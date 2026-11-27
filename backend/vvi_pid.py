"""Pluggable VVI / Altitude-hold PID controller for -force tests and PID assist.

This module isolates the VVI PID interface so an operator-supplied VVI PID
can be dropped into ``VviPidController`` later without altering the
``2.c.(1)-force``, ``2.c.(2a)-force``, or ``2.c.(2b)-force`` test sequences,
reporting, or the uncontrolled Dynamics tests.
"""
from __future__ import annotations

import math
import threading
from typing import Callable, Optional

from backend.bank_pid import slew_position
from backend.multi_pid_runner import PitchPID
from interfaces.multi_pid_transport import MultiPidBrunnerTransport


DEFAULT_VVI_GAINS = {
    'kp_alt': 0.5,       # ft -> fpm outer altitude-to-VVI loop
    'kp_vvi': 0.012,     # fpm -> deg pitch demand
    'ki_vvi': 0.003,     # fpm*s -> deg integrator
    'kd_vvi': 0.006,     # (fpm/s) -> deg damping
    'max_vvi_fpm': 400.0,
    'min_pitch_deg': -15.0,
    'max_pitch_deg': 15.0,
}


class VviPidController:
    """Two-loop Altitude/VVI controller commanding Brunner Pitch axis (0x01).

    Sign convention (matching ``backend.multi_pid_runner.PitchPID``):
      - ``alt_ft``: MSL altitude in feet (+ = climbing).
      - ``vertical_speed_fpm``: rate of climb in ft/min (+ = climbing, - = descending).
      - ``raw_pitch_deg``: raw P3D ``PLANE PITCH DEGREES`` (- = nose up, + = nose down).
        Internally negated to ``control_pitch = -raw_pitch_deg`` so nose-up is +.
      - Output: normalized Brunner Pitch command in [-0.55, +0.65], where positive
        commands pitch stick nose-up.

    To drop in a custom VVI PID later, either:
      1) Pass ``custom_updater(target_vvi_fpm, vertical_speed_fpm, alt_ft, target_alt_ft, control_pitch_deg, dt) -> float``
         returning pitch attitude (deg) for ``output_mode='pitch_deg'`` or normalized stick
         command for ``output_mode='stick'``; OR
      2) Replace ``_compute_pitch_demand`` / ``update`` in this class directly.
    """

    def __init__(
        self,
        target_alt_ft: Optional[float] = None,
        target_vvi_fpm: float = 0.0,
        *,
        kp_alt: float = DEFAULT_VVI_GAINS['kp_alt'],
        kp_vvi: float = DEFAULT_VVI_GAINS['kp_vvi'],
        ki_vvi: float = DEFAULT_VVI_GAINS['ki_vvi'],
        kd_vvi: float = DEFAULT_VVI_GAINS['kd_vvi'],
        max_vvi_fpm: float = DEFAULT_VVI_GAINS['max_vvi_fpm'],
        min_pitch_deg: float = DEFAULT_VVI_GAINS['min_pitch_deg'],
        max_pitch_deg: float = DEFAULT_VVI_GAINS['max_pitch_deg'],
        pitch_pid: Optional[PitchPID] = None,
        custom_updater: Optional[Callable[..., float]] = None,
        output_mode: str = 'pitch_deg',
    ):
        self.target_alt_ft = None if target_alt_ft is None else float(target_alt_ft)
        self.target_vvi_fpm = float(target_vvi_fpm)
        self.kp_alt = float(kp_alt)
        self.kp_vvi = float(kp_vvi)
        self.ki_vvi = float(ki_vvi)
        self.kd_vvi = float(kd_vvi)
        self.max_vvi_fpm = float(max_vvi_fpm)
        self.min_pitch_deg = float(min_pitch_deg)
        self.max_pitch_deg = float(max_pitch_deg)
        self.pitch_pid = pitch_pid if pitch_pid is not None else PitchPID()
        self.custom_updater = custom_updater
        if output_mode not in ('pitch_deg', 'stick'):
            raise ValueError("output_mode must be 'pitch_deg' or 'stick'")
        self.output_mode = output_mode

        self.vvi_integral = 0.0
        self.last_vvi: Optional[float] = None
        self.vvi_rate = 0.0
        self.cmd_pitch_deg = 0.0
        self.cmd_vvi_fpm = self.target_vvi_fpm
        self.stick_command = 0.0
        self.stick_velocity = 0.0
        self._initialized = False

    def reset(
        self,
        *,
        target_alt_ft: Optional[float] = None,
        target_vvi_fpm: Optional[float] = None,
        initial_stick_pos: float = 0.0,
        initial_pitch_deg: float = 0.0,
    ) -> None:
        if target_alt_ft is not None:
            self.target_alt_ft = float(target_alt_ft)
        if target_vvi_fpm is not None:
            self.target_vvi_fpm = float(target_vvi_fpm)
        control_pitch = max(self.min_pitch_deg, min(self.max_pitch_deg, -float(initial_pitch_deg)))
        self.vvi_integral = control_pitch
        self.last_vvi = None
        self.vvi_rate = 0.0
        self.cmd_pitch_deg = control_pitch
        self.cmd_vvi_fpm = self.target_vvi_fpm
        self.pitch_pid.reset(initial_pos=float(initial_stick_pos))
        self.stick_command = float(initial_stick_pos)
        self.stick_velocity = 0.0
        self._initialized = True

    def _desired_vvi_fpm(self, alt_ft: Optional[float]) -> float:
        if self.target_alt_ft is None or alt_ft is None or not math.isfinite(alt_ft):
            self.cmd_vvi_fpm = self.target_vvi_fpm
            return self.cmd_vvi_fpm
        alt_err = self.target_alt_ft - float(alt_ft)
        trim_vvi = max(-self.max_vvi_fpm, min(self.max_vvi_fpm, self.kp_alt * alt_err))
        self.cmd_vvi_fpm = max(-self.max_vvi_fpm, min(self.max_vvi_fpm, self.target_vvi_fpm + trim_vvi))
        return self.cmd_vvi_fpm

    def _compute_pitch_demand(self, target_vvi: float, actual_vvi: float, dt: float) -> float:
        if dt <= 0.0001:
            return self.cmd_pitch_deg
        vvi_err = target_vvi - actual_vvi
        if self.last_vvi is None:
            self.last_vvi = actual_vvi
        raw_rate = (actual_vvi - self.last_vvi) / dt
        self.last_vvi = actual_vvi
        self.vvi_rate += (dt / (0.50 + dt)) * (raw_rate - self.vvi_rate)

        p_term = self.kp_vvi * vvi_err
        d_term = -self.kd_vvi * self.vvi_rate
        trial_i = max(self.min_pitch_deg, min(self.max_pitch_deg, self.vvi_integral + self.ki_vvi * vvi_err * dt))
        trial_raw = p_term + trial_i + d_term
        pushes_further = (trial_raw >= self.max_pitch_deg and vvi_err > 0) or (
            trial_raw <= self.min_pitch_deg and vvi_err < 0
        )
        if not pushes_further:
            self.vvi_integral = trial_i
        demand = max(self.min_pitch_deg, min(self.max_pitch_deg, p_term + self.vvi_integral + d_term))
        max_step = 1.5 * dt
        delta = demand - self.cmd_pitch_deg
        if abs(delta) > max_step:
            self.cmd_pitch_deg += max_step if delta > 0 else -max_step
        else:
            self.cmd_pitch_deg = demand
        return self.cmd_pitch_deg

    def update(
        self,
        *,
        alt_ft: Optional[float],
        vertical_speed_fpm: float,
        raw_pitch_deg: float,
        dt: float,
        measured_stick_pos: Optional[float] = None,
    ) -> float:
        """Compute bounded Brunner Pitch (0x01) command to hold VVI / original altitude."""
        if not self._initialized:
            self.reset(
                initial_stick_pos=measured_stick_pos if measured_stick_pos is not None else 0.0,
                initial_pitch_deg=raw_pitch_deg,
            )
        control_pitch = -float(raw_pitch_deg)
        target_vvi = self._desired_vvi_fpm(alt_ft)

        if self.custom_updater is not None:
            custom_out = float(
                self.custom_updater(
                    target_vvi,
                    float(vertical_speed_fpm),
                    alt_ft,
                    self.target_alt_ft,
                    control_pitch,
                    float(dt),
                )
            )
            if self.output_mode == 'stick':
                desired_stick = max(-0.55, min(0.65, custom_out))
            else:
                self.cmd_pitch_deg = max(self.min_pitch_deg, min(self.max_pitch_deg, custom_out))
                desired_stick = self.pitch_pid.update(self.cmd_pitch_deg, control_pitch, float(dt))
        else:
            cmd_pitch = self._compute_pitch_demand(target_vvi, float(vertical_speed_fpm), float(dt))
            desired_stick = self.pitch_pid.update(cmd_pitch, control_pitch, float(dt))

        self.stick_command, self.stick_velocity = slew_position(
            self.stick_command, self.stick_velocity, desired_stick, float(dt), max_rate=0.28, max_accel=0.90
        )
        if measured_stick_pos is not None and math.isfinite(measured_stick_pos):
            self.stick_command = max(
                float(measured_stick_pos) - 0.28,
                min(float(measured_stick_pos) + 0.28, self.stick_command),
            )
        self.stick_command = max(-0.55, min(0.65, self.stick_command))
        return self.stick_command


def read_pitch_virtual_force(brunner) -> float:
    """Best-effort read of Brunner 0x51 pitch virtual/holding force (N)."""
    if not brunner or not getattr(brunner, 'sock', None) or not hasattr(brunner, 'read_axis'):
        return float('nan')
    try:
        _, _, v_force = brunner.read_axis(0x01)
        val = float(v_force)
        return val if math.isfinite(val) else float('nan')
    except Exception:
        return float('nan')


class ForceAltitudeHold:
    """Pitch-only VVI altitude-hold session helper shared by -force tests."""

    def __init__(
        self,
        target_alt_ft: float,
        target_vvi_fpm: float = 0.0,
        *,
        controller: Optional[VviPidController] = None,
        transport_factory=None,
    ):
        self.target_alt_ft = float(target_alt_ft)
        self.target_vvi_fpm = float(target_vvi_fpm)
        self.controller = controller if controller is not None else VviPidController(
            target_alt_ft=self.target_alt_ft,
            target_vvi_fpm=self.target_vvi_fpm,
        )
        self.custom_transport_factory = transport_factory is not None
        self.transport_factory = transport_factory or (
            lambda b: MultiPidBrunnerTransport(b, threading.Event(), pitch_enabled=True, bank_enabled=False, yaw_enabled=False)
        )
        self.transport = None
        self.last_time: Optional[float] = None
        self.measured_pitch_pos = 0.0
        self.last_command = 0.0

    def engage(self, brunner, now: float, *, alt_ft: Optional[float] = None, pitch_deg: float = 0.0) -> None:
        if alt_ft is not None and math.isfinite(alt_ft):
            self.target_alt_ft = float(alt_ft)
        start_pitch = 0.0
        sock = getattr(brunner, 'sock', None) if brunner is not None else None
        can_send = self.custom_transport_factory or callable(getattr(sock, 'send', None))
        if brunner and sock and can_send:
            self.transport = self.transport_factory(brunner)
            engaged_pitch, _, _ = self.transport.engage()
            if engaged_pitch is not None and math.isfinite(engaged_pitch):
                start_pitch = float(engaged_pitch)
        self.measured_pitch_pos = start_pitch
        self.last_command = start_pitch
        self.last_time = float(now)
        self.controller.reset(
            target_alt_ft=self.target_alt_ft,
            target_vvi_fpm=self.target_vvi_fpm,
            initial_stick_pos=start_pitch,
            initial_pitch_deg=pitch_deg,
        )

    def step(self, frame: dict, now: float) -> float:
        dt = 0.04 if self.last_time is None else max(0.005, min(0.5, float(now) - self.last_time))
        self.last_time = float(now)
        if self.transport is not None:
            self.transport.refresh_watchdog()
            pitch_pos, _, _ = self.transport.read_positions()
            if pitch_pos is not None and math.isfinite(pitch_pos):
                self.measured_pitch_pos = float(pitch_pos)
        alt_val = frame.get('alt_ft', frame.get('pressure_alt'))
        try:
            alt_ft = float(alt_val) if alt_val is not None else None
            if alt_ft is not None and not math.isfinite(alt_ft):
                alt_ft = None
        except (TypeError, ValueError):
            alt_ft = None
        vvi_val = frame.get('vertical_speed', frame.get('vertical_speed_fpm', 0.0))
        try:
            vvi_fpm = float(vvi_val) if math.isfinite(float(vvi_val)) else 0.0
        except (TypeError, ValueError):
            vvi_fpm = 0.0
        raw_pitch = float(frame.get('pitch_deg', frame.get('pitch_angle', 0.0)))
        self.last_command = self.controller.update(
            alt_ft=alt_ft,
            vertical_speed_fpm=vvi_fpm,
            raw_pitch_deg=raw_pitch,
            dt=dt,
            measured_stick_pos=self.measured_pitch_pos,
        )
        if self.transport is not None:
            self.transport.command(pitch_pos=self.last_command, roll_pos=None, rudder_pos=None)
        return self.last_command

    def release(self) -> None:
        if self.transport is not None:
            try:
                self.transport.release()
            finally:
                self.transport = None
