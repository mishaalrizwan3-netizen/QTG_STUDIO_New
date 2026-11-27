"""PID Assist for Automatic Tests.

Allows running any combination of Airspeed PID (pitch axis), Bank PID (roll axis),
and Yaw PID (rudder axis) concurrently with an automatic test, while preserving
the test's own automatic movement on its owned axis.
"""
from __future__ import annotations

import math
import threading
import time
from typing import Dict, Any, Optional

from backend.bank_pid import BankPID, slew_position
from backend.yaw_pid import YawPID, HeadingController, HeadingBankHandoff
from backend.multi_pid_runner import PitchPID, AirspeedController
from interfaces.multi_pid_transport import MultiPidBrunnerTransport


def _get_num(data: Dict[str, Any], *keys: str) -> float:
    if not isinstance(data, dict):
        return float('nan')
    for key in keys:
        if key in data:
            try:
                val = float(data[key])
                if math.isfinite(val):
                    return val
            except (TypeError, ValueError):
                pass
    return float('nan')


class PidAssist:
    """Manages secondary PID axis control alongside an automatic test."""

    def __init__(self, context: Dict[str, Any], owned_axis_mask: int = 0):
        """
        owned_axis_mask: bitmask of axes controlled by the test itself
          0x01: Pitch (e.g. 2.a.(1), 2.c.(8b))
          0x02: Roll  (e.g. 2.a.(2))
          0x04: Rudder (e.g. 2.a.(3))
        PIDs on owned axes will not fight the test's own commands.
        """
        self.context = dict(context or {})
        # Test runners publish their context in each status snapshot. Keep only
        # lightweight command feedback there; no test sequence is changed.
        self._published_context = context if isinstance(context, dict) else None
        self.owned_axis_mask = int(owned_axis_mask)
        active_pids = self.context.get('active_pids') or {}

        self.pitch_requested = bool(active_pids.get('airspeed', False))
        self.bank_requested = bool(active_pids.get('bank', False))
        self.yaw_requested = bool(active_pids.get('yaw', False))

        # Only enable PID on an axis if the test itself does NOT own that axis
        self.pitch_enabled = self.pitch_requested and not bool(self.owned_axis_mask & 0x01)
        # Yaw heading hold also needs roll when that axis is free.
        self.bank_enabled = (self.bank_requested or self.yaw_requested) and not bool(self.owned_axis_mask & 0x02)
        self.yaw_enabled = self.yaw_requested and not bool(self.owned_axis_mask & 0x04)

        self.pid_axis_mask = 0
        if self.pitch_enabled:
            self.pid_axis_mask |= 0x01
        if self.bank_enabled:
            self.pid_axis_mask |= 0x02
        if self.yaw_enabled:
            self.pid_axis_mask |= 0x04

        if self.pitch_enabled and self.context.get('pitch_sweep_airspeed_capture'):
            # Faster, better-damped capture only for Test 2.a.(1). Generic and
            # normal-climb PID construction remains exactly as before.
            self.airspeed_ctrl = AirspeedController(
                kp=0.24, ki=0.008, kd=0.32,
                rate_filter_tau=0.60, pitch_slew_rate=1.25,
                capture_band=4.0, capture_brake_gain=0.55, capture_slew_rate=2.50)
        else:
            self.airspeed_ctrl = AirspeedController() if self.pitch_enabled else None
        self.pitch_pid = PitchPID() if self.pitch_enabled else None
        self.bank_pid = BankPID() if self.bank_enabled else None
        self.yaw_pid = YawPID() if self.yaw_enabled else None
        self.heading_ctrl = HeadingController() if (self.yaw_enabled and self.bank_enabled) else None

        pid_cond = self.context.get('pid_conditions') or {}
        init_cond = self.context.get('initial_conditions') or {}
        raw_ias = pid_cond.get('target_ias', init_cond.get('ias_kt', 80.0))
        try:
            self.target_ias = float(raw_ias)
        except (ValueError, TypeError):
            self.target_ias = 80.0
        raw_minimum_ias = self.context.get('pid_minimum_airspeed')
        self.minimum_airspeed = (float(raw_minimum_ias)
                                 if raw_minimum_ias is not None else None)

        raw_hdg = pid_cond.get('target_heading', self.context.get('target_heading_mag', init_cond.get('heading_deg', 0.0)))
        try:
            self.target_hdg = float(raw_hdg) % 360.0
        except (ValueError, TypeError):
            self.target_hdg = 0.0

        raw_bank = pid_cond.get('target_bank', self.context.get('target_bank', init_cond.get('bank_deg', 0.0)))
        try:
            self.target_bank = float(raw_bank)
        except (ValueError, TypeError):
            self.target_bank = 0.0

        self.last_time: Optional[float] = None
        self.latest_commands: Dict[str, float] = {}
        self.transport = None
        self.pitch_command = self.roll_command = self.rudder_command = 0.0
        self.pitch_velocity = self.roll_velocity = self.rudder_velocity = 0.0
        self.measured_pitch = self.measured_roll = self.measured_rudder = 0.0
        self.bank_handoff = HeadingBankHandoff()
        self._flight_pitch_initialized = False
        self.stable_since: Optional[float] = None
        self.stabilization_started: Optional[float] = None
        self.stabilization_hold_s = 5.0
        tolerances = self.context.get('pid_tolerances') or {}
        self.tolerances = {
            'airspeed': float(tolerances.get('airspeed', 3.0)),
            'bank': float(tolerances.get('bank', 1.0)),
            'yaw': float(tolerances.get('yaw', 2.0)),
        }
        self.stabilization_achieved = False
        self.stabilization_fault: Optional[str] = None

    @property
    def has_active_pids(self) -> bool:
        return bool(self.pid_axis_mask)

    def total_axis_mask(self) -> int:
        return self.owned_axis_mask | self.pid_axis_mask

    def reset(self, initial_pitch: float = 0.0, initial_roll: float = 0.0, initial_rudder: float = 0.0):
        if self.airspeed_ctrl:
            self.airspeed_ctrl.reset(initial_pitch=initial_pitch)
        if self.pitch_pid:
            self.pitch_pid.reset(initial_pos=initial_pitch)
        if self.bank_pid:
            self.bank_pid.reset(initial_output=initial_roll)
        if self.yaw_pid:
            self.yaw_pid.reset(initial_output=initial_rudder)
        if self.heading_ctrl:
            self.heading_ctrl.reset(initial_bank=0.0)
        self.last_time = None
        self.latest_commands.clear()

    def begin_stabilization(self, now: float, brunner: Any) -> None:
        """Engage only permitted PID axes after aircraft initialization."""
        conflicting = []
        if self.pitch_requested and not self.pitch_enabled:
            conflicting.append('Airspeed PID conflicts with the test pitch controller')
        if self.bank_requested and not self.bank_enabled:
            conflicting.append('Bank PID conflicts with the test roll controller')
        if self.yaw_requested and not self.yaw_enabled:
            conflicting.append('Yaw PID conflicts with the test rudder controller')
        if conflicting:
            self.stabilization_fault = '; '.join(conflicting)
            raise RuntimeError(self.stabilization_fault)
        if not self.has_active_pids:
            return
        if not brunner or not getattr(brunner, 'sock', None):
            self.stabilization_fault = 'Selected PID failed to start: CLS2sim is not connected'
            raise RuntimeError(self.stabilization_fault)
        try:
            self.transport = MultiPidBrunnerTransport(
                brunner, threading.Event(), pitch_enabled=self.pitch_enabled,
                bank_enabled=self.bank_requested, yaw_enabled=self.yaw_enabled)
            start_pitch, start_roll, start_rudder = self.transport.engage()
            brunner._multi_pid_transport = self.transport
        except Exception as exc:
            self.stabilization_fault = f'Selected PID failed to start: {exc}'
            raise RuntimeError(self.stabilization_fault) from exc
        self.reset(start_pitch, start_roll, start_rudder)
        self.pitch_command, self.roll_command, self.rudder_command = start_pitch, start_roll, start_rudder
        self.measured_pitch, self.measured_roll, self.measured_rudder = start_pitch, start_roll, start_rudder
        self.pitch_velocity = self.roll_velocity = self.rudder_velocity = 0.0
        self._flight_pitch_initialized = False
        self.stabilization_started = float(now)
        self.stable_since = None
        self.stabilization_achieved = False
        self.stabilization_fault = None

    @staticmethod
    def _heading_error(actual: float, target: float) -> float:
        return ((actual - target + 180.0) % 360.0) - 180.0

    def update_stabilization(self, now: float, telemetry: Dict[str, Any], brunner: Any) -> bool:
        """Run selected PIDs and require one simultaneous continuous 5 s window."""
        if not self.has_active_pids:
            return True
        if self.stabilization_started is None:
            raise RuntimeError('PID stabilization was not started after initialization')
        commands = self.update(now, telemetry, brunner)
        expected = set()
        if self.pitch_enabled:
            expected.add('pitch')
        if self.bank_enabled:
            expected.add('roll')
        if self.yaw_enabled:
            expected.add('rudder')
        command_ok = expected.issubset(commands)
        ias = _get_num(telemetry, 'airspeed_indicated', 'indicated_airspeed', 'ias_kt')
        bank = _get_num(telemetry, 'bank_angle', 'bank_deg')
        heading = _get_num(telemetry, 'true_heading', 'magnetic_heading', 'heading_deg')
        checks = {}
        if self.pitch_requested:
            checks['airspeed'] = (ias, self.target_ias, self.tolerances['airspeed'])
        if self.bank_requested:
            checks['bank'] = (bank, self.target_bank, self.tolerances['bank'])
        if self.yaw_requested:
            checks['yaw'] = (heading, self.target_hdg, self.tolerances['yaw'])
        axes = {}
        within_all = command_ok
        for key, (actual, target, tolerance) in checks.items():
            error = self._heading_error(actual, target) if key == 'yaw' and math.isfinite(actual) else actual - target
            within = math.isfinite(actual) and abs(error) <= tolerance
            axes[key] = dict(target=target, actual=actual if math.isfinite(actual) else None,
                             tolerance=tolerance, within=within, running=command_ok)
            if (key == 'airspeed' and self.minimum_airspeed is not None
                    and (not math.isfinite(actual) or actual < self.minimum_airspeed)):
                within = False
                axes[key]['within'] = False
                axes[key]['minimum'] = self.minimum_airspeed
            within_all = within_all and within
        if within_all:
            if self.stable_since is None:
                self.stable_since = float(now)
        else:
            self.stable_since = None
        stable_elapsed = 0.0 if self.stable_since is None else max(0.0, now - self.stable_since)
        self.stabilization_achieved = stable_elapsed >= self.stabilization_hold_s
        runtime = dict(command_axes=sorted(commands), updated_at=time.time(), axes=axes,
                       stabilization_elapsed_s=stable_elapsed,
                       stabilization_remaining_s=max(0.0, self.stabilization_hold_s - stable_elapsed),
                       stabilization_required_s=self.stabilization_hold_s,
                       stabilization_achieved=self.stabilization_achieved,
                       stabilization_fault=self.stabilization_fault)
        if self._published_context is not None:
            self._published_context['pid_runtime'] = runtime
        return self.stabilization_achieved

    def release_for_test_handoff(self) -> float:
        """Release a stabilization-only pitch PID and return its measured position."""
        if not self.pitch_enabled or self.bank_enabled or self.yaw_enabled:
            raise RuntimeError('Pitch sweep handoff requires Airspeed PID only')
        position = float(self.measured_pitch)
        if not math.isfinite(position):
            raise RuntimeError('PID pitch handoff position is unavailable')
        if self.transport is None:
            raise RuntimeError('PID pitch transport is unavailable for handoff')
        errors = self.transport.release()
        if errors:
            raise RuntimeError('PID pitch handoff failed: ' + '; '.join(errors))
        self.transport = None
        self.pitch_enabled = False
        self.pid_axis_mask &= ~0x01
        self.latest_commands.clear()
        if self._published_context is not None:
            runtime = dict(self._published_context.get('pid_runtime') or {})
            runtime.update(handoff_complete=True, handoff_pitch_position=position,
                           command_axes=[], stabilization_achieved=True)
            self._published_context['pid_runtime'] = runtime
        return max(-0.95, min(0.95, position))

    def update(self, now: float, telemetry: Dict[str, Any], brunner: Any = None) -> Dict[str, float]:
        """
        Compute PID outputs from current telemetry and write commands to Brunner.
        Returns dict with keys: 'pitch', 'roll', 'rudder' for active axes.
        """
        if not self.has_active_pids:
            return {}

        dt = 0.025 if self.last_time is None else max(0.001, min(0.5, now - self.last_time))
        self.last_time = now

        ias = _get_num(telemetry, 'airspeed_indicated', 'indicated_airspeed', 'ias_kt')
        pitch = _get_num(telemetry, 'pitch_angle', 'pitch_deg')
        bank = _get_num(telemetry, 'bank_angle', 'bank_deg')
        hdg = _get_num(telemetry, 'true_heading', 'magnetic_heading', 'heading_deg')
        sideslip = _get_num(telemetry, 'sideslip', 'sideslip_deg', 'beta')
        yaw_rate = _get_num(telemetry, 'yaw_rate', 'yaw_rate_deg_s')
        if not math.isfinite(yaw_rate):
            yaw_rate = 0.0

        commands: Dict[str, float] = {}

        # Original all-three PID convention: P3D pitch is negated before the
        # attitude loop. Loader commands remain in measured CLS2Sim polarity.
        control_pitch = -pitch if math.isfinite(pitch) else pitch
        if self.pitch_enabled and math.isfinite(control_pitch) and not self._flight_pitch_initialized:
            self.airspeed_ctrl.reset(initial_pitch=control_pitch)
            self._flight_pitch_initialized = True
        if self.transport is not None:
            self.transport.refresh_watchdog()
            self.transport.read_positions()
            if self.transport.last_pitch_feedback is not None:
                self.measured_pitch = self.transport.last_pitch_feedback
            if self.transport.last_roll_feedback is not None:
                self.measured_roll = self.transport.last_roll_feedback
            if self.transport.last_rudder_feedback is not None:
                self.measured_rudder = self.transport.last_rudder_feedback

        # 1. Airspeed PID on Pitch axis (0x01)
        if self.pitch_enabled and math.isfinite(ias) and math.isfinite(pitch):
            cmd_pitch = self.airspeed_ctrl.update(self.target_ias, ias, dt, control_pitch)
            desired = self.pitch_pid.update(cmd_pitch, control_pitch, dt)
            self.pitch_command, self.pitch_velocity = slew_position(
                self.pitch_command, self.pitch_velocity, desired, dt, max_rate=.28, max_accel=.90)
            self.pitch_command = max(self.measured_pitch - .28,
                                     min(self.measured_pitch + .28, self.pitch_command))
            commands['pitch'] = self.pitch_command

        # 2. Bank PID on Roll axis (0x02)
        if self.bank_enabled and math.isfinite(bank):
            if self.heading_ctrl and math.isfinite(hdg):
                cmd_bank = self.heading_ctrl.update(self.target_hdg, hdg, yaw_rate, dt)
            else:
                cmd_bank = self.target_bank
            desired = self.bank_pid.update(cmd_bank, bank, dt)
            self.roll_command, self.roll_velocity = slew_position(
                self.roll_command, self.roll_velocity, desired, dt, max_rate=.28, max_accel=.90)
            self.roll_command = max(self.measured_roll - .28,
                                    min(self.measured_roll + .28, self.roll_command))
            commands['roll'] = self.roll_command

        # 3. Yaw PID on Rudder axis (0x04)
        if self.yaw_enabled and math.isfinite(sideslip):
            desired = self.yaw_pid.update(sideslip, yaw_rate, dt)
            self.rudder_command, self.rudder_velocity = slew_position(
                self.rudder_command, self.rudder_velocity, desired, dt, max_rate=.28, max_accel=.90)
            self.rudder_command = max(self.measured_rudder - .28,
                                      min(self.measured_rudder + .28, self.rudder_command))
            self.rudder_command = max(-.40, min(.40, self.rudder_command))
            commands['rudder'] = self.rudder_command

        if self.transport is not None:
            self.transport.command(commands.get('pitch'), commands.get('roll'), commands.get('rudder'))
        elif brunner and getattr(brunner, 'sock', None) and hasattr(brunner, 'write_setting'):
            # Compatibility for isolated unit users that have not engaged the
            # shared pre-record transport.
            for axis, key in ((0x01, 'pitch'), (0x02, 'roll'), (0x04, 'rudder')):
                if key in commands:
                    brunner.write_setting(axis, 0x80, 'f', commands[key])

        self.latest_commands = commands
        if self._published_context is not None:
            self._published_context['pid_runtime'] = {
                'command_axes': list(commands),
                'updated_at': time.time(),
            }
            started = self.stabilization_started if self.stabilization_started is not None else now
            self._published_context['climb_pid_sample'] = {
                'time_s': max(0.0, now - started),
                'ias': ias if math.isfinite(ias) else None,
                'pitch': pitch if math.isfinite(pitch) else None,
                'bank': bank if math.isfinite(bank) else None,
                'heading': hdg if math.isfinite(hdg) else None,
                'target_ias': self.target_ias,
                'target_pitch': commands.get('pitch'),
                'target_bank': self.target_bank,
                'target_heading': self.target_hdg,
                'pitch_cmd': commands.get('pitch'),
                'roll_cmd': commands.get('roll'),
                'rudder_cmd': commands.get('rudder'),
                'pitch_pos': self.measured_pitch if self.pitch_enabled else None,
                'roll_pos': self.measured_roll if self.bank_enabled else None,
                'rudder_pos': self.measured_rudder if self.yaw_enabled else None,
            }
        return commands
