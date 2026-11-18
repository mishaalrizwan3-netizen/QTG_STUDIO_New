"""
Multi-PID Runner for QTG Studio.
Supports running any individual PID or any combination of PIDs simultaneously:
  - Airspeed PID (Pitch Axis 1)
  - Bank PID (Roll Axis 2 - Wings Level 0.0°)
  - Yaw PID (Roll Axis 2 & Rudder Axis 4 - Heading Hold & Yaw Damper)

When all 3 are ticked, all 3 operate simultaneously on the Brunner hardware
using the exact coordinated control logic from PID Controller.
"""
from __future__ import annotations

import csv
from datetime import datetime
import json
import math
import os
from pathlib import Path
import struct
import sys
import threading
import time
from typing import Optional, Tuple

from backend.bank_pid import BankPID, slew_position
from backend.yaw_pid import HeadingController, YawPID, HeadingBankHandoff
from interfaces.multi_pid_transport import MultiPidBrunnerTransport


class PitchPID:
    """Inner-loop pitch attitude controller driving Brunner Elevator (Axis 1)."""
    def __init__(self, kp=0.030, ki=0.004, kd=0.045, invert=False):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.invert = bool(invert)
        self.integral = 0.0
        self.last_pitch = None
        self.pitch_rate = 0.0
        self.output = 0.0

    def reset(self, initial_pos=0.0):
        self.integral = -float(initial_pos) if self.invert else float(initial_pos)
        self.last_pitch = None
        self.pitch_rate = 0.0
        self.output = float(initial_pos)

    def update(self, target_deg: float, actual_deg: float, dt: float) -> float:
        if dt <= 0.0001:
            return self.output
        error = target_deg - actual_deg
        if self.last_pitch is None:
            self.last_pitch = actual_deg
        raw_rate = (actual_deg - self.last_pitch) / dt
        self.last_pitch = actual_deg
        self.pitch_rate += (dt / (0.24 + dt)) * (raw_rate - self.pitch_rate)

        p_term = self.kp * error
        d_term = -self.kd * self.pitch_rate
        trial_i = max(-0.55, min(0.65, self.integral + self.ki * error * dt))
        trial_raw = p_term + trial_i + d_term
        low, high = (-0.65, 0.55) if self.invert else (-0.55, 0.65)
        pushes_further = (trial_raw >= high and error > 0) or (trial_raw <= low and error < 0)
        if abs(error) > 0.12 and not pushes_further:
            self.integral = trial_i
        effort = p_term + self.integral + d_term
        if self.invert:
            effort = -effort
        self.output = max(-0.55, min(0.65, effort))
        return self.output


class AirspeedController:
    """Outer-loop airspeed controller commanding pitch attitude to maintain target IAS."""
    def __init__(self, kp=0.24, ki=0.008, kd=0.32, min_pitch=-15.0, max_pitch=15.0):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.min_pitch = float(min_pitch)
        self.max_pitch = float(max_pitch)
        self.integral = 4.0
        self.last_ias = None
        self.ias_rate = 0.0
        self.cmd_pitch = 4.0

    def reset(self, initial_pitch=4.0):
        self.integral = max(self.min_pitch, min(self.max_pitch, float(initial_pitch)))
        self.last_ias = None
        self.ias_rate = 0.0
        self.cmd_pitch = float(initial_pitch)

    def update(self, target_ias: float, actual_ias: float, dt: float, current_pitch: float) -> float:
        if dt <= 0.0001:
            return self.cmd_pitch
        spd_error = actual_ias - target_ias
        if self.last_ias is None:
            self.last_ias = actual_ias
        raw_rate = (actual_ias - self.last_ias) / dt
        self.last_ias = actual_ias
        self.ias_rate += (dt / (0.60 + dt)) * (raw_rate - self.ias_rate)

        p_term = self.kp * spd_error
        if not ((self.cmd_pitch >= self.max_pitch and spd_error > 0) or
                (self.cmd_pitch <= self.min_pitch and spd_error < 0)):
            self.integral += self.ki * spd_error * dt
            self.integral = max(self.min_pitch, min(self.max_pitch, self.integral))

        i_term = self.integral
        d_term = self.kd * self.ias_rate
        pitch_demand = p_term + i_term + d_term

        desired_target = max(self.min_pitch, min(self.max_pitch, pitch_demand))
        max_step = 1.25 * dt
        delta = desired_target - self.cmd_pitch
        if abs(delta) > max_step:
            self.cmd_pitch += max_step if delta > 0 else -max_step
        else:
            self.cmd_pitch = desired_target
        return self.cmd_pitch


class MultiPidStabilizationGate:
    """
    Stabilization gate for multi-PID engagement.
    All ticked controllers must be simultaneously stable for hold_s (5.0s) continuously.
    If any active axis leaves its tolerance, the timer resets to 0.
    """
    def __init__(self, pitch_enabled: bool, bank_enabled: bool, yaw_enabled: bool,
                 hold_s: float = 5.0, ias_tol: float = 1.5, bank_tol: float = 3.0, hdg_tol: float = 2.0):
        self.pitch_enabled = pitch_enabled
        self.bank_enabled = bank_enabled
        self.yaw_enabled = yaw_enabled
        self.hold_s = float(hold_s)
        self.ias_tol = float(ias_tol)
        self.bank_tol = float(bank_tol)
        self.hdg_tol = float(hdg_tol)
        self.since: Optional[float] = None

    def reset(self):
        self.since = None

    def elapsed_stable(self, now: float) -> float:
        return 0.0 if self.since is None else max(0.0, now - self.since)

    def is_currently_within_tolerances(self, ias: float, bank: float, hdg: float, target_ias: float, target_hdg: float) -> bool:
        if self.pitch_enabled:
            if not math.isfinite(ias) or abs(ias - target_ias) > self.ias_tol:
                return False

        if self.yaw_enabled:
            if not math.isfinite(hdg) or not math.isfinite(bank):
                return False
            hdg_err = (target_hdg - hdg + 180.0) % 360.0 - 180.0
            if abs(hdg_err) > self.hdg_tol or abs(bank) > self.bank_tol:
                return False
        elif self.bank_enabled:
            if not math.isfinite(bank) or abs(bank) > self.bank_tol:
                return False

        return True

    def update(self, now: float, ias: float, bank: float, hdg: float, target_ias: float, target_hdg: float) -> bool:
        within = self.is_currently_within_tolerances(ias, bank, hdg, target_ias, target_hdg)
        if not within:
            self.since = None
            return False
        if self.since is None:
            self.since = now
        return (now - self.since) >= self.hold_s


def fresh_telemetry(window, timeout_s=2.0, max_retries=3):
    last_exc = None
    for attempt in range(max_retries):
        try:
            seq = window.p3d.get_telemetry_sequence()
            if not window.p3d.request_telemetry_once():
                time.sleep(0.05)
                continue
            data = window.p3d.wait_for_fresh_telemetry(seq, timeout_sec=timeout_s)
            if data is not None:
                return data
        except Exception as exc:
            last_exc = exc
            time.sleep(0.05)
    raise RuntimeError(f'P3D telemetry stream unavailable: {last_exc or "timeout"}')


def prepare(window, base):
    """Apply the selected conditions through the existing Manual preparation."""
    if window._session_context.get('initial_conditions') is None:
        raise RuntimeError('PID mode requires selected initial conditions')
    window.active_manual_ref_path = None
    window.selected_ref_csv = None
    window.sig_update_stage.emit(
        getattr(base, 'STAGE_INITIALIZING', 'Initializing aircraft'),
        'Multi-PID: Initializing aircraft state in Prepar3D...'
    )
    return base.SuperMushakLoggerApp.start_manual_test(window)


def loop(window, base, owner_alive=None):
    """
    Main Multi-PID execution loop.
    Controls enabled axes, waits for 5-second continuous stabilization, then records.
    """
    device = window.brunner
    context = getattr(window, '_session_context', {})

    active_pids = context.get('active_pids', {})
    pitch_enabled = bool(active_pids.get('airspeed', False))
    bank_enabled = bool(active_pids.get('bank', False))
    yaw_enabled = bool(active_pids.get('yaw', False))

    # Backward compatibility with single-selection method names
    method = getattr(window, '_control_method', None) or context.get('control_method')
    if not (pitch_enabled or bank_enabled or yaw_enabled):
        if method == 'bank_pid':
            bank_enabled = True
        elif method == 'yaw_pid':
            yaw_enabled = True
        elif method == 'pid':
            pitch_enabled = True
        else:
            # Default to all 3 if multi_pid called without explicit active_pids
            pitch_enabled = bank_enabled = yaw_enabled = True

    target_ias = float((context.get('pid_conditions') or {}).get('target_ias', 80.0))
    target_hdg = float(context.get('target_heading_mag',
                        (context.get('initial_conditions') or {}).get('heading_deg', 0.0)))
    target_hdg = target_hdg % 360.0

    hold_duration_s = float(context.get('multi_pid_hold_sec', 5.0))

    transport = MultiPidBrunnerTransport(device, window.stop_worker_event,
                                         pitch_enabled=pitch_enabled,
                                         bank_enabled=bank_enabled,
                                         yaw_enabled=yaw_enabled)
    device._multi_pid_transport = transport
    window.multi_pid_transport = transport
    window.multi_pid_fault = None

    roll_needed = bank_enabled or yaw_enabled
    rows = []  # Cleanup must also work if the first telemetry request fails.

    try:
        # Aircraft preparation has finished and deliberately leaves P3D paused.
        # Complete one unpause handoff before taking physical axis ownership.
        window.p3d.unpause_sim()
        ready_deadline = time.monotonic() + 20.
        while True:
            if window.stop_worker_event.is_set():
                raise RuntimeError('Multi-PID startup cancelled')
            data = fresh_telemetry(window, timeout_s=2.0, max_retries=3)
            if window.p3d.simulation_running():
                break
            if time.monotonic() >= ready_deadline:
                raise RuntimeError('P3D did not confirm running state after initialization')
            window.stop_worker_event.wait(.04)
        start_pitch, start_roll, start_rudder = transport.engage()

        # Instantiate enabled controllers
        ias_ctrl = AirspeedController(kp=0.24, ki=0.008, kd=0.32, min_pitch=-15.0, max_pitch=15.0)
        pitch_pid = PitchPID(kp=0.030, ki=0.004, kd=0.045, invert=False)
        pitch_pid.reset(initial_pos=start_pitch)
        actual_pitch = -float(data.get('pitch_angle', 0.0))
        ias_ctrl.reset(initial_pitch=actual_pitch)

        heading_ctrl = HeadingController(kp=0.80, ki=0.005, kd=0.70, max_bank=20.0, invert=False)
        bank_pid = BankPID(kp=0.045, ki=0.0025, kd=0.035, invert=False)
        bank_pid.reset(initial_output=start_roll)
        bank_handoff = HeadingBankHandoff()
        yaw_pid = YawPID(kp=0.025, ki=0.0020, kd=0.035, invert=False)
        yaw_pid.reset(initial_output=start_rudder)

        actual_bank = float(data.get('bank_angle', 0.0))
        heading_ctrl.reset(initial_bank=actual_bank)

        pitch_cmd = start_pitch
        pitch_vel = 0.0
        roll_cmd = start_roll
        roll_vel = 0.0
        rudder_cmd = start_rudder
        rudder_vel = 0.0

        gate = MultiPidStabilizationGate(pitch_enabled=pitch_enabled,
                                         bank_enabled=bank_enabled,
                                         yaw_enabled=yaw_enabled,
                                         hold_s=hold_duration_s)

        phase = 'stabilizing'
        control_started = time.monotonic()
        last_loop_time = control_started
        recording_started = None

        mismatch_pitch = None
        mismatch_roll = None
        mismatch_rudder = None

        pause_started = None
        rows = []

        window.multi_pid_active = window._external_control_active = True

        while not window.stop_worker_event.is_set():
            now = time.monotonic()
            dt = max(0.001, min(0.10, now - last_loop_time))
            last_loop_time = now

            if window.stop_worker_event.is_set():
                break

            data = fresh_telemetry(window, timeout_s=1.5, max_retries=2)
            is_paused = not window.p3d.simulation_running()

            if is_paused:
                gate.reset()  # Every pause breaks the continuous five-second hold.
                if phase == 'recording':
                    raise RuntimeError('Prepar3D paused during active recording run.')
                if pause_started is None:
                    pause_started = now
                if now - pause_started > 2.0:
                    gate.reset()
                    window.sig_update_stage.emit(
                        getattr(base, 'STAGE_INITIALIZING', 'Initializing'),
                        'P3D paused; waiting for unpause before resuming stabilization...'
                    )
                time.sleep(0.04)
                continue
            else:
                pause_started = None

            actual_ias = float(data.get('airspeed_indicated', data.get('ias_kt', 80.0)))
            actual_pitch = -float(data.get('pitch_angle', 0.0))
            actual_bank = float(data.get('bank_angle', 0.0))
            actual_hdg = float(data.get('magnetic_heading', data.get('heading_deg', 0.0))) % 360.0
            yaw_rate = float(data.get('yaw_rate', 0.0))
            sideslip = float(data.get('sideslip', 0.0))

            # 1. Pitch Axis Control (Airspeed PID)
            if pitch_enabled:
                desired_target_pitch = ias_ctrl.update(target_ias, actual_ias, dt, actual_pitch)
                desired_pitch_effort = pitch_pid.update(desired_target_pitch, actual_pitch, dt)
                pitch_cmd, pitch_vel = slew_position(pitch_cmd, pitch_vel, desired_pitch_effort, dt, max_rate=0.28, max_accel=0.90)

            # 2. Roll & Rudder Control (Bank PID & Yaw/Heading PID with Handoff)
            if roll_needed:
                if yaw_enabled:
                    demanded_bank = heading_ctrl.update(target_hdg, actual_hdg, yaw_rate, dt)
                else:
                    demanded_bank = 0.0

                captured = bank_handoff.update(
                    bank_enabled, yaw_enabled, target_hdg, actual_hdg, actual_bank,
                    heading_ctrl.hdg_rate, dt, True
                )

                if bank_enabled and yaw_enabled and captured:
                    effective_target_bank = 0.0
                    heading_ctrl.integral = 0.0
                    heading_ctrl.cmd_bank = 0.0
                elif yaw_enabled:
                    effective_target_bank = demanded_bank
                elif bank_enabled:
                    effective_target_bank = 0.0
                else:
                    effective_target_bank = 0.0

                desired_roll = bank_pid.update(effective_target_bank, actual_bank, dt)
                roll_cmd, roll_vel = slew_position(roll_cmd, roll_vel, desired_roll, dt, max_rate=0.28, max_accel=0.90)

                if yaw_enabled and transport.rudder_available:
                    desired_yaw = yaw_pid.update(sideslip, yaw_rate, dt, heading_ctrl.hdg_rate)
                    rudder_cmd, rudder_vel = slew_position(rudder_cmd, rudder_vel, desired_yaw, dt, max_rate=0.28, max_accel=0.90)

            # Refresh enabled physical feedback before watchdog-checked commands.
            transport.read_positions()

            # Send hardware commands
            transport.command(
                pitch_pos=pitch_cmd if pitch_enabled else None,
                roll_pos=roll_cmd if roll_needed else None,
                rudder_pos=rudder_cmd if (yaw_enabled and transport.rudder_available) else None
            )

            # Feedback tracking validation
            measured_pitch = transport.last_pitch_feedback or 0.0
            measured_roll = transport.last_roll_feedback or 0.0
            measured_rudder = transport.last_rudder_feedback or 0.0

            if pitch_enabled:
                if abs(pitch_cmd - measured_pitch) > 0.30:
                    mismatch_pitch = now if mismatch_pitch is None else mismatch_pitch
                    if now - mismatch_pitch > 2.5:
                        raise RuntimeError('BRUNNER pitch loader did not track commands within 2.5s')
                else:
                    mismatch_pitch = None

            if roll_needed:
                if abs(roll_cmd - measured_roll) > 0.30:
                    mismatch_roll = now if mismatch_roll is None else mismatch_roll
                    if now - mismatch_roll > 2.5:
                        raise RuntimeError('BRUNNER roll loader did not track commands within 2.5s')
                else:
                    mismatch_roll = None

            if yaw_enabled and transport.rudder_available:
                if abs(rudder_cmd - measured_rudder) > 0.30:
                    mismatch_rudder = now if mismatch_rudder is None else mismatch_rudder
                    if now - mismatch_rudder > 2.5:
                        raise RuntimeError('BRUNNER rudder loader did not track commands within 2.5s')
                else:
                    mismatch_rudder = None

            # Stabilization Phase & 5-Second Gate
            if phase == 'stabilizing':
                stable = gate.update(now, actual_ias, actual_bank, actual_hdg, target_ias, target_hdg)
                stable_elapsed = gate.elapsed_stable(now)

                if stable:
                    phase = 'recording'
                    recording_started = now
                    window.is_recording = True
                    window.sig_update_stage.emit(
                        getattr(base, 'STAGE_RECORDING', 'Recording'),
                        f'Multi-PID stabilized for {hold_duration_s:.1f}s. Recording active.'
                    )
                else:
                    active_desc = []
                    if pitch_enabled:
                        active_desc.append(f'IAS {actual_ias:.1f}/{target_ias:.1f}kt')
                    if yaw_enabled:
                        active_desc.append(f'Hdg {actual_hdg:.1f}/{target_hdg:.1f}°')
                    if bank_enabled:
                        active_desc.append(f'Bank {actual_bank:+.1f}/0.0°')
                    status_text = f"Stabilizing: {', '.join(active_desc)} — Held {stable_elapsed:.1f}/{hold_duration_s:.1f}s"
                    window.sig_update_stage.emit(getattr(base, 'STAGE_INITIALIZING', 'Initializing'), status_text)

            # Recording Phase
            if phase == 'recording':
                rec_elapsed = now - recording_started
                if rec_elapsed >= window.target_duration_sec:
                    break

                axes = []
                for ax in (1, 2, 4):
                    try:
                        axes.append(device.read_axis(ax))
                    except Exception:
                        axes.append((0.0, 0.0, 0.0))

                window.rec_time.append(rec_elapsed)
                window.p3d_tele_buf.append(dict(data))
                for values, pos_buf, force_buf in zip(
                    axes,
                    (window.p_pos_buf, window.r_pos_buf, window.y_pos_buf),
                    (window.p_force_buf, window.r_force_buf, window.y_force_buf)
                ):
                    pos_buf.append(values[0])
                    force_buf.append(values[1])

                rows.append({
                    'time_s': round(rec_elapsed, 4),
                    'ias_kt': round(actual_ias, 2),
                    'pitch_deg': round(actual_pitch, 2),
                    'bank_deg': round(actual_bank, 2),
                    'heading_deg': round(actual_hdg, 2),
                    'pitch_command': round(pitch_cmd, 4) if pitch_enabled else 0.0,
                    'pitch_measured': round(measured_pitch, 4) if pitch_enabled else 0.0,
                    'roll_command': round(roll_cmd, 4) if roll_needed else 0.0,
                    'roll_measured': round(measured_roll, 4) if roll_needed else 0.0,
                    'rudder_command': round(rudder_cmd, 4) if (yaw_enabled and transport.rudder_available) else 0.0,
                    'rudder_measured': round(measured_rudder, 4) if (yaw_enabled and transport.rudder_available) else 0.0,
                    'active_pids': f"P:{int(pitch_enabled)}_B:{int(bank_enabled)}_Y:{int(yaw_enabled)}"
                })

            time.sleep(0.02)

    except Exception as exc:
        window.multi_pid_fault = str(exc)
        window.sig_update_stage.emit(getattr(base, 'STAGE_CONNECTION_ERROR', 'Connection error'), str(exc))
    finally:
        try:
            transport.release()
        except Exception:
            pass
        window.multi_pid_active = window._external_control_active = False

        # The host logger owns the official CSV and the selected-graph QTG PDF.
        # Do not publish the teammate's separate report format in this workflow.
        if rows:
            window.sig_recording_finished.emit()

