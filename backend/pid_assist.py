"""PID Assist for Automatic Tests.

Allows running any combination of Airspeed PID (pitch axis), Bank PID (roll axis),
and Yaw PID (rudder axis) concurrently with an automatic test, while preserving
the test's own automatic movement on its owned axis.
"""
from __future__ import annotations

import math
from typing import Dict, Any, Optional

from backend.bank_pid import BankPID
from backend.yaw_pid import YawPID, HeadingController
from backend.multi_pid_runner import PitchPID, AirspeedController


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
        self.owned_axis_mask = int(owned_axis_mask)
        active_pids = self.context.get('active_pids') or {}

        self.pitch_requested = bool(active_pids.get('airspeed', False))
        self.bank_requested = bool(active_pids.get('bank', False))
        self.yaw_requested = bool(active_pids.get('yaw', False))

        # Only enable PID on an axis if the test itself does NOT own that axis
        self.pitch_enabled = self.pitch_requested and not bool(self.owned_axis_mask & 0x01)
        self.bank_enabled = self.bank_requested and not bool(self.owned_axis_mask & 0x02)
        self.yaw_enabled = self.yaw_requested and not bool(self.owned_axis_mask & 0x04)

        self.pid_axis_mask = 0
        if self.pitch_enabled:
            self.pid_axis_mask |= 0x01
        if self.bank_enabled:
            self.pid_axis_mask |= 0x02
        if self.yaw_enabled:
            self.pid_axis_mask |= 0x04

        self.airspeed_ctrl = AirspeedController() if self.pitch_enabled else None
        self.pitch_pid = PitchPID() if self.pitch_enabled else None
        self.bank_pid = BankPID() if self.bank_enabled else None
        self.yaw_pid = YawPID() if self.yaw_enabled else None
        self.heading_ctrl = HeadingController() if (self.yaw_enabled and self.bank_enabled) else None

        pid_cond = self.context.get('pid_conditions') or {}
        init_cond = self.context.get('initial_conditions') or {}
        raw_ias = pid_cond.get('target_ias') or init_cond.get('ias_kt') or 80.0
        try:
            self.target_ias = float(raw_ias)
        except (ValueError, TypeError):
            self.target_ias = 80.0

        raw_hdg = pid_cond.get('target_heading') or self.context.get('target_heading_mag') or init_cond.get('heading_deg') or 0.0
        try:
            self.target_hdg = float(raw_hdg) % 360.0
        except (ValueError, TypeError):
            self.target_hdg = 0.0

        raw_bank = pid_cond.get('target_bank') or self.context.get('target_bank') or init_cond.get('bank_deg') or 0.0
        try:
            self.target_bank = float(raw_bank)
        except (ValueError, TypeError):
            self.target_bank = 0.0

        self.last_time: Optional[float] = None
        self.latest_commands: Dict[str, float] = {}

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

        # 1. Airspeed PID on Pitch axis (0x01)
        if self.pitch_enabled and math.isfinite(ias) and math.isfinite(pitch):
            cmd_pitch = self.airspeed_ctrl.update(self.target_ias, ias, dt, pitch)
            pitch_cmd = self.pitch_pid.update(cmd_pitch, pitch, dt)
            pitch_cmd = max(-0.55, min(0.65, pitch_cmd))
            commands['pitch'] = pitch_cmd
            if brunner and getattr(brunner, 'sock', None) and hasattr(brunner, 'write_setting'):
                try:
                    brunner.write_setting(0x01, 0x80, "f", pitch_cmd)
                except Exception:
                    pass

        # 2. Bank PID on Roll axis (0x02)
        if self.bank_enabled and math.isfinite(bank):
            if self.heading_ctrl and math.isfinite(hdg):
                cmd_bank = self.heading_ctrl.update(self.target_hdg, hdg, yaw_rate, dt)
            else:
                cmd_bank = self.target_bank
            roll_cmd = self.bank_pid.update(cmd_bank, bank, dt)
            roll_cmd = max(-0.65, min(0.65, roll_cmd))
            commands['roll'] = roll_cmd
            if brunner and getattr(brunner, 'sock', None) and hasattr(brunner, 'write_setting'):
                try:
                    brunner.write_setting(0x02, 0x80, "f", roll_cmd)
                except Exception:
                    pass

        # 3. Yaw PID on Rudder axis (0x04)
        if self.yaw_enabled and math.isfinite(sideslip):
            rudder_cmd = self.yaw_pid.update(sideslip, yaw_rate, dt)
            rudder_cmd = max(-0.40, min(0.40, rudder_cmd))
            commands['rudder'] = rudder_cmd
            if brunner and getattr(brunner, 'sock', None) and hasattr(brunner, 'write_setting'):
                try:
                    brunner.write_setting(0x04, 0x80, "f", rudder_cmd)
                except Exception:
                    pass

        self.latest_commands = commands
        return commands
