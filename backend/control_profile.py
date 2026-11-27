"""Control profile generator for Static Control Testing (2.a.(1), 2.a.(2), 2.a.(3)).

Generates smooth, rate-limited, bounded trajectories:
Center (0) -> First Limit (-L) -> Second Limit (+L) -> Center (0).

Axis mapping:
- 2.a.(1) Pitch: Forward limit (push, negative) -> Backward limit (pull, positive) -> Center
- 2.a.(2) Roll: Left limit (negative) -> Right limit (positive) -> Center
- 2.a.(3) Yaw: Left pedal limit (negative) -> Right pedal limit (positive) -> Center
"""

import math
from dataclasses import dataclass
from typing import Dict, Tuple

# Supported test references
TEST_PITCH = '2.a.(1)'
TEST_ROLL = '2.a.(2)'
TEST_YAW = '2.a.(3)'

STATIC_CONTROL_REFERENCES = (TEST_PITCH, TEST_ROLL, TEST_YAW)

# Brunner axis masks (matching BrunnerInterface constants)
AXIS_PITCH = 0x01
AXIS_ROLL = 0x02
AXIS_RUDDER = 0x04

AXIS_INFO: Dict[str, dict] = {
    TEST_PITCH: {
        'name': 'Pitch',
        'axis_mask': AXIS_PITCH,
        'limit': 1.0,           # Full travel to -1 and +1 via Autopilot
        'rate': 0.10,           # 10% normalized travel per second for the pitch sweep
        'motor_speed': 300,
        'max_safe_force_n': 350.0,
        'first_direction': 'Forward (-1.0)',
        'second_direction': 'Backward (+1.0)',
        'position_col': 'Joystick Pitch Position (mm)',
        'force_col': 'Joystick Pitch Force (N)',
        'surface_col': 'elevator_deflection_deg',
    },
    TEST_ROLL: {
        'name': 'Roll',
        'axis_mask': AXIS_ROLL,
        'limit': 1.0,           # Full travel to -1 and +1 via Autopilot
        'rate': 0.20,
        'motor_speed': 200,
        'max_safe_force_n': 250.0,
        'first_direction': 'Left (-1.0)',
        'second_direction': 'Right (+1.0)',
        'position_col': 'Joystick Roll Position (mm)',
        'force_col': 'Joystick Roll Force (N)',
        'surface_col': 'aileron_left_deflection_deg',
    },
    TEST_YAW: {
        'name': 'Yaw',
        'axis_mask': AXIS_RUDDER,
        'limit': 1.0,           # Full travel to -1 and +1 via Autopilot
        'rate': 0.20,
        'motor_speed': 200,
        'max_safe_force_n': 450.0,
        'first_direction': 'Left Pedal (-1.0)',
        'second_direction': 'Right Pedal (+1.0)',
        'position_col': 'Rudder Pedal Position (mm)',
        'force_col': 'Rudder Pedal Force (N)',
        'surface_col': 'rudder_deflection_deg',
    },
}


@dataclass
class ProfileSegment:
    start_pos: float
    end_pos: float
    duration: float
    start_time: float
    end_time: float
    name: str


def validate_sweep_rate(value):
    try:
        rate = float(value)
    except (TypeError, ValueError):
        raise ValueError('Sweep speed must be a positive number in normalized units/s.') from None
    if not math.isfinite(rate) or rate <= 0 or not math.isfinite(4.0 / rate):
        raise ValueError('Sweep speed must be a positive finite number in normalized units/s.')
    return rate


def validate_motor_speed(value):
    try:
        speed = float(value)
    except (TypeError, ValueError):
        raise ValueError('CLS2Sim motor speed must be a whole number from 5 to 9999.') from None
    if not 5 <= speed <= 9999 or not speed.is_integer():
        raise ValueError('CLS2Sim motor speed must be a whole number from 5 to 9999.')
    return int(speed)


class StaticControlProfile:
    """Computes smooth trajectory for static control testing."""

    def __init__(self, test_reference: str, baseline_seconds: float = 1.0, settle_seconds: float = 1.0, *, sweep_rate=None, start_position=0.0):
        if test_reference not in AXIS_INFO:
            raise ValueError(f"Unknown static control reference: {test_reference}")

        self.reference = test_reference
        self.info = AXIS_INFO[test_reference]
        self.limit = float(self.info['limit'])
        self.rate = self.info['rate'] if sweep_rate is None else validate_sweep_rate(sweep_rate)
        self.start_position = max(-self.limit, min(self.limit, float(start_position)))
        self.baseline_seconds = float(baseline_seconds)
        self.settle_seconds = float(settle_seconds)

        # Timing for each phase
        t1 = abs(self.start_position + self.limit) / self.rate
        t2 = (2.0 * self.limit) / self.rate      # -Limit to +Limit
        t3 = abs(self.limit - self.start_position) / self.rate

        t = 0.0
        self.segments = []

        # 0. Baseline at the handoff position (zero for the established workflow).
        self.segments.append(ProfileSegment(self.start_position, self.start_position, self.baseline_seconds, t, t + self.baseline_seconds, 'Baseline Start Position'))
        t += self.baseline_seconds

        # 1. Center -> First limit (-limit)
        self.segments.append(ProfileSegment(self.start_position, -self.limit, t1, t, t + t1, f"Start position to {self.info['first_direction']} Limit"))
        t += t1

        # 2. First limit -> Second limit (+limit)
        self.segments.append(ProfileSegment(-self.limit, self.limit, t2, t, t + t2, f"{self.info['first_direction']} to {self.info['second_direction']} Limit"))
        t += t2

        # 3. Second limit -> original handoff position
        self.segments.append(ProfileSegment(self.limit, self.start_position, t3, t, t + t3, f"{self.info['second_direction']} Limit to Start Position"))
        t += t3

        # 4. Settle at original handoff position
        self.segments.append(ProfileSegment(self.start_position, self.start_position, self.settle_seconds, t, t + self.settle_seconds, 'Start Position Settle'))
        t += self.settle_seconds

        self.total_duration = t

    def target(self, elapsed: float) -> Tuple[float, str, bool]:
        """Returns (target_position, segment_name, is_complete).

        Uses smooth cosine blending between endpoints to eliminate jerk.
        """
        if elapsed < 0.0:
            return self.start_position, self.segments[0].name, False

        if elapsed >= self.total_duration:
            return self.start_position, 'Completed', True

        for seg in self.segments:
            if seg.start_time <= elapsed < seg.end_time:
                dt = elapsed - seg.start_time
                fraction = dt / seg.duration if seg.duration > 0 else 1.0
                fraction = max(0.0, min(1.0, fraction))
                # Smooth sinusoidal S-curve interpolation
                blend = 0.5 * (1.0 - math.cos(math.pi * fraction))
                pos = seg.start_pos + (seg.end_pos - seg.start_pos) * blend
                return pos, seg.name, False

        return self.start_position, 'Start Position Settle', False

