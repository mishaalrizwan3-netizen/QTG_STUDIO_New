"""Heading and Yaw PID algorithms reused verbatim from PID Controller.

Outer-loop: HeadingController (Heading-to-Bank Guidance)
Inner-loop Roll: BankPID (Bank-to-Roll Guidance, imported from backend.bank_pid)
Inner-loop Yaw: YawPID (Sideslip & Yaw Rate to Rudder Guidance)
Coordination: HeadingBankHandoff (Captures heading and coordinates wings-level dwell)
"""
import math
from backend.bank_pid import BankPID, slew_position


class HeadingController:
    """
    Outer-Loop Heading Autopilot (Heading-to-Bank Guidance).
    Computes commanded bank angle to steer the aircraft to target heading:
    - Wraps heading error seamlessly across 0°/360° north boundary [-180..+180].
    - Natural polarity: right turn needed (hdg_err > 0) -> demand positive bank -> BankPID drives right turn.
    - Demands proportional bank angle up to standard-rate turn limit (default ±20.0°).
    - Uses true Heading Rate (d(Heading)/dt) for negative damping to lead-out of the turn.
    - Eliminates bank oscillations, hunting, and Dutch roll cross-coupling.
    - Washes commanded bank angle smoothly to exact 0.00° as heading error reaches zero.
    - When heading is captured, bank = 0.00° and yaw = 0.00° (wings level straight flight).
    """
    def __init__(self, kp=0.80, ki=0.005, kd=0.70, max_bank=20.0, invert=False):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.max_bank = float(max_bank)
        self.invert = bool(invert)
        self.integral = 0.0
        self.cmd_bank = 0.0
        self.last_hdg = None
        self.hdg_rate = 0.0

    def reset(self, initial_bank=0.0):
        self.integral = 0.0
        self.cmd_bank = float(initial_bank)
        self.last_hdg = None
        self.hdg_rate = 0.0

    def update(self, target_hdg: float, actual_hdg: float, yaw_rate: float, dt: float) -> float:
        if dt <= 0.0001:
            return self.cmd_bank

        # Shortest-turn heading error wrapped to [-180 .. +180]
        norm_tgt = target_hdg % 360.0
        norm_act = actual_hdg % 360.0
        hdg_err = (norm_tgt - norm_act + 180.0) % 360.0 - 180.0

        # Compute true heading rate directly from consecutive heading telemetry
        if self.last_hdg is None:
            self.last_hdg = actual_hdg
        raw_rate = ((actual_hdg - self.last_hdg + 180.0) % 360.0 - 180.0) / dt
        self.last_hdg = actual_hdg
        # 0.20s low-pass filter to reject telemetry discretization steps
        self.hdg_rate += (dt / (0.20 + dt)) * (raw_rate - self.hdg_rate)

        # Brunner Roll Motor polarity (BankPID invert=True):
        # Right Turn (hdg_err > 0) -> demand POSITIVE bank -> BankPID invert produces NEGATIVE Brunner roll (right turn).
        # Left Turn  (hdg_err < 0) -> demand NEGATIVE bank -> BankPID invert produces POSITIVE Brunner roll (left turn).
        p_term = self.kp * hdg_err

        # Lead-out damping:
        # Counteract turn rate to smoothly wash out bank as heading approaches target
        d_term = -self.kd * self.hdg_rate

        # Anti-windup conditional integration within ±10.0° of target heading
        if abs(hdg_err) <= 10.0:
            self.integral = max(-5.0, min(5.0, self.integral + self.ki * hdg_err * dt))
        else:
            self.integral = 0.0

        turn_cmd = p_term + self.integral + d_term

        # Optional user polarity inversion toggle
        if self.invert:
            turn_cmd = -turn_cmd

        # Progressive heading-capture bank limit (prevents excessive bank on capture):
        capture_limit = min(self.max_bank, max(2.5, abs(hdg_err) * 1.25))
        demanded = max(-capture_limit, min(capture_limit, turn_cmd))

        # Smooth slew rate limit on commanded bank (6.0 deg/sec for responsive, oscillation-free flight)
        max_step = 6.0 * dt
        delta = demanded - self.cmd_bank
        if abs(delta) > max_step:
            self.cmd_bank += max_step if delta > 0 else -max_step
        else:
            self.cmd_bank = demanded
        return self.cmd_bank


class YawPID:
    """
    Yaw / Rudder PID Controller for physical Brunner CLS-P Rudder Loader (AX_RUDDER = 0x0004).
    Maintains coordinated flight (zero sideslip, zero adverse yaw):
    - When turning: dynamically counteracts adverse yaw using yaw rate damping (Kd).
    - When on heading: drives sideslip angle to 0.00° and centers rudder pedals at 0.00.
    - Zero P3D writes: drives physical Brunner rudder loader, NO P3D writes!
    """
    def __init__(self, kp=0.025, ki=0.0020, kd=0.035, invert=False):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.invert = bool(invert)
        self.integral = 0.0
        self.output = 0.0

    def reset(self, initial_output=0.0):
        seed = -float(initial_output) if self.invert else float(initial_output)
        self.integral = max(-0.10, min(0.10, seed))
        self.output = float(initial_output)

    def update(self, sideslip_deg: float, yaw_rate_deg_sec: float, dt: float, coordinated_rate_deg_sec: float = 0.0) -> float:
        if dt <= 0.0001:
            return self.output

        # Coordinated flight error: target sideslip = 0.0
        err_beta = 0.0 - sideslip_deg
        p_term = self.kp * err_beta
        d_term = -self.kd * (yaw_rate_deg_sec - coordinated_rate_deg_sec)

        # Anti-windup conditional integration
        if abs(err_beta) <= 4.0:
            trial_i = max(-0.15, min(0.15, self.integral + self.ki * err_beta * dt))
            trial = p_term + trial_i + d_term
            if abs(trial) <= 0.40 or trial * err_beta < 0.0:
                self.integral = trial_i

        effort = p_term + self.integral + d_term
        if self.invert:
            effort = -effort

        # Authority clamp for rudder pedal loader (±0.40)
        self.output = max(-0.40, min(0.40, effort))
        return self.output


class HeadingBankHandoff:
    """Select one roll target, with capture dwell and drift hysteresis."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.captured = False
        self.settled_seconds = 0.0
        self.target = None

    def update(self, bank_on, heading_on, target, heading, bank, heading_rate, dt, valid=True):
        if not (bank_on and heading_on):
            self.reset()
            return False
        target = target % 360.0
        target_changed = self.target is None or abs((target - self.target + 180.0) % 360.0 - 180.0) > 0.1
        self.target = target
        error = (target - heading + 180.0) % 360.0 - 180.0
        if target_changed:
            self.captured = False
            self.settled_seconds = 0.0
        if not valid:
            self.settled_seconds = 0.0
            return self.captured
        if self.captured:
            if abs(error) > 4.0:
                self.captured = False
                self.settled_seconds = 0.0
        elif abs(error) <= 1.5 and abs(bank) <= 3.0 and abs(heading_rate) <= 0.5:
            self.settled_seconds += max(0.0, min(dt, 0.1))
            if self.settled_seconds >= 2.0:
                self.captured = True
        else:
            self.settled_seconds = 0.0
        return self.captured

