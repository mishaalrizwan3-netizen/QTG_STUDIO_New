"""Bank control extracted verbatim from the supplied reference, without its GUI.
Source: SMK_IAS_PID_BRUNNER_ONLY_V4_1.py
Source SHA256: 3e66f0c686af8b1e8e131829e6c93be0622402f11fd5499439525ed8140b6228
QTG bank_angle is already right-positive. Do not negate it again.
"""

def slew_position(current, velocity, desired, dt, max_rate=0.50, max_accel=1.80):
    """Smooth acceleration & velocity limiter to prevent sudden physical motor jerks."""
    if dt <= 0:
        return current, velocity
    req_v = max(-max_rate, min(max_rate, (desired - current) / dt))
    next_v = max(velocity - max_accel * dt, min(velocity + max_accel * dt, req_v))
    next_p = current + next_v * dt
    if (desired - current) * (desired - next_p) <= 0:
        return desired, 0.0
    return next_p, next_v

class BankPID:
    """
    Bank Angle (Roll Attitude) PID Controller.
    Commands the physical Brunner CLS-P Roll Motor to actively hold Bank = target_deg:
    - Purely hardware-driven: drives physical Brunner roll loader, NO P3D writes!
    - Natural polarity (invert=False): right bank (+10°) -> negative roll command (left yoke) -> rolls left to 0.00°.
    - Zero deadband: integrates any residual error continually down to 0.00°!
    - Anti-windup conditional integration: prevents stick latching during disturbances.
    - Strong roll-rate damping (Kd) prevents limit cycles and pilot-induced oscillations.
    """
    def __init__(self, kp=0.015, ki=0.0010, kd=0.020, invert=False):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.invert = bool(invert)
        self.integral = 0.0
        self.last_bank = None
        self.bank_rate = 0.0
        self.output = 0.0

    def reset(self, initial_output=0.0):
        init_i = max(-0.25, min(0.25, float(initial_output)))
        self.integral = -init_i if self.invert else init_i
        self.last_bank = None
        self.bank_rate = 0.0
        self.output = float(initial_output)

    def update(self, target_deg: float, actual_deg: float, dt: float) -> float:
        if dt <= 0.0001:
            return self.output

        error = target_deg - actual_deg

        if self.last_bank is None:
            self.last_bank = actual_deg
        raw_rate = (actual_deg - self.last_bank) / dt
        self.last_bank = actual_deg
        # 0.08s time constant filter: smoothly filters roll rate without derivative kick
        self.bank_rate += (dt / (0.08 + dt)) * (raw_rate - self.bank_rate)

        p_term = self.kp * error
        d_term = -self.kd * self.bank_rate

        # Anti-windup conditional integration:
        # 1. Integrate when |error| <= 4.0° to eliminate steady-state trim offsets
        # 2. Clamped to ±0.25 authority to allow full aerodynamic trim while preventing runaway
        if abs(error) <= 4.0:
            int_mult = 1.5 if abs(error) < 0.50 else 1.0
            trial_i = max(-0.25, min(0.25, self.integral + self.ki * error * dt * int_mult))
            trial_raw = p_term + trial_i + d_term
            trial_effort = trial_raw  # saturation test precedes motor polarity
            # Anti-saturation: freeze integration if actuator is already near clamp (±0.60)
            pushes_further = (trial_effort >= 0.58 and error > 0) or \
                             (trial_effort <= -0.58 and error < 0)
            if not pushes_further:
                self.integral = trial_i

        effort = p_term + self.integral + d_term

        # Smooth deadband-free micro-trim:
        if abs(error) < 0.50:
            stiction_bias = (error / 0.50) * 0.005
            effort += stiction_bias

        if self.invert:
            effort = -effort

        # Autopilot roll authority clamped to ±0.65:
        self.output = max(-0.65, min(0.65, effort))
        return self.output
