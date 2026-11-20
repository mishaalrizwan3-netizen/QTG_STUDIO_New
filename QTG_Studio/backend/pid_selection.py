"""Explicit, opt-in PID axis selection for Automatic tests."""

PID_CHOICES = {
    "Airspeed PID": (True, False, False),
    "Bank PID": (False, True, False),
    "Yaw PID": (False, False, True),
    "Airspeed + Bank PID": (True, True, False),
    "Airspeed + Yaw PID": (True, False, True),
    "Bank + Yaw PID": (False, True, True),
    "Airspeed + Bank + Yaw PID": (True, True, True),
}


def axes_for(label):
    airspeed, bank, yaw = PID_CHOICES[label]
    return {"airspeed": airspeed, "bank": bank, "yaw": yaw}


def validate(reference, mode, active_pids):
    if mode != "auto":
        raise ValueError("PID axes require Automatic mode.")
    if not isinstance(active_pids, dict) or set(active_pids) != {"airspeed", "bank", "yaw"}:
        raise ValueError("Select a valid PID combination.")
    axes = tuple(active_pids[key] for key in ("airspeed", "bank", "yaw"))
    if any(type(value) is not bool for value in axes) or axes not in PID_CHOICES.values():
        raise ValueError("Select a valid PID combination.")
    return active_pids
