"""Control-method ownership policy used before starting a run."""


def validate_method(reference, method):
    if method not in ("replay", "pid", "engine_step", "static_control", "pitch_trim", "stall"):
        raise ValueError("Unknown Automatic control method")
    if method == "engine_step" and reference not in ("1.f.(1)", "1.f.(2)"):
        raise ValueError("Engine throttle step is supported only for engine tests.")
    if method == "pid" and reference != "1.c.(1)":
        raise ValueError("PID is supported only for normal climb.")
    if method == "static_control" and reference not in ("2.a.(1)", "2.a.(2)", "2.a.(3)"):
        raise ValueError("Static control testing is supported only for 2.a.(1), 2.a.(2), 2.a.(3).")
    if method == "pitch_trim" and reference != "2.a.(4)":
        raise ValueError("Pitch trim testing is supported only for 2.a.(4).")
    if method == 'stall' and reference != '2.c.(8b)':
        raise ValueError('Stall control is supported only for the Stall Warning test.')
    return method
