"""Control-method ownership policy used before starting a run."""


def validate_method(reference, method):
    if method not in ("replay", "pid", "multi_pid", "engine_step", "static_control", "pitch_trim", "stall", "power_dynamics", "flap_dynamics", "roll_response", "longitudinal_trim", "manoeuvring_stability"):
        raise ValueError("Unknown Automatic control method")
    if method == "engine_step" and reference not in ("1.f.(1)", "1.f.(2)"):
        raise ValueError("Engine throttle step is supported only for engine tests.")
    if method == "power_dynamics" and reference not in ("2.c.(1)", "2.b.(11)", "2.c.(1)-force"):
        raise ValueError("Power Change Dynamics is supported only for its dedicated test.")
    if method == "flap_dynamics" and reference not in ("2.c.(2a)", "2.c.(2a)-force", "2.c.(2b)", "2.c.(2b)-force"):
        raise ValueError("Automatic Flap Change Dynamics/Force is supported only for the configured retraction and extension tests.")
    if method == "roll_response" and reference not in ("2.d.(2a)", "2.d.(2b)"):
        raise ValueError("Automatic Roll Response is supported only for 2.d.(2a) Approach and 2.d.(2b) Cruise.")
    if method == "longitudinal_trim" and reference not in ("2.c.(5a)", "2.c.(5b)"):
        raise ValueError("Automatic Longitudinal Trim is supported only for 2.c.(5a) Cruise and 2.c.(5b) Approach.")
    if method == "manoeuvring_stability" and reference not in ("2.c.(6a)", "2.c.(6b)"):
        raise ValueError("Automatic Longitudinal Manoeuvring Stability is supported only for Cruise and Approach variants.")
    if method == "pid" and reference != "1.c.(1)":
        raise ValueError("PID is supported only for normal climb.")
    if method == "static_control" and reference not in ("2.a.(1)", "2.a.(2)", "2.a.(3)"):
        raise ValueError("Static control testing is supported only for 2.a.(1), 2.a.(2), 2.a.(3).")
    if method == "pitch_trim" and reference not in ("2.a.(7)", "2.a.(4)"):
        raise ValueError("Pitch trim testing is supported only for pitch trim tests.")
    if method == 'stall' and reference != '2.c.(8b)':
        raise ValueError('Stall control is supported only for the Stall Warning test.')
    return method
