"""Required initial power and flight conditions for engine tests."""
from backend.verifier import validate_conditions


def enforce(reference, conditions):
    if conditions is None or reference not in ('1.f.(1)', '1.f.(2)'):
        return validate_conditions(conditions)
    result = dict(conditions)
    if reference == '1.f.(1)':
        result['throttle'] = 5.
    else:
        result.update(on_ground=True, ias_kt=0.)
    return validate_conditions(result)


def deceleration_overrides(overrides):
    result = dict(overrides or {})
    result.update(on_ground=True, ias_kt=0.)
    return result
