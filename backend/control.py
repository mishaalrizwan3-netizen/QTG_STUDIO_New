"""Control-method ownership policy used before starting a run."""


def validate_method(reference, method):
    if method not in ("replay", "pid", "bank_pid", "yaw_pid", "multi_pid"):
        raise ValueError("Unknown Automatic control method")
    if method == "pid" and reference != "1.c.(1)":
        raise ValueError("PID is supported only for normal climb.")
    return method


def method_label(context):
    method = context.get('control_method', 'replay')
    if method == 'multi_pid':
        selected = context.get('active_pids') or {}
        labels = [label for key, label in (('airspeed', 'Airspeed'), ('bank', 'Bank'), ('yaw', 'Yaw'))
                  if selected.get(key)]
        return ' + '.join(labels) + ' PID' if labels else 'Multiple PIDs'
    return {'replay': 'Recorded Replay', 'pid': 'Airspeed PID',
            'bank_pid': 'Bank PID', 'yaw_pid': 'Yaw PID'}.get(method, 'Unknown control method')
