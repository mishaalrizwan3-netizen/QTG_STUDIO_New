"""Measurements for the approach-to-stall test; no control commands."""
import math
import struct


CHANNELS = (
    ('ias_kt', 'AIRSPEED INDICATED', 'knots'),
    ('aoa_deg', 'INCIDENCE ALPHA', 'degrees'),
    ('pitch_deg', 'PLANE PITCH DEGREES', 'degrees'),
    ('bank_deg', 'PLANE BANK DEGREES', 'degrees'),
    ('stall_warning', 'STALL WARNING', 'bool'),
    ('altitude_ft', 'PLANE ALTITUDE', 'feet'),
    ('sim_time_s', 'SIM TIME', 'seconds'),
    ('terrain_ft', 'GROUND ALTITUDE', 'feet'),
    ('fuel_flow_pph', 'GENERAL ENG FUEL FLOW PPH:1', 'pounds per hour'),
    ('cg_long_fraction', 'CG PERCENT', 'percent over 100'),
    ('cg_lateral_fraction', 'CG PERCENT LATERAL', 'percent over 100'),
)


def measurement_frame(frame):
    """Return finite readings; unavailable or invalid readings stay unavailable."""
    result = {}
    for name, _, _ in CHANNELS:
        value = frame.get(name)
        try:
            value = float(value)
        except (TypeError, ValueError, OverflowError):
            value = None
        if value is not None and not math.isfinite(value):
            value = None
        if name == 'stall_warning' and value not in (0.0, 1.0):
            value = None
        result[name] = value
    return result


def telemetry_fields(frame, now, max_age=0.5):
    """Convert a fresh correlated sample to the existing recorder's fields."""
    empty = {key: float('nan') for key in ('airspeed_indicated', 'aoa', 'pitch_angle',
                                            'bank_angle', 'pressure_alt', 'stall_warning',
                                            'stall_ground_alt', 'stall_fuel_flow_pph',
                                            'stall_cg_long_pct', 'stall_cg_lateral_pct')}
    if not frame:
        return empty
    try:
        age = float(now) - float(frame['receipt_time'])
    except (KeyError, TypeError, ValueError, OverflowError):
        return empty
    if not math.isfinite(age) or not 0 <= age <= max_age:
        return empty
    values = measurement_frame(frame)
    for source, dest in (('ias_kt', 'airspeed_indicated'), ('aoa_deg', 'aoa'),
                         ('pitch_deg', 'pitch_angle'), ('bank_deg', 'bank_angle'),
                         ('altitude_ft', 'pressure_alt'), ('stall_warning', 'stall_warning'),
                         ('terrain_ft', 'stall_ground_alt'), ('fuel_flow_pph', 'stall_fuel_flow_pph')):
        if values[source] is not None:
            empty[dest] = values[source]
    for source, dest in (('cg_long_fraction', 'stall_cg_long_pct'),
                         ('cg_lateral_fraction', 'stall_cg_lateral_pct')):
        if values[source] is not None:
            empty[dest] = values[source] * 100.0
    return empty


def parse_control_response(packet, data_id):
    """Decode one CLS2Sim device; 0x22 explicitly specifies N versus Nm."""
    record_size = 6 if data_id in (0x11, 0x21) else 8 if data_id == 0x22 else 0
    if not record_size or len(packet) < 3 + record_size or (len(packet) - 3) % record_size:
        raise ValueError('Missing or ambiguous CLS2Sim axis response')
    if struct.unpack_from('<H', packet)[0] != len(packet) - 2 or packet[2] != 0:
        raise ValueError('Invalid CLS2Sim response')
    if data_id in (0x11, 0x21):
        if len(packet) != 3 + record_size:
            raise ValueError('Multiple CLS2Sim devices for normalized axis response')
        value = struct.unpack_from('<f', packet, 5)[0]
        if not math.isfinite(value) or abs(value) > 1.05:
            raise ValueError('Invalid normalized control value')
        return value, 'normalized'
    readings = []
    for offset in range(3, len(packet), record_size):
        _, movement, value = struct.unpack_from('<HHf', packet, offset)
        if movement == 0:
            continue
        if movement not in (1, 2) or not math.isfinite(value):
            raise ValueError('Invalid engineering force or movement type')
        readings.append((movement, value))
    if len(readings) != 1:
        raise ValueError('Missing or ambiguous active CLS2Sim force device')
    movement, value = readings[0]
    return value, 'N' if movement == 1 else 'Nm'


def read_control_axis(brunner, axis):
    """Read position and engineering force without commanding the controls."""
    missing = float('nan')
    result = [missing, missing, '']
    if not getattr(brunner, 'sock', None):
        return tuple(result)
    with brunner._io_lock:
        sock = brunner.sock
        try:
            sock.setblocking(False)
            while True:
                try:
                    sock.recv(1024)
                except BlockingIOError:
                    break
        finally:
            sock.settimeout(brunner.timeout)
        for data_id, index in ((0x11, 0), (0x22, 1), (0x21, 1)):
            if data_id == 0x21 and result[2]:
                continue
            packet = None
            try:
                sock.send(struct.pack('<III', 0xD0, axis, data_id))
                packet = sock.recv(1024)
                value, unit = parse_control_response(packet, data_id)
                result[index] = value
                if index == 1:
                    result[2] = unit
            except (OSError, ValueError, struct.error) as exc:
                reported = getattr(brunner, '_stall_read_errors', None)
                if reported is None:
                    reported = brunner._stall_read_errors = set()
                key = (axis, data_id)
                if key not in reported:
                    reported.add(key)
                    detail = f', response={packet.hex()}' if packet is not None else ''
                    print(f'[Stall warning] CLS2Sim axis {axis:#x}, data {data_id:#x}: {exc}{detail}', flush=True)
    return tuple(result)


def parse_axis_calibration(packet):
    """Read CLS2Sim 0x23 device factors for a single translational axis."""
    layout = '<HHffIHHiiii'
    size = struct.calcsize(layout)
    if len(packet) < 3 + size or (len(packet) - 3) % size:
        raise ValueError('Missing or ambiguous CLS2Sim axis calibration')
    if struct.unpack_from('<H', packet)[0] != len(packet) - 2 or packet[2] != 0:
        raise ValueError('Invalid CLS2Sim axis calibration response')
    records = [struct.unpack_from(layout, packet, offset) for offset in range(3, len(packet), size)]
    active = [record for record in records if record[1] != 0]
    if len(active) != 1:
        raise ValueError('Missing or ambiguous active CLS2Sim calibration device')
    _, movement, force_factor, position_factor, _, _, _, _, _, _, _ = active[0]
    if movement == 2:
        raise ValueError('Rotational axis reports degrees and Nm; mm/N require approved control geometry')
    if (movement != 1 or not math.isfinite(force_factor)
            or not math.isfinite(position_factor) or force_factor <= 0 or position_factor <= 0):
        raise ValueError('CLS2Sim linear-axis conversion factors unavailable')
    return position_factor, force_factor


def parse_raw_axis(packet):
    """Return a single device's signed raw position or force integer."""
    if len(packet) != 9 or struct.unpack_from('<H', packet)[0] != 7 or packet[2] != 0:
        raise ValueError('Missing or ambiguous CLS2Sim raw axis response')
    return struct.unpack_from('<i', packet, 5)[0]


def read_engineering_axis(brunner, axis, force, unit):
    """Return mm/N only when CLS2Sim identifies and calibrates a linear axis."""
    missing = float('nan')
    force_n = force if unit == 'N' and math.isfinite(force) else missing
    if not getattr(brunner, 'sock', None):
        return missing, force_n
    cache = getattr(brunner, '_stall_axis_calibration', None)
    if cache is None:
        cache = brunner._stall_axis_calibration = {}
    with brunner._io_lock:
        sock = brunner.sock

        def request(data_id, parser):
            packet = None
            try:
                sock.send(struct.pack('<III', 0xD0, axis, data_id))
                packet = sock.recv(1024)
                return parser(packet)
            except (OSError, ValueError, struct.error) as exc:
                reported = getattr(brunner, '_stall_read_errors', None)
                if reported is None:
                    reported = brunner._stall_read_errors = set()
                key = (axis, data_id)
                if key not in reported:
                    reported.add(key)
                    detail = f', response={packet.hex()}' if packet is not None else ''
                    print(f'[Stall warning] CLS2Sim axis {axis:#x}, data {data_id:#x}: {exc}{detail}', flush=True)
                return None

        if axis not in cache:
            cache[axis] = request(0x23, parse_axis_calibration)
        calibration = cache[axis]
        if calibration is None:
            return missing, force_n
        position_factor, force_factor = calibration
        raw_position = request(0x12, parse_raw_axis)
        position_mm = raw_position * position_factor if raw_position is not None else missing
        if not math.isfinite(force_n):
            raw_force = request(0x20, parse_raw_axis)
            if raw_force is not None:
                force_n = raw_force * force_factor
        return position_mm, force_n
