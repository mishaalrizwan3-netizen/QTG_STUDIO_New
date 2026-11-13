"""Correlated, read-only engine measurements using the existing SDK DLL.

No BRUNNER imports, actuator writes, or dependency on the legacy telemetry layout.
All calls and dispatch run on the worker's single owner thread.
"""
import ctypes as C
import math
from pathlib import Path
import time


CHANNELS = (
    ('rpm', 'GENERAL ENG RPM:1', 'rpm'),
    ('throttle', 'GENERAL ENG THROTTLE LEVER POSITION:1', 'percent'),
    ('parking_brake', 'BRAKE PARKING POSITION', 'bool'),
    ('sim_time', 'SIM TIME', 'seconds'),
    ('alt_ft', 'PLANE ALTITUDE', 'feet'),
    ('ias_kt', 'AIRSPEED INDICATED', 'knots'),
    ('lat', 'PLANE LATITUDE', 'degrees'),
    ('lon', 'PLANE LONGITUDE', 'degrees'),
    ('flaps', 'FLAPS HANDLE INDEX', 'number'),
    ('on_ground', 'SIM ON GROUND', 'bool'),
    ('temperature_c', 'AMBIENT TEMPERATURE', 'celsius'),
    ('ground_speed', 'GROUND VELOCITY', 'knots'),
    ('heading_deg', 'PLANE HEADING DEGREES TRUE', 'degrees'),
    ('pitch_deg', 'PLANE PITCH DEGREES', 'degrees'),
    ('bank_deg', 'PLANE BANK DEGREES', 'degrees'),
    ('prop_rpm', 'PROP RPM:1', 'rpm'),
    ('manifold_pressure_psi', 'RECIP ENG MANIFOLD PRESSURE:1', 'psi'),
    ('vertical_speed', 'VERTICAL SPEED', 'feet per minute'),
    ('total_weight', 'TOTAL WEIGHT', 'kilograms'),
    ('cg_percent', 'CG PERCENT', 'percent'),
    ('height_agl_ft', 'PLANE ALT ABOVE GROUND', 'feet'),
)


class Measurements:
    def __init__(self, dll_path, *, clock=time.monotonic):
        self.clock = clock
        self.dll = C.CDLL(str(Path(dll_path)))
        self.handle = C.c_void_p()
        self.pending = None
        self.sequence = 0
        self.latest = None
        self.error = ''
        self.object_id = None
        self.requested_at = None
        u, p = C.c_uint32, C.c_void_p
        self.callback_type = C.CFUNCTYPE(None, p, u, p)
        signatures = {
            'SimConnect_Open': [C.POINTER(p), C.c_char_p, p, u, p, u],
            'SimConnect_Close': [p],
            'SimConnect_AddToDataDefinition': [p, u, C.c_char_p, C.c_char_p, u, C.c_float, u],
            'SimConnect_RequestDataOnSimObject': [p, u, u, u, u, u, u, u, u],
            'SimConnect_CallDispatch': [p, self.callback_type, p],
        }
        for name, args in signatures.items():
            function = getattr(self.dll, name)
            function.argtypes, function.restype = args, C.c_int32
        self.callback = self.callback_type(self._receive)
        try:
            self._check(self.dll.SimConnect_Open(C.byref(self.handle), b'QTG engine measurements', None, 0, None, 0))
            for _, name, unit in CHANNELS:
                self._check(self.dll.SimConnect_AddToDataDefinition(
                    self.handle, 1, name.encode(), unit.encode(), 4, 0., 0xFFFFFFFF))
        except Exception:
            self.close()
            raise

    @staticmethod
    def _check(result):
        if result < 0:
            raise RuntimeError('Engine measurement SimConnect call failed: '+hex(result & 0xFFFFFFFF))

    def _receive(self, pointer, size, _context):
        # SIMCONNECT_RECV_SIMOBJECT_DATA: ten DWORDs followed by packed doubles.
        try:
            if size < 12:
                raise RuntimeError('Truncated SimConnect message')
            header = C.cast(pointer, C.POINTER(C.c_uint32))
            kind = header[2]
            if kind in (1, 3):  # EXCEPTION / QUIT
                raise RuntimeError('Simulator exception or disconnect in engine measurement session')
            if kind != 8:
                return
            if size < 40 + 8 * len(CHANNELS):
                raise RuntimeError('Truncated engine measurement response')
            if header[3] != self.pending:
                return  # Delayed response cannot refresh the current request.
            if header[5] != 1 or header[6] != 0 or header[9] != len(CHANNELS):
                raise RuntimeError('Invalid correlated engine measurement response: '
                                   f'definition={header[5]}, flags={header[6]}, count={header[9]}')
            # SIMCONNECT_OBJECT_ID_USER (zero) selects the user aircraft in the
            # request. The server returns its actual object ID, which can be nonzero.
            # Bind only a valid, correlated response and fail on an aircraft change.
            if self.object_id is not None and header[4] != self.object_id:
                raise RuntimeError(f'User aircraft changed during measurement: {self.object_id} to {header[4]}')
            received_at = self.clock()
            if self.requested_at is None or not 0 <= received_at - self.requested_at <= 1.0:
                raise RuntimeError('Engine measurement response was delayed or has an invalid clock')
            values = C.cast(pointer + 40, C.POINTER(C.c_double))
            measured = {key: float(values[i]) for i, (key, _, _) in enumerate(CHANNELS)}
            essential = ('rpm', 'throttle', 'sim_time', 'alt_ft', 'ias_kt')
            if not all(math.isfinite(measured[k]) for k in essential if k in measured):
                raise RuntimeError('Nonfinite engine measurement')
            if measured['parking_brake'] not in (0., 1.) or measured['on_ground'] not in (0., 1.):
                raise RuntimeError('Invalid engine measurement boolean')
            self.object_id = header[4]
            self.sequence += 1
            self.latest = dict(measured, sequence=self.sequence, receipt_time=received_at, request_id=self.pending,
                               object_id=self.object_id)
            self.pending = None
            self.requested_at = None
        except Exception as exc:
            self.error = str(exc)

    def poll(self):
        self._check(self.dll.SimConnect_CallDispatch(self.handle, self.callback, None))
        if self.error:
            raise RuntimeError(self.error)
        if self.pending is None:
            self.pending = self.sequence + 100
            self.requested_at = self.clock()
            self._check(self.dll.SimConnect_RequestDataOnSimObject(self.handle, self.pending, 1, 0, 1, 0, 0, 0, 0))
        elif self.clock() - self.requested_at > 1.0:
            raise RuntimeError('Timed out waiting for a fresh engine measurement response')
        return self.latest

    def close(self):
        if self.handle.value:
            self.dll.SimConnect_Close(self.handle)
            self.handle = C.c_void_p()
