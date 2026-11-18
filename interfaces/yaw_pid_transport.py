"""Dual-axis (Roll + Rudder) CLS2Sim adapter using the worker's existing socket and RLock.
Pitch (Axis 1) is NEVER commanded, armed, or modified.
Roll (Axis 2) and Rudder (Axis 4) are coordinated for heading hold and turn coordination.
"""
import math
import struct
import time

# Hardware CAN node mappings from PID Controller:
# Axis 2 (Roll):   CAN node 1 (fallback 2)
# Axis 4 (Rudder): CAN node 5 (fallback 4, 3)
# Axis 1 (Pitch):  EXCLUDED completely
AXIS_PREFERRED_NODES = {
    2: [1, 2],
    4: [5, 4, 3],
}

def parse_cls2sim_detailed(blob: bytes, target_axis: int = None) -> dict:
    """
    Detailed diagnostic parser for CLS2Sim UDP telemetry packets.
    Extracts telemetry data and flags specific protocol errors.
    """
    res = {
        'valid': False,
        'val': None,
        'format': 'Unknown',
        'status': None,
        'node': None,
        'raw_hex': blob.hex(' ').upper() if blob else '--',
        'length': len(blob) if blob else 0,
        'error': None
    }
    if not blob or len(blob) < 3:
        res['format'] = 'Truncated (<3B)'
        res['error'] = f'Empty or truncated packet ({len(blob) if blob else 0} bytes)'
        return res

    candidates = AXIS_PREFERRED_NODES.get(target_axis, [target_axis] if target_axis is not None else None)

    # 0. 3-byte empty/status response (e.g. b'\x01\x00\x00' -> axis has no motor loader/sensor connected)
    if len(blob) == 3:
        st = blob[2]
        res['format'] = '3B-Status'
        res['status'] = st
        res['error'] = f'No position sensor or loader attached (3 bytes, status {st})'
        return res

    # 1. Multi-Node Compact format (9+ bytes, where (len - 3) % 6 == 0):
    if len(blob) >= 9 and (len(blob) - 3) % 6 == 0:
        st = blob[2]
        res['format'] = f'Multi-Node ({len(blob)}B)'
        res['status'] = st
        if st != 0:
            res['error'] = f'Status error code {st}'
            return res

        pairs = {}
        for off in range(3, len(blob) - 5, 6):
            node, val = struct.unpack_from('<Hf', blob, off)
            pairs[node] = val

        chosen_node = None
        chosen_val = None
        if candidates is not None:
            for cand in candidates:
                if cand in pairs:
                    chosen_node = cand
                    chosen_val = pairs[cand]
                    break
            if chosen_node is None:
                present = list(pairs.keys())
                res['node'] = present[0] if present else None
                res['error'] = f'Wrong axis reply: nodes {present} present, expected {candidates}'
                return res
        else:
            if pairs:
                chosen_node = list(pairs.keys())[0]
                chosen_val = pairs[chosen_node]

        if chosen_val is not None:
            res['node'] = chosen_node
            if not math.isfinite(chosen_val):
                res['error'] = 'Non-finite float (NaN or Inf)'
            elif abs(chosen_val) > 1.05:
                res['error'] = f'Value out of bounds ({chosen_val:+.4f})'
            else:
                res['valid'] = True
                res['val'] = float(chosen_val)
            return res

    # 2. Protocol v2 with 32-bit Message ID (17+ bytes):
    if len(blob) >= 17:
        st = struct.unpack_from('<I', blob, 2)[0]
        dev_id = blob[10]
        did = struct.unpack_from('<H', blob, 11)[0]
        val = struct.unpack_from('<f', blob, 13)[0]
        res['format'] = 'v2-MsgID (17B)'
        res['status'] = st
        res['node'] = dev_id
        if st != 0:
            res['error'] = f'Status error code {st}'
        elif did != 0x11:
            res['error'] = f'Unexpected data_id {hex(did)} (expected 0x11 POS_NORM)'
        elif candidates is not None and dev_id not in candidates:
            res['error'] = f'Wrong axis reply: got node {dev_id}, expected in {candidates}'
        elif not math.isfinite(val):
            res['error'] = 'Non-finite float (NaN or Inf)'
        elif abs(val) > 1.05:
            res['error'] = f'Value out of bounds ({val:+.4f})'
        else:
            res['valid'] = True
            res['val'] = float(val)
        return res

    # 3. Protocol v2 standard (13+ bytes):
    if len(blob) >= 13:
        st = struct.unpack_from('<I', blob, 2)[0]
        dev_id = blob[6]
        did = struct.unpack_from('<H', blob, 7)[0]
        val = struct.unpack_from('<f', blob, 9)[0]
        res['format'] = 'v2-Standard (13B)'
        res['status'] = st
        res['node'] = dev_id
        if st != 0:
            res['error'] = f'Status error code {st}'
        elif did != 0x11:
            res['error'] = f'Unexpected data_id {hex(did)} (expected 0x11 POS_NORM)'
        elif candidates is not None and dev_id not in candidates:
            res['error'] = f'Wrong axis reply: got node {dev_id}, expected in {candidates}'
        elif not math.isfinite(val):
            res['error'] = 'Non-finite float (NaN or Inf)'
        elif abs(val) > 1.05:
            res['error'] = f'Value out of bounds ({val:+.4f})'
        else:
            res['valid'] = True
            res['val'] = float(val)
        return res

    res['format'] = f'Malformed ({len(blob)}B)'
    res['error'] = f'Unrecognized packet length ({len(blob)} bytes)'
    return res


class YawBrunnerTransport:
    """
    Dual-Axis Transport: Controls Roll (Axis 2) and Rudder (Axis 4) simultaneously.
    Never touches Pitch (Axis 1).
    """
    def __init__(self, device, stop_event):
        self.device = device
        self.stop_event = stop_event
        self.owned = False
        self.closed = False
        self.rudder_available = True
        self.last_roll_feedback = None
        self.last_rudder_feedback = None
        self.roll_feedback_time = None
        self.rudder_feedback_time = None
        self.last_watchdog = 0.0

    def exchange(self, packet):
        device = self.device
        with device._io_lock:
            if not device.sock:
                raise RuntimeError('BRUNNER disconnected')
            device.sock.setblocking(False)
            try:
                while True:
                    device.sock.recv(4096)
            except BlockingIOError:
                pass
            finally:
                device.sock.settimeout(device.timeout)
            device.sock.send(packet)
            return device.sock.recv(4096)

    def write(self, packet):
        reply = self.exchange(packet)
        if len(reply) < 3 or reply[2] != 0:
            raise RuntimeError('CLS2Sim command rejected or malformed acknowledgement')

    def setting(self, axis, setting_id, fmt, value):
        self.write(struct.pack('<III', 0xCE, int(axis), int(setting_id)) + struct.pack('<' + fmt, value))

    def overrides(self, mask):
        # Override IDs contain GLOBAL masks. Only Roll (0x02) and Rudder (0x04) are owned.
        for setting_code in (3, 4, 2):
            self.write(struct.pack('<III', 0xD1, int(mask), setting_code))

    def read_axis_position(self, axis: int) -> float:
        with self.device._io_lock:
            reply = self.exchange(struct.pack('<III', 0xD0, int(axis), 0x11))
            if len(reply) < 3 or struct.unpack_from('<H', reply)[0] != len(reply) - 2:
                raise RuntimeError(f'Malformed position packet length for axis {axis}')
            result = parse_cls2sim_detailed(reply, axis)
            if not result['valid']:
                # If Rudder (Axis 4) has no active motorized loader (e.g. 3-byte status reply),
                # handle gracefully: turn coordination/yaw damping is inactive on unmotorized pedals,
                # while Heading Hold proceeds with full roll authority.
                if axis == 4 and (len(reply) == 3 or 'No position' in (result['error'] or '') or 'Unrecognized' in (result['error'] or '')):
                    self.rudder_available = False
                    self.last_rudder_feedback = 0.0
                    self.rudder_feedback_time = time.monotonic()
                    return 0.0
                raise RuntimeError(f'Invalid feedback for axis {axis}: {result["error"]}')
            now = time.monotonic()
            if axis == 2:
                self.last_roll_feedback = result['val']
                self.roll_feedback_time = now
            elif axis == 4:
                self.last_rudder_feedback = result['val']
                self.rudder_feedback_time = now
            return result['val']

    def read_positions(self) -> tuple[float, float]:
        roll = self.read_axis_position(2)
        rudder = self.read_axis_position(4)
        return roll, rudder

    def engage(self) -> tuple[float, float]:
        with self.device._io_lock:
            if self.closed or self.stop_event.is_set():
                raise RuntimeError('Yaw engagement cancelled')
            if getattr(self.device, '_pid_transport', None) is not None:
                raise RuntimeError('Airspeed PID still owns the hardware')
            if getattr(self.device, '_bank_transport', None) is not None and getattr(self.device._bank_transport, 'owned', False):
                raise RuntimeError('Bank PID still owns the hardware')

            # Seed positions from physical hardware
            start_roll = self.read_axis_position(2)
            start_rudder = self.read_axis_position(4)

            self.owned = True
            try:
                # Arm watchdog
                self.write(struct.pack('<IIH', 0xD8, 0, 1000))
                self.last_watchdog = time.monotonic()

                # Enable overrides: Roll (0x02) and Rudder (0x04 if available)
                # Pitch (Axis 1) is completely excluded from the mask!
                mask = 0x02 | (0x04 if self.rudder_available else 0x00)
                self.overrides(mask)

                # Configure Roll (Axis 2)
                self.setting(2, 0x50, 'H', 2400)
                self.setting(2, 0x60, 'H', 80)
                self.setting(2, 0x80, 'f', start_roll)
                self.setting(2, 0x70, 'B', 1)

                # Configure Rudder (Axis 4) only if hardware is available
                if self.rudder_available:
                    self.setting(4, 0x50, 'H', 2400)
                    self.setting(4, 0x60, 'H', 80)
                    self.setting(4, 0x80, 'f', start_rudder)
                    self.setting(4, 0x70, 'B', 1)

            except Exception:
                self.release()
                raise
            return start_roll, start_rudder

    def command(self, roll_pos: float, rudder_pos: float):
        with self.device._io_lock:
            if self.closed or self.stop_event.is_set() or not self.owned:
                raise RuntimeError('Yaw/Heading roll and rudder ownership released')
            now = time.monotonic()
            if self.roll_feedback_time is None or now - self.roll_feedback_time > 0.35:
                raise RuntimeError('Roll position feedback stale')
            if self.rudder_available:
                if self.rudder_feedback_time is None or now - self.rudder_feedback_time > 0.35:
                    raise RuntimeError('Rudder position feedback stale')

            if not (math.isfinite(roll_pos) and (math.isfinite(rudder_pos) or not self.rudder_available)):
                raise RuntimeError('Invalid Yaw/Heading motor command: non-finite position')

            # Clamping authorities from PID Controller:
            # Roll: [-0.95 .. +0.95]
            # Rudder: [-0.40 .. +0.40] (authority limit for rudder loader)
            clamped_roll = max(-0.95, min(0.95, float(roll_pos)))

            if now - self.last_watchdog >= 0.4:
                self.write(struct.pack('<IIH', 0xD8, 0, 1000))
                self.last_watchdog = now

            self.setting(2, 0x80, 'f', clamped_roll)
            if self.rudder_available:
                clamped_rudder = max(-0.40, min(0.40, float(rudder_pos)))
                self.setting(4, 0x80, 'f', clamped_rudder)

    def release(self):
        with self.device._io_lock:
            self.closed = True
            if not self.owned:
                return
            errors = []
            # Disarm owned axes without unwanted centering
            operations = [
                lambda: self.setting(2, 0x70, 'B', 0),
            ]
            if self.rudder_available:
                operations.append(lambda: self.setting(4, 0x70, 'B', 0))
            operations.extend([
                lambda: self.overrides(0),
                lambda: self.write(struct.pack('<III', 0xD1, 0, 6)),
            ])
            for operation in operations:
                try:
                    operation()
                except Exception as exc:
                    errors.append(str(exc))
            if errors:
                raise RuntimeError('Yaw release incomplete: ' + '; '.join(errors))
            self.write(struct.pack('<IIH', 0xD8, 0, 0))
            self.owned = False

