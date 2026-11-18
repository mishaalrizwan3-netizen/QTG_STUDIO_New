"""Roll-only CLS2Sim adapter using the worker's existing socket and RLock.
Reference parser and engagement settings are retained. No socket is created.
"""
import math
import struct
import time

AXIS_PREFERRED_NODES = {2: [1, 2]}

def parse_cls2sim_detailed(blob: bytes, target_axis: int = None) -> dict:
    """
    Detailed diagnostic parser for CLS2Sim UDP telemetry packets.
    Extracts telemetry data and flags specific protocol errors:
    - 'valid': bool (True if finite float within [-1.05..1.05] and matching target axis)
    - 'val': float or None
    - 'format': 'Multi-Node' | 'v2-MsgID' | 'v2-Standard' | 'Malformed' | 'Truncated'
    - 'status': int or None (reported status code, 0 = OK)
    - 'node': int or None (node/device ID reported in packet)
    - 'raw_hex': hex string of full received packet
    - 'length': int (byte length)
    - 'error': str or None (specific human-readable error description)
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

    # 1. Multi-Node Compact format (9+ bytes, where (len - 3) % 6 == 0):
    # [u16 plen][u8 st] { [u16 node][float val] }*
    # 9B (1 node), 15B (2 nodes), 21B (3 nodes)
    if len(blob) >= 9 and (len(blob) - 3) % 6 == 0:
        plen = struct.unpack_from('<H', blob, 0)[0]
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
    # [u16 len][u32 st=0][u32 msg_id][u8 dev_id][u16 data_id=0x11][float val]
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
    # [u16 len][u32 st=0][u8 dev_id][u16 data_id=0x11][float val]
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

class BankRollTransport:
    def __init__(self, device, stop_event):
        self.device = device
        self.stop_event = stop_event
        self.owned = False
        self.closed = False
        self.last_feedback = None
        self.feedback_time = None
        self.last_watchdog = 0.

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
            raise RuntimeError('CLS2Sim roll command rejected or malformed acknowledgement')

    def setting(self, setting, fmt, value):
        self.write(struct.pack('<III', 0xCE, 2, setting) + struct.pack('<'+fmt, value))

    def overrides(self, mask):
        # Override IDs contain GLOBAL masks. Only roll is owned by this mode.
        for setting in (3, 4, 2):
            self.write(struct.pack('<III', 0xD1, mask, setting))

    def read_position(self):
        with self.device._io_lock:
            reply = self.exchange(struct.pack('<III', 0xD0, 2, 0x11))
            if len(reply) < 3 or struct.unpack_from('<H', reply)[0] != len(reply)-2:
                raise RuntimeError('Malformed roll position packet length')
            result = parse_cls2sim_detailed(reply, 2)
            if not result['valid']:
                raise RuntimeError('Invalid roll feedback: '+str(result['error']))
            self.last_feedback = result['val']
            self.feedback_time = time.monotonic()
            return self.last_feedback

    def engage(self):
        with self.device._io_lock:
            if self.closed or self.stop_event.is_set():
                raise RuntimeError('Bank engagement cancelled')
            if getattr(self.device, '_pid_transport', None) is not None:
                raise RuntimeError('Airspeed PID still owns the hardware')
            start = self.read_position()
            # Mark ownership before any write so partial engagement also releases.
            self.owned = True
            try:
                self.write(struct.pack('<IIH', 0xD8, 0, 1000))
                self.last_watchdog = time.monotonic()
                self.overrides(2)
                self.setting(0x50, 'H', 2400)
                self.setting(0x60, 'H', 80)
                self.setting(0x80, 'f', start)
                self.setting(0x70, 'B', 1)
            except Exception:
                self.release()
                raise
            return start

    def command(self, position):
        with self.device._io_lock:
            if self.closed or self.stop_event.is_set() or not self.owned:
                raise RuntimeError('Bank roll ownership released')
            now = time.monotonic()
            if self.feedback_time is None or now-self.feedback_time > .35:
                raise RuntimeError('Roll position feedback stale')
            if not math.isfinite(position):
                raise RuntimeError('Invalid Bank roll command')
            position = max(-.95, min(.95, float(position)))  # Reference position_roll clamp.
            if now-self.last_watchdog >= .4:
                self.write(struct.pack('<IIH', 0xD8, 0, 1000))
                self.last_watchdog = now
            self.setting(0x80, 'f', position)

    def release(self):
        with self.device._io_lock:
            self.closed = True  # No producer can rearm or write after release.
            if not self.owned:
                return
            errors = []
            for operation in (
                lambda: self.setting(0x70, 'B', 0),
                lambda: self.overrides(0),
                lambda: self.write(struct.pack('<III', 0xD1, 0, 6)),
            ):
                try:
                    operation()
                except Exception as exc:
                    errors.append(str(exc))
            if errors:
                # Keep ownership flagged, allowing cleanup to retry. Watchdog is backup.
                raise RuntimeError('Bank release incomplete: '+'; '.join(errors))
            self.write(struct.pack('<IIH', 0xD8, 0, 0))
            self.owned = False
