"""
Unified Multi-Axis Brunner Transport.
Controls any combination of:
  - Pitch (Axis 1) -> Elevator (Airspeed PID)
  - Roll (Axis 2)  -> Ailerons (Bank PID / Heading Steer)
  - Rudder (Axis 4)-> Rudder (Yaw Damper / Turn Coordination)

STRICT RULE: Only the enabled axes are claimed and commanded.
If an axis is not enabled, its Brunner motor is NEVER commanded and remains 100% manual.
"""
from __future__ import annotations

import math
import struct
import time
from typing import Optional, Tuple

AXIS_PREFERRED_NODES = {
    1: [2, 1],        # Elevator loader CAN node 2 (fallback 1)
    2: [1, 2],        # Aileron loader CAN node 1 (fallback 2)
    4: [5, 4, 3],     # Rudder loader CAN node 5 (fallback 4, 3)
}


def parse_cls2sim_detailed(blob: bytes, target_axis: Optional[int] = None) -> dict:
    """Detailed diagnostic parser for CLS2Sim UDP telemetry packets."""
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


class MultiPidBrunnerTransport:
    """
    Multi-Axis Transport supporting any combination of:
      - Pitch (Axis 1)
      - Roll (Axis 2)
      - Rudder (Axis 4)
    """
    def __init__(self, device, stop_event, pitch_enabled: bool = False, bank_enabled: bool = False, yaw_enabled: bool = False):
        self.device = device
        self.stop_event = stop_event
        self.pitch_enabled = bool(pitch_enabled)
        self.bank_enabled = bool(bank_enabled)
        self.yaw_enabled = bool(yaw_enabled)

        # Roll is commanded if either Bank PID or Yaw PID is enabled
        self.roll_needed = self.bank_enabled or self.yaw_enabled

        self.rudder_available = True
        self.owned = False
        self.closed = False

        self.last_pitch_feedback = None
        self.last_roll_feedback = None
        self.last_rudder_feedback = None

        self.pitch_feedback_time = None
        self.roll_feedback_time = None
        self.rudder_feedback_time = None

        self.last_watchdog = 0.0

    def exchange(self, packet: bytes) -> bytes:
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

    def write(self, packet: bytes):
        reply = self.exchange(packet)
        if len(reply) < 3 or reply[2] != 0:
            raise RuntimeError('CLS2Sim command rejected or malformed acknowledgement')

    def setting(self, axis: int, setting_id: int, fmt: str, value):
        self.write(struct.pack('<III', 0xCE, int(axis), int(setting_id)) + struct.pack('<' + fmt, value))

    def overrides(self, mask: int):
        for setting_code in (3, 4, 2):
            self.write(struct.pack('<III', 0xD1, int(mask), setting_code))

    def read_axis_position(self, axis: int) -> float:
        with self.device._io_lock:
            reply = self.exchange(struct.pack('<III', 0xD0, int(axis), 0x11))
            if len(reply) < 3 or struct.unpack_from('<H', reply)[0] != len(reply) - 2:
                raise RuntimeError(f'Malformed position packet length for axis {axis}')
            result = parse_cls2sim_detailed(reply, axis)
            if not result['valid']:
                # Axis 4 (Rudder) may be unmotorized / absent
                if axis == 4 and (len(reply) == 3 or 'No position' in (result['error'] or '') or 'Unrecognized' in (result['error'] or '')):
                    self.rudder_available = False
                    self.last_rudder_feedback = 0.0
                    self.rudder_feedback_time = time.monotonic()
                    return 0.0
                raise RuntimeError(f'Invalid feedback for axis {axis}: {result["error"]}')
            now = time.monotonic()
            val = float(result['val'])
            if axis == 1:
                self.last_pitch_feedback = val
                self.pitch_feedback_time = now
            elif axis == 2:
                self.last_roll_feedback = val
                self.roll_feedback_time = now
            elif axis == 4:
                self.last_rudder_feedback = val
                self.rudder_feedback_time = now
            return val

    def read_positions(self) -> Tuple[float, float, float]:
        """Reads initial positions for all enabled axes. Non-enabled axes return 0.0."""
        pitch = self.read_axis_position(1) if self.pitch_enabled else 0.0
        roll = self.read_axis_position(2) if self.roll_needed else 0.0
        rudder = self.read_axis_position(4) if self.yaw_enabled else 0.0
        return pitch, roll, rudder

    def engage(self) -> Tuple[float, float, float]:
        with self.device._io_lock:
            if self.closed or self.stop_event.is_set():
                raise RuntimeError('PID engagement cancelled')

            # Seed positions
            start_pitch, start_roll, start_rudder = self.read_positions()

            self.owned = True
            try:
                # Arm watchdog
                self.write(struct.pack('<IIH', 0xD8, 0, 1000))
                self.last_watchdog = time.monotonic()

                # Build exact override mask
                mask = 0
                if self.pitch_enabled:
                    mask |= 0x01
                if self.roll_needed:
                    mask |= 0x02
                if self.yaw_enabled and self.rudder_available:
                    mask |= 0x04

                self.overrides(mask)

                # Configure Pitch (Axis 1)
                if self.pitch_enabled:
                    self.setting(1, 0x50, 'H', 2400)
                    self.setting(1, 0x60, 'H', 80)
                    self.setting(1, 0x80, 'f', start_pitch)
                    self.setting(1, 0x70, 'B', 1)

                # Configure Roll (Axis 2)
                if self.roll_needed:
                    self.setting(2, 0x50, 'H', 2400)
                    self.setting(2, 0x60, 'H', 80)
                    self.setting(2, 0x80, 'f', start_roll)
                    self.setting(2, 0x70, 'B', 1)

                # Configure Rudder (Axis 4)
                if self.yaw_enabled and self.rudder_available:
                    self.setting(4, 0x50, 'H', 2400)
                    self.setting(4, 0x60, 'H', 80)
                    self.setting(4, 0x80, 'f', start_rudder)
                    self.setting(4, 0x70, 'B', 1)

            except Exception:
                self.release()
                raise

            return start_pitch, start_roll, start_rudder

    def command(self, pitch_pos: Optional[float] = None, roll_pos: Optional[float] = None, rudder_pos: Optional[float] = None):
        with self.device._io_lock:
            if self.closed or self.stop_event.is_set() or not self.owned:
                raise RuntimeError('PID control ownership released')
            now = time.monotonic()

            if now - self.last_watchdog >= 0.4:
                self.write(struct.pack('<IIH', 0xD8, 0, 1000))
                self.last_watchdog = now

            if self.pitch_enabled and pitch_pos is not None:
                if self.pitch_feedback_time is None or now - self.pitch_feedback_time > 0.35:
                    raise RuntimeError('Pitch position feedback stale')
                if not math.isfinite(pitch_pos):
                    raise RuntimeError('Invalid Pitch motor command: non-finite position')
                clamped_pitch = max(-0.95, min(0.95, float(pitch_pos)))
                self.setting(1, 0x80, 'f', clamped_pitch)

            if self.roll_needed and roll_pos is not None:
                if self.roll_feedback_time is None or now - self.roll_feedback_time > 0.35:
                    raise RuntimeError('Roll position feedback stale')
                if not math.isfinite(roll_pos):
                    raise RuntimeError('Invalid Roll motor command: non-finite position')
                clamped_roll = max(-0.95, min(0.95, float(roll_pos)))
                self.setting(2, 0x80, 'f', clamped_roll)

            if self.yaw_enabled and self.rudder_available and rudder_pos is not None:
                if self.rudder_feedback_time is None or now - self.rudder_feedback_time > 0.35:
                    raise RuntimeError('Rudder position feedback stale')
                if not math.isfinite(rudder_pos):
                    raise RuntimeError('Invalid Rudder motor command: non-finite position')
                clamped_rudder = max(-0.40, min(0.40, float(rudder_pos)))
                self.setting(4, 0x80, 'f', clamped_rudder)

    def release(self):
        with self.device._io_lock:
            self.closed = True
            if not self.owned:
                return
            errors = []
            operations = []

            if self.pitch_enabled:
                operations.append(lambda: self.setting(1, 0x70, 'B', 0))
            if self.roll_needed:
                operations.append(lambda: self.setting(2, 0x70, 'B', 0))
            if self.yaw_enabled and self.rudder_available:
                operations.append(lambda: self.setting(4, 0x70, 'B', 0))

            operations.extend([
                lambda: self.overrides(0),
                lambda: self.write(struct.pack('<III', 0xD1, 0, 6)),
            ])

            for op in operations:
                try:
                    op()
                except Exception as exc:
                    errors.append(str(exc))

            if errors:
                raise RuntimeError('PID release incomplete: ' + '; '.join(errors))

            self.write(struct.pack('<IIH', 0xD8, 0, 0))
            self.owned = False

