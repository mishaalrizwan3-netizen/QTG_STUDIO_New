"""
SMK Prepar3D Fast Pitch Angle, Airspeed & Bank Angle Controller (V4.3)
Integrates SimConnect 64-bit Read-Only Native Telemetry Bridge with Brunner CLS2Sim UDP Protocol.

Features:
1. Pure Brunner CLS-P Dual-Axis Hardware Loader Control:
   - Pitch: Controlled purely via Brunner CLS-P Pitch Motor (CLS2Sim forwards stick to P3D).
   - Bank: Controlled purely via Brunner CLS-P Roll Motor (CLS2Sim forwards stick to P3D).
   - Prepar3D SimConnect is strictly READ-ONLY telemetry: ZERO flight control surface writes to P3D.
2. Zero-Deadband Bank Angle PID:
   - Continuously integrates attitude error with no deadband until Bank Error reaches exactly 0.00°.
3. Dual-Tabbed Clean Modern UI:
   - Tab 1: Airspeed & Pitch Hold (FLC / Pitch PID + Live Telemetry Graphs).
   - Tab 2: Bank Angle Hold (Wings Level = 0.0° via Brunner CLS-P Roll Loader + Live Graphs).
4. Independent thread timing:
   - SimConnect Telemetry Bridge: 50 Hz
   - PID / Guidance Worker: 25 Hz
   - GUI / Matplotlib Graphs: 10 Hz
"""

import collections
import csv
from datetime import datetime
import ctypes
import math
import os
import socket
import struct
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# -----------------------------------------------------------------------------
# Brunner CLS2Sim UDP Protocol Constants & Helpers (CLS-P Dual-Axis Loader)
# -----------------------------------------------------------------------------
HOST, PORT = '127.0.0.1', 15090
AX_PITCH  = 0x0001  # Elevator / Pitch motor axis
AX_ROLL   = 0x0002  # Aileron / Roll motor axis (CLS-P loader)
AX_RUDDER = 0x0004  # Rudder / Yaw motor axis (CLS-P loader)

# Brunner CAN Hardware Node mapping:
# On Brunner CLS-P systems, physical motor loaders have hardware CAN Node IDs:
# Node 1 = Aileron / Roll Motor Loader
# Node 2 = Elevator / Pitch Motor Loader
# Node 5 = Rudder Pedals / Yaw Motor Loader (or 4, 3 depending on cockpit installation)
#
# When querying CLS2Sim via UDP CMD_READ (0xD0):
# Outbound query axis mask: AX_PITCH (0x01), AX_ROLL (0x02), AX_RUDDER (0x04)
# In reply, CLS2Sim reports the physical CAN node ID of the hardware drive.
AXIS_PREFERRED_NODES = {
    AX_PITCH:  [2, 1],        # Elevator loader CAN node 2 (fallback 1)
    AX_ROLL:   [1, 2],        # Aileron loader CAN node 1 (fallback 2)
    AX_RUDDER: [5, 4, 3],     # Rudder loader CAN node 5 (fallback 4, 3)
}
CMD_READ, CMD_OVERRIDE, CMD_SETTINGS, CMD_REMOTE_CONFIG = 0xD0, 0xD1, 0xCE, 0xD8
CMD_STATE = 0xCD
ST_INIT, ST_SETOFFSET = 0x01, 0x05
POS_NORM = 0x11
OVR_AP_ENABLE, OVR_AP_POS, OVR_POSITION_TO_SIM = 0x03, 0x04, 0x06
SID_AP_FORCE, SID_AP_SPEED, SID_AP_ENABLE, SID_AP_POSITION = 0x50, 0x60, 0x70, 0x80
AP_FORCE, AP_SPEED = 2400, 80
WATCHDOG_MS = 1000

def pack_state(axis, subcmd):
    return struct.pack('<III', CMD_STATE, int(axis), int(subcmd))

def pack_read(var, axis=0):
    return struct.pack('<III', CMD_READ, int(axis), int(var))

def pack_override(axis, var):
    return struct.pack('<Iii', CMD_OVERRIDE, int(axis), int(var))

def pack_u16(axis, var, value):
    return struct.pack('<IiiH', CMD_SETTINGS, int(axis), int(var), int(value))

def pack_u8(axis, var, value):
    return struct.pack('<IiiB', CMD_SETTINGS, int(axis), int(var), int(value))

def pack_float(axis, var, value):
    return struct.pack('<Iiif', CMD_SETTINGS, int(axis), int(var), float(value))

def pack_watchdog(ms):
    return struct.pack('<IIH', CMD_REMOTE_CONFIG, 0, int(ms))

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


def parse_cls2sim_normalized(blob, target_axis=None):
    """Parses normalized position (-1.0 .. +1.0) from CLS2Sim UDP reply."""
    res = parse_cls2sim_detailed(blob, target_axis)
    return res['val'] if res['valid'] else None

def reply_status(blob):
    if len(blob) < 3:
        raise RuntimeError('Malformed CLS2Sim response')
    if len(blob) >= 6:
        st32 = struct.unpack_from('<I', blob, 2)[0]
        if st32 == 0:
            return 0
    return blob[2]

def reply_normalized(blob, target_axis=None):
    """Parses normalized stick position from CLS2Sim response."""
    val = parse_cls2sim_normalized(blob, target_axis)
    return val if val is not None else 0.0

def reply_positions(blob):
    """
    Parses CLS2Sim normalized stick positions for both Node 2 (Pitch) and Node 1 (Roll).
    Returns (pitch_norm, roll_norm).
    """
    p_pos = parse_cls2sim_normalized(blob, AX_PITCH)
    r_pos = parse_cls2sim_normalized(blob, AX_ROLL)
    return (p_pos if p_pos is not None else 0.0, r_pos if r_pos is not None else 0.0)

def slew_position(current, velocity, desired, dt, max_rate=0.50, max_accel=1.80):
    """Smooth acceleration & velocity limiter to prevent sudden physical motor jerks."""
    if dt <= 0:
        return current, velocity
    req_v = max(-max_rate, min(max_rate, (desired - current) / dt))
    next_v = max(velocity - max_accel * dt, min(velocity + max_accel * dt, req_v))
    next_p = current + next_v * dt
    if (desired - current) * (desired - next_p) <= 0:
        return desired, 0.0
    return next_p, next_v


class AxisDiagState:
    """Diagnostic tracking state for an individual Brunner CLS-P axis."""
    def __init__(self, name: str, mask: int, node_default: int):
        self.name = name
        self.mask = mask
        self.node_default = node_default
        self.connected = False
        self.feedback_valid = False
        self.measured_pos = 0.0
        self.last_query_time = 0.0
        self.last_reply_time = 0.0
        self.feedback_age_ms = -1.0
        self.last_format = "No Telemetry"
        self.last_status = None
        self.last_reported_node = None
        self.last_raw_hex = "--"
        self.last_error = "Awaiting feedback"

        # Telemetry metrics
        self.total_queries = 0
        self.valid_replies = 0
        self.timeouts = 0
        self.malformed_count = 0
        self.wrong_axis_count = 0
        self.bounds_errors = 0

        # Command metrics
        self.last_cmd_pos = 0.0
        self.last_cmd_time = 0.0
        self.last_cmd_ack_received = False
        self.last_cmd_ack_status = None
        self.last_cmd_ack_time = 0.0
        self.last_cmd_ack_age_ms = -1.0
        self.cmd_count = 0
        self.cmd_ack_count = 0
        self.cmd_reject_count = 0
        self.last_cmd_error = None


class CLS2Sim:
    """Robust Brunner CLS2Sim UDP Driver for Dual-Axis (Pitch & Roll) CLS-P Loader."""
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(('127.0.0.1', 0))
        except Exception:
            pass
        self.sock.settimeout(0.06)
        self.io_lock = threading.RLock()
        self.latest_pitch = 0.0
        self.latest_roll = 0.0
        self.latest_yaw = 0.0
        self.latest_time = 0.0
        self.owns_pitch_overrides = False
        self.owns_roll_overrides = False
        self.owns_yaw_overrides = False
        self.watchdog_enabled = False
        self.last_watchdog = 0.0
        self.last_watchdog_ack_status = None
        self.connected = False

        # Diagnostic state per axis (Elevator=CAN Node 2, Aileron=CAN Node 1, Rudder=CAN Node 5)
        self.diag = {
            AX_PITCH: AxisDiagState("Pitch (Elevator)", AX_PITCH, 2),
            AX_ROLL: AxisDiagState("Roll (Aileron)", AX_ROLL, 1),
            AX_RUDDER: AxisDiagState("Yaw (Rudder)", AX_RUDDER, 5),
        }
        self.recent_events = collections.deque(maxlen=150)
        self.total_udp_timeouts = 0

    def record_event(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        entry = f"[{ts}] {msg}"
        with self.io_lock:
            self.recent_events.append(entry)

    def drain(self):
        """Flushes any lingering packets in the UDP socket buffer."""
        try:
            self.sock.setblocking(False)
            while True:
                try:
                    self.sock.recvfrom(4096)
                except (BlockingIOError, socket.error):
                    break
        except Exception:
            pass
        finally:
            self.sock.setblocking(True)
            self.sock.settimeout(0.06)

    def exchange(self, packet: bytes) -> bytes:
        with self.io_lock:
            for attempt in range(2):
                self.drain()
                try:
                    self.sock.sendto(packet, (HOST, PORT))
                    reply, addr = self.sock.recvfrom(4096)
                    if addr[0] != HOST:
                        raise RuntimeError('Unexpected CLS2Sim response IP')
                    self.connected = True
                    return reply
                except (socket.timeout, OSError) as e:
                    if attempt == 1:
                        self.connected = False
                        raise RuntimeError('CLS2Sim UDP timeout/unavailable') from e
                    time.sleep(0.010)

    def write(self, packet: bytes):
        reply = self.exchange(packet)
        status = reply_status(reply)
        if status != 0:
            raise RuntimeError(f'CLS2Sim write rejected with status {status}')

    def _query_axis(self, axis_mask: int):
        diag = self.diag[axis_mask]
        diag.total_queries += 1
        now = time.monotonic()
        diag.last_query_time = now
        try:
            rep = self.exchange(pack_read(POS_NORM, axis_mask))
            # Packet received from CLS2Sim via UDP! Mark link ONLINE
            diag.connected = True
            diag.last_reply_time = time.monotonic()
            diag.feedback_age_ms = 0.0

            parsed = parse_cls2sim_detailed(rep, target_axis=axis_mask)
            diag.last_raw_hex = parsed['raw_hex']
            diag.last_format = parsed['format']
            diag.last_status = parsed['status']
            diag.last_reported_node = parsed['node']

            if parsed['valid']:
                diag.feedback_valid = True
                diag.measured_pos = parsed['val']
                diag.valid_replies += 1
                diag.last_error = None
                return parsed['val']
            else:
                diag.feedback_valid = False
                diag.last_error = parsed['error']
                err_str = parsed['error'] or ''
                if 'Wrong axis' in err_str:
                    diag.wrong_axis_count += 1
                elif 'bounds' in err_str or 'NaN' in err_str:
                    diag.bounds_errors += 1
                else:
                    diag.malformed_count += 1
                self.record_event(f"[{diag.name}] Telemetry invalid: {diag.last_error}")
                return None
        except Exception as exc:
            diag.timeouts += 1
            self.total_udp_timeouts += 1
            diag.feedback_valid = False
            if diag.last_reply_time > 0:
                diag.feedback_age_ms = (time.monotonic() - diag.last_reply_time) * 1000.0
                if diag.feedback_age_ms > 1500.0:
                    diag.connected = False
            else:
                diag.connected = False
                diag.feedback_age_ms = -1.0
            diag.last_error = f"UDP Timeout/Error: {exc}"
            return None

    def _send_cmd(self, axis_mask: int, target_pos: float):
        diag = self.diag[axis_mask]
        diag.last_cmd_pos = target_pos
        diag.last_cmd_time = time.monotonic()
        diag.cmd_count += 1
        try:
            pkt = pack_float(axis_mask, SID_AP_POSITION, target_pos)
            reply = self.exchange(pkt)
            diag.connected = True
            diag.last_reply_time = time.monotonic()
            status = reply_status(reply)
            diag.last_cmd_ack_time = time.monotonic()
            diag.last_cmd_ack_status = status
            diag.last_cmd_ack_age_ms = 0.0
            if status == 0:
                diag.last_cmd_ack_received = True
                diag.cmd_ack_count += 1
                diag.last_cmd_error = None
            else:
                diag.last_cmd_ack_received = False
                diag.cmd_reject_count += 1
                diag.last_cmd_error = f"Rejected status {status}"
                raise RuntimeError(f"[{diag.name}] Position command rejected: status {status}")
        except Exception as exc:
            diag.last_cmd_ack_received = False
            diag.last_cmd_ack_status = -1
            diag.cmd_reject_count += 1
            diag.last_cmd_error = f"Write error: {exc}"
            self.record_event(f"[{diag.name}] Cmd {target_pos:+.4f} failed: {exc}")
            raise

    def get_diag_snapshot(self):
        with self.io_lock:
            now = time.monotonic()
            axes_data = {}
            for mask, d in self.diag.items():
                age_ms = (now - d.last_reply_time) * 1000.0 if d.last_reply_time > 0 else -1.0
                ack_age_ms = (now - d.last_cmd_ack_time) * 1000.0 if d.last_cmd_ack_time > 0 else -1.0
                axes_data[mask] = {
                    'name': d.name,
                    'mask': d.mask,
                    'connected': d.connected and (age_ms < 1500.0 if age_ms >= 0 else False),
                    'feedback_valid': d.feedback_valid and (age_ms < 1500.0 if age_ms >= 0 else False),
                    'measured_pos': d.measured_pos,
                    'feedback_age_ms': age_ms,
                    'last_format': d.last_format,
                    'last_status': d.last_status,
                    'last_node': d.last_reported_node,
                    'last_raw_hex': d.last_raw_hex,
                    'last_error': d.last_error,
                    'total_queries': d.total_queries,
                    'valid_replies': d.valid_replies,
                    'timeouts': d.timeouts,
                    'malformed_count': d.malformed_count,
                    'wrong_axis_count': d.wrong_axis_count,
                    'bounds_errors': d.bounds_errors,
                    'last_cmd_pos': d.last_cmd_pos,
                    'last_cmd_ack_received': d.last_cmd_ack_received,
                    'last_cmd_ack_status': d.last_cmd_ack_status,
                    'last_cmd_ack_age_ms': ack_age_ms,
                    'cmd_count': d.cmd_count,
                    'cmd_ack_count': d.cmd_ack_count,
                    'cmd_reject_count': d.cmd_reject_count,
                    'last_cmd_error': d.last_cmd_error,
                }
            return {
                'connected': self.connected,
                'axes': axes_data,
                'owns_pitch': self.owns_pitch_overrides,
                'owns_roll': self.owns_roll_overrides,
                'owns_yaw': self.owns_yaw_overrides,
                'watchdog_enabled': self.watchdog_enabled,
                'watchdog_age_ms': (now - self.last_watchdog) * 1000.0 if self.last_watchdog > 0 else -1.0,
                'total_udp_timeouts': self.total_udp_timeouts,
                'events': list(self.recent_events),
            }

    def read_positions(self):
        val_p = self._query_axis(AX_PITCH)
        if val_p is not None:
            self.latest_pitch = val_p

        val_r = self._query_axis(AX_ROLL)
        if val_r is not None:
            self.latest_roll = val_r

        val_y = self._query_axis(AX_RUDDER)
        if val_y is not None:
            self.latest_yaw = val_y

        self.latest_time = time.monotonic()
        return self.latest_pitch, self.latest_roll, self.latest_yaw

    def poll(self):
        """Non-throwing poll for background 25 Hz control loop."""
        try:
            p_val, r_val, y_val = self.read_positions()
            return self.connected, p_val, r_val, y_val
        except Exception:
            return False, self.latest_pitch, self.latest_roll, self.latest_yaw

    def watchdog(self):
        """Periodically refresh watchdog (every 400ms) to maintain CLS2Sim motor engagement."""
        now = time.monotonic()
        if self.watchdog_enabled and (now - self.last_watchdog > 0.40):
            try:
                reply = self.exchange(pack_watchdog(WATCHDOG_MS))
                status = reply_status(reply)
                self.last_watchdog = now
                self.last_watchdog_ack_status = status
            except Exception:
                pass

    def _sync_axis_overrides(self, mask=None):
        """Override IDs hold GLOBAL axis masks, not independent per-axis flags."""
        if mask is None:
            mask = ((AX_PITCH if self.owns_pitch_overrides else 0) |
                    (AX_ROLL if self.owns_roll_overrides else 0) |
                    (AX_RUDDER if self.owns_yaw_overrides else 0))
        self.write(pack_override(mask, OVR_AP_ENABLE))
        self.write(pack_override(mask, OVR_AP_POS))
        # Preserve the existing trim ownership for lateral axes only.
        self.write(pack_override(mask & (AX_ROLL | AX_RUDDER), 0x02))

    def _engage_axis(self, axis, ownership, position, name):
        with self.io_lock:
            if not self.watchdog_enabled:
                self.write(pack_watchdog(WATCHDOG_MS))
                self.watchdog_enabled = True
                self.last_watchdog = time.monotonic()
            previously_owned = getattr(self, ownership)
            setattr(self, ownership, True)
            try:
                self._sync_axis_overrides()
                self.write(pack_u16(axis, SID_AP_FORCE, AP_FORCE))
                self.write(pack_u16(axis, SID_AP_SPEED, AP_SPEED))
                self.write(pack_float(axis, SID_AP_POSITION, position))
                self.write(pack_u8(axis, SID_AP_ENABLE, 1))
            except Exception:
                setattr(self, ownership, previously_owned)
                try:
                    self.write(pack_u8(axis, SID_AP_ENABLE, 0))
                    self._sync_axis_overrides()
                except Exception:
                    pass
                raise
            self.record_event(f"[{name}] Overrides armed; other active axes preserved")

    def _release_axis(self, axis, ownership, name):
        with self.io_lock:
            if getattr(self, ownership):
                try:
                    self.write(pack_u8(axis, SID_AP_ENABLE, 0))
                finally:
                    setattr(self, ownership, False)
                    self._sync_axis_overrides()
                self.record_event(f"[{name}] Overrides released; other active axes preserved")
            if not (self.owns_pitch_overrides or self.owns_roll_overrides or self.owns_yaw_overrides):
                self.write(pack_watchdog(0))
                self.watchdog_enabled = False

    def engage_pitch(self, start_pitch: float):
        self._engage_axis(AX_PITCH, 'owns_pitch_overrides', start_pitch, 'Pitch')

    def release_pitch(self):
        self._release_axis(AX_PITCH, 'owns_pitch_overrides', 'Pitch')

    def engage_roll(self, start_roll: float = 0.0):
        self._engage_axis(AX_ROLL, 'owns_roll_overrides', start_roll, 'Roll')

    def release_roll(self):
        self._release_axis(AX_ROLL, 'owns_roll_overrides', 'Roll')

    def engage_yaw(self, start_yaw: float = 0.0):
        self._engage_axis(AX_RUDDER, 'owns_yaw_overrides', start_yaw, 'Rudder')

    def release_yaw(self):
        self._release_axis(AX_RUDDER, 'owns_yaw_overrides', 'Rudder')

    def engage(self, start_pitch: float, start_roll: float = 0.0):
        """Arm Brunner CLS-P motor overrides for Pitch, Roll and Rudder axes."""
        self.engage_pitch(start_pitch)
        self.engage_roll(start_roll)

    def position_pitch(self, target_pos: float):
        """Command target pitch position to Brunner pitch motor [-0.95 .. +0.95]."""
        clamped = max(-0.95, min(0.95, float(target_pos)))
        self._send_cmd(AX_PITCH, clamped)

    def position_roll(self, target_pos: float):
        """Command target roll position to Brunner CLS-P roll motor [-0.95 .. +0.95]."""
        clamped = max(-0.95, min(0.95, float(target_pos)))
        self._send_cmd(AX_ROLL, clamped)

    def position_rudder(self, target_pos: float):
        """Command target rudder position to Brunner CLS-P rudder motor [-0.60 .. +0.60]."""
        clamped = max(-0.60, min(0.60, float(target_pos)))
        self._send_cmd(AX_RUDDER, clamped)

    def release(self):
        """Gracefully release all motor overrides and return manual stick control."""
        if self.owns_pitch_overrides or self.owns_roll_overrides or self.owns_yaw_overrides:
            for pkt in (
                pack_u8(AX_PITCH, SID_AP_ENABLE, 0),
                pack_override(0, OVR_AP_ENABLE),
                pack_u8(AX_ROLL, SID_AP_ENABLE, 0),
                pack_override(0, OVR_AP_POS),
                pack_u8(AX_RUDDER, SID_AP_ENABLE, 0),
                pack_override(0, 0x02),
                pack_watchdog(0)
            ):
                try:
                    self.write(pkt)
                except Exception:
                    pass
        self.watchdog_enabled = False
        self.owns_pitch_overrides = False
        self.owns_roll_overrides = False
        self.owns_yaw_overrides = False

    def close(self):
        self.release()
        try:
            self.sock.close()
        except Exception:
            pass


# -----------------------------------------------------------------------------
# SimConnect C++ Bridge Interface (Strictly Read-Only Telemetry)
# -----------------------------------------------------------------------------
class PitchBridgeState(ctypes.Structure):
    _fields_ = [
        ('pitchDeg', ctypes.c_double),
        ('elevatorPosition', ctypes.c_double),
        ('yokeYPosition', ctypes.c_double),
        ('iasKt', ctypes.c_double),
        ('altitudeFt', ctypes.c_double),
        ('throttlePercent', ctypes.c_double),
        ('sentElevator', ctypes.c_double),
        ('sentYokeY', ctypes.c_double),
        ('sentAxis', ctypes.c_double),
        ('telemetryAgeMs', ctypes.c_uint64),
        ('connected', ctypes.c_int),
        ('telemetryValid', ctypes.c_int),
        ('writingEnabled', ctypes.c_int),
        ('controlMode', ctypes.c_int),
        ('lastWriteOk', ctypes.c_int),
        ('lastError', ctypes.c_int),
        ('totalWeightKg', ctypes.c_double),
        ('ambientTempC', ctypes.c_double),
        ('isPaused', ctypes.c_int),
        ('bankDeg', ctypes.c_double),
        ('headingDeg', ctypes.c_double),
        ('yawRateDegSec', ctypes.c_double),
        ('sideslipDeg', ctypes.c_double),
        ('rudderPosition', ctypes.c_double),
    ]


class SimConnectBridge:
    def __init__(self):
        search_dirs = []
        if getattr(sys, 'frozen', False):
            search_dirs.append(os.path.dirname(sys.executable))
            if hasattr(sys, '_MEIPASS'):
                search_dirs.append(sys._MEIPASS)
        search_dirs.append(os.path.dirname(os.path.abspath(__file__)))
        search_dirs.append(os.getcwd())

        dll_path = None
        base_dir = search_dirs[0]
        for d in search_dirs:
            for name in ('smk_simbridge.dll', 'smk_simbridge_v4.dll'):
                candidate = os.path.join(d, name)
                if os.path.isfile(candidate):
                    dll_path = candidate
                    base_dir = d
                    break
            if dll_path:
                break

        if not dll_path:
            raise FileNotFoundError("Native SimConnect bridge DLL 'smk_simbridge.dll' not found in search paths")

        if sys.platform == 'win32':
            try:
                os.add_dll_directory(base_dir)
            except Exception:
                pass

        self.dll = ctypes.CDLL(dll_path)
        if not hasattr(self.dll, 'SC_GetAbiVersion'):
            raise RuntimeError('Old bridge DLL detected. Compile the included V4.4 C++ source.')
        self.dll.SC_GetAbiVersion.restype = ctypes.c_int
        abi = self.dll.SC_GetAbiVersion()
        if abi != 44:
            raise RuntimeError(f'Wrong SimConnect DLL ABI ({abi}). Expected ABI 44.')
        
        self.dll.SC_Start.restype = ctypes.c_int
        self.dll.SC_GetState.argtypes = [ctypes.c_void_p]
        self.dll.SC_GetState.restype = ctypes.c_int
        self.dll.SC_SetControlMode.argtypes = [ctypes.c_int]
        self.dll.SC_Command.argtypes = [ctypes.c_double]
        self.dll.SC_Enable.argtypes = [ctypes.c_int]
        self.dll.SC_Stop.restype = None

        if hasattr(self.dll, 'SC_IsPaused'):
            self.dll.SC_IsPaused.restype = ctypes.c_int

        if self.dll.SC_Start() != 1:
            raise RuntimeError("Failed to start SimConnect background worker")

    def read(self) -> PitchBridgeState:
        state = PitchBridgeState()
        self.dll.SC_GetState(ctypes.byref(state))
        return state

    def is_paused(self) -> bool:
        if hasattr(self.dll, 'SC_IsPaused'):
            try:
                return bool(self.dll.SC_IsPaused())
            except Exception:
                pass
        return False

    def set_mode(self, mode: int):
        self.dll.SC_SetControlMode(int(mode))

    def enable(self, enabled: bool):
        self.dll.SC_Enable(1 if enabled else 0)

    def close(self):
        try:
            self.enable(False)
            self.dll.SC_Stop()
        except Exception:
            pass


# -----------------------------------------------------------------------------
# Inner-Loop: Pitch Attitude PID Controller (Brunner Pitch Motor)
# -----------------------------------------------------------------------------
class PitchPID:
    def __init__(self, kp=0.030, ki=0.004, kd=0.045, invert=False):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.invert = bool(invert)
        self.integral = 0.0
        self.last_pitch = None
        self.pitch_rate = 0.0
        self.output = 0.0

    def reset(self, initial_pos=0.0):
        self.integral = -float(initial_pos) if self.invert else float(initial_pos)
        self.last_pitch = None
        self.pitch_rate = 0.0
        self.output = float(initial_pos)

    def update(self, target_deg: float, actual_deg: float, dt: float) -> float:
        if dt <= 0.0001:
            return self.output

        error = target_deg - actual_deg

        if self.last_pitch is None:
            self.last_pitch = actual_deg
        raw_rate = (actual_deg - self.last_pitch) / dt
        self.last_pitch = actual_deg
        self.pitch_rate += (dt / (0.24 + dt)) * (raw_rate - self.pitch_rate)

        p_term = self.kp * error
        d_term = -self.kd * self.pitch_rate
        trial_i = max(-0.55, min(0.65, self.integral + self.ki * error * dt))
        trial_raw = p_term + trial_i + d_term
        trial_effort = trial_raw
        low, high = (-0.65, 0.55) if self.invert else (-0.55, 0.65)
        pushes_further = (trial_effort >= high and error > 0) or \
                         (trial_effort <= low and error < 0)
        if abs(error) > 0.12 and not pushes_further:
            self.integral = trial_i
        effort = p_term + self.integral + d_term
        if self.invert:
            effort = -effort
        self.output = max(-0.55, min(0.65, effort))
        return self.output


# -----------------------------------------------------------------------------
# Outer-Loop: Airspeed Controller (Pitch-for-Speed / FLC Mode)
# -----------------------------------------------------------------------------
class AirspeedController:
    def __init__(self, kp=0.24, ki=0.008, kd=0.32, min_pitch=-15.0, max_pitch=15.0):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.min_pitch = float(min_pitch)
        self.max_pitch = float(max_pitch)
        self.integral = 4.0
        self.last_ias = None
        self.ias_rate = 0.0
        self.cmd_pitch = 4.0

    def reset(self, initial_pitch=4.0):
        self.integral = max(self.min_pitch, min(self.max_pitch, float(initial_pitch)))
        self.last_ias = None
        self.ias_rate = 0.0
        self.cmd_pitch = float(initial_pitch)

    def update(self, target_ias: float, actual_ias: float, dt: float, current_pitch: float) -> float:
        if dt <= 0.0001:
            return self.cmd_pitch

        spd_error = actual_ias - target_ias

        if self.last_ias is None:
            self.last_ias = actual_ias
        raw_rate = (actual_ias - self.last_ias) / dt
        self.last_ias = actual_ias
        self.ias_rate += (dt / (0.60 + dt)) * (raw_rate - self.ias_rate)

        p_term = self.kp * spd_error

        if not ((self.cmd_pitch >= self.max_pitch and spd_error > 0) or
                (self.cmd_pitch <= self.min_pitch and spd_error < 0)):
            self.integral += self.ki * spd_error * dt
            self.integral = max(self.min_pitch, min(self.max_pitch, self.integral))

        i_term = self.integral
        d_term = self.kd * self.ias_rate
        pitch_demand = p_term + i_term + d_term

        desired_target = max(self.min_pitch, min(self.max_pitch, pitch_demand))
        max_step = 1.25 * dt
        delta = desired_target - self.cmd_pitch
        if abs(delta) > max_step:
            self.cmd_pitch += max_step if delta > 0 else -max_step
        else:
            self.cmd_pitch = desired_target

        return self.cmd_pitch


# -----------------------------------------------------------------------------
# Bank Angle PID Controller (Brunner CLS-P Roll Motor Control — ZERO DEADBAND)
# -----------------------------------------------------------------------------
class BankPID:
    """
    Bank Angle (Roll Attitude) PID Controller.
    Commands the physical Brunner CLS-P Roll Motor to actively hold Bank = target_deg:
    - Purely hardware-driven: drives physical Brunner roll loader, NO P3D writes!
    - Natural polarity (invert=False): right bank (+10°) -> negative roll command (left yoke) -> rolls left to 0.00°.
    - Zero deadband: integrates any residual error continually down to 0.00°!
    - Anti-windup conditional integration: prevents stick latching during disturbances.
    - Strong roll-rate damping (Kd) prevents limit cycles and pilot-induced oscillations.
    """
    def __init__(self, kp=0.015, ki=0.0010, kd=0.020, invert=False):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.invert = bool(invert)
        self.integral = 0.0
        self.last_bank = None
        self.bank_rate = 0.0
        self.output = 0.0

    def reset(self, initial_output=0.0):
        init_i = max(-0.25, min(0.25, float(initial_output)))
        self.integral = -init_i if self.invert else init_i
        self.last_bank = None
        self.bank_rate = 0.0
        self.output = float(initial_output)

    def update(self, target_deg: float, actual_deg: float, dt: float) -> float:
        if dt <= 0.0001:
            return self.output

        error = target_deg - actual_deg

        if self.last_bank is None:
            self.last_bank = actual_deg
        raw_rate = (actual_deg - self.last_bank) / dt
        self.last_bank = actual_deg
        # 0.08s time constant filter: smoothly filters roll rate without derivative kick
        self.bank_rate += (dt / (0.08 + dt)) * (raw_rate - self.bank_rate)

        p_term = self.kp * error
        d_term = -self.kd * self.bank_rate

        # Anti-windup conditional integration:
        # 1. Integrate when |error| <= 4.0° to eliminate steady-state trim offsets
        # 2. Clamped to ±0.25 authority to allow full aerodynamic trim while preventing runaway
        if abs(error) <= 4.0:
            int_mult = 1.5 if abs(error) < 0.50 else 1.0
            trial_i = max(-0.25, min(0.25, self.integral + self.ki * error * dt * int_mult))
            trial_raw = p_term + trial_i + d_term
            trial_effort = trial_raw  # saturation test precedes motor polarity
            # Anti-saturation: freeze integration if actuator is already near clamp (±0.60)
            pushes_further = (trial_effort >= 0.58 and error > 0) or \
                             (trial_effort <= -0.58 and error < 0)
            if not pushes_further:
                self.integral = trial_i

        effort = p_term + self.integral + d_term

        # Smooth deadband-free micro-trim:
        if abs(error) < 0.50:
            stiction_bias = (error / 0.50) * 0.005
            effort += stiction_bias

        if self.invert:
            effort = -effort

        # Autopilot roll authority clamped to ±0.65:
        self.output = max(-0.65, min(0.65, effort))
        return self.output


# -----------------------------------------------------------------------------
# Outer-Loop: Heading Controller (Heading-to-Bank Guidance)
# -----------------------------------------------------------------------------
class HeadingController:
    """
    Outer-Loop Heading Autopilot (Heading-to-Bank Guidance).
    Computes commanded bank angle to steer the aircraft to target heading:
    - Wraps heading error seamlessly across 0°/360° north boundary [-180..+180].
    - Natural polarity: right turn needed (hdg_err > 0) -> demand positive bank -> BankPID drives right turn.
    - Demands proportional bank angle up to standard-rate turn limit (default ±20.0°).
    - Uses true Heading Rate (d(Heading)/dt) for negative damping to lead-out of the turn.
    - Eliminates bank oscillations, hunting, and Dutch roll cross-coupling.
    - Washes commanded bank angle smoothly to exact 0.00° as heading error reaches zero.
    - When heading is captured, bank = 0.00° and yaw = 0.00° (wings level straight flight).
    """
    def __init__(self, kp=0.80, ki=0.005, kd=0.70, max_bank=20.0, invert=False):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.max_bank = float(max_bank)
        self.invert = bool(invert)
        self.integral = 0.0
        self.cmd_bank = 0.0
        self.last_hdg = None
        self.hdg_rate = 0.0

    def reset(self, initial_bank=0.0):
        self.integral = 0.0
        self.cmd_bank = float(initial_bank)
        self.last_hdg = None
        self.hdg_rate = 0.0

    def update(self, target_hdg: float, actual_hdg: float, yaw_rate: float, dt: float) -> float:
        if dt <= 0.0001:
            return self.cmd_bank

        # Shortest-turn heading error wrapped to [-180 .. +180]
        # target_hdg may be any value (e.g. 400° -> 40°)
        norm_tgt = target_hdg % 360.0
        norm_act = actual_hdg % 360.0
        hdg_err = (norm_tgt - norm_act + 180.0) % 360.0 - 180.0

        # Compute true heading rate directly from consecutive heading telemetry
        if self.last_hdg is None:
            self.last_hdg = actual_hdg
        raw_rate = ((actual_hdg - self.last_hdg + 180.0) % 360.0 - 180.0) / dt
        self.last_hdg = actual_hdg
        # 0.20s low-pass filter to reject telemetry discretization steps
        self.hdg_rate += (dt / (0.20 + dt)) * (raw_rate - self.hdg_rate)

        # Brunner Roll Motor polarity (BankPID invert=True):
        # Right Turn (hdg_err > 0) -> demand POSITIVE bank -> BankPID invert produces NEGATIVE Brunner roll (right turn).
        # Left Turn  (hdg_err < 0) -> demand NEGATIVE bank -> BankPID invert produces POSITIVE Brunner roll (left turn).
        p_term = self.kp * hdg_err

        # Lead-out damping:
        # Counteract turn rate to smoothly wash out bank as heading approaches target
        d_term = -self.kd * self.hdg_rate

        # Anti-windup conditional integration within ±10.0° of target heading
        if abs(hdg_err) <= 10.0:
            self.integral = max(-5.0, min(5.0, self.integral + self.ki * hdg_err * dt))
        else:
            self.integral = 0.0

        turn_cmd = p_term + self.integral + d_term

        # Optional user polarity inversion toggle
        if self.invert:
            turn_cmd = -turn_cmd

        # Progressive heading-capture bank limit (prevents excessive bank on capture):
        capture_limit = min(self.max_bank, max(2.5, abs(hdg_err) * 1.25))
        demanded = max(-capture_limit, min(capture_limit, turn_cmd))

        # Smooth slew rate limit on commanded bank (6.0 deg/sec for responsive, oscillation-free flight)
        max_step = 6.0 * dt
        delta = demanded - self.cmd_bank
        if abs(delta) > max_step:
            self.cmd_bank += max_step if delta > 0 else -max_step
        else:
            self.cmd_bank = demanded
        return self.cmd_bank


# -----------------------------------------------------------------------------
# Inner-Loop: Yaw / Rudder Loader PID Controller (Coordinated Flight & Yaw Damper)
# -----------------------------------------------------------------------------
class YawPID:
    """
    Yaw / Rudder PID Controller for physical Brunner CLS-P Rudder Loader (AX_RUDDER = 0x0004).
    Maintains coordinated flight (zero sideslip, zero adverse yaw):
    - When turning: dynamically counteracts adverse yaw using yaw rate damping (Kd).
    - When on heading: drives sideslip angle to 0.00° and centers rudder pedals at 0.00.
    - Zero P3D writes: drives physical Brunner rudder loader, NO P3D writes!
    """
    def __init__(self, kp=0.025, ki=0.0020, kd=0.035, invert=False):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.invert = bool(invert)
        self.integral = 0.0
        self.output = 0.0

    def reset(self, initial_output=0.0):
        seed = -float(initial_output) if self.invert else float(initial_output)
        self.integral = max(-0.10, min(0.10, seed))
        self.output = float(initial_output)

    def update(self, sideslip_deg: float, yaw_rate_deg_sec: float, dt: float, coordinated_rate_deg_sec: float = 0.0) -> float:
        if dt <= 0.0001:
            return self.output

        # Coordinated flight error: target sideslip = 0.0
        err_beta = 0.0 - sideslip_deg
        p_term = self.kp * err_beta
        d_term = -self.kd * (yaw_rate_deg_sec - coordinated_rate_deg_sec)

        # Anti-windup conditional integration
        if abs(err_beta) <= 4.0:
            trial_i = max(-0.15, min(0.15, self.integral + self.ki * err_beta * dt))
            trial = p_term + trial_i + d_term
            if abs(trial) <= 0.40 or trial * err_beta < 0.0:
                self.integral = trial_i

        effort = p_term + self.integral + d_term
        if self.invert:
            effort = -effort

        # Authority clamp for rudder pedal loader (±0.40)
        self.output = max(-0.40, min(0.40, effort))
        return self.output


# -----------------------------------------------------------------------------
# GUI Application with Dual Tabs & Pure Brunner CLS-P Actuation
# -----------------------------------------------------------------------------
class HeadingBankHandoff:
    """Select one roll target, with capture dwell and drift hysteresis."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.captured = False
        self.settled_seconds = 0.0
        self.target = None

    def update(self, bank_on, heading_on, target, heading, bank, heading_rate, dt, valid=True):
        if not (bank_on and heading_on):
            self.reset()
            return False
        target = target % 360.0
        target_changed = self.target is None or abs((target-self.target+180.0)%360.0-180.0) > 0.1
        self.target = target
        error = (target-heading+180.0)%360.0-180.0
        if target_changed:
            self.captured = False
            self.settled_seconds = 0.0
        if not valid:
            self.settled_seconds = 0.0
            return self.captured
        if self.captured:
            if abs(error) > 4.0:
                self.captured = False
                self.settled_seconds = 0.0
        elif abs(error) <= 1.5 and abs(bank) <= 3.0 and abs(heading_rate) <= 0.5:
            self.settled_seconds += max(0.0, min(dt, 0.1))
            if self.settled_seconds >= 2.0:
                self.captured = True
        else:
            self.settled_seconds = 0.0
        return self.captured


class PitchPIDApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Prepar3D Flight Controller V4.4 — Brunner CLS-P Pitch, Roll & Rudder Control")
        self.root.geometry(f"{min(1600, self.root.winfo_screenwidth())}x{min(900, self.root.winfo_screenheight()-70)}")
        self.root.minsize(1200, 650)
        self.root.state("zoomed")

        # Dark theme styling
        self.bg_color = "#1e222b"
        self.panel_bg = "#272c38"
        self.text_color = "#e6edf3"
        self.accent_color = "#388bfd"
        self.root.configure(bg=self.bg_color)

        # Subsystems & Controllers
        self.bridge = SimConnectBridge()
        self.cls = CLS2Sim()
        self.pid = PitchPID(kp=0.030, ki=0.004, kd=0.045, invert=False)
        self.ias_ctrl = AirspeedController(kp=0.24, ki=0.008, kd=0.32, min_pitch=-15.0, max_pitch=15.0)
        self.bank_pid = BankPID(kp=0.015, ki=0.0010, kd=0.020, invert=False)
        self.heading_ctrl = HeadingController(kp=0.80, ki=0.005, kd=0.70, max_bank=20.0, invert=False)
        self.yaw_pid = YawPID(kp=0.025, ki=0.0020, kd=0.035, invert=False)

        self.hdg_kp_var = tk.DoubleVar(value=0.80)
        self.hdg_ki_var = tk.DoubleVar(value=0.005)
        self.hdg_kd_var = tk.DoubleVar(value=0.70)
        self.yaw_kp_var = tk.DoubleVar(value=0.025)
        self.yaw_ki_var = tk.DoubleVar(value=0.0020)
        self.yaw_kd_var = tk.DoubleVar(value=0.035)

        # Thread synchronization
        self.lock = threading.Lock()
        self.running = True
        self.after_id = None

        self.pitch_enabled = False
        self.bank_enabled = False
        self.yaw_enabled = False
        self.active_tab_index = 0
        self.guidance_mode = "IAS"
        self.target_pitch = 0.0
        self.target_ias = 80.0
        self.target_bank = 0.0
        self.target_heading = 360.0
        self.bank_invert_var = tk.BooleanVar(value=False)
        self.heading_invert_var = tk.BooleanVar(value=False)
        self.yaw_invert_var = tk.BooleanVar(value=False)
        self.cmd_pitch = 0.0
        self.demanded_bank = 0.0
        self.bank_handoff = HeadingBankHandoff()
        self.snap_bank_phase = "OFF"
        self.control_mode = 3

        # Brunner motor state (Pitch, Roll & Yaw/Rudder)
        self.brunner_pitch_cmd = 0.0
        self.brunner_pitch_vel = 0.0
        self.brunner_roll_cmd = 0.0
        self.brunner_roll_vel = 0.0
        self.brunner_yaw_cmd = 0.0
        self.brunner_yaw_vel = 0.0

        # Pause tracking
        self.snap_paused = False
        self.was_paused = False

        # Logging
        self.log_lock = threading.Lock()
        self.log_file = None
        self.log_writer = None
        self.log_path = ''
        self.last_fault = ''
        self.failure_start = None
        self.run_started = None
        self.log_counter = 0

        # Snapshots shared from control worker to GUI
        self.snap_connected = False
        self.snap_telemetry_age = 9999
        self.snap_brunner_connected = False
        self.snap_brunner_pitch = 0.0
        self.snap_brunner_roll = 0.0
        self.snap_brunner_yaw = 0.0
        self.snap_pitch_actual = 0.0
        self.snap_pitch_target = 0.0
        self.snap_pitch_error = 0.0
        self.snap_ias_actual = 0.0
        self.snap_ias_target = 80.0
        self.snap_ias_error = 0.0
        self.snap_cmd_pitch = 0.0
        self.snap_guidance_mode = "IAS"
        self.snap_throttle = 0.0
        self.snap_elev_pos = 0.0
        self.snap_yoke_pos = 0.0
        self.snap_weight = float('nan')
        self.snap_temp = float('nan')
        self.measured_hz = 25.0
        self.snap_measured_hz = 25.0

        # Bank angle snapshots
        self.snap_bank_actual = 0.0
        self.snap_bank_target = 0.0
        self.snap_bank_error = 0.0

        # Heading & Yaw snapshots
        self.snap_heading_actual = 360.0
        self.snap_heading_target = 360.0
        self.snap_heading_error = 0.0
        self.snap_yaw_rate = 0.0
        self.snap_sideslip = 0.0
        self.snap_rudder_pos = 0.0
        self.snap_demanded_bank = 0.0

        # Rolling telemetry buffer for live graphs (30 seconds at 10 Hz = 300 pts)
        self.history_len = 300
        self.time_history = collections.deque(maxlen=self.history_len)
        self.pitch_actual_hist = collections.deque(maxlen=self.history_len)
        self.pitch_target_hist = collections.deque(maxlen=self.history_len)
        self.pitch_error_hist = collections.deque(maxlen=self.history_len)
        self.ias_actual_hist = collections.deque(maxlen=self.history_len)
        self.ias_target_hist = collections.deque(maxlen=self.history_len)
        self.ias_error_hist = collections.deque(maxlen=self.history_len)
        self.brunner_pitch_hist = collections.deque(maxlen=self.history_len)
        self.brunner_pitch_cmd_hist = collections.deque(maxlen=self.history_len)

        # Bank Angle & Brunner Roll Graph Buffers
        self.bank_actual_hist = collections.deque(maxlen=self.history_len)
        self.bank_target_hist = collections.deque(maxlen=self.history_len)
        self.bank_error_hist = collections.deque(maxlen=self.history_len)
        self.brunner_roll_hist = collections.deque(maxlen=self.history_len)
        self.brunner_roll_cmd_hist = collections.deque(maxlen=self.history_len)

        # Heading & Yaw Graph Buffers
        self.heading_actual_hist = collections.deque(maxlen=self.history_len)
        self.heading_target_hist = collections.deque(maxlen=self.history_len)
        self.heading_error_hist = collections.deque(maxlen=self.history_len)
        self.demanded_bank_hist = collections.deque(maxlen=self.history_len)
        self.sideslip_hist = collections.deque(maxlen=self.history_len)
        self.yaw_rate_hist = collections.deque(maxlen=self.history_len)
        self.brunner_yaw_hist = collections.deque(maxlen=self.history_len)
        self.brunner_yaw_cmd_hist = collections.deque(maxlen=self.history_len)

        self.start_time = time.monotonic()
        self.active_tab_index = 0

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # 1. Start dedicated high-precision PID control worker (25 Hz = 40 ms)
        self.worker_thread = threading.Thread(target=self._pid_worker_loop, name="PIDControlWorker", daemon=True)
        self.worker_thread.start()

        # 2. Start smooth GUI chart rendering loop (10 Hz = 100 ms)
        self.after_id = self.root.after(100, self.gui_tick)

    def _build_ui(self):
        style = ttk.Style()
        style.theme_use('default')
        style.configure('.', background=self.panel_bg, foreground=self.text_color)
        style.configure('TLabel', background=self.panel_bg, foreground=self.text_color, font=('Segoe UI', 10))
        style.configure('TCheckbutton', background=self.panel_bg, foreground=self.text_color, font=('Segoe UI', 10, 'bold'))
        style.configure('TRadiobutton', background=self.panel_bg, foreground=self.text_color, font=('Segoe UI', 10))

        # Notebook tabs styling
        style.configure('TNotebook', background=self.bg_color, borderwidth=0)
        style.configure('TNotebook.Tab', background='#21262d', foreground='#c9d1d9',
                        font=('Segoe UI', 10, 'bold'), padding=[20, 7])
        style.map('TNotebook.Tab',
                  background=[('selected', '#1f6feb'), ('active', '#30363d')],
                  foreground=[('selected', '#ffffff'), ('active', '#ffffff')])

        # ----------------- Top Header Bar -----------------
        top_bar = tk.Frame(self.root, bg=self.panel_bg, height=56, padx=14, pady=8)
        top_bar.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(10, 4))

        title_lbl = tk.Label(top_bar, text="P3D BRUNNER CLS-P FLIGHT CONTROLLER",
                             font=('Segoe UI', 12, 'bold'), bg=self.panel_bg, fg="#58a6ff")
        title_lbl.pack(side=tk.LEFT)

        # Status pills
        self.conn_badge = tk.Label(top_bar, text="SimConnect: Disconnected",
                                   font=('Segoe UI', 9, 'bold'), bg="#484f58", fg="#f0f6fc",
                                   padx=10, pady=3)
        self.conn_badge.pack(side=tk.LEFT, padx=5)

        self.brunner_badge = tk.Label(top_bar, text="Brunner: Checking...",
                                      font=('Segoe UI', 9, 'bold'), bg="#484f58", fg="#f0f6fc",
                                      padx=10, pady=3)
        self.brunner_badge.pack(side=tk.LEFT, padx=4)

        self.mode_badge = tk.Label(top_bar, text="CONTROLLERS: IDLE",
                                   font=('Segoe UI', 9, 'bold'), bg="#484f58", fg="#ffffff",
                                   padx=10, pady=3)
        self.mode_badge.pack(side=tk.LEFT, padx=4)

        self.loop_badge = tk.Label(top_bar, text="PID Rate: 25.0 Hz",
                                   font=('Segoe UI', 9, 'bold'), bg="#1f6feb", fg="#ffffff",
                                   padx=10, pady=3)
        self.loop_badge.pack(side=tk.LEFT, padx=4)

        self.log_badge = tk.Label(top_bar, text="CSV: idle", font=('Segoe UI', 9),
                                  bg="#484f58", fg="#ffffff", padx=8, pady=3)
        self.log_badge.pack(side=tk.LEFT, padx=4)

        # Disarm All emergency button (Red)
        self.disarm_btn = tk.Button(top_bar, text="🚨 DISARM ALL",
                                    font=('Segoe UI', 10, 'bold'), bg="#da3633", fg="#ffffff",
                                    activebackground="#b62324", activeforeground="#ffffff",
                                    relief=tk.FLAT, padx=12, pady=4, cursor="hand2",
                                    command=self.disarm_all)
        self.disarm_btn.pack(side=tk.RIGHT, padx=4)

        # Contextual Start / Stop Active Tab Button
        self.engage_btn = tk.Button(top_bar, text="START ACTIVE CONTROLLER",
                                    font=('Segoe UI', 10, 'bold'), bg="#238636", fg="#ffffff",
                                    activebackground="#2ea043", activeforeground="#ffffff",
                                    relief=tk.FLAT, padx=14, pady=4, cursor="hand2",
                                    command=self.toggle_pid)
        self.engage_btn.pack(side=tk.RIGHT, padx=4)

        self.open_logs_btn = tk.Button(top_bar, text="📁 Logs",
                                       font=('Segoe UI', 9, 'bold'), bg="#21262d", fg="#58a6ff",
                                       activebackground="#30363d", activeforeground="#58a6ff",
                                       relief=tk.FLAT, padx=8, pady=4, cursor="hand2",
                                       command=self.open_logs_folder)
        self.open_logs_btn.pack(side=tk.RIGHT, padx=4)

        # ----------------- Notebook Tab Control -----------------
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=(2, 6))
        self.notebook.bind("<<NotebookTabChanged>>", self.on_tab_changed)

        self.dashboard = tk.Frame(self.notebook, bg=self.bg_color)
        self.notebook.add(self.dashboard, text="  Flight Controls  ")
        self.dashboard.rowconfigure(0, weight=1)
        self.tab_ias = tk.Frame(self.dashboard, bg=self.bg_color)
        self.tab_bank = tk.Frame(self.dashboard, bg=self.bg_color)
        self.tab_yaw = tk.Frame(self.dashboard, bg=self.bg_color)
        for column, panel in enumerate((self.tab_ias, self.tab_bank, self.tab_yaw)):
            self.dashboard.columnconfigure(column, weight=1, uniform="controllers")
            panel.grid(row=0, column=column, sticky="nsew", padx=3)
        self.tab_diag = tk.Frame(self.notebook, bg=self.bg_color)
        self.notebook.add(self.tab_diag, text="  BRUNNER Diagnostics  ")

        self._build_tab_ias(self.tab_ias)
        self._build_tab_bank(self.tab_bank)
        self._build_tab_yaw(self.tab_yaw)
        self._build_tab_diagnostics(self.tab_diag)
        self._layout_dashboard()

    def _layout_dashboard(self):
        """Arrange the existing controls without changing their variables or callbacks."""
        header = self.conn_badge.master
        items = header.winfo_children()
        for widget in items:
            widget.pack_forget()
        items[0].configure(text="P3D BRUNNER FLIGHT CONTROLLER", font=("Segoe UI", 11, "bold"))
        items[0].grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 5))
        self.open_logs_btn.grid(row=0, column=4, sticky="e")
        self.disarm_btn.grid(row=0, column=5, sticky="e")
        # Each column has its own explicit start/stop button.
        for column, badge in enumerate((self.conn_badge, self.brunner_badge,
                                       self.mode_badge, self.loop_badge, self.log_badge)):
            header.columnconfigure(column, weight=1, uniform="status")
            badge.configure(font=("Segoe UI", 8, "bold"), padx=3, wraplength=210)
            badge.grid(row=1, column=column, sticky="ew", padx=2)
        header.columnconfigure(5, weight=1, uniform="status")

        for index, panel in enumerate((self.tab_ias, self.tab_bank, self.tab_yaw)):
            controls, graph = panel.winfo_children()
            controls.configure(padx=4, pady=3)
            controls.pack_configure(padx=0, pady=2)
            graph.pack_configure(padx=0, pady=2)
            row1, row2 = controls.winfo_children()
            boxes = row1.winfo_children()
            for box in boxes:
                box.pack_forget()
            row1.columnconfigure(0, weight=1)
            row1.columnconfigure(1, weight=1)
            if index == 0:
                # The fixed guidance/actuator descriptions are replaced by the column title.
                target, polarity, toggle = boxes[1], boxes[3], boxes[4]
                target.grid(row=0, column=0, columnspan=2, sticky="ew")
                polarity.grid(row=1, column=0, columnspan=2, sticky="ew")
                toggle.grid(row=2, column=0, columnspan=2, sticky="ew", pady=3)
            else:
                boxes[0].grid(row=0, column=0, columnspan=2, sticky="ew")
                for c, gains in enumerate(boxes[1:-1]):
                    gains.grid(row=1, column=c, columnspan=2 if index == 1 else 1, sticky="ew")
                boxes[-1].grid(row=2, column=0, columnspan=2, sticky="ew", pady=3)
            # Stack the IAS gain box above its readouts; other readouts already fill a row.
            for child in row2.winfo_children():
                child.pack_configure(side=tk.TOP, fill=tk.X, expand=False, padx=0)
            def compact(widget):
                if isinstance(widget, (tk.Frame, tk.LabelFrame)):
                    widget.configure(padx=2, pady=1)
                if isinstance(widget, tk.Scale):
                    widget.configure(length=65, width=10, sliderlength=15)
                if isinstance(widget, (tk.Label, tk.Checkbutton, tk.Button)):
                    widget.configure(font=("Segoe UI", 8, "bold"))
                children = widget.winfo_children()
                gridded = [w for w in children if w.winfo_manager() == "grid"]
                # Existing gain groups use label/slider/entry triples across nine columns.
                if len(gridded) == 9 and any(isinstance(w, tk.Scale) for w in gridded):
                    for w in gridded:
                        col = int(w.grid_info()["column"])
                        w.grid_configure(row=col//3, column=col%3, padx=2, pady=0)
                    widget.columnconfigure(1, weight=1)
                elif gridded and all(isinstance(w, tk.Label) for w in gridded):
                    for n, w in enumerate(children):
                        w.grid_configure(row=n//2, column=n%2, columnspan=1, sticky="w", padx=2)
                        w.configure(wraplength=180, justify=tk.LEFT)
                        if str(w.cget("text")).startswith("Hardware Drive:"):
                            w.grid_remove()
                for child in children:
                    compact(child)
            compact(controls)
            # Place polarity below target controls instead of extending their width.
            target_box = self.ias_box if index == 0 else boxes[0]
            for w in target_box.winfo_children():
                w.pack_forget()
            for n, w in enumerate(target_box.winfo_children()):
                if isinstance(w, tk.Checkbutton):
                    w.grid(row=1, column=0, columnspan=8, sticky="w")
                else:
                    w.grid(row=0, column=n, padx=1, sticky="ew")
            target_box.columnconfigure(0, weight=1)
            heading = tk.Label(panel, text=("IAS / PITCH", "BANK", "HEADING / YAW")[index],
                               bg="#1f6feb", fg="white", font=("Segoe UI", 11, "bold"), pady=5)
            heading.pack(side=tk.TOP, fill=tk.X, before=controls)
        self.root.update_idletasks()
        control_frames = [panel.winfo_children()[0] for panel in (self.tab_ias, self.tab_bank, self.tab_yaw)]
        control_height = max(frame.winfo_reqheight() for frame in control_frames)
        for frame in control_frames:
            frame.configure(height=control_height)
            frame.pack_propagate(False)
        titles = (("Pitch tracking", "Airspeed tracking", "Pitch motor"),
                  ("Bank tracking", "Roll motor"), ("Heading tracking", "Rudder / roll motors"))
        for figure, names in zip((self.fig1, self.fig2, self.fig3), titles):
            figure.set_size_inches(4, 4, forward=False)
            figure.subplots_adjust(left=.16, right=.97, bottom=.10, top=.94, hspace=.55)
            for axis, title in zip(figure.axes, names):
                axis.set_title(title, fontsize=8, pad=3)
                axis.tick_params(labelsize=7)
                axis.xaxis.label.set_size(7)
                axis.yaxis.label.set_size(7)
                legend = axis.get_legend()
                if legend:
                    for text in legend.get_texts():
                        label = text.get_text().replace("Brunner ", "").replace(" (P3D Telemetry)", "")
                        label = label.replace(" (Measured)", " meas").replace(" (Commanded)", " cmd")
                        text.set_text(label)
                        text.set_fontsize(6)

    # -------------------------------------------------------------------------
    # TAB 1: AIRSPEED & PITCH HOLD
    # -------------------------------------------------------------------------
    def _build_tab_ias(self, parent):
        ctrl_frame = tk.Frame(parent, bg=self.panel_bg, padx=12, pady=6)
        ctrl_frame.pack(side=tk.TOP, fill=tk.X, padx=4, pady=4)

        row1 = tk.Frame(ctrl_frame, bg=self.panel_bg)
        row1.pack(fill=tk.X, pady=(0, 4))

        # Guidance Mode
        strategy_box = tk.LabelFrame(row1, text="  Guidance Mode  ",
                                     font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#79c0ff",
                                     padx=8, pady=4)
        strategy_box.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))

        lbl_mode_fixed = tk.Label(strategy_box, text="AIRSPEED HOLD\n(Pitch-for-Speed)",
                                  font=('Segoe UI', 9, 'bold'), fg="#38bdf8", bg=self.panel_bg, justify=tk.LEFT)
        lbl_mode_fixed.pack(anchor=tk.W, padx=4, pady=2)

        # Target Airspeed
        self.ias_box = tk.LabelFrame(row1, text="  Target Airspeed (knots) [ACTIVE]  ",
                                     font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#38bdf8",
                                     padx=8, pady=4)
        self.ias_box.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))

        self.target_ias_var = tk.DoubleVar(value=80.0)
        self.target_ias_slider = tk.Scale(self.ias_box, from_=0.0, to=300.0, resolution=1.0,
                                          showvalue=False, orient=tk.HORIZONTAL, variable=self.target_ias_var,
                                          bg=self.panel_bg, fg=self.text_color, troughcolor="#161b22",
                                          highlightthickness=0, length=110, command=self.on_target_ias_change)
        self.target_ias_slider.pack(side=tk.LEFT, padx=3)

        self.target_ias_entry = tk.Entry(self.ias_box, width=5, font=('Segoe UI', 10, 'bold'),
                                         bg="#161b22", fg="#38bdf8", insertbackground="#ffffff", justify=tk.CENTER)
        self.target_ias_entry.insert(0, "80")
        self.target_ias_entry.pack(side=tk.LEFT, padx=4)
        self.target_ias_entry.bind("<KeyRelease>", self.on_target_ias_keyrelease)
        self.target_ias_entry.bind("<Return>", self.on_target_ias_entry)
        self.target_ias_entry.bind("<FocusOut>", self.on_target_ias_entry)

        for val in [65, 80, 90, 100]:
            btn = tk.Button(self.ias_box, text=f"{val}", font=('Segoe UI', 8, 'bold'),
                            bg="#30363d", fg=self.text_color, relief=tk.FLAT, padx=4, pady=1,
                            command=lambda v=val: self.set_target_ias(v))
            btn.pack(side=tk.LEFT, padx=1)

        # Actuator Channel (Brunner Pitch Motor)
        mode_box = tk.LabelFrame(row1, text="  Pitch Actuator  ",
                                 font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#79c0ff",
                                 padx=8, pady=4)
        mode_box.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))

        self.mode_var = tk.IntVar(value=3)
        rb4 = ttk.Radiobutton(mode_box, text="BRUNNER CLS-P (Pitch Motor)", value=3, variable=self.mode_var)
        rb4.pack(anchor=tk.W, padx=4, pady=2)

        lbl_hint = tk.Label(mode_box, text="P3D elevator writes 100% OFF", font=('Segoe UI', 8, 'italic'),
                            fg="#8b949e", bg=self.panel_bg)
        lbl_hint.pack(anchor=tk.W, padx=4, pady=(0, 2))

        # Polarity
        sign_box = tk.LabelFrame(row1, text="  Pitch Polarity  ",
                                 font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#79c0ff",
                                 padx=8, pady=4)
        sign_box.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))

        self.invert_var = tk.BooleanVar(value=False)
        inv_chk = ttk.Checkbutton(sign_box, text="Invert Pitch Sign (-/+)",
                                  variable=self.invert_var, command=self.on_invert_change)
        inv_chk.pack(anchor=tk.W, padx=4, pady=4)

        self.btn_pitch_toggle = tk.Button(row1, text="START AIRSPEED & PITCH HOLD",
                                          font=('Segoe UI', 10, 'bold'), bg="#238636", fg="#ffffff",
                                          activebackground="#2ea043", activeforeground="#ffffff",
                                          relief=tk.FLAT, padx=14, pady=6, cursor="hand2",
                                          command=self.toggle_pitch)
        self.btn_pitch_toggle.pack(side=tk.RIGHT, padx=4)

        # Row 2: Realtime PID Sliders & Live Telemetry Readout
        row2 = tk.Frame(ctrl_frame, bg=self.panel_bg)
        row2.pack(fill=tk.X, pady=(2, 0))

        # Realtime Airspeed PID Gains
        pid_box = tk.LabelFrame(row2, text="  Airspeed (IAS) PID Gains  ",
                                font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#79c0ff",
                                padx=8, pady=4)
        pid_box.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))

        tk.Label(pid_box, text="IAS Kp:", font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#38bdf8").grid(row=0, column=0, padx=3, sticky=tk.W)
        self.ias_kp_var = tk.DoubleVar(value=0.24)
        self.ias_kp_scale = tk.Scale(pid_box, from_=0.000, to=10.000, resolution=0.01, orient=tk.HORIZONTAL,
                                     showvalue=False, variable=self.ias_kp_var, bg=self.panel_bg, fg=self.text_color, troughcolor="#161b22",
                                     highlightthickness=0, length=80, command=self.on_ias_pid_change)
        self.ias_kp_scale.grid(row=0, column=1, padx=3)
        self.ias_kp_entry = tk.Entry(pid_box, width=6, font=('Segoe UI', 9, 'bold'),
                                     bg="#161b22", fg="#38bdf8", insertbackground="#ffffff", justify=tk.CENTER)
        self.ias_kp_entry.insert(0, "0.24")
        self.ias_kp_entry.grid(row=0, column=2, padx=3)
        self.ias_kp_entry.bind("<KeyRelease>", self.on_ias_kp_keyrelease)
        self.ias_kp_entry.bind("<Return>", self.on_ias_kp_entry)
        self.ias_kp_entry.bind("<FocusOut>", self.on_ias_kp_entry)

        tk.Label(pid_box, text="Ki:", font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#38bdf8").grid(row=0, column=3, padx=3, sticky=tk.W)
        self.ias_ki_var = tk.DoubleVar(value=0.0080)
        self.ias_ki_scale = tk.Scale(pid_box, from_=0.000, to=2.000, resolution=0.001, orient=tk.HORIZONTAL,
                                     showvalue=False, variable=self.ias_ki_var, bg=self.panel_bg, fg=self.text_color, troughcolor="#161b22",
                                     highlightthickness=0, length=80, command=self.on_ias_pid_change)
        self.ias_ki_scale.grid(row=0, column=4, padx=3)
        self.ias_ki_entry = tk.Entry(pid_box, width=6, font=('Segoe UI', 9, 'bold'),
                                     bg="#161b22", fg="#38bdf8", insertbackground="#ffffff", justify=tk.CENTER)
        self.ias_ki_entry.insert(0, "0.0080")
        self.ias_ki_entry.grid(row=0, column=5, padx=3)
        self.ias_ki_entry.bind("<KeyRelease>", self.on_ias_ki_keyrelease)
        self.ias_ki_entry.bind("<Return>", self.on_ias_ki_entry)
        self.ias_ki_entry.bind("<FocusOut>", self.on_ias_ki_entry)

        tk.Label(pid_box, text="Kd:", font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#38bdf8").grid(row=0, column=6, padx=3, sticky=tk.W)
        self.ias_kd_var = tk.DoubleVar(value=0.32)
        self.ias_kd_scale = tk.Scale(pid_box, from_=0.000, to=10.000, resolution=0.005, orient=tk.HORIZONTAL,
                                     showvalue=False, variable=self.ias_kd_var, bg=self.panel_bg, fg=self.text_color, troughcolor="#161b22",
                                     highlightthickness=0, length=80, command=self.on_ias_pid_change)
        self.ias_kd_scale.grid(row=0, column=7, padx=3)
        self.ias_kd_entry = tk.Entry(pid_box, width=6, font=('Segoe UI', 9, 'bold'),
                                     bg="#161b22", fg="#38bdf8", insertbackground="#ffffff", justify=tk.CENTER)
        self.ias_kd_entry.insert(0, "0.32")
        self.ias_kd_entry.grid(row=0, column=8, padx=3)
        self.ias_kd_entry.bind("<KeyRelease>", self.on_ias_kd_keyrelease)
        self.ias_kd_entry.bind("<Return>", self.on_ias_kd_entry)
        self.ias_kd_entry.bind("<FocusOut>", self.on_ias_kd_entry)

        # Digital Readouts
        data_box = tk.LabelFrame(row2, text="  Live Pitch & Speed Readouts  ",
                                 font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#79c0ff",
                                 padx=8, pady=4)
        data_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.lbl_pitch_act = tk.Label(data_box, text="Pitch: 0.00°", font=('Segoe UI', 9, 'bold'), fg="#39d353", bg=self.panel_bg)
        self.lbl_pitch_act.grid(row=0, column=0, padx=6, sticky=tk.W)

        self.lbl_pitch_err = tk.Label(data_box, text="Pitch Err: 0.00°", font=('Segoe UI', 9, 'bold'), fg="#f85149", bg=self.panel_bg)
        self.lbl_pitch_err.grid(row=0, column=1, padx=6, sticky=tk.W)

        self.lbl_throttle = tk.Label(data_box, text="Throttle: 0.0%", font=('Segoe UI', 9, 'bold'), fg="#e3b341", bg=self.panel_bg)
        self.lbl_throttle.grid(row=0, column=2, padx=6, sticky=tk.W)

        self.lbl_ias_act = tk.Label(data_box, text="IAS: 0.0 kt", font=('Segoe UI', 9, 'bold'), fg="#38bdf8", bg=self.panel_bg)
        self.lbl_ias_act.grid(row=1, column=0, padx=6, sticky=tk.W)

        self.lbl_ias_err = tk.Label(data_box, text="IAS Err: 0.0 kt", font=('Segoe UI', 9, 'bold'), fg="#ffa657", bg=self.panel_bg)
        self.lbl_ias_err.grid(row=1, column=1, padx=6, sticky=tk.W)

        self.lbl_cmd_pitch = tk.Label(data_box, text="Cmd Pitch: 0.00°", font=('Segoe UI', 9, 'bold'), fg="#d2a8ff", bg=self.panel_bg)
        self.lbl_cmd_pitch.grid(row=1, column=2, padx=6, sticky=tk.W)

        self.lbl_brunner_pitch = tk.Label(data_box, text="Brunner Pitch (Meas/Cmd): 0.00 / 0.00", font=('Segoe UI', 9, 'bold'), fg="#58a6ff", bg=self.panel_bg)
        self.lbl_brunner_pitch.grid(row=2, column=0, columnspan=2, padx=6, sticky=tk.W)

        self.lbl_fault = tk.Label(data_box, text="", font=('Segoe UI', 8, 'bold'), fg="#f85149", bg=self.panel_bg)
        self.lbl_fault.grid(row=2, column=2, padx=6, sticky=tk.W)

        # Tab 1 Live Graphs
        graph_frame = tk.Frame(parent, bg="#161b22")
        graph_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=4, pady=4)

        plt.style.use('dark_background')
        self.fig1, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, sharex=True, figsize=(10, 5.8), dpi=100)
        self.fig1.patch.set_facecolor('#161b22')

        # Subplot 1: Pitch
        self.ax1.set_facecolor('#0d1117')
        self.ax1.set_ylabel("Pitch (°)", fontsize=8, color="#39d353")
        self.ax1.tick_params(colors="#39d353")
        self.ax1.grid(True, linestyle="--", alpha=0.3, color="#30363d")
        self.ax1.set_ylim(-15.0, 15.0)
        self.line_actual, = self.ax1.plot([], [], label="Pitch Actual", color="#39d353", linewidth=1.8)
        self.line_target, = self.ax1.plot([], [], label="Pitch Target / Cmd", color="#e3b341", linestyle="--", linewidth=1.6)
        self.line_error, = self.ax1.plot([], [], label="Pitch Error", color="#f85149", linewidth=1.2)
        self.ax1.legend(loc="upper left", facecolor="#161b22", edgecolor="#30363d", fontsize=8)
        self.ax1.set_title("Pitch Attitude Tracking & Error (deg)", fontsize=9, fontweight="bold", color="#58a6ff", pad=2)

        # Subplot 2: IAS
        self.ax2.set_facecolor('#0d1117')
        self.ax2.set_ylabel("IAS (kt)", fontsize=8, color="#38bdf8")
        self.ax2.tick_params(colors="#38bdf8")
        self.ax2.grid(True, linestyle="--", alpha=0.3, color="#30363d")
        self.line_ias_act, = self.ax2.plot([], [], label="Actual Airspeed (IAS)", color="#38bdf8", linewidth=1.8)
        self.line_ias_tgt, = self.ax2.plot([], [], label="Target Airspeed", color="#c084fc", linestyle="--", linewidth=1.6)
        self.line_ias_err, = self.ax2.plot([], [], label="Airspeed Error", color="#ffa657", linewidth=1.2)
        self.ax2.legend(loc="upper left", facecolor="#161b22", edgecolor="#30363d", fontsize=8)
        self.ax2.set_title("Airspeed Tracking & Error (knots)", fontsize=9, fontweight="bold", color="#38bdf8", pad=2)

        # Subplot 3: Brunner Pitch Stick
        self.ax3.set_facecolor('#0d1117')
        self.ax3.set_xlabel("Time (seconds)", fontsize=8, color="#c9d1d9")
        self.ax3.set_ylabel("Stick Pitch [-1..1]", fontsize=8, color="#ffa657")
        self.ax3.grid(True, linestyle="--", alpha=0.3, color="#30363d")
        self.line_brunner, = self.ax3.plot([], [], label="Brunner Pitch (Measured)", color="#ffa657", linewidth=1.8)
        self.line_brunner_cmd, = self.ax3.plot([], [], label="Brunner Pitch (Commanded)", color="#38bdf8", linestyle="--", linewidth=1.4)
        self.ax3.legend(loc="upper left", facecolor="#161b22", edgecolor="#30363d", fontsize=8)
        self.ax3.set_title("Brunner CLS-P Pitch Stick Motor Drive", fontsize=9, fontweight="bold", color="#ffa657", pad=2)

        self.fig1.tight_layout()
        self.canvas1 = FigureCanvasTkAgg(self.fig1, master=graph_frame)
        self.canvas1.draw()
        self.canvas1.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # -------------------------------------------------------------------------
    # TAB 2: BANK ANGLE HOLD (WINGS LEVEL = 0.0° VIA BRUNNER CLS-P ROLL MOTOR)
    # -------------------------------------------------------------------------
    def _build_tab_bank(self, parent):
        ctrl_frame = tk.Frame(parent, bg=self.panel_bg, padx=14, pady=10)
        ctrl_frame.pack(side=tk.TOP, fill=tk.X, padx=4, pady=4)

        row1 = tk.Frame(ctrl_frame, bg=self.panel_bg)
        row1.pack(fill=tk.X, pady=(0, 6))

        # Target Bank Angle (0.00° Wings Level)
        bank_target_box = tk.LabelFrame(row1, text="  Target Bank Angle (deg)  ",
                                        font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#38bdf8",
                                        padx=12, pady=6)
        bank_target_box.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        self.target_bank_var = tk.DoubleVar(value=0.0)
        self.target_bank_slider = tk.Scale(bank_target_box, from_=-45.0, to=45.0, resolution=0.5,
                                           showvalue=False, orient=tk.HORIZONTAL, variable=self.target_bank_var,
                                           bg=self.panel_bg, fg=self.text_color, troughcolor="#161b22",
                                           highlightthickness=0, length=140, command=self.on_target_bank_change)
        self.target_bank_slider.pack(side=tk.LEFT, padx=4)

        self.target_bank_entry = tk.Entry(bank_target_box, width=6, font=('Segoe UI', 10, 'bold'),
                                          bg="#161b22", fg="#38bdf8", insertbackground="#ffffff", justify=tk.CENTER)
        self.target_bank_entry.insert(0, "0.0")
        self.target_bank_entry.pack(side=tk.LEFT, padx=6)
        self.target_bank_entry.bind("<KeyRelease>", self.on_target_bank_keyrelease)
        self.target_bank_entry.bind("<Return>", self.on_target_bank_entry)
        self.target_bank_entry.bind("<FocusOut>", self.on_target_bank_entry)

        btn_level = tk.Button(bank_target_box, text="0.00° Level", font=('Segoe UI', 8, 'bold'),
                              bg="#238636", fg="#ffffff", relief=tk.FLAT, padx=8, pady=2,
                              command=lambda: self.set_target_bank(0.0))
        btn_level.pack(side=tk.LEFT, padx=4)

        self.chk_bank_inv = tk.Checkbutton(bank_target_box, text="Invert Motor Polarity",
                                           variable=self.bank_invert_var, command=self.on_bank_invert_change,
                                           bg=self.panel_bg, fg="#e3b341", selectcolor="#161b22",
                                           activebackground=self.panel_bg, activeforeground="#ffffff",
                                           font=('Segoe UI', 9, 'bold'))
        self.chk_bank_inv.pack(side=tk.LEFT, padx=8)

        # Realtime Bank PID Gains (Brunner Roll Motor Loop)
        bank_pid_box = tk.LabelFrame(row1, text="  Realtime Bank PID Gains (Brunner Roll Motor)  ",
                                     font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#79c0ff",
                                     padx=10, pady=6)
        bank_pid_box.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        tk.Label(bank_pid_box, text="Bank Kp:", font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#38bdf8").grid(row=0, column=0, padx=4, sticky=tk.W)
        self.bank_kp_var = tk.DoubleVar(value=0.015)
        self.bank_kp_scale = tk.Scale(bank_pid_box, from_=0.000, to=10.000, resolution=0.001, orient=tk.HORIZONTAL,
                                      showvalue=False, variable=self.bank_kp_var, bg=self.panel_bg, fg=self.text_color, troughcolor="#161b22",
                                      highlightthickness=0, length=90, command=self.on_bank_pid_change)
        self.bank_kp_scale.grid(row=0, column=1, padx=4)
        self.bank_kp_entry = tk.Entry(bank_pid_box, width=6, font=('Segoe UI', 9, 'bold'),
                                      bg="#161b22", fg="#38bdf8", insertbackground="#ffffff", justify=tk.CENTER)
        self.bank_kp_entry.insert(0, "0.015")
        self.bank_kp_entry.grid(row=0, column=2, padx=4)
        self.bank_kp_entry.bind("<KeyRelease>", self.on_bank_kp_keyrelease)
        self.bank_kp_entry.bind("<Return>", self.on_bank_kp_entry)
        self.bank_kp_entry.bind("<FocusOut>", self.on_bank_kp_entry)

        tk.Label(bank_pid_box, text="Ki:", font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#38bdf8").grid(row=0, column=3, padx=4, sticky=tk.W)
        self.bank_ki_var = tk.DoubleVar(value=0.0010)
        self.bank_ki_scale = tk.Scale(bank_pid_box, from_=0.000, to=2.000, resolution=0.001, orient=tk.HORIZONTAL,
                                      showvalue=False, variable=self.bank_ki_var, bg=self.panel_bg, fg=self.text_color, troughcolor="#161b22",
                                      highlightthickness=0, length=90, command=self.on_bank_pid_change)
        self.bank_ki_scale.grid(row=0, column=4, padx=4)
        self.bank_ki_entry = tk.Entry(bank_pid_box, width=6, font=('Segoe UI', 9, 'bold'),
                                      bg="#161b22", fg="#38bdf8", insertbackground="#ffffff", justify=tk.CENTER)
        self.bank_ki_entry.insert(0, "0.0010")
        self.bank_ki_entry.grid(row=0, column=5, padx=4)
        self.bank_ki_entry.bind("<KeyRelease>", self.on_bank_ki_keyrelease)
        self.bank_ki_entry.bind("<Return>", self.on_bank_ki_entry)
        self.bank_ki_entry.bind("<FocusOut>", self.on_bank_ki_entry)

        tk.Label(bank_pid_box, text="Kd:", font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#38bdf8").grid(row=0, column=6, padx=4, sticky=tk.W)
        self.bank_kd_var = tk.DoubleVar(value=0.020)
        self.bank_kd_scale = tk.Scale(bank_pid_box, from_=0.000, to=10.000, resolution=0.001, orient=tk.HORIZONTAL,
                                      showvalue=False, variable=self.bank_kd_var, bg=self.panel_bg, fg=self.text_color, troughcolor="#161b22",
                                      highlightthickness=0, length=90, command=self.on_bank_pid_change)
        self.bank_kd_scale.grid(row=0, column=7, padx=4)
        self.bank_kd_entry = tk.Entry(bank_pid_box, width=6, font=('Segoe UI', 9, 'bold'),
                                      bg="#161b22", fg="#38bdf8", insertbackground="#ffffff", justify=tk.CENTER)
        self.bank_kd_entry.insert(0, "0.020")
        self.bank_kd_entry.grid(row=0, column=8, padx=4)
        self.bank_kd_entry.bind("<KeyRelease>", self.on_bank_kd_keyrelease)
        self.bank_kd_entry.bind("<Return>", self.on_bank_kd_entry)
        self.bank_kd_entry.bind("<FocusOut>", self.on_bank_kd_entry)

        self.btn_bank_toggle = tk.Button(row1, text="START BANK HOLD (0.0°)",
                                         font=('Segoe UI', 10, 'bold'), bg="#238636", fg="#ffffff",
                                         activebackground="#2ea043", activeforeground="#ffffff",
                                         relief=tk.FLAT, padx=14, pady=6, cursor="hand2",
                                         command=self.toggle_bank)
        self.btn_bank_toggle.pack(side=tk.RIGHT, padx=4)

        # Row 2: Live Digital Readouts for Bank Attitude
        row2 = tk.Frame(ctrl_frame, bg=self.panel_bg)
        row2.pack(fill=tk.X, pady=(4, 0))

        bank_data_box = tk.LabelFrame(row2, text="  Live Bank Attitude Tracking  ",
                                      font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#79c0ff",
                                      padx=12, pady=6)
        bank_data_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.lbl_bank_act = tk.Label(bank_data_box, text="Bank: 0.00°", font=('Segoe UI', 11, 'bold'), fg="#38bdf8", bg=self.panel_bg)
        self.lbl_bank_act.grid(row=0, column=0, padx=12, sticky=tk.W)

        self.lbl_bank_err = tk.Label(bank_data_box, text="Bank Err: 0.00°", font=('Segoe UI', 11, 'bold'), fg="#f85149", bg=self.panel_bg)
        self.lbl_bank_err.grid(row=0, column=1, padx=12, sticky=tk.W)

        self.lbl_bank_target_info = tk.Label(bank_data_box, text="Target: 0.00° (Wings Level)", font=('Segoe UI', 10, 'bold'), fg="#e3b341", bg=self.panel_bg)
        self.lbl_bank_target_info.grid(row=0, column=2, padx=12, sticky=tk.W)

        lbl_p3d_status = tk.Label(bank_data_box, text="Hardware Drive: Pure Brunner CLS-P (P3D Writes: NONE)", font=('Segoe UI', 9, 'italic'), fg="#7ee787", bg=self.panel_bg)
        lbl_p3d_status.grid(row=0, column=3, padx=12, sticky=tk.E)

        # Tab 2 Live Graphs (Bank Angle Tracking & Brunner Roll Motor Drive)
        bank_graph_frame = tk.Frame(parent, bg="#161b22")
        bank_graph_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=4, pady=4)

        self.fig2, (self.bank_ax1, self.bank_ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 5.8), dpi=100)
        self.fig2.patch.set_facecolor('#161b22')

        # Subplot 1: Bank Angle Tracking (deg)
        self.bank_ax1.set_facecolor('#0d1117')
        self.bank_ax1.set_ylabel("Bank (°)", fontsize=9, color="#38bdf8")
        self.bank_ax1.tick_params(colors="#38bdf8")
        self.bank_ax1.grid(True, linestyle="--", alpha=0.3, color="#30363d")
        self.bank_ax1.set_ylim(-15.0, 15.0)
        self.line_bank_act, = self.bank_ax1.plot([], [], label="Actual Bank Angle (P3D Telemetry)", color="#38bdf8", linewidth=1.8)
        self.line_bank_tgt, = self.bank_ax1.plot([], [], label="Target Bank (0.00° Wings Level)", color="#e3b341", linestyle="--", linewidth=1.6)
        self.line_bank_err, = self.bank_ax1.plot([], [], label="Bank Error", color="#f85149", linewidth=1.2)
        self.bank_ax1.legend(loc="upper left", facecolor="#161b22", edgecolor="#30363d", fontsize=8)
        self.bank_ax1.set_title("Bank Angle Tracking & Error (deg) — Closed-Loop Wings-Level Hold", fontsize=9, fontweight="bold", color="#38bdf8", pad=2)

        # Subplot 2: Brunner Roll Motor Channel
        self.bank_ax2.set_facecolor('#0d1117')
        self.bank_ax2.set_xlabel("Time (seconds)", fontsize=9, color="#c9d1d9")
        self.bank_ax2.set_ylabel("Stick Roll [-1..1]", fontsize=9, color="#ffa657")
        self.bank_ax2.tick_params(colors="#ffa657")
        self.bank_ax2.grid(True, linestyle="--", alpha=0.3, color="#30363d")
        self.bank_ax2.set_ylim(-0.85, 0.85)
        self.line_b_roll_meas, = self.bank_ax2.plot([], [], label="Brunner Roll Stick (Measured)", color="#ffa657", linewidth=1.8)
        self.line_b_roll_cmd, = self.bank_ax2.plot([], [], label="Brunner Roll Motor (Commanded)", color="#38bdf8", linestyle="--", linewidth=1.6)
        self.bank_ax2.legend(loc="upper left", facecolor="#161b22", edgecolor="#30363d", fontsize=8)
        self.bank_ax2.set_title("Brunner CLS-P Roll Motor Drive Channel", fontsize=9, fontweight="bold", color="#ffa657", pad=2)

        self.fig2.tight_layout()
        self.canvas2 = FigureCanvasTkAgg(self.fig2, master=bank_graph_frame)
        self.canvas2.draw()
        self.canvas2.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # -------------------------------------------------------------------------
    # TAB 3: HEADING & YAW HOLD (COORDINATED FLIGHT VIA BRUNNER CLS-P RUDDER)
    # -------------------------------------------------------------------------
    def _build_tab_yaw(self, parent):
        ctrl_frame = tk.Frame(parent, bg=self.panel_bg, padx=14, pady=10)
        ctrl_frame.pack(side=tk.TOP, fill=tk.X, padx=4, pady=4)

        row1 = tk.Frame(ctrl_frame, bg=self.panel_bg)
        row1.pack(fill=tk.X, pady=(0, 6))

        # Target Heading Box
        hdg_target_box = tk.LabelFrame(row1, text="  Target Heading (deg)  ",
                                       font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#38bdf8",
                                       padx=12, pady=6)
        hdg_target_box.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        self.target_hdg_var = tk.DoubleVar(value=360.0)
        self.target_hdg_slider = tk.Scale(hdg_target_box, from_=0.0, to=500.0, resolution=1.0,
                                          showvalue=False, orient=tk.HORIZONTAL, variable=self.target_hdg_var,
                                          bg=self.panel_bg, fg=self.text_color, troughcolor="#161b22",
                                          highlightthickness=0, length=140, command=self.on_target_hdg_change)
        self.target_hdg_slider.pack(side=tk.LEFT, padx=4)

        self.target_hdg_entry = tk.Entry(hdg_target_box, width=6, font=('Segoe UI', 10, 'bold'),
                                         bg="#161b22", fg="#38bdf8", insertbackground="#ffffff", justify=tk.CENTER)
        self.target_hdg_entry.insert(0, "360")
        self.target_hdg_entry.pack(side=tk.LEFT, padx=6)
        self.target_hdg_entry.bind("<KeyRelease>", self.on_target_hdg_keyrelease)
        self.target_hdg_entry.bind("<Return>", self.on_target_hdg_entry)
        self.target_hdg_entry.bind("<FocusOut>", self.on_target_hdg_entry)

        for name, val in [("N", 360), ("E", 90), ("S", 180), ("W", 270)]:
            btn = tk.Button(hdg_target_box, text=f"{name}", font=('Segoe UI', 8, 'bold'),
                            bg="#30363d", fg=self.text_color, relief=tk.FLAT, padx=6, pady=2,
                            command=lambda v=val: self.set_target_hdg(v))
            btn.pack(side=tk.LEFT, padx=2)

        self.chk_hdg_inv = tk.Checkbutton(hdg_target_box, text="Invert Turn Direction",
                                          variable=self.heading_invert_var, command=self.on_heading_invert_change,
                                          bg=self.panel_bg, fg="#e3b341", selectcolor="#161b22",
                                          activebackground=self.panel_bg, activeforeground="#ffffff",
                                          font=('Segoe UI', 9, 'bold'))
        self.chk_hdg_inv.pack(side=tk.LEFT, padx=8)

        # Realtime Heading PID Gains (Steering & Heading Hold Loop)
        hdg_pid_box = tk.LabelFrame(row1, text="  Heading PID (Outer Loop)  ",
                                    font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#79c0ff",
                                    padx=8, pady=6)
        hdg_pid_box.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 6))

        tk.Label(hdg_pid_box, text="Hdg Kp:", font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#38bdf8").grid(row=0, column=0, padx=2, sticky=tk.W)
        self.hdg_kp_scale = tk.Scale(hdg_pid_box, from_=0.000, to=50.000, resolution=0.01, orient=tk.HORIZONTAL,
                                     showvalue=False, variable=self.hdg_kp_var, bg=self.panel_bg, fg=self.text_color, troughcolor="#161b22",
                                     highlightthickness=0, length=70, command=self.on_hdg_pid_change)
        self.hdg_kp_scale.grid(row=0, column=1, padx=2)
        self.hdg_kp_entry = tk.Entry(hdg_pid_box, width=5, font=('Segoe UI', 9, 'bold'),
                                     bg="#161b22", fg="#38bdf8", insertbackground="#ffffff", justify=tk.CENTER)
        self.hdg_kp_entry.insert(0, "0.80")
        self.hdg_kp_entry.grid(row=0, column=2, padx=2)
        self.hdg_kp_entry.bind("<KeyRelease>", self.on_hdg_kp_keyrelease)
        self.hdg_kp_entry.bind("<Return>", self.on_hdg_kp_entry)
        self.hdg_kp_entry.bind("<FocusOut>", self.on_hdg_kp_entry)

        tk.Label(hdg_pid_box, text="Ki:", font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#38bdf8").grid(row=0, column=3, padx=2, sticky=tk.W)
        self.hdg_ki_scale = tk.Scale(hdg_pid_box, from_=0.000, to=10.000, resolution=0.001, orient=tk.HORIZONTAL,
                                     showvalue=False, variable=self.hdg_ki_var, bg=self.panel_bg, fg=self.text_color, troughcolor="#161b22",
                                     highlightthickness=0, length=70, command=self.on_hdg_pid_change)
        self.hdg_ki_scale.grid(row=0, column=4, padx=2)
        self.hdg_ki_entry = tk.Entry(hdg_pid_box, width=5, font=('Segoe UI', 9, 'bold'),
                                     bg="#161b22", fg="#38bdf8", insertbackground="#ffffff", justify=tk.CENTER)
        self.hdg_ki_entry.insert(0, "0.005")
        self.hdg_ki_entry.grid(row=0, column=5, padx=2)
        self.hdg_ki_entry.bind("<KeyRelease>", self.on_hdg_ki_keyrelease)
        self.hdg_ki_entry.bind("<Return>", self.on_hdg_ki_entry)
        self.hdg_ki_entry.bind("<FocusOut>", self.on_hdg_ki_entry)

        tk.Label(hdg_pid_box, text="Kd:", font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#38bdf8").grid(row=0, column=6, padx=2, sticky=tk.W)
        self.hdg_kd_scale = tk.Scale(hdg_pid_box, from_=0.000, to=20.000, resolution=0.01, orient=tk.HORIZONTAL,
                                     showvalue=False, variable=self.hdg_kd_var, bg=self.panel_bg, fg=self.text_color, troughcolor="#161b22",
                                     highlightthickness=0, length=70, command=self.on_hdg_pid_change)
        self.hdg_kd_scale.grid(row=0, column=7, padx=2)
        self.hdg_kd_entry = tk.Entry(hdg_pid_box, width=5, font=('Segoe UI', 9, 'bold'),
                                     bg="#161b22", fg="#38bdf8", insertbackground="#ffffff", justify=tk.CENTER)
        self.hdg_kd_entry.insert(0, "0.70")
        self.hdg_kd_entry.grid(row=0, column=8, padx=2)
        self.hdg_kd_entry.bind("<KeyRelease>", self.on_hdg_kd_keyrelease)
        self.hdg_kd_entry.bind("<Return>", self.on_hdg_kd_entry)
        self.hdg_kd_entry.bind("<FocusOut>", self.on_hdg_kd_entry)

        # Realtime Yaw / Rudder PID Gains (Coordination & Damper)
        yaw_pid_box = tk.LabelFrame(row1, text="  Yaw / Rudder PID (Inner Loop)  ",
                                    font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#a371f7",
                                    padx=8, pady=6)
        yaw_pid_box.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 6))

        tk.Label(yaw_pid_box, text="Yaw Kp:", font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#a371f7").grid(row=0, column=0, padx=2, sticky=tk.W)
        self.yaw_kp_scale = tk.Scale(yaw_pid_box, from_=0.000, to=2.000, resolution=0.001, orient=tk.HORIZONTAL,
                                     showvalue=False, variable=self.yaw_kp_var, bg=self.panel_bg, fg=self.text_color, troughcolor="#161b22",
                                     highlightthickness=0, length=65, command=self.on_yaw_pid_change)
        self.yaw_kp_scale.grid(row=0, column=1, padx=2)
        self.yaw_kp_entry = tk.Entry(yaw_pid_box, width=5, font=('Segoe UI', 9, 'bold'),
                                     bg="#161b22", fg="#a371f7", insertbackground="#ffffff", justify=tk.CENTER)
        self.yaw_kp_entry.insert(0, "0.025")
        self.yaw_kp_entry.grid(row=0, column=2, padx=2)
        self.yaw_kp_entry.bind("<KeyRelease>", self.on_yaw_kp_keyrelease)
        self.yaw_kp_entry.bind("<Return>", self.on_yaw_kp_entry)
        self.yaw_kp_entry.bind("<FocusOut>", self.on_yaw_kp_entry)

        tk.Label(yaw_pid_box, text="Ki:", font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#a371f7").grid(row=0, column=3, padx=2, sticky=tk.W)
        self.yaw_ki_scale = tk.Scale(yaw_pid_box, from_=0.000, to=0.500, resolution=0.0005, orient=tk.HORIZONTAL,
                                     showvalue=False, variable=self.yaw_ki_var, bg=self.panel_bg, fg=self.text_color, troughcolor="#161b22",
                                     highlightthickness=0, length=65, command=self.on_yaw_pid_change)
        self.yaw_ki_scale.grid(row=0, column=4, padx=2)
        self.yaw_ki_entry = tk.Entry(yaw_pid_box, width=5, font=('Segoe UI', 9, 'bold'),
                                     bg="#161b22", fg="#a371f7", insertbackground="#ffffff", justify=tk.CENTER)
        self.yaw_ki_entry.insert(0, "0.0020")
        self.yaw_ki_entry.grid(row=0, column=5, padx=2)
        self.yaw_ki_entry.bind("<KeyRelease>", self.on_yaw_ki_keyrelease)
        self.yaw_ki_entry.bind("<Return>", self.on_yaw_ki_entry)
        self.yaw_ki_entry.bind("<FocusOut>", self.on_yaw_ki_entry)

        tk.Label(yaw_pid_box, text="Kd:", font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#a371f7").grid(row=0, column=6, padx=2, sticky=tk.W)
        self.yaw_kd_scale = tk.Scale(yaw_pid_box, from_=0.000, to=1.000, resolution=0.001, orient=tk.HORIZONTAL,
                                     showvalue=False, variable=self.yaw_kd_var, bg=self.panel_bg, fg=self.text_color, troughcolor="#161b22",
                                     highlightthickness=0, length=65, command=self.on_yaw_pid_change)
        self.yaw_kd_scale.grid(row=0, column=7, padx=2)
        self.yaw_kd_entry = tk.Entry(yaw_pid_box, width=5, font=('Segoe UI', 9, 'bold'),
                                     bg="#161b22", fg="#a371f7", insertbackground="#ffffff", justify=tk.CENTER)
        self.yaw_kd_entry.insert(0, "0.035")
        self.yaw_kd_entry.grid(row=0, column=8, padx=2)
        self.yaw_kd_entry.bind("<KeyRelease>", self.on_yaw_kd_keyrelease)
        self.yaw_kd_entry.bind("<Return>", self.on_yaw_kd_entry)
        self.yaw_kd_entry.bind("<FocusOut>", self.on_yaw_kd_entry)

        # Tab 3 Independent Start/Stop Toggle Button
        self.btn_yaw_toggle = tk.Button(row1, text="START HEADING & YAW HOLD",
                                        font=('Segoe UI', 10, 'bold'), bg="#238636", fg="#ffffff",
                                        activebackground="#2ea043", activeforeground="#ffffff",
                                        relief=tk.FLAT, padx=14, pady=6, cursor="hand2",
                                        command=self.toggle_yaw)
        self.btn_yaw_toggle.pack(side=tk.RIGHT, padx=4)

        # Row 2: Live Digital Readouts for Heading Attitude
        row2 = tk.Frame(ctrl_frame, bg=self.panel_bg)
        row2.pack(fill=tk.X, pady=(4, 0))

        yaw_data_box = tk.LabelFrame(row2, text="  Live Heading Attitude Tracking  ",
                                     font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#79c0ff",
                                     padx=12, pady=6)
        yaw_data_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.lbl_hdg_act = tk.Label(yaw_data_box, text="Heading: 360.0°", font=('Segoe UI', 11, 'bold'), fg="#38bdf8", bg=self.panel_bg)
        self.lbl_hdg_act.grid(row=0, column=0, padx=10, sticky=tk.W)

        self.lbl_hdg_err = tk.Label(yaw_data_box, text="Heading Err: 0.0°", font=('Segoe UI', 11, 'bold'), fg="#f85149", bg=self.panel_bg)
        self.lbl_hdg_err.grid(row=0, column=1, padx=10, sticky=tk.W)

        self.lbl_hdg_target_info = tk.Label(yaw_data_box, text="Target: 360.0°", font=('Segoe UI', 10, 'bold'), fg="#e3b341", bg=self.panel_bg)
        self.lbl_hdg_target_info.grid(row=0, column=2, padx=10, sticky=tk.W)

        self.lbl_hdg_bank_info = tk.Label(yaw_data_box, text="Demanded Bank: 0.00°", font=('Segoe UI', 10, 'bold'), fg="#ffa657", bg=self.panel_bg)
        self.lbl_hdg_bank_info.grid(row=0, column=3, padx=10, sticky=tk.W)

        lbl_p3d_status = tk.Label(yaw_data_box, text="Hardware Drive: Pure Brunner CLS-P (P3D Writes: NONE)", font=('Segoe UI', 9, 'italic'), fg="#7ee787", bg=self.panel_bg)
        lbl_p3d_status.grid(row=0, column=4, padx=10, sticky=tk.E)

        # Tab 3 Live Graphs (Heading Tracking & Brunner Rudder Loader Drive Channel)
        yaw_graph_frame = tk.Frame(parent, bg="#161b22")
        yaw_graph_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=4, pady=4)

        self.fig3, (self.yaw_ax1, self.yaw_ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 5.8), dpi=100)
        self.fig3.patch.set_facecolor('#161b22')

        # Subplot 1: Heading Tracking (deg)
        self.yaw_ax1.set_facecolor('#0d1117')
        self.yaw_ax1.set_ylabel("Heading (°)", fontsize=9, color="#38bdf8")
        self.yaw_ax1.tick_params(colors="#38bdf8")
        self.yaw_ax1.grid(True, linestyle="--", alpha=0.3, color="#30363d")
        self.yaw_ax1.set_ylim(-5.0, 365.0)
        self.line_hdg_act, = self.yaw_ax1.plot([], [], label="Actual Heading (P3D)", color="#38bdf8", linewidth=1.8)
        self.line_hdg_tgt, = self.yaw_ax1.plot([], [], label="Target Heading", color="#e3b341", linestyle="--", linewidth=1.6)
        self.line_hdg_err, = self.yaw_ax1.plot([], [], label="Heading Error", color="#f85149", linewidth=1.2)
        self.yaw_ax1.legend(loc="upper left", facecolor="#161b22", edgecolor="#30363d", fontsize=8)
        self.yaw_ax1.set_title("Heading Angle Tracking & Error (deg)", fontsize=9, fontweight="bold", color="#38bdf8", pad=2)

        # Subplot 2: Brunner CLS-P Rudder & Roll Drive Channels (Measured vs Commanded)
        self.yaw_ax2.set_facecolor('#0d1117')
        self.yaw_ax2.set_xlabel("Time (seconds)", fontsize=9, color="#c9d1d9")
        self.yaw_ax2.set_ylabel("Motor / Stick [-1..1]", fontsize=9, color="#ffa657")
        self.yaw_ax2.tick_params(colors="#ffa657")
        self.yaw_ax2.grid(True, linestyle="--", alpha=0.3, color="#30363d")
        self.yaw_ax2.set_ylim(-0.75, 0.75)
        self.line_b_yaw_meas, = self.yaw_ax2.plot([], [], label="Brunner Rudder Stick (Measured)", color="#ffa657", linewidth=1.8)
        self.line_b_yaw_cmd, = self.yaw_ax2.plot([], [], label="Brunner Rudder (Commanded)", color="#38bdf8", linestyle="--", linewidth=1.6)
        self.line_b_roll_cmd_tab3, = self.yaw_ax2.plot([], [], label="Brunner Roll Motor (Commanded)", color="#a371f7", linestyle=":", linewidth=1.4)
        self.line_bank_track_act = self.line_b_yaw_meas
        self.line_bank_track_cmd = self.line_b_yaw_cmd
        self.yaw_ax2.axhline(0.0, color="#6e7681", linestyle=":", linewidth=1.0)
        self.yaw_ax2.legend(loc="upper left", facecolor="#161b22", edgecolor="#30363d", fontsize=8)
        self.yaw_ax2.set_title("Brunner CLS-P Hardware Drive Channel (Rudder & Roll Actuation)", fontsize=9, fontweight="bold", color="#ffa657", pad=2)

        self.fig3.tight_layout()
        self.canvas3 = FigureCanvasTkAgg(self.fig3, master=yaw_graph_frame)
        self.canvas3.draw()
        self.canvas3.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # -------------------------------------------------------------------------
    # TAB 4: BRUNNER CLS-P HARDWARE & PROTOCOL DIAGNOSTICS (READ-ONLY)
    # -------------------------------------------------------------------------
    def _get_base_dir(self):
        return os.path.dirname(sys.executable if getattr(sys, 'frozen', False) else os.path.abspath(__file__))

    def _build_tab_diagnostics(self, parent):
        main_frame = tk.Frame(parent, bg=self.bg_color, padx=10, pady=8)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Top Control / Action Bar
        top_frame = tk.Frame(main_frame, bg=self.panel_bg, padx=12, pady=8)
        top_frame.pack(fill=tk.X, pady=(0, 6))

        tk.Label(top_frame, text="🔍 BRUNNER CLS-P HARDWARE & PROTOCOL DIAGNOSTICS",
                 font=('Segoe UI', 11, 'bold'), bg=self.panel_bg, fg="#58a6ff").pack(side=tk.LEFT)

        tk.Label(top_frame, text=" (STRICTLY READ-ONLY) ",
                 font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#7ee787").pack(side=tk.LEFT, padx=6)

        self.btn_export_diag_report = tk.Button(
            top_frame, text="📄 Export Diagnostic Report",
            font=('Segoe UI', 9, 'bold'), bg="#21262d", fg="#58a6ff",
            activebackground="#30363d", activeforeground="#ffffff",
            relief=tk.FLAT, padx=10, pady=4, cursor="hand2",
            command=self.export_diag_report
        )
        self.btn_export_diag_report.pack(side=tk.RIGHT, padx=4)

        self.btn_export_diag_csv = tk.Button(
            top_frame, text="📊 Export Diagnostic CSV",
            font=('Segoe UI', 9, 'bold'), bg="#21262d", fg="#3fb950",
            activebackground="#30363d", activeforeground="#ffffff",
            relief=tk.FLAT, padx=10, pady=4, cursor="hand2",
            command=self.export_diag_csv
        )
        self.btn_export_diag_csv.pack(side=tk.RIGHT, padx=4)

        self.btn_clear_diag_log = tk.Button(
            top_frame, text="🔄 Clear Log",
            font=('Segoe UI', 9, 'bold'), bg="#21262d", fg="#e3b341",
            activebackground="#30363d", activeforeground="#ffffff",
            relief=tk.FLAT, padx=8, pady=4, cursor="hand2",
            command=self.clear_diag_log
        )
        self.btn_clear_diag_log.pack(side=tk.RIGHT, padx=4)

        # Status & Override Ownership Banner
        status_banner = tk.LabelFrame(main_frame, text="  CLS2Sim Remote Interface & Hardware Override Ownership  ",
                                      font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#79c0ff",
                                      padx=12, pady=6)
        status_banner.pack(fill=tk.X, pady=(0, 6))

        caps_row = tk.Frame(status_banner, bg=self.panel_bg)
        caps_row.pack(fill=tk.X, pady=(0, 4))

        self.lbl_diag_udp_target = tk.Label(caps_row, text=f"Target: {HOST}:{PORT} (UDP)",
                                            font=('Segoe UI', 9, 'bold'), bg="#161b22", fg="#79c0ff",
                                            padx=8, pady=3)
        self.lbl_diag_udp_target.pack(side=tk.LEFT, padx=(0, 6))

        self.lbl_diag_ovr_pitch = tk.Label(caps_row, text="Pitch Override: MANUAL",
                                           font=('Segoe UI', 9, 'bold'), bg="#21262d", fg="#8b949e",
                                           padx=8, pady=3)
        self.lbl_diag_ovr_pitch.pack(side=tk.LEFT, padx=4)

        self.lbl_diag_ovr_roll = tk.Label(caps_row, text="Roll Override: MANUAL",
                                          font=('Segoe UI', 9, 'bold'), bg="#21262d", fg="#8b949e",
                                          padx=8, pady=3)
        self.lbl_diag_ovr_roll.pack(side=tk.LEFT, padx=4)

        self.lbl_diag_ovr_yaw = tk.Label(caps_row, text="Rudder Override: MANUAL",
                                         font=('Segoe UI', 9, 'bold'), bg="#21262d", fg="#8b949e",
                                         padx=8, pady=3)
        self.lbl_diag_ovr_yaw.pack(side=tk.LEFT, padx=4)

        self.lbl_diag_watchdog = tk.Label(caps_row, text="Watchdog: IDLE",
                                          font=('Segoe UI', 9, 'bold'), bg="#21262d", fg="#8b949e",
                                          padx=8, pady=3)
        self.lbl_diag_watchdog.pack(side=tk.LEFT, padx=4)

        self.lbl_diag_overall_badge = tk.Label(caps_row, text="LINK: CHECKING...",
                                               font=('Segoe UI', 9, 'bold'), bg="#30363d", fg="#ffffff",
                                               padx=10, pady=3)
        self.lbl_diag_overall_badge.pack(side=tk.RIGHT, padx=4)

        advisory_box = tk.Frame(status_banner, bg="#161b22", padx=8, pady=4)
        advisory_box.pack(fill=tk.X, pady=(2, 0))
        tk.Label(advisory_box,
                 text="⚠️ ADVISORY: UDP protocol ACK confirms network packet receipt by CLS2Sim software. "
                      "Physical motor movement additionally requires CLS2Sim CAN bus active, motor drive power ON, and hardware safety interlocks closed.",
                 font=('Segoe UI', 8, 'italic'), bg="#161b22", fg="#e3b341", justify=tk.LEFT).pack(anchor=tk.W)

        # Middle Area: 3 Diagnostic Cards for Pitch, Roll, Rudder
        cards_frame = tk.Frame(main_frame, bg=self.bg_color)
        cards_frame.pack(fill=tk.X, pady=(0, 6))
        cards_frame.columnconfigure(0, weight=1, uniform="diag_col")
        cards_frame.columnconfigure(1, weight=1, uniform="diag_col")
        cards_frame.columnconfigure(2, weight=1, uniform="diag_col")

        self.diag_card_widgets = {}
        axes_cfg = [
            (AX_PITCH, "Pitch Axis (Elevator | CAN Node 2)", "#38bdf8", 0),
            (AX_ROLL, "Roll Axis (Aileron | CAN Node 1)", "#e3b341", 1),
            (AX_RUDDER, "Rudder Axis (Yaw | CAN Node 5)", "#a371f7", 2),
        ]

        for mask, title, color, col in axes_cfg:
            box = tk.LabelFrame(cards_frame, text=f"  {title}  ",
                                font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg=color,
                                padx=10, pady=6)
            box.grid(row=0, column=col, sticky="nsew", padx=4)

            # Row 1: Connection Badge & Feedback Validity & Age
            r1 = tk.Frame(box, bg=self.panel_bg)
            r1.pack(fill=tk.X, pady=(0, 4))
            lbl_link = tk.Label(r1, text="OFFLINE", font=('Segoe UI', 8, 'bold'), bg="#484f58", fg="#ffffff", padx=6, pady=2)
            lbl_link.pack(side=tk.LEFT, padx=(0, 4))
            lbl_valid = tk.Label(r1, text="NO TELEMETRY", font=('Segoe UI', 8, 'bold'), bg="#21262d", fg="#8b949e", padx=6, pady=2)
            lbl_valid.pack(side=tk.LEFT, padx=2)
            lbl_age = tk.Label(r1, text="Age: --", font=('Segoe UI', 8), bg=self.panel_bg, fg="#8b949e")
            lbl_age.pack(side=tk.RIGHT)

            # Row 2: Measured Position (Large)
            r2 = tk.Frame(box, bg=self.panel_bg)
            r2.pack(fill=tk.X, pady=(2, 2))
            tk.Label(r2, text="Measured Stick:", font=('Segoe UI', 9), bg=self.panel_bg, fg=self.text_color).pack(side=tk.LEFT)
            lbl_pos = tk.Label(r2, text="+0.0000", font=('Segoe UI', 12, 'bold'), bg=self.panel_bg, fg=color)
            lbl_pos.pack(side=tk.RIGHT)

            # Position Meter Canvas [-1.0 .. 0 .. +1.0]
            cvs_bar = tk.Canvas(box, height=20, bg="#161b22", highlightthickness=1, highlightbackground="#30363d")
            cvs_bar.pack(fill=tk.X, pady=(2, 4))

            # Row 3: Command & ACK
            r3 = tk.Frame(box, bg=self.panel_bg)
            r3.pack(fill=tk.X, pady=(2, 2))
            lbl_cmd = tk.Label(r3, text="Last Cmd: +0.0000", font=('Segoe UI', 8, 'bold'), bg=self.panel_bg, fg=self.text_color)
            lbl_cmd.pack(side=tk.LEFT)
            lbl_ack = tk.Label(r3, text="ACK: --", font=('Segoe UI', 8, 'bold'), bg="#21262d", fg="#8b949e", padx=4, pady=1)
            lbl_ack.pack(side=tk.RIGHT)

            # Row 4: Packet Telemetry Inspector
            r4 = tk.Frame(box, bg=self.panel_bg)
            r4.pack(fill=tk.X, pady=(2, 2))
            lbl_format = tk.Label(r4, text="Format: -- | Node: --", font=('Segoe UI', 8), bg=self.panel_bg, fg="#8b949e")
            lbl_format.pack(side=tk.LEFT)
            lbl_hex = tk.Label(box, text="Hex: --", font=('Consolas', 7), bg="#161b22", fg="#79c0ff", anchor=tk.W, padx=4, pady=2)
            lbl_hex.pack(fill=tk.X, pady=(2, 4))

            # Row 5: Protocol Error Counters
            r5 = tk.Frame(box, bg=self.panel_bg)
            r5.pack(fill=tk.X, pady=(2, 2))
            lbl_errs = tk.Label(r5, text="Timeouts: 0 | WrongAxis: 0 | Malformed: 0", font=('Segoe UI', 7, 'bold'), bg=self.panel_bg, fg="#e3b341")
            lbl_errs.pack(side=tk.LEFT)

            # Row 6: Last Error
            lbl_last_err = tk.Label(box, text="Status: OK", font=('Segoe UI', 8), bg=self.panel_bg, fg="#3fb950", anchor=tk.W)
            lbl_last_err.pack(fill=tk.X, pady=(2, 0))

            self.diag_card_widgets[mask] = {
                'lbl_link': lbl_link,
                'lbl_valid': lbl_valid,
                'lbl_age': lbl_age,
                'lbl_pos': lbl_pos,
                'cvs_bar': cvs_bar,
                'lbl_cmd': lbl_cmd,
                'lbl_ack': lbl_ack,
                'lbl_format': lbl_format,
                'lbl_hex': lbl_hex,
                'lbl_errs': lbl_errs,
                'lbl_last_err': lbl_last_err,
                'color': color,
            }

        # Bottom Section: Live Diagnostic Event Console
        log_frame = tk.LabelFrame(main_frame, text="  Live Diagnostic Event Log  ",
                                  font=('Segoe UI', 9, 'bold'), bg=self.panel_bg, fg="#79c0ff",
                                  padx=8, pady=6)
        log_frame.pack(fill=tk.BOTH, expand=True)

        log_ctrl_bar = tk.Frame(log_frame, bg=self.panel_bg)
        log_ctrl_bar.pack(fill=tk.X, pady=(0, 4))

        self.chk_diag_autoscroll = tk.BooleanVar(value=True)
        chk_scroll = tk.Checkbutton(log_ctrl_bar, text="Auto-scroll to latest event",
                                    variable=self.chk_diag_autoscroll, bg=self.panel_bg, fg=self.text_color,
                                    selectcolor="#161b22", activebackground=self.panel_bg, activeforeground="#ffffff",
                                    font=('Segoe UI', 8))
        chk_scroll.pack(side=tk.LEFT)

        self.lbl_diag_event_count = tk.Label(log_ctrl_bar, text="Events: 0", font=('Segoe UI', 8), bg=self.panel_bg, fg="#8b949e")
        self.lbl_diag_event_count.pack(side=tk.RIGHT)

        txt_container = tk.Frame(log_frame, bg="#161b22")
        txt_container.pack(fill=tk.BOTH, expand=True)

        self.txt_diag_log = tk.Text(txt_container, bg="#161b22", fg="#c9d1d9",
                                    insertbackground="#ffffff", font=('Consolas', 9),
                                    wrap=tk.NONE, height=8, relief=tk.FLAT)
        scrollbar_y = tk.Scrollbar(txt_container, orient=tk.VERTICAL, command=self.txt_diag_log.yview)
        scrollbar_x = tk.Scrollbar(txt_container, orient=tk.HORIZONTAL, command=self.txt_diag_log.xview)
        self.txt_diag_log.configure(xscrollcommand=scrollbar_x.set, yscrollcommand=scrollbar_y.set)

        scrollbar_y.pack(side=tk.RIGHT, fill=tk.Y)
        scrollbar_x.pack(side=tk.BOTTOM, fill=tk.X)
        self.txt_diag_log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.txt_diag_log.tag_configure("INFO", foreground="#79c0ff")
        self.txt_diag_log.tag_configure("WARN", foreground="#e3b341")
        self.txt_diag_log.tag_configure("ERROR", foreground="#f85149")
        self.txt_diag_log.tag_configure("ACK", foreground="#7ee787")
        self._last_diag_event_count = 0

    def _update_tab_diagnostics(self):
        if not hasattr(self, 'diag_card_widgets'):
            return

        snap = self.cls.get_diag_snapshot()

        # Update Overrides & Watchdog status banner
        if hasattr(self, 'lbl_diag_ovr_pitch'):
            p_ovr = snap['owns_pitch']
            self.lbl_diag_ovr_pitch.configure(
                text=f"Pitch Override: {'ARMED' if p_ovr else 'MANUAL'}",
                bg="#238636" if p_ovr else "#21262d",
                fg="#ffffff" if p_ovr else "#8b949e"
            )
        if hasattr(self, 'lbl_diag_ovr_roll'):
            r_ovr = snap['owns_roll']
            self.lbl_diag_ovr_roll.configure(
                text=f"Roll Override: {'ARMED' if r_ovr else 'MANUAL'}",
                bg="#238636" if r_ovr else "#21262d",
                fg="#ffffff" if r_ovr else "#8b949e"
            )
        if hasattr(self, 'lbl_diag_ovr_yaw'):
            y_ovr = snap['owns_yaw']
            self.lbl_diag_ovr_yaw.configure(
                text=f"Rudder Override: {'ARMED' if y_ovr else 'MANUAL'}",
                bg="#238636" if y_ovr else "#21262d",
                fg="#ffffff" if y_ovr else "#8b949e"
            )
        if hasattr(self, 'lbl_diag_watchdog'):
            wd_on = snap['watchdog_enabled']
            wd_age = snap['watchdog_age_ms']
            self.lbl_diag_watchdog.configure(
                text=f"Watchdog: {'ACTIVE (' + (f'{wd_age:.0f}ms' if wd_age >= 0 else 'OK') + ')' if wd_on else 'IDLE'}",
                bg="#1f6feb" if wd_on else "#21262d",
                fg="#ffffff" if wd_on else "#8b949e"
            )

        # Overall Status Badge
        all_online = all(snap['axes'][m]['connected'] for m in (AX_PITCH, AX_ROLL, AX_RUDDER))
        all_valid = all(snap['axes'][m]['feedback_valid'] for m in (AX_PITCH, AX_ROLL, AX_RUDDER))
        if hasattr(self, 'lbl_diag_overall_badge'):
            if all_online and all_valid:
                self.lbl_diag_overall_badge.configure(
                    text="ALL 3 AXES ONLINE & VALID", bg="#238636", fg="#ffffff"
                )
            elif all_online:
                self.lbl_diag_overall_badge.configure(
                    text="TELEMETRY WARNING / INVALID", bg="#d29922", fg="#ffffff"
                )
            else:
                self.lbl_diag_overall_badge.configure(
                    text="AXIS TIMEOUT / OFFLINE", bg="#da3633", fg="#ffffff"
                )

        # Update 3 Axis Cards
        for mask in (AX_PITCH, AX_ROLL, AX_RUDDER):
            w = self.diag_card_widgets.get(mask)
            if not w:
                continue
            d = snap['axes'][mask]
            conn = d['connected']
            valid = d['feedback_valid']
            age = d['feedback_age_ms']
            pos = d['measured_pos']
            cmd_pos = d['last_cmd_pos']

            # Link badge
            w['lbl_link'].configure(
                text="ONLINE" if conn else "TIMEOUT",
                bg="#238636" if conn else "#da3633",
                fg="#ffffff"
            )
            # Validity badge
            w['lbl_valid'].configure(
                text="VALID TELEMETRY" if valid else ("WAITING" if age < 0 else "INVALID / ERR"),
                fg="#3fb950" if valid else ("#8b949e" if age < 0 else "#f85149")
            )
            # Feedback Age
            if age >= 0:
                age_color = "#3fb950" if age < 250 else ("#e3b341" if age < 1000 else "#f85149")
                w['lbl_age'].configure(text=f"Age: {age:.0f} ms", fg=age_color)
            else:
                w['lbl_age'].configure(text="Age: --", fg="#8b949e")

            # Measured Position
            w['lbl_pos'].configure(text=f"{pos:+.4f}")

            # Draw Canvas Position Bar
            cvs = w['cvs_bar']
            cvs.delete("all")
            cw = cvs.winfo_width()
            if cw < 40:
                cw = 240
            ch = 20
            cx = cw / 2.0
            scale = (cw - 30.0) / 2.0

            # Background track and zero tick
            cvs.create_line(15, ch/2, cw-15, ch/2, fill="#30363d", width=3)
            cvs.create_line(cx, 3, cx, ch-3, fill="#8b949e", width=1)
            cvs.create_text(15, ch-4, text="-1", fill="#6e7681", font=('Segoe UI', 6), anchor="s")
            cvs.create_text(cx, ch-4, text="0", fill="#6e7681", font=('Segoe UI', 6), anchor="s")
            cvs.create_text(cw-15, ch-4, text="+1", fill="#6e7681", font=('Segoe UI', 6), anchor="s")

            # Draw commanded marker (orange marker)
            x_cmd = max(15.0, min(cw - 15.0, cx + cmd_pos * scale))
            cvs.create_polygon(x_cmd, 2, x_cmd-4, 7, x_cmd+4, 7, fill="#ffa657", outline="")

            # Draw measured fill & needle
            x_meas = max(15.0, min(cw - 15.0, cx + pos * scale))
            if abs(x_meas - cx) > 1:
                cvs.create_rectangle(min(cx, x_meas), ch/2 - 3, max(cx, x_meas), ch/2 + 3,
                                     fill=w['color'], outline="")
            cvs.create_line(x_meas, 3, x_meas, ch-3, fill="#ffffff", width=2)

            # Last Commanded & ACK
            w['lbl_cmd'].configure(text=f"Last Cmd: {cmd_pos:+.4f}")
            if d['last_cmd_ack_received']:
                ack_st = d['last_cmd_ack_status']
                ack_age = d['last_cmd_ack_age_ms']
                if ack_st == 0:
                    w['lbl_ack'].configure(text=f"ACK OK (0) [{ack_age:.0f}ms]", bg="#1f6feb", fg="#ffffff")
                else:
                    w['lbl_ack'].configure(text=f"ACK ERR ({ack_st})", bg="#da3633", fg="#ffffff")
            elif d['cmd_count'] > 0:
                w['lbl_ack'].configure(text="NO ACK / TIMEOUT", bg="#da3633", fg="#ffffff")
            else:
                w['lbl_ack'].configure(text="ACK: IDLE", bg="#21262d", fg="#8b949e")

            # Packet format, node, hex
            node_txt = f"Node: {d['last_node']}" if d['last_node'] is not None else "Node: --"
            st_txt = f"st={d['last_status']}" if d['last_status'] is not None else ""
            w['lbl_format'].configure(text=f"Format: {d['last_format']} | {node_txt} {st_txt}")
            w['lbl_hex'].configure(text=f"Hex: {d['last_raw_hex']}")

            # Statistics & Error counters
            tot = d['total_queries']
            val_cnt = d['valid_replies']
            pct = (val_cnt / tot * 100.0) if tot > 0 else 0.0
            w['lbl_errs'].configure(
                text=f"Timeouts: {d['timeouts']} | WrongAxis: {d['wrong_axis_count']} | Bad: {d['malformed_count']} | OOB: {d['bounds_errors']}"
            )

            # Last error message
            if d['last_error']:
                w['lbl_last_err'].configure(text=f"Err: {d['last_error']}", fg="#f85149")
            elif valid:
                w['lbl_last_err'].configure(text=f"Status: Normal (Queries: {tot}, Valid: {pct:.1f}%)", fg="#3fb950")
            else:
                w['lbl_last_err'].configure(text="Status: Awaiting feedback", fg="#8b949e")

        # Update Event Log Console
        events = snap['events']
        num_events = len(events)
        if hasattr(self, 'lbl_diag_event_count'):
            self.lbl_diag_event_count.configure(text=f"Events: {num_events}")

        if hasattr(self, 'txt_diag_log'):
            if num_events > self._last_diag_event_count:
                new_items = events[self._last_diag_event_count:]
                for item in new_items:
                    tag = "INFO"
                    low = item.lower()
                    if "invalid" in low or "error" in low or "failed" in low:
                        tag = "ERROR"
                    elif "timeout" in low or "rejected" in low or "warn" in low:
                        tag = "WARN"
                    elif "ack" in low or "armed" in low:
                        tag = "ACK"
                    self.txt_diag_log.insert(tk.END, item + "\n", tag)
                self._last_diag_event_count = num_events
                if self.chk_diag_autoscroll.get():
                    self.txt_diag_log.see(tk.END)

    def export_diag_report(self):
        try:
            folder = os.path.join(self._get_base_dir(), 'PID_LOGS')
            os.makedirs(folder, exist_ok=True)
            fname = 'BRUNNER_DIAG_REPORT_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.txt'
            path = os.path.join(folder, fname)
            snap = self.cls.get_diag_snapshot()

            with open(path, 'w', encoding='utf-8') as f:
                f.write("=" * 80 + "\n")
                f.write("BRUNNER CLS-P HARDWARE & PROTOCOL DIAGNOSTIC REPORT\n")
                f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Application: Prepar3D Flight Controller V4.4 (Brunner-Only Tri-Axis)\n")
                f.write(f"CLS2Sim UDP Target: {HOST}:{PORT}\n")
                with self.lock:
                    p3d_conn = self.snap_connected
                    p3d_age = self.snap_telemetry_age
                f.write(f"Prepar3D SimConnect State: {'Connected' if p3d_conn else 'Disconnected'} (Age: {p3d_age:.0f} ms) [Strictly READ-ONLY]\n")
                f.write("=" * 80 + "\n\n")

                f.write("1. HARDWARE OVERRIDE OWNERSHIP & SAFETY INTERLOCKS\n")
                f.write("-" * 80 + "\n")
                f.write(f"* Pitch Motor Override (Elevator): {'ARMED (Autopilot Driving)' if snap['owns_pitch'] else 'RELEASED (Manual Cockpit)'}\n")
                f.write(f"* Roll Motor Override (Aileron):   {'ARMED (Autopilot Driving)' if snap['owns_roll'] else 'RELEASED (Manual Cockpit)'}\n")
                f.write(f"* Rudder Motor Override (Yaw):     {'ARMED (Autopilot Driving)' if snap['owns_yaw'] else 'RELEASED (Manual Cockpit)'}\n")
                f.write(f"* Watchdog State:                 {'ACTIVE' if snap['watchdog_enabled'] else 'IDLE'} (Age: {snap['watchdog_age_ms']:.0f} ms)\n")
                f.write(f"* Total UDP Timeout Count:        {snap['total_udp_timeouts']}\n\n")
                f.write("SAFETY ADVISORY:\n")
                f.write("UDP Protocol Acknowledgement verifies network communication with CLS2Sim software.\n")
                f.write("Physical motor drive engagement additionally requires CLS2Sim CAN bus active,\n")
                f.write("motor drive power ON, and motor clutch/safety interlocks closed.\n\n")

                f.write("2. TRI-AXIS TELEMETRY & COMMAND SUMMARY\n")
                f.write("-" * 80 + "\n")
                hdr = f"{'Axis':<16} {'Mask':<6} {'CAN Node':<9} {'Link':<9} {'Valid':<8} {'Meas Pos':<10} {'Age(ms)':<8} {'Cmd Pos':<9} {'Cmd ACK':<9} {'Timeouts':<9} {'WrongNode':<10} {'BadPkts':<8}\n"
                f.write(hdr)
                f.write("-" * 80 + "\n")
                for mask in (AX_PITCH, AX_ROLL, AX_RUDDER):
                    d = snap['axes'][mask]
                    link_s = "ONLINE" if d['connected'] else "TIMEOUT"
                    val_s = "VALID" if d['feedback_valid'] else "INVALID"
                    meas_s = f"{d['measured_pos']:+.4f}"
                    age_s = f"{d['feedback_age_ms']:.0f}" if d['feedback_age_ms'] >= 0 else "--"
                    cmd_s = f"{d['last_cmd_pos']:+.4f}"
                    ack_s = f"ACK {d['last_cmd_ack_status']}" if d['last_cmd_ack_received'] else "NO_ACK"
                    node_s = str(d['last_node']) if d['last_node'] is not None else "--"
                    line = f"{d['name']:<16} {hex(mask):<6} {node_s:<9} {link_s:<9} {val_s:<8} {meas_s:<10} {age_s:<8} {cmd_s:<9} {ack_s:<9} {d['timeouts']:<9} {d['wrong_axis_count']:<10} {d['malformed_count']:<8}\n"
                    f.write(line)
                f.write("\n")

                f.write("3. PROTOCOL INSPECTOR & PACKET TELEMETRY (DEEP DIVE)\n")
                f.write("-" * 80 + "\n")
                for mask in (AX_PITCH, AX_ROLL, AX_RUDDER):
                    d = snap['axes'][mask]
                    pct = (d['valid_replies'] / max(1, d['total_queries']) * 100.0) if d['total_queries'] > 0 else 0.0
                    f.write(f"[{d['name']} - Node Mask 0x{mask:04X}]\n")
                    f.write(f"  * Detected Packet Format: {d['last_format']}\n")
                    f.write(f"  * Status Code Reported:   {d['last_status']}\n")
                    f.write(f"  * Node Reported in Reply: {d['last_node']}\n")
                    f.write(f"  * Last Raw Hex Packet:    {d['last_raw_hex']}\n")
                    f.write(f"  * Query Count / Valid:    {d['total_queries']} queries, {d['valid_replies']} valid ({pct:.1f}%)\n")
                    f.write(f"  * Protocol Errors:        Timeouts={d['timeouts']}, WrongAxis={d['wrong_axis_count']}, Malformed={d['malformed_count']}, BoundsErrors={d['bounds_errors']}\n")
                    f.write(f"  * Last Telemetry Error:   {d['last_error'] or 'None (Normal)'}\n")
                    f.write(f"  * Commands Sent / ACK:    {d['cmd_count']} sent, {d['cmd_ack_count']} acked, {d['cmd_reject_count']} rejected\n")
                    f.write(f"  * Last Command ACK Age:   {d['last_cmd_ack_age_ms']:.0f} ms\n")
                    f.write(f"  * Last Command Error:     {d['last_cmd_error'] or 'None (Normal)'}\n\n")

                f.write("4. RECENT DIAGNOSTIC EVENT LOG\n")
                f.write("-" * 80 + "\n")
                for ev in snap['events']:
                    f.write(ev + "\n")
                f.write("=" * 80 + "\n")
                f.write("END OF REPORT\n")

            messagebox.showinfo("Diagnostic Report Saved", f"Diagnostic report saved successfully:\n\n{path}")
        except Exception as exc:
            messagebox.showerror("Export Failed", f"Failed to save diagnostic report:\n{exc}")

    def export_diag_csv(self):
        try:
            folder = os.path.join(self._get_base_dir(), 'PID_LOGS')
            os.makedirs(folder, exist_ok=True)
            fname = 'BRUNNER_DIAG_SNAPSHOT_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.csv'
            path = os.path.join(folder, fname)
            snap = self.cls.get_diag_snapshot()

            fields = [
                'timestamp_local', 'axis_name', 'axis_mask', 'connected', 'feedback_valid',
                'measured_pos', 'feedback_age_ms', 'packet_format', 'status_code',
                'reported_node', 'raw_hex', 'last_cmd_pos', 'cmd_ack_received',
                'cmd_ack_status', 'cmd_ack_age_ms', 'timeouts', 'wrong_axis_count',
                'malformed_count', 'bounds_errors', 'total_queries', 'valid_replies',
                'cmd_count', 'cmd_ack_count', 'cmd_reject_count', 'last_error',
                'owns_overrides', 'watchdog_enabled'
            ]
            now_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            with open(path, 'w', newline='', encoding='utf-8-sig') as f:
                writer = csv.DictWriter(f, fieldnames=fields)
                writer.writeheader()
                for mask in (AX_PITCH, AX_ROLL, AX_RUDDER):
                    d = snap['axes'][mask]
                    owns = (snap['owns_pitch'] if mask == AX_PITCH else
                            snap['owns_roll'] if mask == AX_ROLL else snap['owns_yaw'])
                    writer.writerow({
                        'timestamp_local': now_str,
                        'axis_name': d['name'],
                        'axis_mask': mask,
                        'connected': 1 if d['connected'] else 0,
                        'feedback_valid': 1 if d['feedback_valid'] else 0,
                        'measured_pos': round(d['measured_pos'], 5),
                        'feedback_age_ms': round(d['feedback_age_ms'], 1),
                        'packet_format': d['last_format'],
                        'status_code': d['last_status'] if d['last_status'] is not None else '',
                        'reported_node': d['last_node'] if d['last_node'] is not None else '',
                        'raw_hex': d['last_raw_hex'],
                        'last_cmd_pos': round(d['last_cmd_pos'], 5),
                        'cmd_ack_received': 1 if d['last_cmd_ack_received'] else 0,
                        'cmd_ack_status': d['last_cmd_ack_status'] if d['last_cmd_ack_status'] is not None else '',
                        'cmd_ack_age_ms': round(d['last_cmd_ack_age_ms'], 1),
                        'timeouts': d['timeouts'],
                        'wrong_axis_count': d['wrong_axis_count'],
                        'malformed_count': d['malformed_count'],
                        'bounds_errors': d['bounds_errors'],
                        'total_queries': d['total_queries'],
                        'valid_replies': d['valid_replies'],
                        'cmd_count': d['cmd_count'],
                        'cmd_ack_count': d['cmd_ack_count'],
                        'cmd_reject_count': d['cmd_reject_count'],
                        'last_error': d['last_error'] or '',
                        'owns_overrides': 1 if owns else 0,
                        'watchdog_enabled': 1 if snap['watchdog_enabled'] else 0
                    })

            messagebox.showinfo("Diagnostic CSV Saved", f"Diagnostic snapshot CSV saved successfully:\n\n{path}")
        except Exception as exc:
            messagebox.showerror("Export Failed", f"Failed to save diagnostic CSV:\n{exc}")

    def clear_diag_log(self):
        with self.cls.io_lock:
            self.cls.recent_events.clear()
        if hasattr(self, 'txt_diag_log'):
            self.txt_diag_log.delete("1.0", tk.END)
        self._last_diag_event_count = 0

    def on_tab_changed(self, event=None):
        try:
            self.active_tab_index = 3 if self.notebook.select() == str(self.tab_diag) else 0
            self._update_engage_buttons()
        except Exception:
            pass

    LOG_FIELDS = (
        'timestamp_local', 'elapsed_s', 'event',
        'target_ias_kt', 'actual_ias_kt', 'ias_error_kt',
        'target_pitch_deg', 'actual_pitch_deg', 'pitch_error_deg',
        'target_bank_deg', 'actual_bank_deg', 'bank_error_deg',
        'target_heading_deg', 'actual_heading_deg', 'heading_error_deg',
        'bank_phase', 'demanded_bank_deg', 'sideslip_deg', 'yaw_rate_deg_sec',
        'pitch_pid_output', 'ias_pitch_integral', 'pitch_stick_integral',
        'bank_pid_output', 'bank_integral',
        'yaw_pid_output', 'yaw_integral',
        'brunner_pitch_cmd', 'brunner_pitch_meas',
        'brunner_roll_cmd', 'brunner_roll_meas',
        'brunner_yaw_cmd', 'brunner_yaw_meas',
        'ias_kp', 'ias_ki', 'ias_kd', 'pitch_kp', 'pitch_ki', 'pitch_kd',
        'bank_kp', 'bank_ki', 'bank_kd',
        'heading_kp', 'heading_ki', 'heading_kd',
        'yaw_kp', 'yaw_ki', 'yaw_kd',
        'elevator_position', 'yoke_y_position', 'rudder_position', 'throttle_percent',
        'aircraft_weight_kg', 'ambient_temp_c', 'p3d_connected',
        'brunner_connected', 'p3d_paused', 'bridge_error', 'fault',
    )

    def _start_log(self):
        base = os.path.dirname(sys.executable if getattr(sys, 'frozen', False) else os.path.abspath(__file__))
        folder = os.path.join(base, 'PID_LOGS')
        os.makedirs(folder, exist_ok=True)
        filename = 'SMK_Brunner_TriAxis_V4_4_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.csv'
        path = os.path.join(folder, filename)
        with self.log_lock:
            if self.log_file is not None:
                self.log_file.flush()
                self.log_file.close()
            self.log_file = open(path, 'w', newline='', encoding='utf-8-sig')
            self.log_writer = csv.DictWriter(self.log_file, fieldnames=self.LOG_FIELDS)
            self.log_writer.writeheader()
            self.log_path = path
            self.log_counter = 0
        self.run_started = time.monotonic()
        return path

    def _log_cycle(self, row):
        with self.log_lock:
            if self.log_writer is None:
                return
            self.log_writer.writerow({field: (datetime.now().astimezone().isoformat(timespec='milliseconds')
                                            if field == 'timestamp_local' else row.get(field, ''))
                                           for field in self.LOG_FIELDS})
            self.log_counter += 1
            if self.log_counter % 25 == 0:
                self.log_file.flush()

    def _stop_log(self):
        with self.log_lock:
            if self.log_file is not None:
                self.log_file.flush()
                self.log_file.close()
            self.log_file = None
            self.log_writer = None

    def open_logs_folder(self):
        try:
            base = os.path.dirname(sys.executable if getattr(sys, 'frozen', False) else os.path.abspath(__file__))
            folder = os.path.join(base, 'PID_LOGS')
            os.makedirs(folder, exist_ok=True)
            if sys.platform == 'win32':
                os.startfile(folder)
            else:
                subprocess.Popen(['xdg-open', folder])
        except Exception as e:
            print(f"[Open Logs Error] Could not open logs folder: {e}")

    def _stop_for_fault(self, message):
        with self.lock:
            self.pitch_enabled = False
            self.bank_enabled = False
            self.yaw_enabled = False
            self.last_fault = message
        try:
            self.cls.release()
            self.bridge.enable(False)
        except Exception:
            pass
        try:
            self._update_engage_buttons()
        except Exception:
            pass
        try:
            self._log_cycle({'elapsed_s': round(time.monotonic() - (self.run_started or time.monotonic()), 3),
                             'event': 'FAULT', 'fault': message})
        finally:
            self._stop_log()

    # ----------------- Dedicated 25 Hz Control Thread -----------------
    def _pid_worker_loop(self):
        last_t = time.monotonic()
        while self.running:
            now = time.monotonic()
            dt = now - last_t
            last_t = now
            dt = min(0.1, max(0.01, dt))

            inst_hz = 1.0 / dt if dt > 0.0001 else 25.0
            self.measured_hz = 0.90 * self.measured_hz + 0.10 * inst_hz

            # 1. Read Prepar3D telemetry (STRICTLY READ-ONLY)
            state = self.bridge.read()
            connected = (state.connected == 1 and state.telemetryValid == 1 and state.telemetryAgeMs < 1500)
            is_paused = (getattr(state, 'isPaused', 0) == 1) or self.bridge.is_paused()

            valid_telemetry = (math.isfinite(state.pitchDeg) and math.isfinite(state.bankDeg) and
                               math.isfinite(state.headingDeg) and math.isfinite(state.iasKt))
            if connected and valid_telemetry:
                pitch_actual = -state.pitchDeg
                # P3D bank is positive left; controller bank is positive right.
                bank_actual = -state.bankDeg
                heading_actual = state.headingDeg
                yaw_rate_actual = state.yawRateDegSec if math.isfinite(state.yawRateDegSec) else 0.0
                sideslip_actual = state.sideslipDeg if math.isfinite(state.sideslipDeg) else 0.0
                rudder_pos = state.rudderPosition if math.isfinite(state.rudderPosition) else 0.0
                ias_actual = state.iasKt
                elev_pos = state.elevatorPosition if math.isfinite(state.elevatorPosition) else 0.0
                yoke_pos = state.yokeYPosition if math.isfinite(state.yokeYPosition) else 0.0
                throttle_val = state.throttlePercent if math.isfinite(state.throttlePercent) else 0.0
                weight_val = state.totalWeightKg if math.isfinite(state.totalWeightKg) else float('nan')
                temp_val = state.ambientTempC if math.isfinite(state.ambientTempC) else float('nan')

                self.last_valid_pitch = pitch_actual
                self.last_valid_bank = bank_actual
                self.last_valid_heading = heading_actual
                self.last_valid_ias = ias_actual
            else:
                connected = False
                pitch_actual = getattr(self, 'last_valid_pitch', 0.0)
                bank_actual = getattr(self, 'last_valid_bank', 0.0)
                heading_actual = getattr(self, 'last_valid_heading', 360.0)
                yaw_rate_actual = 0.0
                sideslip_actual = 0.0
                rudder_pos = 0.0
                ias_actual = getattr(self, 'last_valid_ias', 0.0)
                elev_pos = 0.0
                yoke_pos = 0.0
                throttle_val = 0.0
                weight_val = float('nan')
                temp_val = float('nan')

            # 2. Poll Brunner CLS-P stick positions (Pitch, Roll, Rudder) & refresh watchdog
            brunner_conn, b_pitch_pos, b_roll_pos, b_yaw_pos = self.cls.poll()
            self.cls.watchdog()

            with self.lock:
                pitch_on = self.pitch_enabled
                bank_on = self.bank_enabled
                yaw_on = self.yaw_enabled
                any_enabled = pitch_on or bank_on or yaw_on
                guidance = self.guidance_mode
                mode = self.control_mode
                target_p = self.target_pitch
                target_ias = self.target_ias
                target_bank = self.target_bank
                target_hdg = self.target_heading

            # If Prepar3D is paused, freeze controller execution
            if is_paused:
                self.was_paused = True
                self.bank_handoff.settled_seconds = 0.0
                last_t = now
                if connected and brunner_conn and any_enabled and mode == 3:
                    self.cls.watchdog()
                    try:
                        if pitch_on:
                            self.cls.position_pitch(self.brunner_pitch_cmd)
                        if bank_on or yaw_on:
                            self.cls.position_roll(self.brunner_roll_cmd)
                        if yaw_on:
                            self.cls.position_rudder(self.brunner_yaw_cmd)
                    except Exception as exc:
                        self._stop_for_fault('BRUNNER command failed while paused: ' + str(exc))
                with self.lock:
                    self.snap_paused = True
                    self.snap_connected = connected
                    self.snap_telemetry_age = state.telemetryAgeMs
                    self.snap_measured_hz = 0.0
                    self.snap_brunner_connected = brunner_conn
                    self.snap_brunner_pitch = b_pitch_pos
                    self.snap_brunner_roll = b_roll_pos
                    self.snap_brunner_yaw = b_yaw_pos
                    self.snap_pitch_actual = pitch_actual
                    self.snap_bank_actual = bank_actual
                    self.snap_ias_actual = ias_actual
                    self.snap_heading_actual = heading_actual
                    self.snap_yaw_rate = yaw_rate_actual
                    self.snap_sideslip = sideslip_actual
                if self.log_file is not None:
                    self._log_cycle({
                        'elapsed_s': round(time.monotonic() - (self.run_started or time.monotonic()), 3),
                        'event': 'PAUSED', 'actual_ias_kt': ias_actual,
                        'actual_pitch_deg': pitch_actual, 'actual_bank_deg': bank_actual,
                        'actual_heading_deg': heading_actual,
                        'brunner_pitch_meas': b_pitch_pos, 'brunner_pitch_cmd': self.brunner_pitch_cmd,
                        'brunner_roll_meas': b_roll_pos, 'brunner_roll_cmd': self.brunner_roll_cmd,
                        'brunner_yaw_meas': b_yaw_pos, 'brunner_yaw_cmd': self.brunner_yaw_cmd,
                        'p3d_paused': 1,
                    })
                time.sleep(0.040)
                continue

            # Bumpless resumption when unpausing
            if self.was_paused:
                self.was_paused = False
                last_t = now
                if guidance == "IAS":
                    self.ias_ctrl.reset(initial_pitch=pitch_actual)
                self.pid.reset(initial_pos=self.brunner_pitch_cmd)
                self.bank_pid.reset(initial_output=self.brunner_roll_cmd)
                self.yaw_pid.reset(initial_output=self.brunner_yaw_cmd)
                self.heading_ctrl.reset(initial_bank=bank_actual)

            # 3. Guidance Errors & Targets
            if guidance == "IAS":
                if ias_actual >= 10.0 and pitch_on:
                    active_target_pitch = self.ias_ctrl.update(target_ias, ias_actual, dt, pitch_actual)
                else:
                    active_target_pitch = 0.0
                    if not pitch_on:
                        self.ias_ctrl.reset(initial_pitch=pitch_actual)
            else:
                active_target_pitch = target_p

            pitch_error = pitch_actual - active_target_pitch
            ias_error = ias_actual - target_ias

            # Heading guidance retains roll ownership until heading is settled.
            hdg_err = (target_hdg - heading_actual + 180.0) % 360.0 - 180.0
            was_captured = self.bank_handoff.captured
            if yaw_on:
                demanded_bank = self.heading_ctrl.update(target_hdg, heading_actual, yaw_rate_actual, dt)
            else:
                demanded_bank = 0.0
            captured = self.bank_handoff.update(
                bank_on, yaw_on, target_hdg, heading_actual, bank_actual,
                self.heading_ctrl.hdg_rate, dt, connected and brunner_conn)
            if yaw_on and was_captured and not captured:
                # Reacquire from actual bank, not an accumulated hidden turn demand.
                self.heading_ctrl.reset(initial_bank=bank_actual)
                demanded_bank = self.heading_ctrl.update(target_hdg, heading_actual, yaw_rate_actual, dt)
            if bank_on and yaw_on and captured:
                effective_target_bank = 0.0
                demanded_bank = 0.0
                # Keep heading-rate feedback updated, without latent integral or bank demand.
                self.heading_ctrl.integral = 0.0
                self.heading_ctrl.cmd_bank = 0.0
                bank_phase = "WINGS LEVEL"
            elif yaw_on:
                effective_target_bank = demanded_bank
                bank_phase = "ARMED / WAITING" if bank_on else "HEADING STEER"
            elif bank_on:
                effective_target_bank = target_bank
                bank_phase = "BANK HOLD"
            else:
                effective_target_bank = 0.0
                bank_phase = "OFF"

            bank_error = effective_target_bank - bank_actual

            # Telemetry watchdog
            if any_enabled and not is_paused:
                if not connected or not brunner_conn:
                    if self.failure_start is None:
                        self.failure_start = time.monotonic()
                    elif time.monotonic() - self.failure_start > 1.5:
                        self._stop_for_fault('P3D/BRUNNER feedback lost for more than 1.5 s')
                        pitch_on = False
                        bank_on = False
                        yaw_on = False
                else:
                    self.failure_start = None

            # 4. Hardware Actuation: Brunner CLS-P Pitch, Roll & Rudder SEPARATED
            if connected and brunner_conn and mode == 3:
                # --- PITCH AXIS (Airspeed & Pitch Hold) ---
                if pitch_on:
                    if not self.cls.owns_pitch_overrides or not self.cls.watchdog_enabled:
                        self.cls.engage_pitch(self.brunner_pitch_cmd)

                    desired_pitch = self.pid.update(active_target_pitch, pitch_actual, dt)
                    next_pitch_p, next_pitch_v = slew_position(self.brunner_pitch_cmd, self.brunner_pitch_vel,
                                                               desired_pitch, dt, max_rate=0.28, max_accel=0.90)
                    self.brunner_pitch_cmd = next_pitch_p
                    self.brunner_pitch_vel = next_pitch_v

                    try:
                        self.cls.position_pitch(self.brunner_pitch_cmd)
                    except Exception as exc:
                        self._stop_for_fault('BRUNNER pitch motor error: ' + str(exc))
                        pitch_on = False
                        with self.lock:
                            self.pitch_enabled = False
                else:
                    if self.cls.owns_pitch_overrides:
                        self.cls.release_pitch()

                # --- ROLL AXIS (Bank Angle Hold / Heading Steer) ---
                if bank_on or yaw_on:
                    if not self.cls.owns_roll_overrides or not self.cls.watchdog_enabled:
                        self.cls.engage_roll(self.brunner_roll_cmd)

                    desired_roll = self.bank_pid.update(effective_target_bank, bank_actual, dt)
                    next_roll_p, next_roll_v = slew_position(self.brunner_roll_cmd, self.brunner_roll_vel,
                                                             desired_roll, dt, max_rate=0.28, max_accel=0.90)
                    self.brunner_roll_cmd = next_roll_p
                    self.brunner_roll_vel = next_roll_v

                    try:
                        self.cls.position_roll(self.brunner_roll_cmd)
                    except Exception as exc:
                        self._stop_for_fault('BRUNNER roll motor error: ' + str(exc))
                        bank_on = False
                        yaw_on = False
                        with self.lock:
                            self.bank_enabled = False
                            self.yaw_enabled = False
                else:
                    if self.cls.owns_roll_overrides:
                        self.cls.release_roll()

                # --- YAW / RUDDER AXIS (Coordinated Flight & Yaw Damper) ---
                if yaw_on:
                    if not self.cls.owns_yaw_overrides or not self.cls.watchdog_enabled:
                        self.cls.engage_yaw(self.brunner_yaw_cmd)

                    # Preserve the sustained yaw rate of a coordinated turn.
                    desired_yaw = self.yaw_pid.update(sideslip_actual, yaw_rate_actual, dt, self.heading_ctrl.hdg_rate)
                    next_yaw_p, next_yaw_v = slew_position(self.brunner_yaw_cmd, self.brunner_yaw_vel,
                                                           desired_yaw, dt, max_rate=0.28, max_accel=0.90)
                    self.brunner_yaw_cmd = next_yaw_p
                    self.brunner_yaw_vel = next_yaw_v

                    try:
                        self.cls.position_rudder(self.brunner_yaw_cmd)
                    except Exception as exc:
                        self._stop_for_fault('BRUNNER rudder motor error: ' + str(exc))
                        pitch_on = bank_on = yaw_on = False
                else:
                    if self.cls.owns_yaw_overrides:
                        self.cls.release_yaw()
                    self.brunner_yaw_cmd = 0.0
                    self.brunner_yaw_vel = 0.0
            else:
                if not pitch_on and self.cls.owns_pitch_overrides:
                    self.cls.release_pitch()
                if not bank_on and not yaw_on and self.cls.owns_roll_overrides:
                    self.cls.release_roll()
                if not yaw_on and self.cls.owns_yaw_overrides:
                    self.cls.release_yaw()

            # Store snapshot for UI
            with self.lock:
                self.snap_paused = False
                self.snap_connected = connected
                self.snap_telemetry_age = state.telemetryAgeMs
                self.snap_measured_hz = self.measured_hz
                self.snap_brunner_connected = brunner_conn
                self.snap_brunner_pitch = b_pitch_pos
                self.snap_brunner_roll = b_roll_pos
                self.snap_brunner_yaw = b_yaw_pos
                self.snap_pitch_actual = pitch_actual
                self.snap_pitch_target = active_target_pitch
                self.snap_pitch_error = pitch_error
                self.snap_ias_actual = ias_actual
                self.snap_ias_target = target_ias
                self.snap_ias_error = ias_error
                self.snap_cmd_pitch = active_target_pitch
                self.snap_guidance_mode = guidance
                self.snap_throttle = throttle_val
                self.snap_elev_pos = elev_pos
                self.snap_yoke_pos = yoke_pos
                self.snap_weight = weight_val
                self.snap_temp = temp_val

                # Bank snapshots
                self.snap_bank_actual = bank_actual
                self.snap_bank_target = effective_target_bank
                self.snap_bank_error = bank_error
                self.snap_bank_phase = bank_phase

                # Heading & Yaw snapshots
                self.snap_heading_actual = heading_actual
                self.snap_heading_target = target_hdg
                self.snap_heading_error = hdg_err
                self.snap_demanded_bank = demanded_bank
                self.snap_yaw_rate = yaw_rate_actual
                self.snap_sideslip = sideslip_actual
                self.snap_rudder_pos = rudder_pos

            if self.run_started is not None and (pitch_on or bank_on or yaw_on):
                self._log_cycle({
                    'elapsed_s': round(time.monotonic() - self.run_started, 3),
                    'event': 'RUN',
                    'target_ias_kt': target_ias, 'actual_ias_kt': ias_actual, 'ias_error_kt': ias_error,
                    'target_pitch_deg': active_target_pitch, 'actual_pitch_deg': pitch_actual, 'pitch_error_deg': pitch_error,
                    'target_bank_deg': effective_target_bank, 'actual_bank_deg': bank_actual, 'bank_error_deg': bank_error,
                    'target_heading_deg': target_hdg, 'actual_heading_deg': heading_actual, 'heading_error_deg': hdg_err,
                    'bank_phase': bank_phase, 'demanded_bank_deg': demanded_bank, 'sideslip_deg': sideslip_actual, 'yaw_rate_deg_sec': yaw_rate_actual,
                    'pitch_pid_output': self.pid.output, 'ias_pitch_integral': self.ias_ctrl.integral,
                    'pitch_stick_integral': self.pid.integral,
                    'bank_pid_output': self.bank_pid.output, 'bank_integral': self.bank_pid.integral,
                    'yaw_pid_output': self.yaw_pid.output, 'yaw_integral': self.yaw_pid.integral,
                    'brunner_pitch_cmd': self.brunner_pitch_cmd, 'brunner_pitch_meas': b_pitch_pos,
                    'brunner_roll_cmd': self.brunner_roll_cmd, 'brunner_roll_meas': b_roll_pos,
                    'brunner_yaw_cmd': self.brunner_yaw_cmd, 'brunner_yaw_meas': b_yaw_pos,
                    'ias_kp': self.ias_ctrl.kp, 'ias_ki': self.ias_ctrl.ki, 'ias_kd': self.ias_ctrl.kd,
                    'pitch_kp': self.pid.kp, 'pitch_ki': self.pid.ki, 'pitch_kd': self.pid.kd,
                    'bank_kp': self.bank_pid.kp, 'bank_ki': self.bank_pid.ki, 'bank_kd': self.bank_pid.kd,
                    'heading_kp': self.heading_ctrl.kp, 'yaw_kp': self.yaw_pid.kp, 'yaw_ki': self.yaw_pid.ki, 'yaw_kd': self.yaw_pid.kd,
                    'elevator_position': elev_pos, 'yoke_y_position': yoke_pos, 'rudder_position': rudder_pos,
                    'throttle_percent': throttle_val,
                    'aircraft_weight_kg': weight_val, 'ambient_temp_c': temp_val,
                    'p3d_connected': int(connected), 'brunner_connected': int(brunner_conn),
                    'p3d_paused': int(is_paused), 'bridge_error': state.lastError,
                    'fault': self.last_fault,
                })
            time.sleep(0.040)

    # ----------------- Smooth 10 Hz GUI Tick Loop (100 ms) -----------------
    def _plot_limits(self, axis, x=None, y=None):
        """Only invalidate a plot background when its limits actually change."""
        changed = False
        if x is not None and tuple(axis.get_xlim()) != tuple(x):
            axis.set_xlim(*x)
            changed = True
        if y is not None and tuple(axis.get_ylim()) != tuple(y):
            axis.set_ylim(*y)
            changed = True
        if changed:
            axis.figure.canvas._live_background = None

    def _draw_live_plot(self, canvas):
        """Reuse axes, labels and legends; redraw only live telemetry lines."""
        if not hasattr(canvas, '_live_initialized'):
            canvas._live_initialized = True
            canvas._live_background = None
            key = tk.Frame(canvas.get_tk_widget().master, bg="#161b22")
            key.pack(side=tk.TOP, fill=tk.X, before=canvas.get_tk_widget())
            for axis in canvas.figure.axes:
                labels = []
                for line in axis.lines:
                    label = line.get_label()
                    if label and not label.startswith('_'):
                        labels.append((label, line.get_color()))
                row = tk.Frame(key, bg="#161b22")
                row.pack(fill=tk.X)
                for label, color in labels:
                    short = label.replace('Brunner ', '').replace(' (P3D Telemetry)', '').replace(' (Measured)', ' meas').replace(' (Commanded)', ' cmd')
                    short = short.replace('Target Bank (0.00? Wings Level)', 'Target bank')
                    tk.Label(row, text='? '+short, fg=color, bg="#161b22", font=('Segoe UI', 7)).pack(side=tk.LEFT)

            for axis in canvas.figure.axes:
                for line in axis.lines:
                    line.set_animated(True)
                if axis.get_legend() is not None:
                    axis.get_legend().set_visible(False)
            def capture_background(event):
                canvas._live_background = canvas.copy_from_bbox(canvas.figure.bbox)
            canvas.mpl_connect('draw_event', capture_background)
            canvas.mpl_connect('resize_event', lambda event: setattr(canvas, '_live_background', None))
        if canvas._live_background is None:
            canvas.draw()
        canvas.restore_region(canvas._live_background)
        for axis in canvas.figure.axes:
            for line in axis.lines:
                axis.draw_artist(line)
            # Legend keys live above the plotting area, outside the cached raster.
        canvas.blit(canvas.figure.bbox)

    def gui_tick(self):
        if not self.running:
            return

        with self.lock:
            conn = self.snap_connected
            age = self.snap_telemetry_age
            hz = self.snap_measured_hz
            is_paused = self.snap_paused
            b_conn = self.snap_brunner_connected
            b_pitch_pos = self.snap_brunner_pitch
            b_roll_pos = self.snap_brunner_roll
            b_yaw_pos = self.snap_brunner_yaw
            pitch_act = self.snap_pitch_actual
            pitch_tgt = self.snap_pitch_target
            pitch_err = self.snap_pitch_error
            ias_act = self.snap_ias_actual
            ias_tgt = self.snap_ias_target
            ias_err = self.snap_ias_error
            cmd_p = self.snap_cmd_pitch
            guidance = self.snap_guidance_mode
            throttle = self.snap_throttle

            # Bank
            bank_act = self.snap_bank_actual
            bank_tgt = self.snap_bank_target
            bank_err = self.snap_bank_error

            # Heading & Yaw
            heading_act = self.snap_heading_actual
            heading_tgt = self.snap_heading_target
            heading_err = self.snap_heading_error
            cmd_bank = self.snap_demanded_bank
            yaw_rate_act = self.snap_yaw_rate
            sideslip_act = self.snap_sideslip

            p_on = self.pitch_enabled
            b_on = self.bank_enabled
            y_on = self.yaw_enabled

        # 1. Update Connection Badges
        if is_paused:
            self.conn_badge.configure(text="P3D: PAUSED", bg="#d29922", fg="#000000")
            self.loop_badge.configure(text="PID: PAUSED", bg="#d29922", fg="#000000")
        elif conn:
            self.conn_badge.configure(text=f"SimConnect: Connected ({age}ms)", bg="#238636", fg="#ffffff")
            self.loop_badge.configure(text=f"PID Rate: {hz:.1f} Hz", bg="#1f6feb", fg="#ffffff")
        else:
            self.conn_badge.configure(text="SimConnect: Disconnected / Waiting", bg="#484f58", fg="#f0f6fc")
            self.loop_badge.configure(text=f"PID Rate: {hz:.1f} Hz", bg="#1f6feb", fg="#ffffff")

        if b_conn:
            self.brunner_badge.configure(text=f"Brunner: Connected (P:{b_pitch_pos:+.2f} R:{b_roll_pos:+.2f} Y:{b_yaw_pos:+.2f})", bg="#238636", fg="#ffffff")
        else:
            self.brunner_badge.configure(text="Brunner: Offline (Port 15090)", bg="#484f58", fg="#f0f6fc")

        # 2. Update Tab 1 Digital Labels
        pause_suffix = " [PAUSED]" if is_paused else ""
        self.lbl_pitch_act.configure(text=f"Pitch: {pitch_act:+.2f}°{pause_suffix}")
        self.lbl_pitch_err.configure(text=f"Pitch Err: {pitch_err:+.2f}°")
        self.lbl_throttle.configure(text=f"Throttle: {throttle:.1f}%")
        self.lbl_ias_act.configure(text=f"IAS: {ias_act:.1f} kt{pause_suffix}")
        self.lbl_ias_err.configure(text=f"IAS Err: {ias_err:+.1f} kt")
        self.lbl_cmd_pitch.configure(text=f"Cmd Pitch: {cmd_p:+.2f}°")
        self.lbl_brunner_pitch.configure(text=f"Stick Pitch (Meas/Cmd): {b_pitch_pos:+.2f} / {self.brunner_pitch_cmd:+.2f}")
        self.lbl_fault.configure(text=self.last_fault[:95])

        # 3. Update Tab 2 Digital Labels (Clean: Bank Actual & Bank Error Only)
        disp_bank = 0.0 if abs(bank_act) < 0.005 else bank_act
        disp_bank_err = 0.0 if abs(bank_err) < 0.005 else bank_err
        self.lbl_bank_act.configure(text=f"Bank: {disp_bank:+.2f}°{pause_suffix}")
        self.lbl_bank_err.configure(text=f"Bank Err: {disp_bank_err:+.2f}°")
        self.lbl_bank_target_info.configure(
            text=f"Active target: {bank_tgt:+.2f} deg | {self.snap_bank_phase}")

        # 4. Update Tab 3 Digital Labels (Clean Heading Attitude Tracking)
        if hasattr(self, 'lbl_hdg_act'):
            disp_hdg = heading_act
            # If target is 360 and aircraft is right around North (0°..10°), display as 360°..370° so it doesn't flip to 0.0
            if abs(heading_tgt - 360.0) < 0.5 and heading_act < 15.0:
                disp_hdg = 360.0 + heading_act
            self.lbl_hdg_act.configure(text=f"Heading: {disp_hdg:.1f}°{pause_suffix}")
            self.lbl_hdg_err.configure(text=f"Heading Err: {heading_err:+.1f}°")
            self.lbl_hdg_target_info.configure(text=f"Target: {heading_tgt:.1f}°")
            if hasattr(self, 'lbl_hdg_bank_info'):
                self.lbl_hdg_bank_info.configure(text=f"Demanded Bank: {cmd_bank:+.2f}°")

        self.log_badge.configure(text=('CSV: saving' if self.log_file else 'CSV: idle'),
                                 bg='#238636' if self.log_file else '#484f58')
        button_state = (p_on, b_on, y_on, ias_tgt, heading_tgt, self.active_tab_index, self.snap_bank_phase)
        if button_state != getattr(self, '_last_button_state', None):
            self._last_button_state = button_state
            self._update_engage_buttons()

        if is_paused:
            if self.running:
                try:
                    if self.root.winfo_exists():
                        self.after_id = self.root.after(100, self.gui_tick)
                except Exception:
                    pass
            return

        # 5. Append to Rolling Graph History
        elapsed = time.monotonic() - self.start_time
        self.time_history.append(elapsed)
        self.pitch_actual_hist.append(pitch_act)
        self.pitch_target_hist.append(pitch_tgt)
        self.pitch_error_hist.append(pitch_err)
        self.ias_actual_hist.append(ias_act)
        self.ias_target_hist.append(ias_tgt)
        self.ias_error_hist.append(ias_err)
        self.brunner_pitch_hist.append(b_pitch_pos)
        self.brunner_pitch_cmd_hist.append(self.brunner_pitch_cmd)

        self.bank_actual_hist.append(bank_act)
        self.bank_target_hist.append(bank_tgt)
        self.bank_error_hist.append(bank_err)
        self.brunner_roll_hist.append(b_roll_pos)
        self.brunner_roll_cmd_hist.append(self.brunner_roll_cmd)

        # For plotting: prevent 360/0 wrap cliff when target is near 360
        plot_hdg_act = heading_act
        if heading_tgt >= 300.0 and heading_act < 60.0:
            plot_hdg_act = 360.0 + heading_act
        elif heading_tgt <= 60.0 and heading_act > 300.0:
            plot_hdg_act = heading_act - 360.0

        self.heading_actual_hist.append(plot_hdg_act)
        self.heading_target_hist.append(heading_tgt)
        self.heading_error_hist.append(heading_err)
        self.demanded_bank_hist.append(cmd_bank)
        self.sideslip_hist.append(sideslip_act)
        self.yaw_rate_hist.append(yaw_rate_act)
        self.brunner_yaw_hist.append(b_yaw_pos)
        self.brunner_yaw_cmd_hist.append(self.brunner_yaw_cmd)

        # 6. Redraw Active Tab's Matplotlib Plot
        if len(self.time_history) > 1:
            t_list = list(self.time_history)
            t_max = max(30.0, (math.floor(t_list[-1] / 5.0) + 1) * 5.0)
            t_min = max(0.0, t_max - 30.0)

            if self.active_tab_index == 0:
                # Tab 1: Airspeed & Pitch Plot
                self.line_actual.set_data(t_list, list(self.pitch_actual_hist))
                self.line_target.set_data(t_list, list(self.pitch_target_hist))
                self.line_error.set_data(t_list, list(self.pitch_error_hist))

                self.line_ias_act.set_data(t_list, list(self.ias_actual_hist))
                self.line_ias_tgt.set_data(t_list, list(self.ias_target_hist))
                self.line_ias_err.set_data(t_list, list(self.ias_error_hist))

                self.line_brunner.set_data(t_list, list(self.brunner_pitch_hist))
                self.line_brunner_cmd.set_data(t_list, list(self.brunner_pitch_cmd_hist))

                self._plot_limits(self.ax1, x=(t_min, t_max))
                self._plot_limits(self.ax1, y=(-15.0, 15.0))

                spd_min = min(min(self.ias_actual_hist), min(self.ias_target_hist), min(self.ias_error_hist))
                spd_max = max(max(self.ias_actual_hist), max(self.ias_target_hist), max(self.ias_error_hist))
                spd_margin = max(8.0, (spd_max - spd_min) * 0.15)
                self._plot_limits(self.ax2, y=(max(-20.0, math.floor((spd_min - spd_margin)/10)*10), math.ceil((spd_max + spd_margin)/10)*10))
                self._plot_limits(self.ax3, y=(-1.15, 1.15))

                if self.running and self.root.winfo_exists():
                    try:
                        self._draw_live_plot(self.canvas1)
                    except Exception:
                        pass

            if self.active_tab_index == 0:
                # Tab 2: Bank Angle & Brunner Roll Plot
                self.line_bank_act.set_data(t_list, list(self.bank_actual_hist))
                self.line_bank_tgt.set_data(t_list, list(self.bank_target_hist))
                self.line_bank_err.set_data(t_list, list(self.bank_error_hist))

                self.line_b_roll_meas.set_data(t_list, list(self.brunner_roll_hist))
                self.line_b_roll_cmd.set_data(t_list, list(self.brunner_roll_cmd_hist))

                self._plot_limits(self.bank_ax1, x=(t_min, t_max))
                self._plot_limits(self.bank_ax1, y=(-15.0, 15.0))
                self._plot_limits(self.bank_ax2, x=(t_min, t_max))
                self._plot_limits(self.bank_ax2, y=(-0.85, 0.85))

                if self.running and self.root.winfo_exists():
                    try:
                        self._draw_live_plot(self.canvas2)
                    except Exception:
                        pass

            if self.active_tab_index == 0:
                # Tab 3: Heading Angle & Brunner Motor Plot
                self.line_hdg_act.set_data(t_list, list(self.heading_actual_hist))
                self.line_hdg_tgt.set_data(t_list, list(self.heading_target_hist))
                self.line_hdg_err.set_data(t_list, list(self.heading_error_hist))

                self.line_b_yaw_meas.set_data(t_list, list(self.brunner_yaw_hist))
                self.line_b_yaw_cmd.set_data(t_list, list(self.brunner_yaw_cmd_hist))
                self.line_b_roll_cmd_tab3.set_data(t_list, list(self.brunner_roll_cmd_hist))

                self._plot_limits(self.yaw_ax1, x=(t_min, t_max))
                max_hdg_disp = max(375.0, self.target_heading + 15.0)
                min_hdg_disp = min(-5.0, self.target_heading - 15.0)
                self._plot_limits(self.yaw_ax1, y=(min_hdg_disp, max_hdg_disp))
                self._plot_limits(self.yaw_ax2, x=(t_min, t_max))
                self._plot_limits(self.yaw_ax2, y=(-0.75, 0.75))

                if self.running and self.root.winfo_exists():
                    try:
                        self._draw_live_plot(self.canvas3)
                    except Exception:
                        pass

        # Tab 4: BRUNNER Diagnostics real-time update
        if self.active_tab_index == 3:
            self._update_tab_diagnostics()

        if self.running:
            try:
                if self.root.winfo_exists():
                    self.after_id = self.root.after(100, self.gui_tick)
            except Exception:
                pass

    # ----------------- Control Event Handlers (Pitch, Bank & Yaw Separated) -----------------
    def toggle_pitch(self):
        with self.lock:
            self.pitch_enabled = not self.pitch_enabled
            enabled = self.pitch_enabled
            other_enabled = self.bank_enabled or self.yaw_enabled

        if enabled:
            with self.lock:
                self.last_fault = ''
            self.failure_start = None
            if not other_enabled:
                try:
                    self._start_log()
                except Exception as exc:
                    with self.lock:
                        self.pitch_enabled = False
                        self.last_fault = 'Cannot open PID CSV: ' + str(exc)
                    self._update_engage_buttons()
                    return

            with self.lock:
                curr_pitch = self.snap_pitch_actual
            self.ias_ctrl.reset(initial_pitch=curr_pitch)
            curr_pitch_pos = self.cls.latest_pitch
            self.brunner_pitch_cmd = curr_pitch_pos
            self.brunner_pitch_vel = 0.0

            try:
                self.cls.engage_pitch(curr_pitch_pos)
            except Exception as exc:
                print(f"[Notice] Pitch engage will complete in background worker: {exc}")

            self.pid.reset(initial_pos=curr_pitch_pos)
            self.bridge.set_mode(3)
            self.bridge.enable(True)
        else:
            self.cls.release_pitch()
            if not other_enabled:
                self.bridge.enable(False)
                self._stop_log()

        self._update_engage_buttons()

    def toggle_bank(self):
        with self.lock:
            self.bank_enabled = not self.bank_enabled
            enabled = self.bank_enabled
            other_enabled = self.pitch_enabled or self.yaw_enabled

        if enabled:
            with self.lock:
                self.last_fault = ''
            self.failure_start = None
            if not other_enabled:
                try:
                    self._start_log()
                except Exception as exc:
                    with self.lock:
                        self.bank_enabled = False
                        self.last_fault = 'Cannot open PID CSV: ' + str(exc)
                    self._update_engage_buttons()
                    return

            if not self.yaw_enabled:
                curr_roll_pos = self.cls.latest_roll
                self.brunner_roll_cmd = curr_roll_pos
                self.brunner_roll_vel = 0.0

                try:
                    self.cls.engage_roll(curr_roll_pos)
                except Exception as exc:
                    print(f"[Notice] Roll engage will complete in background worker: {exc}")

                self.bank_pid.reset(initial_output=curr_roll_pos)
            self.bridge.set_mode(3)
            self.bridge.enable(True)
        else:
            if not self.yaw_enabled:
                self.cls.release_roll()
            if not other_enabled:
                self.bridge.enable(False)
                self._stop_log()

        self._update_engage_buttons()

    def toggle_yaw(self):
        with self.lock:
            self.yaw_enabled = not self.yaw_enabled
            enabled = self.yaw_enabled
            other_enabled = self.pitch_enabled or self.bank_enabled

        if enabled:
            with self.lock:
                self.last_fault = ''
            self.failure_start = None
            if not other_enabled:
                try:
                    self._start_log()
                except Exception as exc:
                    with self.lock:
                        self.yaw_enabled = False
                        self.last_fault = 'Cannot open PID CSV: ' + str(exc)
                    self._update_engage_buttons()
                    return

            # 1. Rudder/Yaw axis initialization (AX_RUDDER)
            curr_yaw_pos = self.cls.latest_yaw
            self.brunner_yaw_cmd = curr_yaw_pos
            self.brunner_yaw_vel = 0.0
            self.yaw_pid.reset(initial_output=curr_yaw_pos)
            try:
                self.cls.engage_yaw(curr_yaw_pos)
            except Exception as exc:
                print(f"[Notice] Yaw engage: {exc}")

            # 2. Roll axis initialization for Heading guidance turn
            curr_roll_pos = self.cls.latest_roll
            self.brunner_roll_cmd = curr_roll_pos
            self.brunner_roll_vel = 0.0
            self.bank_pid.reset(initial_output=curr_roll_pos)
            try:
                self.cls.engage_roll(curr_roll_pos)
            except Exception as exc:
                print(f"[Notice] Roll engage will complete in background worker: {exc}")

            with self.lock:
                curr_bank = self.snap_bank_actual
            self.heading_ctrl.reset(initial_bank=curr_bank)
            self.bridge.set_mode(3)
            self.bridge.enable(True)
        else:
            if not self.bank_enabled:
                self.cls.release_roll()
            self.cls.release_yaw()
            self.brunner_yaw_cmd = 0.0
            self.brunner_yaw_vel = 0.0
            if not other_enabled:
                self.bridge.enable(False)
                self._stop_log()

        self._update_engage_buttons()

    def disarm_all(self):
        with self.lock:
            self.pitch_enabled = False
            self.bank_enabled = False
            self.yaw_enabled = False
        try:
            self.cls.release()
            self.bridge.enable(False)
        except Exception:
            pass
        self._update_engage_buttons()
        self._stop_log()

    def toggle_pid(self):
        """Header button: toggles whichever controller corresponds to the active tab."""
        try:
            tab_idx = self.notebook.index(self.notebook.select())
        except Exception:
            tab_idx = self.active_tab_index
        if tab_idx == 0:
            self.toggle_pitch()
        elif tab_idx == 1:
            self.toggle_bank()
        elif tab_idx == 2:
            self.toggle_yaw()
        else:
            # Tab 4: BRUNNER Diagnostics is strictly read-only
            pass

    def _update_engage_buttons(self):
        with self.lock:
            p_on = self.pitch_enabled
            b_on = self.bank_enabled
            y_on = self.yaw_enabled
            ias_tgt = self.snap_ias_target
            hdg_tgt = self.snap_heading_target

        try:
            tab_idx = self.notebook.index(self.notebook.select())
        except Exception:
            tab_idx = self.active_tab_index

        # Update Tab-specific toggle buttons if present
        if hasattr(self, 'btn_pitch_toggle'):
            self.btn_pitch_toggle.configure(
                text="STOP PITCH HOLD" if p_on else "START AIRSPEED & PITCH HOLD",
                bg="#da3633" if p_on else "#238636"
            )
        if hasattr(self, 'btn_bank_toggle'):
            waiting = b_on and y_on and self.snap_bank_phase != "WINGS LEVEL"
            self.btn_bank_toggle.configure(
                text=("BANK ARMED / WAITING - STOP" if waiting
                      else "WINGS LEVEL ACTIVE - STOP" if b_on and y_on
                      else "STOP BANK HOLD" if b_on else "START BANK HOLD"),
                bg="#9e6a03" if waiting else "#da3633" if b_on else "#238636")
        if hasattr(self, 'btn_yaw_toggle'):
            self.btn_yaw_toggle.configure(
                text="STOP HEADING & YAW" if y_on else "START HEADING & YAW HOLD",
                bg="#da3633" if y_on else "#238636"
            )

        if hasattr(self, 'engage_btn'):
            if tab_idx == 0:
                self.engage_btn.configure(
                    text="STOP PITCH HOLD" if p_on else "START PITCH HOLD",
                    bg="#da3633" if p_on else "#238636", fg="#ffffff", state="normal"
                )
            elif tab_idx == 1:
                self.engage_btn.configure(
                    text="STOP BANK HOLD (0.0°)" if b_on else "START BANK HOLD (0.0°)",
                    bg="#da3633" if b_on else "#238636", fg="#ffffff", state="normal"
                )
            elif tab_idx == 2:
                self.engage_btn.configure(
                    text="STOP HEADING & YAW" if y_on else "START HEADING & YAW",
                    bg="#da3633" if y_on else "#238636", fg="#ffffff", state="normal"
                )
            else:
                self.engage_btn.configure(
                    text="DIAGNOSTICS (READ ONLY)",
                    bg="#30363d", fg="#8b949e", state="disabled"
                )

        if hasattr(self, 'mode_badge'):
            active_list = []
            if p_on:
                active_list.append(f"Pitch ({ias_tgt:.0f}kt)")
            if b_on:
                active_list.append("Bank: " + self.snap_bank_phase)
            if y_on:
                active_list.append(f"Hdg ({hdg_tgt:.0f}°)")

            if active_list:
                self.mode_badge.configure(text="ACTIVE: " + " + ".join(active_list), bg="#238636", fg="#ffffff")
            else:
                self.mode_badge.configure(text="CONTROLLERS: IDLE (100% Manual Cockpit Control)", bg="#484f58", fg="#f0f6fc")

    # ----------------- Tab 1: Airspeed Handlers -----------------
    def on_target_ias_keyrelease(self, event=None):
        try:
            txt = self.target_ias_entry.get().strip()
            if not txt:
                return
            val = float(txt)
            with self.lock:
                self.target_ias = val
            if val > self.target_ias_slider['to']:
                self.target_ias_slider.configure(to=val * 1.25)
            if val < self.target_ias_slider['from']:
                self.target_ias_slider.configure(from_=val)
            self._editing_entry = True
            try:
                self.target_ias_var.set(val)
            finally:
                self._editing_entry = False
        except ValueError:
            pass

    def on_target_ias_entry(self, event=None):
        self.on_target_ias_keyrelease(event)
        try:
            val = float(self.target_ias_entry.get().strip())
            self.target_ias_entry.delete(0, tk.END)
            self.target_ias_entry.insert(0, f"{val:.0f}" if val.is_integer() else f"{val:.1f}")
        except ValueError:
            pass

    def on_target_ias_change(self, val):
        if getattr(self, '_editing_entry', False):
            return
        try:
            v = float(val)
            with self.lock:
                self.target_ias = v
            self.target_ias_entry.delete(0, tk.END)
            self.target_ias_entry.insert(0, f"{v:.0f}" if v.is_integer() else f"{v:.1f}")
        except ValueError:
            pass

    def set_target_ias(self, val: float):
        v = float(val)
        with self.lock:
            self.target_ias = v
        if v > self.target_ias_slider['to']:
            self.target_ias_slider.configure(to=v * 1.25)
        if v < self.target_ias_slider['from']:
            self.target_ias_slider.configure(from_=v)
        self.target_ias_var.set(v)
        self.target_ias_entry.delete(0, tk.END)
        self.target_ias_entry.insert(0, f"{v:.0f}" if v.is_integer() else f"{v:.1f}")

    def on_ias_pid_change(self, event=None):
        if getattr(self, '_editing_entry', False):
            return
        kp = float(self.ias_kp_var.get())
        ki = float(self.ias_ki_var.get())
        kd = float(self.ias_kd_var.get())
        if kp >= self.ias_kp_scale['to'] * 0.95:
            self.ias_kp_scale.configure(to=self.ias_kp_scale['to'] * 1.5)
        if ki >= self.ias_ki_scale['to'] * 0.95:
            self.ias_ki_scale.configure(to=self.ias_ki_scale['to'] * 1.5)
        if kd >= self.ias_kd_scale['to'] * 0.95:
            self.ias_kd_scale.configure(to=self.ias_kd_scale['to'] * 1.5)
        with self.lock:
            self.ias_ctrl.kp = kp
            self.ias_ctrl.ki = ki
            self.ias_ctrl.kd = kd
        if hasattr(self, 'ias_kp_entry'):
            self.ias_kp_entry.delete(0, tk.END)
            self.ias_kp_entry.insert(0, f"{kp:.4f}".rstrip('0').rstrip('.'))
            self.ias_ki_entry.delete(0, tk.END)
            self.ias_ki_entry.insert(0, f"{ki:.5f}".rstrip('0').rstrip('.'))
            self.ias_kd_entry.delete(0, tk.END)
            self.ias_kd_entry.insert(0, f"{kd:.4f}".rstrip('0').rstrip('.'))

    def on_ias_kp_keyrelease(self, event=None):
        try:
            txt = self.ias_kp_entry.get().strip()
            if not txt:
                return
            val = float(txt)
            with self.lock:
                self.ias_ctrl.kp = val
            if val > self.ias_kp_scale['to']:
                self.ias_kp_scale.configure(to=val * 1.5)
            if val < self.ias_kp_scale['from']:
                self.ias_kp_scale.configure(from_=val)
            self._editing_entry = True
            try:
                self.ias_kp_var.set(val)
            finally:
                self._editing_entry = False
        except ValueError:
            pass

    def on_ias_kp_entry(self, event=None):
        self.on_ias_kp_keyrelease(event)
        try:
            val = float(self.ias_kp_entry.get().strip())
            self.ias_kp_entry.delete(0, tk.END)
            self.ias_kp_entry.insert(0, f"{val:.4f}".rstrip('0').rstrip('.'))
        except ValueError:
            pass

    def on_ias_ki_keyrelease(self, event=None):
        try:
            txt = self.ias_ki_entry.get().strip()
            if not txt:
                return
            val = float(txt)
            with self.lock:
                self.ias_ctrl.ki = val
            if val > self.ias_ki_scale['to']:
                self.ias_ki_scale.configure(to=val * 1.5)
            if val < self.ias_ki_scale['from']:
                self.ias_ki_scale.configure(from_=val)
            self._editing_entry = True
            try:
                self.ias_ki_var.set(val)
            finally:
                self._editing_entry = False
        except ValueError:
            pass

    def on_ias_ki_entry(self, event=None):
        self.on_ias_ki_keyrelease(event)
        try:
            val = float(self.ias_ki_entry.get().strip())
            self.ias_ki_entry.delete(0, tk.END)
            self.ias_ki_entry.insert(0, f"{val:.5f}".rstrip('0').rstrip('.'))
        except ValueError:
            pass

    def on_ias_kd_keyrelease(self, event=None):
        try:
            txt = self.ias_kd_entry.get().strip()
            if not txt:
                return
            val = float(txt)
            with self.lock:
                self.ias_ctrl.kd = val
            if val > self.ias_kd_scale['to']:
                self.ias_kd_scale.configure(to=val * 1.5)
            if val < self.ias_kd_scale['from']:
                self.ias_kd_scale.configure(from_=val)
            self._editing_entry = True
            try:
                self.ias_kd_var.set(val)
            finally:
                self._editing_entry = False
        except ValueError:
            pass

    def on_ias_kd_entry(self, event=None):
        self.on_ias_kd_keyrelease(event)
        try:
            val = float(self.ias_kd_entry.get().strip())
            self.ias_kd_entry.delete(0, tk.END)
            self.ias_kd_entry.insert(0, f"{val:.4f}".rstrip('0').rstrip('.'))
        except ValueError:
            pass

    def on_invert_change(self):
        inv = bool(self.invert_var.get())
        with self.lock:
            self.pid.invert = inv
            self.pid.integral = 0.0

    # ----------------- Tab 2: Bank Angle Handlers -----------------
    def on_target_bank_keyrelease(self, event=None):
        try:
            txt = self.target_bank_entry.get().strip()
            if not txt:
                return
            val = float(txt)
            with self.lock:
                self.target_bank = val
            lim = max(abs(val) * 1.25, 20.0)
            if abs(val) > abs(self.target_bank_slider['to']):
                self.target_bank_slider.configure(from_=-lim, to=lim)
            self._editing_entry = True
            try:
                self.target_bank_var.set(val)
            finally:
                self._editing_entry = False
        except ValueError:
            pass

    def on_target_bank_entry(self, event=None):
        self.on_target_bank_keyrelease(event)
        try:
            val = float(self.target_bank_entry.get().strip())
            self.target_bank_entry.delete(0, tk.END)
            self.target_bank_entry.insert(0, f"{val:.2f}")
        except ValueError:
            pass

    def on_target_bank_change(self, val):
        if getattr(self, '_editing_entry', False):
            return
        try:
            v = float(val)
            with self.lock:
                self.target_bank = v
            self.target_bank_entry.delete(0, tk.END)
            self.target_bank_entry.insert(0, f"{v:.2f}")
        except ValueError:
            pass

    def set_target_bank(self, val: float):
        v = float(val)
        with self.lock:
            self.target_bank = v
        lim = max(abs(v) * 1.25, 20.0)
        if abs(v) > abs(self.target_bank_slider['to']):
            self.target_bank_slider.configure(from_=-lim, to=lim)
        self.target_bank_var.set(v)
        self.target_bank_entry.delete(0, tk.END)
        self.target_bank_entry.insert(0, f"{v:.2f}")

    def on_bank_pid_change(self, event=None):
        if getattr(self, '_editing_entry', False):
            return
        kp = float(self.bank_kp_var.get())
        ki = float(self.bank_ki_var.get())
        kd = float(self.bank_kd_var.get())
        if kp >= self.bank_kp_scale['to'] * 0.95:
            self.bank_kp_scale.configure(to=self.bank_kp_scale['to'] * 1.5)
        if ki >= self.bank_ki_scale['to'] * 0.95:
            self.bank_ki_scale.configure(to=self.bank_ki_scale['to'] * 1.5)
        if kd >= self.bank_kd_scale['to'] * 0.95:
            self.bank_kd_scale.configure(to=self.bank_kd_scale['to'] * 1.5)
        with self.lock:
            self.bank_pid.kp = kp
            self.bank_pid.ki = ki
            self.bank_pid.kd = kd
        if hasattr(self, 'bank_kp_entry'):
            self.bank_kp_entry.delete(0, tk.END)
            self.bank_kp_entry.insert(0, f"{kp:.4f}".rstrip('0').rstrip('.'))
            self.bank_ki_entry.delete(0, tk.END)
            self.bank_ki_entry.insert(0, f"{ki:.5f}".rstrip('0').rstrip('.'))
            self.bank_kd_entry.delete(0, tk.END)
            self.bank_kd_entry.insert(0, f"{kd:.4f}".rstrip('0').rstrip('.'))

    def on_bank_kp_keyrelease(self, event=None):
        try:
            txt = self.bank_kp_entry.get().strip()
            if not txt:
                return
            val = float(txt)
            with self.lock:
                self.bank_pid.kp = val
            if val > self.bank_kp_scale['to']:
                self.bank_kp_scale.configure(to=val * 1.5)
            if val < self.bank_kp_scale['from']:
                self.bank_kp_scale.configure(from_=val)
            self._editing_entry = True
            try:
                self.bank_kp_var.set(val)
            finally:
                self._editing_entry = False
        except ValueError:
            pass

    def on_bank_kp_entry(self, event=None):
        self.on_bank_kp_keyrelease(event)
        try:
            val = float(self.bank_kp_entry.get().strip())
            self.bank_kp_entry.delete(0, tk.END)
            self.bank_kp_entry.insert(0, f"{val:.4f}".rstrip('0').rstrip('.'))
        except ValueError:
            pass

    def on_bank_ki_keyrelease(self, event=None):
        try:
            txt = self.bank_ki_entry.get().strip()
            if not txt:
                return
            val = float(txt)
            with self.lock:
                self.bank_pid.ki = val
            if val > self.bank_ki_scale['to']:
                self.bank_ki_scale.configure(to=val * 1.5)
            if val < self.bank_ki_scale['from']:
                self.bank_ki_scale.configure(from_=val)
            self._editing_entry = True
            try:
                self.bank_ki_var.set(val)
            finally:
                self._editing_entry = False
        except ValueError:
            pass

    def on_bank_ki_entry(self, event=None):
        self.on_bank_ki_keyrelease(event)
        try:
            val = float(self.bank_ki_entry.get().strip())
            self.bank_ki_entry.delete(0, tk.END)
            self.bank_ki_entry.insert(0, f"{val:.5f}".rstrip('0').rstrip('.'))
        except ValueError:
            pass

    def on_bank_kd_keyrelease(self, event=None):
        try:
            txt = self.bank_kd_entry.get().strip()
            if not txt:
                return
            val = float(txt)
            with self.lock:
                self.bank_pid.kd = val
            if val > self.bank_kd_scale['to']:
                self.bank_kd_scale.configure(to=val * 1.5)
            if val < self.bank_kd_scale['from']:
                self.bank_kd_scale.configure(from_=val)
            self._editing_entry = True
            try:
                self.bank_kd_var.set(val)
            finally:
                self._editing_entry = False
        except ValueError:
            pass

    def on_bank_kd_entry(self, event=None):
        self.on_bank_kd_keyrelease(event)
        try:
            val = float(self.bank_kd_entry.get().strip())
            self.bank_kd_entry.delete(0, tk.END)
            self.bank_kd_entry.insert(0, f"{val:.4f}".rstrip('0').rstrip('.'))
        except ValueError:
            pass

    def on_bank_invert_change(self):
        inv = bool(self.bank_invert_var.get())
        with self.lock:
            self.bank_pid.invert = inv
            self.bank_pid.integral = 0.0

    # ----------------- Tab 3: Heading & Yaw Handlers -----------------
    def on_target_hdg_keyrelease(self, event=None):
        try:
            txt = self.target_hdg_entry.get().strip()
            if not txt:
                return
            val = float(txt)
            with self.lock:
                self.target_heading = val
            if val > self.target_hdg_slider['to']:
                self.target_hdg_slider.configure(to=max(500.0, val * 1.25))
            if val < self.target_hdg_slider['from']:
                self.target_hdg_slider.configure(from_=val)
            self._editing_entry = True
            try:
                self.target_hdg_var.set(val)
            finally:
                self._editing_entry = False
        except ValueError:
            pass

    def on_target_hdg_entry(self, event=None):
        self.on_target_hdg_keyrelease(event)
        try:
            val = float(self.target_hdg_entry.get().strip())
            self.target_hdg_entry.delete(0, tk.END)
            self.target_hdg_entry.insert(0, f"{val:.0f}" if val.is_integer() else f"{val:.1f}")
        except ValueError:
            pass

    def on_target_hdg_change(self, val):
        if getattr(self, '_editing_entry', False):
            return
        try:
            v = float(val)
            if v >= self.target_hdg_slider['to'] * 0.95:
                self.target_hdg_slider.configure(to=self.target_hdg_slider['to'] + 100.0)
            with self.lock:
                self.target_heading = v
            self.target_hdg_entry.delete(0, tk.END)
            self.target_hdg_entry.insert(0, f"{v:.0f}" if v.is_integer() else f"{v:.1f}")
        except ValueError:
            pass

    def set_target_hdg(self, val: float):
        v = float(val)
        with self.lock:
            self.target_heading = v
        if v > self.target_hdg_slider['to']:
            self.target_hdg_slider.configure(to=max(500.0, v * 1.25))
        if v < self.target_hdg_slider['from']:
            self.target_hdg_slider.configure(from_=v)
        self.target_hdg_var.set(v)
        self.target_hdg_entry.delete(0, tk.END)
        self.target_hdg_entry.insert(0, f"{v:.0f}" if v.is_integer() else f"{v:.1f}")

    def on_hdg_pid_change(self, event=None):
        if getattr(self, '_editing_entry', False):
            return
        kp = float(self.hdg_kp_var.get())
        ki = float(self.hdg_ki_var.get())
        kd = float(self.hdg_kd_var.get())
        if kp >= self.hdg_kp_scale['to'] * 0.95:
            self.hdg_kp_scale.configure(to=self.hdg_kp_scale['to'] * 1.5)
        if ki >= self.hdg_ki_scale['to'] * 0.95:
            self.hdg_ki_scale.configure(to=self.hdg_ki_scale['to'] * 1.5)
        if kd >= self.hdg_kd_scale['to'] * 0.95:
            self.hdg_kd_scale.configure(to=self.hdg_kd_scale['to'] * 1.5)
        with self.lock:
            self.heading_ctrl.kp = kp
            self.heading_ctrl.ki = ki
            self.heading_ctrl.kd = kd
        if hasattr(self, 'hdg_kp_entry'):
            self.hdg_kp_entry.delete(0, tk.END)
            self.hdg_kp_entry.insert(0, f"{kp:.4f}".rstrip('0').rstrip('.'))
            self.hdg_ki_entry.delete(0, tk.END)
            self.hdg_ki_entry.insert(0, f"{ki:.5f}".rstrip('0').rstrip('.'))
            self.hdg_kd_entry.delete(0, tk.END)
            self.hdg_kd_entry.insert(0, f"{kd:.4f}".rstrip('0').rstrip('.'))

    def on_hdg_kp_keyrelease(self, event=None):
        try:
            txt = self.hdg_kp_entry.get().strip()
            if not txt:
                return
            val = float(txt)
            with self.lock:
                self.heading_ctrl.kp = val
            if val > self.hdg_kp_scale['to']:
                self.hdg_kp_scale.configure(to=val * 1.5)
            if val < self.hdg_kp_scale['from']:
                self.hdg_kp_scale.configure(from_=val)
            self._editing_entry = True
            try:
                self.hdg_kp_var.set(val)
            finally:
                self._editing_entry = False
        except ValueError:
            pass

    def on_hdg_kp_entry(self, event=None):
        self.on_hdg_kp_keyrelease(event)
        try:
            val = float(self.hdg_kp_entry.get().strip())
            self.hdg_kp_entry.delete(0, tk.END)
            self.hdg_kp_entry.insert(0, f"{val:.4f}".rstrip('0').rstrip('.'))
        except ValueError:
            pass

    def on_hdg_ki_keyrelease(self, event=None):
        try:
            txt = self.hdg_ki_entry.get().strip()
            if not txt:
                return
            val = float(txt)
            with self.lock:
                self.heading_ctrl.ki = val
            if val > self.hdg_ki_scale['to']:
                self.hdg_ki_scale.configure(to=val * 1.5)
            if val < self.hdg_ki_scale['from']:
                self.hdg_ki_scale.configure(from_=val)
            self._editing_entry = True
            try:
                self.hdg_ki_var.set(val)
            finally:
                self._editing_entry = False
        except ValueError:
            pass

    def on_hdg_ki_entry(self, event=None):
        self.on_hdg_ki_keyrelease(event)
        try:
            val = float(self.hdg_ki_entry.get().strip())
            self.hdg_ki_entry.delete(0, tk.END)
            self.hdg_ki_entry.insert(0, f"{val:.5f}".rstrip('0').rstrip('.'))
        except ValueError:
            pass

    def on_hdg_kd_keyrelease(self, event=None):
        try:
            txt = self.hdg_kd_entry.get().strip()
            if not txt:
                return
            val = float(txt)
            with self.lock:
                self.heading_ctrl.kd = val
            if val > self.hdg_kd_scale['to']:
                self.hdg_kd_scale.configure(to=val * 1.5)
            if val < self.hdg_kd_scale['from']:
                self.hdg_kd_scale.configure(from_=val)
            self._editing_entry = True
            try:
                self.hdg_kd_var.set(val)
            finally:
                self._editing_entry = False
        except ValueError:
            pass

    def on_hdg_kd_entry(self, event=None):
        self.on_hdg_kd_keyrelease(event)
        try:
            val = float(self.hdg_kd_entry.get().strip())
            self.hdg_kd_entry.delete(0, tk.END)
            self.hdg_kd_entry.insert(0, f"{val:.4f}".rstrip('0').rstrip('.'))
        except ValueError:
            pass

    # Dedicated Yaw / Rudder PID Handlers (Completely separated from Heading PID)
    def on_yaw_pid_change(self, event=None):
        if getattr(self, '_editing_entry', False):
            return
        kp = float(self.yaw_kp_var.get())
        ki = float(self.yaw_ki_var.get())
        kd = float(self.yaw_kd_var.get())
        if hasattr(self, 'yaw_kp_scale') and kp >= self.yaw_kp_scale['to'] * 0.95:
            self.yaw_kp_scale.configure(to=self.yaw_kp_scale['to'] * 1.5)
        if hasattr(self, 'yaw_ki_scale') and ki >= self.yaw_ki_scale['to'] * 0.95:
            self.yaw_ki_scale.configure(to=self.yaw_ki_scale['to'] * 1.5)
        if hasattr(self, 'yaw_kd_scale') and kd >= self.yaw_kd_scale['to'] * 0.95:
            self.yaw_kd_scale.configure(to=self.yaw_kd_scale['to'] * 1.5)
        with self.lock:
            self.yaw_pid.kp = kp
            self.yaw_pid.ki = ki
            self.yaw_pid.kd = kd
        if hasattr(self, 'yaw_kp_entry'):
            self.yaw_kp_entry.delete(0, tk.END)
            self.yaw_kp_entry.insert(0, f"{kp:.4f}".rstrip('0').rstrip('.'))
            self.yaw_ki_entry.delete(0, tk.END)
            self.yaw_ki_entry.insert(0, f"{ki:.5f}".rstrip('0').rstrip('.'))
            self.yaw_kd_entry.delete(0, tk.END)
            self.yaw_kd_entry.insert(0, f"{kd:.4f}".rstrip('0').rstrip('.'))

    def on_yaw_kp_keyrelease(self, event=None):
        try:
            txt = self.yaw_kp_entry.get().strip()
            if not txt:
                return
            val = float(txt)
            with self.lock:
                self.yaw_pid.kp = val
            if hasattr(self, 'yaw_kp_scale'):
                if val > self.yaw_kp_scale['to']:
                    self.yaw_kp_scale.configure(to=val * 1.5)
                if val < self.yaw_kp_scale['from']:
                    self.yaw_kp_scale.configure(from_=val)
            self._editing_entry = True
            try:
                self.yaw_kp_var.set(val)
            finally:
                self._editing_entry = False
        except ValueError:
            pass

    def on_yaw_kp_entry(self, event=None):
        self.on_yaw_kp_keyrelease(event)
        try:
            val = float(self.yaw_kp_entry.get().strip())
            self.yaw_kp_entry.delete(0, tk.END)
            self.yaw_kp_entry.insert(0, f"{val:.4f}".rstrip('0').rstrip('.'))
        except ValueError:
            pass

    def on_yaw_ki_keyrelease(self, event=None):
        try:
            txt = self.yaw_ki_entry.get().strip()
            if not txt:
                return
            val = float(txt)
            with self.lock:
                self.yaw_pid.ki = val
            if hasattr(self, 'yaw_ki_scale'):
                if val > self.yaw_ki_scale['to']:
                    self.yaw_ki_scale.configure(to=val * 1.5)
                if val < self.yaw_ki_scale['from']:
                    self.yaw_ki_scale.configure(from_=val)
            self._editing_entry = True
            try:
                self.yaw_ki_var.set(val)
            finally:
                self._editing_entry = False
        except ValueError:
            pass

    def on_yaw_ki_entry(self, event=None):
        self.on_yaw_ki_keyrelease(event)
        try:
            val = float(self.yaw_ki_entry.get().strip())
            self.yaw_ki_entry.delete(0, tk.END)
            self.yaw_ki_entry.insert(0, f"{val:.5f}".rstrip('0').rstrip('.'))
        except ValueError:
            pass

    def on_yaw_kd_keyrelease(self, event=None):
        try:
            txt = self.yaw_kd_entry.get().strip()
            if not txt:
                return
            val = float(txt)
            with self.lock:
                self.yaw_pid.kd = val
            if hasattr(self, 'yaw_kd_scale'):
                if val > self.yaw_kd_scale['to']:
                    self.yaw_kd_scale.configure(to=val * 1.5)
                if val < self.yaw_kd_scale['from']:
                    self.yaw_kd_scale.configure(from_=val)
            self._editing_entry = True
            try:
                self.yaw_kd_var.set(val)
            finally:
                self._editing_entry = False
        except ValueError:
            pass

    def on_yaw_kd_entry(self, event=None):
        self.on_yaw_kd_keyrelease(event)
        try:
            val = float(self.yaw_kd_entry.get().strip())
            self.yaw_kd_entry.delete(0, tk.END)
            self.yaw_kd_entry.insert(0, f"{val:.4f}".rstrip('0').rstrip('.'))
        except ValueError:
            pass

    def on_heading_invert_change(self):
        inv = bool(self.heading_invert_var.get())
        with self.lock:
            self.heading_ctrl.invert = inv
            self.heading_ctrl.integral = 0.0

    def on_yaw_invert_change(self):
        inv = bool(self.yaw_invert_var.get())
        with self.lock:
            self.yaw_pid.invert = inv
            self.yaw_pid.integral = 0.0

    def on_close(self):
        self.running = False
        if self.after_id:
            try:
                self.root.after_cancel(self.after_id)
            except Exception:
                pass
            self.after_id = None
        if hasattr(self, 'worker_thread') and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=2.0)
        self._stop_log()
        self.cls.close()
        self.bridge.close()
        try:
            plt.close('all')
        except Exception:
            pass
        try:
            if self.root.winfo_exists():
                self.root.destroy()
        except Exception:
            pass


def main():
    root = tk.Tk()
    app = PitchPIDApp(root)
    try:
        root.mainloop()
    except (KeyboardInterrupt, SystemExit):
        app.on_close()


if __name__ == '__main__':
    main()
