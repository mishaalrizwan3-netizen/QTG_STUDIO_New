"""Offline Multi-PID integration tests. All simulator and socket objects are fakes."""
import csv
import math
from pathlib import Path
import struct
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'interfaces/brunner_base/src'))

from backend.bank_pid import BankPID, slew_position
from backend.yaw_pid import HeadingController, YawPID, HeadingBankHandoff
from backend.multi_pid_runner import (
    MultiPidStabilizationGate, loop, fresh_telemetry,
    PitchPID, AirspeedController
)
from interfaces.multi_pid_transport import MultiPidBrunnerTransport
from reports.multi_pid_report import report as generate_multi_pid_pdf_report
from brunner_interface import BrunnerInterface
import qtg_common_engine as engine


class FakeSocket:
    def __init__(self, rudder_3byte=False):
        self.blocking = True
        self.packets = []
        self.pitch_pos = 0.02
        self.roll_pos = 0.05
        self.rudder_pos = -0.02
        self.rudder_3byte = rudder_3byte
        self.reject = False

    def setblocking(self, value): self.blocking = value
    def settimeout(self, value): self.blocking = True
    def send(self, packet): self.packets.append(packet)

    def recv(self, size):
        if not self.blocking:
            raise BlockingIOError()
        packet = self.packets[-1]
        cmd = struct.unpack_from('<I', packet)[0]
        if cmd == 0xD0:  # read axis
            axis = struct.unpack_from('<I', packet, 4)[0]
            if axis == 4 and self.rudder_3byte:
                return b'\x01\x00\x00'
            pos = self.pitch_pos if axis == 1 else (self.roll_pos if axis == 2 else (self.rudder_pos if axis == 4 else 0.0))
            node_id = 5 if axis == 4 else (1 if axis == 2 else 2)
            return struct.pack('<HBHf', 7, 0, node_id, pos)
        if self.reject:
            return b'\x01\x00\x01'
        return b'\x01\x00\x00'


def make_fake_brunner(rudder_3byte=False):
    d = BrunnerInterface()
    d.sock = FakeSocket(rudder_3byte=rudder_3byte)
    d.read_axis = Mock(side_effect=lambda axis: (
        0.02 if axis == 1 else (0.05 if axis == 2 else (-0.02 if axis == 4 else 0.0)),
        1.0, 2.0
    ))
    return d


class FakeSim:
    def __init__(self, ias=80.0, hdg=270.0, bank=0.0, yaw_rate=0.0, sideslip=0.0):
        self.sequence = 0
        self.connected = True
        self.paused = False
        self.ias = ias
        self.hdg = hdg
        self.bank = bank
        self.yaw_rate = yaw_rate
        self.sideslip = sideslip

    def is_connected(self): return self.connected
    def get_telemetry_sequence(self): return self.sequence
    def request_telemetry_once(self): self.sequence += 1; return True
    def wait_for_fresh_telemetry(self, sequence, timeout_sec=1.0):
        return {
            'airspeed_indicated': self.ias,
            'magnetic_heading': self.hdg,
            'bank_angle': self.bank,
            'yaw_rate': self.yaw_rate,
            'sideslip': self.sideslip,
            'bridge_telemetry_sequence': self.sequence
        }
    def unpause_sim(self): self.paused = False
    def pause_sim(self): self.paused = True
    def simulation_running(self): return not self.paused


class TestMultiPidTransport(unittest.TestCase):
    def test_transport_axis_mask_all_three(self):
        brunner = make_fake_brunner()
        event = threading.Event()
        transport = MultiPidBrunnerTransport(brunner, event, pitch_enabled=True, bank_enabled=True, yaw_enabled=True)
        self.assertTrue(transport.pitch_enabled)
        self.assertTrue(transport.bank_enabled)
        self.assertTrue(transport.yaw_enabled)
        transport.engage()
        # Find override packets (length 12: cmd=0xD1, mask, setting_code): 0x01 | 0x02 | 0x04 = 0x07
        override_packets = [pkt for pkt in brunner.sock.packets if len(pkt) == 12 and struct.unpack_from('<I', pkt)[0] == 0xD1]
        self.assertTrue(len(override_packets) > 0)
        mask = struct.unpack_from('<I', override_packets[-1], 4)[0]
        self.assertEqual(mask, 0x07)

    def test_transport_pitch_isolated_when_airspeed_disabled(self):
        brunner = make_fake_brunner()
        event = threading.Event()
        # Only Bank and Yaw ticked -> Pitch must NOT be enabled or commanded
        transport = MultiPidBrunnerTransport(brunner, event, pitch_enabled=False, bank_enabled=True, yaw_enabled=True)
        self.assertFalse(transport.pitch_enabled)
        transport.engage()
        
        # Override mask should be 0x06 (0x02 | 0x04)
        override_packets = [pkt for pkt in brunner.sock.packets if len(pkt) == 12 and struct.unpack_from('<I', pkt)[0] == 0xD1]
        mask = struct.unpack_from('<I', override_packets[-1], 4)[0]
        self.assertEqual(mask, 0x06)

        # Clear packets and send command
        brunner.sock.packets.clear()
        transport.command(pitch_pos=0.5, roll_pos=0.1, rudder_pos=-0.1)
        # Check packets sent: only roll (axis 2) and rudder (axis 4)
        for pkt in brunner.sock.packets:
            if len(pkt) == 14 and struct.unpack_from('<I', pkt)[0] == 0xD1:  # motor position setting
                axis = struct.unpack_from('<B', pkt, 4)[0]
                self.assertIn(axis, [2, 4], "Pitch axis 1 should NEVER be commanded when pitch_enabled is False!")

    def test_transport_rudder_3byte_graceful(self):
        brunner = make_fake_brunner(rudder_3byte=True)
        event = threading.Event()
        transport = MultiPidBrunnerTransport(brunner, event, pitch_enabled=False, bank_enabled=True, yaw_enabled=True)
        # Should not raise exception and should detect rudder_available = False
        transport.engage()
        self.assertFalse(transport.rudder_available)
        # Override mask should have cleared rudder bit: only roll (0x02)
        override_packets = [pkt for pkt in brunner.sock.packets if len(pkt) == 12 and struct.unpack_from('<I', pkt)[0] == 0xD1]
        mask = struct.unpack_from('<I', override_packets[-1], 4)[0]
        self.assertEqual(mask, 0x02)


class TestMultiPidStabilizationGate(unittest.TestCase):
    def test_gate_all_axes_pass(self):
        gate = MultiPidStabilizationGate(pitch_enabled=True, bank_enabled=True, yaw_enabled=True, hold_s=5.0)
        
        # In tolerance: IAS 80.5 (tol 1.5), Bank 1.0 (tol 3.0), Heading 271.0 (tol 2.0)
        # First sample starts timer
        self.assertFalse(gate.update(10.0, 80.5, 1.0, 271.0, 80.0, 270.0))
        # After 4.9 seconds, still not ready
        self.assertFalse(gate.update(14.9, 80.2, 0.5, 270.5, 80.0, 270.0))
        # After 5.1 seconds, ready!
        self.assertTrue(gate.update(15.1, 80.0, 0.0, 270.0, 80.0, 270.0))

    def test_gate_disturbance_resets_timer(self):
        gate = MultiPidStabilizationGate(pitch_enabled=True, bank_enabled=True, yaw_enabled=True, hold_s=5.0)
        
        gate.update(10.0, 80.0, 0.0, 270.0, 80.0, 270.0)
        gate.update(14.0, 80.0, 0.0, 270.0, 80.0, 270.0)
        # Gust causes bank to exceed 3.0 deg
        self.assertFalse(gate.update(14.5, 80.0, 4.0, 270.0, 80.0, 270.0))
        self.assertIsNone(gate.since, "Timer should reset on disturbance")
        
        # After returning to tolerance, must wait another full 5.0 seconds
        gate.update(15.0, 80.0, 0.0, 270.0, 80.0, 270.0)
        self.assertFalse(gate.update(19.5, 80.0, 0.0, 270.0, 80.0, 270.0))
        self.assertTrue(gate.update(20.1, 80.0, 0.0, 270.0, 80.0, 270.0))

    def test_gate_airspeed_only(self):
        gate = MultiPidStabilizationGate(pitch_enabled=True, bank_enabled=False, yaw_enabled=False, hold_s=5.0)
        
        # Bank and Heading are way off, but shouldn't matter since they are not active
        self.assertFalse(gate.update(1.0, 80.5, 30.0, 120.0, 80.0, 270.0))
        self.assertTrue(gate.update(6.1, 80.5, 30.0, 120.0, 80.0, 270.0))


class TestMultiPidReporting(unittest.TestCase):
    def test_pdf_report_generation(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = Path(tmpdir) / "multi_pid_test.csv"
            with open(csv_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'elapsed_time_s', 'indicated_airspeed', 'target_ias',
                    'bank_angle', 'target_bank_angle',
                    'magnetic_heading', 'target_heading_mag',
                    'pitch_cmd', 'roll_cmd', 'rudder_cmd',
                    'handoff_state', 'is_stabilizing', 'is_recording'
                ])
                for i in range(50):
                    t = i * 0.1
                    writer.writerow([
                        t, 80.0 + math.sin(t), 80.0,
                        math.cos(t), 0.0,
                        270.0 + math.sin(t*0.5), 270.0,
                        0.02, 0.01, 0.0,
                        'BANK_HEADING_HOLD', 0, 1
                    ])
            
            pdf_path = Path(tmpdir) / "multi_pid_test.pdf"
            generate_multi_pid_pdf_report(
                str(csv_path),
                str(pdf_path),
                active_pids={'airspeed': True, 'bank': True, 'yaw': True},
                target_ias=80.0,
                target_heading_mag=270.0
            )
            self.assertTrue(pdf_path.exists())
            self.assertGreater(pdf_path.stat().st_size, 1000)


if __name__ == '__main__':
    unittest.main()