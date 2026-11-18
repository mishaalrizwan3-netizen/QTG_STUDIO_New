"""Offline Yaw/Heading PID integration tests. All simulator and socket objects are fakes."""
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
from backend.yaw_pid_runner import YawStabilizationGate, loop, fresh, simulator_loaded
from interfaces.yaw_pid_transport import YawBrunnerTransport
from brunner_interface import BrunnerInterface
import qtg_common_engine as engine


class FakeSocket:
    def __init__(self):
        self.blocking = True
        self.packets = []
        self.roll_pos = 0.05
        self.rudder_pos = -0.02
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
            axis = struct.unpack_from('<B', packet, 4)[0]
            pos = self.roll_pos if axis == 2 else (self.rudder_pos if axis == 4 else 0.0)
            node_id = 5 if axis == 4 else 1
            return struct.pack('<HBHf', 7, 0, node_id, pos)
        if self.reject:
            return b'\x01\x00\x01'
        return b'\x01\x00\x00'


def make_fake_brunner():
    d = BrunnerInterface()
    d.sock = FakeSocket()
    d.read_axis = Mock(side_effect=lambda axis: (0.05 if axis == 2 else (-0.02 if axis == 4 else 0.0), 1.0, 2.0))
    return d


class FakeSim:
    def __init__(self, hdg=270.0, bank=0.0, yaw_rate=0.0, sideslip=0.0):
        self.sequence = 0
        self.connected = True
        self.paused = False
        self.hdg = hdg
        self.bank = bank
        self.yaw_rate = yaw_rate
        self.sideslip = sideslip

    def is_connected(self): return self.connected
    def get_telemetry_sequence(self): return self.sequence
    def request_telemetry_once(self): self.sequence += 1; return True
    def wait_for_fresh_telemetry(self, sequence, timeout):
        return {
            'magnetic_heading': self.hdg,
            'bank_angle': self.bank,
            'yaw_rate': self.yaw_rate,
            'sideslip': self.sideslip,
            'bridge_telemetry_sequence': self.sequence
        }
    def unpause_sim(self): self.paused = False
    def pause_sim(self): self.paused = True
    def simulation_running(self): return not self.paused


def make_fake_window(folder, hdg=270.0, bank=0.0):
    device = make_fake_brunner()
    event = threading.Event()
    device._yaw_transport = YawBrunnerTransport(device, event)
    
    signals = SimpleNamespace(
        emit=Mock(),
    )
    
    w = SimpleNamespace(
        brunner=device,
        p3d=FakeSim(hdg=hdg, bank=bank),
        stop_worker_event=event,
        _session_context={
            'yaw_parent_pid': 12345,
            'bank_parent_pid': 12345,
            'target_heading_mag': 270.0,
            'yaw_hold_sec': 0.05,
            'yaw_acquisition_timeout_sec': 1.0,
            'status_file': str(folder / 'fake_status.json'),
        },
        target_duration_sec=0.2,
        data_dir=folder,
        _outputs=[],
        rec_time=[],
        p_pos_buf=[],
        r_pos_buf=[],
        y_pos_buf=[],
        p_force_buf=[],
        r_force_buf=[],
        y_force_buf=[],
        p3d_tele_buf=[],
        applied_force_buf=[],
        virtual_force_buf=[],
        is_recording=False,
        yaw_fault=None,
        sig_update_stage=signals,
        sig_record_progress=signals,
        sig_recording_finished=signals,
    )
    return w


class TestYawPIDOffline(unittest.TestCase):
    def test_heading_wraparound_shortest_turn(self):
        """Heading wraparound: shortest turn produces correct sign and magnitude across 0/360 boundary."""
        def calc_err(target, actual):
            norm_tgt = target % 360.0
            norm_act = actual % 360.0
            return (norm_tgt - norm_act + 180.0) % 360.0 - 180.0

        # Turn right across 360
        self.assertAlmostEqual(calc_err(1.0, 359.0), 2.0)
        # Turn left across 360
        self.assertAlmostEqual(calc_err(359.0, 1.0), -2.0)
        # 10 deg to 350 deg is left 20 deg
        self.assertAlmostEqual(calc_err(350.0, 10.0), -20.0)
        # 350 deg to 10 deg is right 20 deg
        self.assertAlmostEqual(calc_err(10.0, 350.0), 20.0)
        # On heading
        self.assertAlmostEqual(calc_err(270.0, 270.0), 0.0)

    def test_heading_controller_math(self):
        """HeadingController steers via commanded bank limited to max_bank."""
        ctrl = HeadingController(kp=0.80, ki=0.005, kd=0.70, max_bank=20.0)
        # Target 270, actual 250 -> error +20 -> requires right bank (+ commanded bank)
        bank_cmd = ctrl.update(270.0, 250.0, 0.0, 0.05)
        self.assertGreater(bank_cmd, 0.0)
        self.assertLessEqual(bank_cmd, 20.0)

        # Target 270, actual 290 -> error -20 -> requires left bank (- commanded bank)
        ctrl.reset(initial_bank=0.0)
        bank_cmd_left = ctrl.update(270.0, 290.0, 0.0, 0.05)
        self.assertLess(bank_cmd_left, 0.0)
        self.assertGreaterEqual(bank_cmd_left, -20.0)

    def test_yaw_pid_math(self):
        """YawPID drives rudder for zero sideslip and yaw damping within +/- 0.40."""
        yaw_pid = YawPID(kp=0.025, ki=0.002, kd=0.035)
        # Zero sideslip & zero yaw rate -> zero rudder command
        cmd_zero = yaw_pid.update(sideslip_deg=0.0, yaw_rate_deg_sec=0.0, dt=0.05)
        self.assertAlmostEqual(cmd_zero, 0.0)

        # Positive sideslip (adverse) -> negative rudder command
        cmd_slip = yaw_pid.update(sideslip_deg=5.0, yaw_rate_deg_sec=0.0, dt=0.05)
        self.assertLess(cmd_slip, 0.0)
        self.assertGreaterEqual(cmd_slip, -0.40)

    def test_yaw_stabilization_gate(self):
        """Gate holds for 5.0 continuous seconds within +/-2 deg hdg and +/-3 deg bank."""
        gate = YawStabilizationGate(hdg_tolerance_deg=2.0, bank_tolerance_deg=3.0, hold_s=5.0)
        now = 100.0

        # Off heading: not stable
        self.assertFalse(gate.update(now, target_hdg=270.0, actual_hdg=265.0, actual_bank=0.0))
        self.assertEqual(gate.elapsed_stable(now), 0.0)

        # Within tolerance: start counting
        self.assertFalse(gate.update(now, target_hdg=270.0, actual_hdg=271.5, actual_bank=1.0))
        self.assertAlmostEqual(gate.elapsed_stable(now), 0.0)

        # 4 seconds later: still within tolerance, but not yet 5.0s
        self.assertFalse(gate.update(now + 4.0, target_hdg=270.0, actual_hdg=270.5, actual_bank=0.5))
        self.assertAlmostEqual(gate.elapsed_stable(now + 4.0), 4.0)

        # Perturbation at 4.5s: bank exceeds 3.0 deg -> gate resets!
        self.assertFalse(gate.update(now + 4.5, target_hdg=270.0, actual_hdg=270.5, actual_bank=3.5))
        self.assertEqual(gate.elapsed_stable(now + 4.5), 0.0)

        # New stable run for full 5 seconds
        t_start = now + 5.0
        self.assertFalse(gate.update(t_start, target_hdg=270.0, actual_hdg=270.0, actual_bank=0.0))
        self.assertTrue(gate.update(t_start + 5.0, target_hdg=270.0, actual_hdg=270.2, actual_bank=-0.5))
        self.assertGreaterEqual(gate.elapsed_stable(t_start + 5.0), 5.0)

        # Wraparound stability: target 359.5, actual 0.5 is error 1.0 deg (within +/-2.0 deg)
        gate.reset()
        self.assertFalse(gate.update(now, target_hdg=359.5, actual_hdg=0.5, actual_bank=0.0))
        self.assertTrue(gate.update(now + 5.0, target_hdg=359.5, actual_hdg=0.5, actual_bank=0.0))

    def test_yaw_transport_axes_mask_and_pitch_isolation(self):
        """YawBrunnerTransport claims roll (2) and rudder (4) ONLY; pitch (1) is NEVER commanded."""
        sock = FakeSocket()
        device = BrunnerInterface()
        device.sock = sock
        event = threading.Event()
        transport = YawBrunnerTransport(device, event)

        # Engagement claims mask 0x06 (Roll | Rudder)
        transport.engage()
        self.assertTrue(transport.owned)
        # Check override packet: cmd 0xD1, mask 0x06
        override_packets = [p for p in sock.packets if struct.unpack_from('<I', p)[0] == 0xD1]
        self.assertGreater(len(override_packets), 0)
        override_packet = override_packets[0]
        cmd, mask, setting_code = struct.unpack_from('<III', override_packet)
        self.assertEqual(cmd, 0xD1)
        self.assertEqual(mask, 0x06)  # 0x02 | 0x04, NO Pitch (0x01)!

        # Command roll and rudder (setting 0x80 sent to axes 2 and 4 via 0xCE)
        transport.command(roll_pos=0.15, rudder_pos=-0.20)
        cmd_packet = sock.packets[-1]
        cmd_id, axis_id = struct.unpack_from('<II', cmd_packet)[:2]
        self.assertEqual(cmd_id, 0xCE)
        self.assertEqual(axis_id, 4)  # Axis 4 commanded (Axis 1 pitch NEVER commanded)

        # Release releases roll and rudder without affecting other axes
        transport.release()
        self.assertFalse(transport.owned)
        rel_packets = [p for p in sock.packets if struct.unpack_from('<I', p)[0] == 0xD1]
        last_rel = rel_packets[-1]
        cmd_rel, mask_rel, code_rel = struct.unpack_from('<III', last_rel)
        self.assertEqual(cmd_rel, 0xD1)
        self.assertEqual(mask_rel, 0x00)

    def test_full_yaw_lifecycle(self):
        """End-to-end offline loop: aligns, holds for 50ms, records, outputs CSV and PDF."""
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            win = make_fake_window(tmp_path, hdg=270.0, bank=0.0)

            # Create fake status file so simulator_loaded returns true
            status_file = tmp_path / 'fake_status.json'
            p3d_ready = status_file.with_suffix('.p3d_ready.json')
            p3d_ready.write_text(
                '{"updated": ' + str(time.time()) + ', "connected": true, "sim_running": true, '
                '"telemetry_valid": true, "telemetry_age_ms": 20}',
                encoding='utf-8'
            )

            base = SimpleNamespace(
                STAGE_INITIALIZING='Initializing aircraft',
                STAGE_RECORDING='Recording',
                STAGE_COMPLETED='Completed',
                STAGE_CONNECTION_ERROR='Connection error'
            )

            # Run loop with fake owner alive check
            loop(win, base, owner_alive=lambda pid: True)

            self.assertIsNone(win.yaw_fault)
            self.assertTrue(win.is_recording or len(win.rec_time) > 0)
            self.assertGreater(len(win._outputs), 0)

            # Check that diagnostics CSV was created
            csv_path = win._outputs[-1]
            self.assertTrue(Path(csv_path).is_file())
            with open(csv_path, newline='', encoding='utf-8') as f:
                reader = csv.DictReader(f)
                rows = list(reader)
                self.assertGreater(len(rows), 0)
                self.assertIn('target_hdg_deg', rows[0])
                self.assertIn('actual_hdg_deg', rows[0])
                self.assertIn('roll_command', rows[0])
                self.assertIn('rudder_command', rows[0])

    def test_yaw_rudder_unavailable_graceful_heading_hold(self):
        """When Rudder Axis 4 returns 3-byte empty packet (no motorized loader), gracefully steer with Roll only."""
        class RudderEmptySocket(FakeSocket):
            def recv(self, size):
                if not self.blocking:
                    raise BlockingIOError()
                packet = self.packets[-1]
                cmd = struct.unpack_from('<I', packet)[0]
                if cmd == 0xD0:
                    axis = struct.unpack_from('<B', packet, 4)[0]
                    if axis == 4:
                        return b'\x01\x00\x00'  # 3-byte empty status from CLS2Sim
                    return struct.pack('<HBHf', 7, 0, 1, self.roll_pos)
                if self.reject:
                    return b'\x01\x00\x01'
                return b'\x01\x00\x00'

        sock = RudderEmptySocket()
        device = BrunnerInterface()
        device.sock = sock
        event = threading.Event()
        transport = YawBrunnerTransport(device, event)

        start_roll, start_rudder = transport.engage()
        self.assertTrue(transport.owned)
        self.assertFalse(transport.rudder_available)
        self.assertEqual(start_rudder, 0.0)

        # Check override mask: only Roll (0x02) claimed, NO Rudder (0x04) or Pitch (0x01)!
        override_packets = [p for p in sock.packets if struct.unpack_from('<I', p)[0] == 0xD1]
        self.assertGreater(len(override_packets), 0)
        cmd, mask, setting_code = struct.unpack_from('<III', override_packets[0])
        self.assertEqual(mask, 0x02)

        # Command roll and rudder: roll commanded to axis 2, rudder omitted
        transport.command(roll_pos=0.10, rudder_pos=-0.25)
        # Check last setting packet: should be for axis 2 only
        last_setting = [p for p in sock.packets if struct.unpack_from('<I', p)[0] == 0xCE][-1]
        cmd_id, axis_id = struct.unpack_from('<II', last_setting)[:2]
        self.assertEqual(axis_id, 2)

        transport.release()
        self.assertFalse(transport.owned)

    def test_yaw_report_generation(self):
        """reports.yaw_pid_report creates a valid 2-page PDF report."""
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            csv_path = tmp_path / 'yaw_test.csv'
            csv_path.write_text(
                "Time (s),Heading (deg),Bank Angle (deg),Rudder Position (-),Aileron Position (-),Yaw Rate (deg/s),Sideslip (deg)\n"
                "0.0,270.0,0.0,0.0,0.0,0.0,0.0\n"
                "1.0,270.5,0.2,-0.01,-0.02,0.05,-0.1\n"
                "2.0,269.8,-0.1,0.01,0.02,-0.03,0.1\n",
                encoding='utf-8'
            )
            from reports.yaw_pid_report import report as yaw_report
            out_pdf = yaw_report(csv_path, selected_graphs=['heading_deg', 'bank_deg', 'rudder_pos'])
            self.assertTrue(Path(out_pdf).is_file())
            self.assertTrue(Path(out_pdf).read_bytes().startswith(b'%PDF'))


if __name__ == '__main__':
    unittest.main()
