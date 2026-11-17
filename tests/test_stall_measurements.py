import unittest
import math
import struct
import threading
from types import SimpleNamespace
from unittest.mock import Mock

from backend.stall_measurements import (CHANNELS, measurement_frame, telemetry_fields, parse_control_response,
                                        read_control_axis, parse_axis_calibration, read_engineering_axis)


class StallMeasurementTests(unittest.TestCase):
    def test_values_and_warning_are_preserved(self):
        data = {'ias_kt': 78.5, 'aoa_deg': 4.0, 'pitch_deg': -1.2,
                'bank_deg': 0.0, 'stall_warning': 1.0, 'altitude_ft': 4000,
                'sim_time_s': 12}
        self.assertEqual({key: measurement_frame(data)[key] for key in data}, data)
        self.assertIn(('stall_warning', 'STALL WARNING', 'bool'), CHANNELS)

    def test_invalid_and_missing_values_are_not_fabricated(self):
        data = measurement_frame({'ias_kt': float('nan'), 'stall_warning': 2,
                                  'bank_deg': 0})
        self.assertIsNone(data['ias_kt'])
        self.assertIsNone(data['stall_warning'])
        self.assertIsNone(data['aoa_deg'])
        self.assertEqual(data['bank_deg'], 0)

    def test_freshness_and_boolean_are_preserved(self):
        frame = {'ias_kt': 78, 'stall_warning': 1, 'receipt_time': 4}
        self.assertEqual(telemetry_fields(frame, 4.1)['stall_warning'], 1)
        self.assertTrue(math.isnan(telemetry_fields(frame, 5)['stall_warning']))
        self.assertTrue(math.isnan(telemetry_fields(dict(frame, stall_warning=7), 4.1)['stall_warning']))

    def test_engineering_force_units_are_not_guessed(self):
        position = struct.pack('<HBHf', 7, 0, 1, .25)
        linear = struct.pack('<HBHHf', 9, 0, 1, 1, 12.5)
        rotary = struct.pack('<HBHHf', 9, 0, 1, 2, -3.)
        self.assertEqual(parse_control_response(position, 0x11), (.25, 'normalized'))
        self.assertEqual(parse_control_response(position, 0x21), (.25, 'normalized'))
        self.assertEqual(parse_control_response(linear, 0x22), (12.5, 'N'))
        self.assertEqual(parse_control_response(rotary, 0x22), (-3., 'Nm'))
        with self.assertRaises(ValueError):
            parse_control_response(struct.pack('<HBHHf', 9, 0, 1, 3, 5.), 0x22)

    def test_simulator_fields_keep_their_units(self):
        frame = {'receipt_time': 10, 'cg_long_fraction': .24, 'cg_lateral_fraction': -.01,
                 'terrain_ft': 745, 'fuel_flow_pph': 50}
        values = telemetry_fields(frame, 10.1)
        self.assertEqual(values['stall_cg_long_pct'], 24)
        self.assertEqual(values['stall_cg_lateral_pct'], -1)
        self.assertEqual(values['stall_ground_alt'], 745)
        self.assertEqual(values['stall_fuel_flow_pph'], 50)

    def test_normalized_force_fallback_is_labeled(self):
        sock = Mock()
        position = struct.pack('<HBHf', 7, 0, 1, .25)
        normalized_force = struct.pack('<HBHf', 7, 0, 1, -.5)
        sock.recv.side_effect = [BlockingIOError(), position, b'\x01\x00\x01', normalized_force]
        brunner = SimpleNamespace(sock=sock, timeout=.05, _io_lock=threading.RLock())
        self.assertEqual(read_control_axis(brunner, 1), (.25, -.5, 'normalized'))
        self.assertEqual(sock.send.call_count, 3)

    def test_device_calibration_converts_raw_counts_to_mm_and_newtons(self):
        calibration = struct.pack('<HBHHffIHHiiii', 37, 0, 1, 1, .5, .25, 300, 1, 0, 100, -100, 80, -80)
        self.assertEqual(parse_axis_calibration(calibration), (.25, .5))
        sock = Mock()
        sock.recv.side_effect = [calibration, struct.pack('<HBHi', 7, 0, 1, 40),
                                 struct.pack('<HBHi', 7, 0, 1, -6)]
        brunner = SimpleNamespace(sock=sock, _io_lock=threading.RLock())
        self.assertEqual(read_engineering_axis(brunner, 1, -.1, 'normalized'), (10., -3.))

    def test_rotary_axis_is_not_misrepresented_as_mm_or_newtons(self):
        calibration = struct.pack('<HBHHffIHHiiii', 37, 0, 1, 2, .5, .25, 300, 1, 0, 100, -100, 80, -80)
        with self.assertRaises(ValueError):
            parse_axis_calibration(calibration)

    def test_observed_multi_device_response_identifies_rotational_axis(self):
        force = bytes.fromhex('11000002000200cc1047be0400000000000000')
        value, unit = parse_control_response(force, 0x22)
        self.assertEqual(unit, 'Nm')
        self.assertAlmostEqual(value, -.1944, places=4)
        calibration = bytes.fromhex('4900000200020052491d3a99bb163c082000000100010052080000aef7ffff52080000aef7ffff040000000000000000000000000000000000000000000000000000000000000000000000')
        with self.assertRaisesRegex(ValueError, 'Rotational axis'):
            parse_axis_calibration(calibration)


if __name__ == '__main__':
    unittest.main()
