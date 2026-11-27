import unittest
from unittest.mock import Mock, patch

from backend.automatic_manoeuvring_stability import (
    AutomaticManoeuvringStability, TARGETS, selected,
)
from backend.automatic_power_change import AutomaticPowerDynamics
from backend.control import validate_method


class SequenceTests(unittest.TestCase):
    def test_variants_and_method_are_isolated(self):
        self.assertEqual(TARGETS['2.c.(6a)'], (0.0, 15.0, 30.0, 45.0))
        self.assertEqual(TARGETS['2.c.(6b)'], (0.0, 15.0, 30.0))
        for ref in TARGETS:
            self.assertTrue(selected(dict(mode='auto', control_method='manoeuvring_stability', test_reference=ref)))
            self.assertEqual(validate_method(ref, 'manoeuvring_stability'), 'manoeuvring_stability')
        self.assertFalse(selected(dict(mode='manual', control_method='manoeuvring_stability', test_reference='2.c.(6a)')))
        with self.assertRaises(ValueError):
            validate_method('2.a.(4)', 'manoeuvring_stability')

    def test_hold_resets_on_drift_and_advances_after_five_continuous_seconds(self):
        worker = object.__new__(AutomaticManoeuvringStability)
        current = [0.0]
        worker.clock = lambda: current[0]
        worker.sim = Mock(is_connected=Mock(return_value=True))
        worker.publish = lambda _state: None
        worker.phase, worker.stage, worker.details = 'BANK_SEQUENCE', 'BANK TARGET 0 DEG', ''
        worker.targets, worker.stage_index = (0.0, 15.0), 0
        worker.stage_started = worker.start_time = 0.0
        worker.stable_since = None
        worker.last_sequence = -1
        worker.latest = None
        worker.last_sample_time = None
        worker.rows, worker.outputs = [], []
        worker.context = {'result_directory': '.', 'test_reference': '2.c.(6b)'}
        worker.brunner = Mock(sock=True)
        worker.pid_assist = Mock()
        worker.pid_assist.transport = None
        worker._ensure_live = lambda _frame, _now: None
        worker.emit = lambda: None
        def feed(sequence, time_s, bank):
            current[0] = time_s
            frame = dict(sequence=sequence, receipt_time=time_s, sim_time=time_s,
                         bank_deg=-bank, ias_kt=90, alt_ft=4000, pitch_deg=0,
                         throttle=50, rpm=2500)
            worker.step(frame)
        with patch.object(AutomaticManoeuvringStability, '_record_frame',
                          lambda self, frame, now: self.rows.append({'Time (s)': now})):
            feed(1, 0.0, 0.0)
            feed(2, 3.0, 3.0)
            self.assertIsNone(worker.stable_since)
            feed(3, 4.0, 0.0)
            feed(4, 8.9, 0.0)
            self.assertEqual(worker.stage_index, 0)
            feed(5, 9.0, 0.0)
            self.assertEqual(worker.stage_index, 1, (worker.phase, worker.details))
            self.assertEqual(worker.pid_assist.target_bank, 0.0)
            feed(6, 9.1, 15.0)
            self.assertEqual(worker.pid_assist.target_bank, 15.0)


if __name__ == '__main__':
    unittest.main()
