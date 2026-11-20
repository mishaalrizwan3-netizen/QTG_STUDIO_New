import unittest

from backend.stall_events import StallWarningCycle


class StallWarningCycleTests(unittest.TestCase):
    def test_completes_only_after_warning_appears_and_clears(self):
        cycle = StallWarningCycle()
        for sequence, warning, expected in [(1, 0, 'APPROACH'), (2, 0, 'APPROACH'),
                                             (3, 1, 'WARNING'), (4, 1, 'WARNING'),
                                             (5, 0, 'COMPLETE')]:
            self.assertEqual(cycle.update(dict(sequence=sequence, receipt_time=1, stall_warning=warning), 1.1), expected)

    def test_stale_duplicate_or_invalid_readings_cannot_finish_cycle(self):
        cycle = StallWarningCycle()
        cycle.update(dict(sequence=1, receipt_time=1, stall_warning=0), 1.1)
        cycle.update(dict(sequence=2, receipt_time=1, stall_warning=1), 1.1)
        for sample, now in [(dict(sequence=3, receipt_time=1, stall_warning=0), 2),
                            (dict(sequence=2, receipt_time=1, stall_warning=0), 1.1),
                            (dict(sequence=3, receipt_time=1, stall_warning=2), 1.1)]:
            with self.assertRaises(ValueError):
                cycle.update(sample, now)
        self.assertFalse(cycle.completed)

    def test_already_active_warning_is_not_a_new_approach(self):
        with self.assertRaisesRegex(ValueError, 'already active'):
            StallWarningCycle().update(dict(sequence=1, receipt_time=1, stall_warning=1), 1.1)
