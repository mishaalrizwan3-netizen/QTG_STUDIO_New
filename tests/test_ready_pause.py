import unittest
from unittest.mock import Mock

from backend.ready_pause import confirm_ready_pause


class ReadyPauseTests(unittest.TestCase):
    def test_pause_is_confirmed_before_readiness(self):
        sim = Mock()
        sim.is_connected.return_value = True
        sim.wait_for_paused.return_value = True
        confirm_ready_pause(sim)
        self.assertLess(sim.mock_calls.index(unittest.mock.call.pause_sim()),
                        sim.mock_calls.index(unittest.mock.call.wait_for_paused(3.0)))

    def test_disconnection_and_missing_pause_acknowledgment_block_readiness(self):
        sim = Mock()
        sim.is_connected.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'disconnected'):
            confirm_ready_pause(sim)
        sim.pause_sim.assert_not_called()
        sim.is_connected.return_value = True
        sim.wait_for_paused.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'did not confirm pause'):
            confirm_ready_pause(sim)


if __name__ == '__main__':
    unittest.main()
