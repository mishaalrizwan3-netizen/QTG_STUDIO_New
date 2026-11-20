"""Offline checks for the separate PID launcher; no hardware is opened."""

from pathlib import Path
import tempfile
import threading
import unittest

from pid_launcher import PidOptions, PidSession


class FakeSim:
    def __init__(self):
        self.paused = False
        self.stopped = False

    def connect_and_run(self, *, on_connected, on_error):
        on_connected()

    def pause_sim(self):
        self.paused = True

    def stop(self):
        self.stopped = True


class FakeDevice:
    def __init__(self):
        self.closed = False

    def connect(self):
        return True

    def close(self):
        self.closed = True


class PidAddonTests(unittest.TestCase):
    def options(self, output):
        return PidOptions(True, False, True, 80.0, 48.4, 15.0, Path(output))

    def test_selected_axes_reach_separate_runner_and_cleanup_runs(self):
        with tempfile.TemporaryDirectory() as folder:
            sim, device, messages = FakeSim(), FakeDevice(), []

            def runner(window, base):
                self.assertEqual(window._session_context['active_pids'],
                                 {'airspeed': True, 'bank': False, 'yaw': True})
                self.assertEqual(window._session_context['pid_conditions']['target_ias'], 80.0)
                self.assertEqual(window._session_context['target_heading_mag'], 48.4)
                self.assertEqual(window.target_duration_sec, 15.0)
                window.multi_pid_fault = None
                window._outputs.append(str(Path(folder) / 'record.csv'))

            session = PidSession(self.options(folder), messages.append,
                                 hardware_factory=lambda: (sim, device), runner=runner)
            session.run()
            self.assertIsNone(session.error)
            self.assertTrue(sim.paused)
            self.assertTrue(sim.stopped)
            self.assertTrue(device.closed)
            self.assertEqual(len(session.outputs), 1)
            self.assertIn('PID run complete', messages[-1])

    def test_runner_failure_still_pauses_and_releases(self):
        with tempfile.TemporaryDirectory() as folder:
            sim, device = FakeSim(), FakeDevice()

            def failed_runner(window, base):
                raise RuntimeError('feedback lost')

            session = PidSession(self.options(folder), lambda *_: None,
                                 hardware_factory=lambda: (sim, device), runner=failed_runner)
            session.run()
            self.assertEqual(session.error, 'feedback lost')
            self.assertTrue(sim.paused)
            self.assertTrue(sim.stopped)
            self.assertTrue(device.closed)

    def test_cancel_before_connection_never_engages(self):
        with tempfile.TemporaryDirectory() as folder:
            sim, device = FakeSim(), FakeDevice()
            waiting = threading.Event()

            def connect_without_handshake(*, on_connected, on_error):
                waiting.set()
                while not sim.stopped:
                    threading.Event().wait(.01)

            sim.connect_and_run = connect_without_handshake
            session = PidSession(self.options(folder), lambda *_: None,
                                 hardware_factory=lambda: (sim, device),
                                 runner=lambda *_: self.fail('PID engaged before P3D connected'))
            thread = threading.Thread(target=session.run)
            thread.start()
            self.assertTrue(waiting.wait(1))
            session.stop()
            thread.join(3)
            self.assertFalse(thread.is_alive())
            self.assertFalse(sim.paused)
            self.assertTrue(sim.stopped)
            self.assertTrue(device.closed)

    def test_empty_pid_selection_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Select at least one'):
            PidOptions(False, False, False, 80.0, 48.4, 15.0, Path('.')).validate()


if __name__ == '__main__':
    unittest.main()
