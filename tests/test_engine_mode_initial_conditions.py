"""Engine Manual/Automatic settings and brake interlock, without simulator access."""
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from backend import initializer
from backend.engine_conditions import enforce
import qtg_common_engine as engine


def conditions():
    return dict(alt_ft=2345, ias_kt=80, lat=32.1, lon=74.2, heading_deg=45,
                pitch_deg=0, bank_deg=0, flaps=0, throttle=42,
                temperature_c=15, on_ground=False)


class EngineInitialConditionsTests(unittest.TestCase):
    def test_both_modes_keep_settings_and_enforce_required_engine_conditions(self):
        with tempfile.TemporaryDirectory() as folder:
            for reference, title in (('1.f.(1)', 'Engine acceleration'), ('1.f.(2)', 'Engine deceleration')):
                for mode in ('manual', 'auto'):
                    with self.subTest(reference=reference, mode=mode):
                        method = 'engine_step' if mode == 'auto' else 'replay'
                        with patch.object(engine.subprocess, 'Popen'):
                            _, context = engine.launch(mode, SimpleNamespace(reference=reference, title=title),
                                Path(folder)/'sessions', control_method=method, initial_conditions=conditions())
                        actual = context['initial_conditions']
                        self.assertEqual(actual['alt_ft'], 2345)
                        self.assertEqual(actual['lat'], 32.1)
                        self.assertEqual(actual['throttle'], 5 if reference == '1.f.(1)' else 100)
                        if reference == '1.f.(2)':
                            self.assertTrue(actual['on_ground'])
                            self.assertEqual(actual['ias_kt'], 0)
                        else:
                            self.assertFalse(actual['on_ground'])
                            self.assertEqual(actual['ias_kt'], 80)

    def test_manual_deceleration_checks_brake_after_position_before_full_power(self):
        calls = []
        sim = Mock()
        sim.is_connected.return_value = True
        sim.wait_for_paused.return_value = True
        sim.queue_position_airborne.side_effect = lambda **kw: calls.append(('position', kw)) or True
        sim.set_initial_throttle.side_effect = lambda value: calls.append(('throttle', value)) or True
        sim.set_initial_flaps.return_value = True
        cfg = enforce('1.f.(2)', conditions())
        actual = dict(pressure_alt=cfg['alt_ft'], plane_latitude=cfg['lat'], plane_longitude=cfg['lon'],
                      measured_flaps_handle_index=0, engine_throttle_percent=100)
        window = SimpleNamespace(p3d=sim, stop_worker_event=threading.Event(),
            _session_context={'test_reference':'1.f.(2)'}, _wait_for_initial_altitude=Mock(),
            _refresh_paused_telemetry=Mock(return_value=actual))
        with patch.dict('sys.modules', {'replay_support': SimpleNamespace(reference_weather_metar=lambda *args: None)}), \
             patch.object(initializer, 'verify_manual_deceleration_brake', side_effect=lambda w: calls.append(('brake',))):
            initializer.apply_manual_conditions(window, conditions())
        self.assertEqual([entry[0] for entry in calls], ['throttle', 'position', 'brake', 'throttle'])
        self.assertEqual(calls[0][1], 5.)
        self.assertEqual(calls[-1][1], 100.)
        self.assertTrue(calls[1][1]['on_ground'])
        self.assertEqual(calls[1][1]['speed_kts'], 0)

    def test_manual_deceleration_brake_failure_never_commands_full_throttle(self):
        sim = Mock()
        sim.is_connected.return_value = sim.wait_for_paused.return_value = True
        sim.set_initial_throttle.return_value = sim.queue_position_airborne.return_value = True
        window = SimpleNamespace(p3d=sim, stop_worker_event=threading.Event(),
            _session_context={'test_reference':'1.f.(2)'}, _wait_for_initial_altitude=Mock())
        with patch.dict('sys.modules', {'replay_support': SimpleNamespace(reference_weather_metar=lambda *args: None)}), \
             patch.object(initializer, 'verify_manual_deceleration_brake', side_effect=RuntimeError('brake not verified')):
            with self.assertRaisesRegex(RuntimeError, 'brake not verified'):
                initializer.apply_manual_conditions(window, conditions())
        sim.set_initial_throttle.assert_called_once_with(5.)

    def test_brake_reader_requires_fresh_brake_and_ground_and_always_closes(self):
        for brake, ground, speed, age, success in ((1, 1, 0, 0, True), (0, 1, 0, 0, False),
                (1, 0, 0, 0, False), (1, 1, 3, 0, False), (1, 1, 0, 2, False)):
            with self.subTest(brake=brake, ground=ground, speed=speed, age=age):
                tick = iter((10., 10., 10., 16.))
                reader = Mock()
                reader.poll.return_value = dict(parking_brake=brake, on_ground=ground,
                                                ground_speed=speed, receipt_time=10.-age, sequence=1)
                window = SimpleNamespace(p3d=Mock(is_connected=Mock(return_value=True)),
                                         stop_worker_event=Mock(is_set=Mock(return_value=False)))
                if success:
                    initializer.verify_manual_deceleration_brake(window, reader_factory=lambda: reader, clock=lambda: next(tick))
                else:
                    with self.assertRaisesRegex(RuntimeError, 'parking brake'):
                        initializer.verify_manual_deceleration_brake(window, reader_factory=lambda: reader, clock=lambda: next(tick))
                reader.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
