from pathlib import Path
from unittest.mock import Mock, patch
import tempfile
import unittest

from backend.automatic_flap_change import (
    AutomaticFlapDynamics, EXTENSION_REFERENCE, RETRACTION_REFERENCE,
    EXTENSION_FORCE_REFERENCE, RETRACTION_FORCE_REFERENCE, save_report,
)


class Clock:
    def __init__(self): self.value = 100.0
    def __call__(self): return self.value
    def advance(self, seconds): self.value += seconds


class Sim:
    def __init__(self):
        self.connected = True
        self.paused = False
        self.flaps = 1.0
        self.throttle = 50.0
        self.events = []
    def is_connected(self): return self.connected
    def pause_sim(self): self.paused = True; self.events.append(('pause',))
    def wait_for_paused(self, timeout): return self.paused
    def unpause_sim(self): self.paused = False; self.events.append(('unpause',))
    def set_initial_throttle(self, value): self.throttle = value; return True
    def queue_position_airborne(self, **kwargs): return True
    def set_initial_flaps(self, value): self.flaps = value; self.events.append(('initial_flaps', value)); return True
    def set_flaps_handle_index(self, value): self.flaps = value; self.events.append(('flap_step', value)); return True
    def get_global_weather_metar(self): return 'METAR OPRW 171000Z 00000KT CAVOK 15/10 Q1013'
    def set_global_weather_metar(self, value): return True


class AutomaticFlapChangeTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.sim = Sim()
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.count = 0

    def context(self, reference):
        initial_flap = 0.0 if reference == EXTENSION_REFERENCE else 1.0
        self.sim.flaps = initial_flap
        return dict(mode='auto', control_method='flap_dynamics', test_reference=reference,
                    title='Flap change dynamics', result_directory=self.tmp.name,
                    initial_conditions=dict(alt_ft=4000.0, ias_kt=90.0, lat=32.235,
                        lon=74.135, heading_deg=48.4, pitch_deg=3.0, bank_deg=0.0,
                        flaps=initial_flap, throttle=50.0, temperature_c=15.0, on_ground=False),
                    selected_graphs=['ias_kt', 'pitch_deg'])

    def feed(self, worker, dt=0.5, **changes):
        self.clock.advance(dt); self.count += 1
        frame = dict(sequence=self.count, receipt_time=self.clock(), sim_time=self.clock(),
                     rpm=2000.0, throttle=self.sim.throttle, flaps=self.sim.flaps,
                     ias_kt=90.0, alt_ft=4000.0, lat=32.235, lon=74.135,
                     height_agl_ft=3000.0, on_ground=0.0, autopilot_active=0.0,
                     temperature_c=15.0, pitch_deg=3.0, bank_deg=0.0,
                     heading_deg=48.4, prop_rpm=2000.0,
                     trailing_edge_flaps_left_angle_rad=0.25,
                     trailing_edge_flaps_right_angle_rad=0.26,
                     manifold_pressure_psi=12.0, vertical_speed=0.0,
                     elevator_input=0.0, elevator_trim_rad=0.0)
        frame.update(changes)
        worker.step(frame)

    def run_direction(self, reference, final_index):
        worker = AutomaticFlapDynamics(self.context(reference), self.sim, lambda state: None,
                                       clock=self.clock)
        for _ in range(10):
            self.feed(worker)
            if worker.phase == 'READY': break
        self.assertEqual(worker.phase, 'READY', worker.details)
        worker.command({'action': 'record'})
        for _ in range(22):
            self.feed(worker)
            if worker.flap_command_time is not None: break
        self.assertIsNotNone(worker.flap_command_time)
        self.assertGreaterEqual(worker.flap_command_time - worker.start_time, 5.0)
        self.assertEqual([event for event in self.sim.events if event[0] == 'flap_step'],
                         [('flap_step', float(final_index))])
        self.feed(worker)
        self.assertEqual(worker.flap_completed_time, self.clock())
        with patch('backend.automatic_flap_change.save_report', return_value=[]):
            for _ in range(35):
                if worker.phase == 'DONE': break
                self.feed(worker)
        self.assertEqual(worker.result, 'COMPLETE', worker.details)
        self.assertGreaterEqual(self.clock() - worker.flap_completed_time, 15.0)

    def test_retraction_automatically_commands_index_zero(self):
        self.run_direction(RETRACTION_REFERENCE, 0)

    def test_extension_automatically_commands_index_one_from_zero(self):
        self.run_direction(EXTENSION_REFERENCE, 1)

    def test_force_tests_reuse_the_exact_retraction_and_extension_sequences(self):
        self.run_direction(RETRACTION_FORCE_REFERENCE, 0)
        self.sim.events.clear(); self.sim.flaps = 1.0
        self.run_direction(EXTENSION_FORCE_REFERENCE, 3)

    def test_force_report_has_separate_identity_and_pitch_force_selection(self):
        import json
        rows = [
            {'Time (s)': 0.0, 'Joystick Pitch Position (mm)': 1.0,
             'Joystick Pitch Force (N)': 2.0, 'Flaps Handle Index': 1.0},
            {'Time (s)': 20.0, 'Joystick Pitch Position (mm)': 1.5,
             'Joystick Pitch Force (N)': 3.0, 'Flaps Handle Index': 0.0},
        ]
        context = self.context(RETRACTION_FORCE_REFERENCE)
        context['selected_graphs'] = ['pitch_position', 'pitch_force', 'flaps_handle_index']
        with patch('reports.plot_graphs._render_report'):
            outputs = save_report(context, rows, 'COMPLETE', 'offline force check',
                                  flap_command_time=105.0, flap_onset_time=105.1,
                                  flap_completed_time=105.2, recording_start=100.0)
        self.assertIn('flap_change_force_retraction_auto_', Path(outputs[0]).name)
        metadata = json.loads(Path(outputs[1]).read_text(encoding='utf-8'))
        self.assertEqual(metadata['kind'], 'Flap Change Force')
        self.assertIn('pitch_force', metadata['graphed_channels'])

    def test_other_references_cannot_use_flap_worker(self):
        context = self.context('2.c.(5)')
        with self.assertRaises(ValueError):
            AutomaticFlapDynamics(context, self.sim, lambda state: None, clock=self.clock)

    def test_records_flap_lever_index_and_percentage_channels(self):
        from interfaces.engine_measurements import CHANNELS
        from reports.power_dynamics import CHANNELS as REPORT_CHANNELS, graph_availability
        self.assertIn(('flaps', 'FLAPS HANDLE INDEX', 'number'), CHANNELS)
        self.assertIn(('flaps_handle_percent', 'FLAPS HANDLE PERCENT', 'percent'), CHANNELS)
        self.assertIn(('trailing_edge_flaps_left_angle_rad', 'TRAILING EDGE FLAPS LEFT ANGLE', 'radians'), CHANNELS)
        self.assertIn(('trailing_edge_flaps_right_angle_rad', 'TRAILING EDGE FLAPS RIGHT ANGLE', 'radians'), CHANNELS)
        self.assertIn('flaps_handle_index', REPORT_CHANNELS)
        self.assertIn('flaps_handle_percent', REPORT_CHANNELS)
        self.assertIn('flaps_left_angle', REPORT_CHANNELS)
        self.assertIn('flaps_right_angle', REPORT_CHANNELS)
        worker = AutomaticFlapDynamics(self.context(RETRACTION_REFERENCE), self.sim, lambda state: None,
                                       clock=self.clock)
        for _ in range(10):
            self.feed(worker, flaps_handle_percent=33.0)
            if worker.phase == 'READY': break
        self.assertEqual(worker.status()['telemetry']['flaps_handle_percent'], 33.0)
        worker.command({'action': 'record'})
        for _ in range(5):
            self.feed(worker, flaps_handle_percent=33.0)
        self.feed(worker, flaps_handle_percent=33.0)
        self.assertEqual(worker.rows[0]['Flaps Handle Index'], 1.0)
        self.assertEqual(worker.rows[0]['Flaps Handle Percent (%)'], 33.0)
        self.assertEqual(worker.rows[0]['TRAILING EDGE FLAPS LEFT ANGLE'], 0.25)
        self.assertEqual(worker.rows[0]['TRAILING EDGE FLAPS RIGHT ANGLE'], 0.26)
        available, unavailable = graph_availability(
            worker.rows, ['flaps_handle_index', 'flaps_handle_percent', 'flaps_left_angle', 'flaps_right_angle'])
        self.assertEqual(available, ['flaps_handle_index', 'flaps_handle_percent', 'flaps_left_angle', 'flaps_right_angle'])
        self.assertEqual(unavailable, [])

    def test_recording_waits_until_measured_flap_angles_have_settled(self):
        worker = AutomaticFlapDynamics(self.context(RETRACTION_REFERENCE), self.sim, lambda state: None,
                                       clock=self.clock)
        for _ in range(10):
            self.feed(worker)
            if worker.phase == 'READY':
                break
        worker.command({'action': 'record'})
        for angle in (0.04, 0.06, 0.08, 0.095):
            self.feed(worker, trailing_edge_flaps_left_angle_rad=angle,
                      trailing_edge_flaps_right_angle_rad=angle)
        self.assertIsNone(worker.start_time)
        self.assertEqual(worker.rows, [])
        for _ in range(3):
            self.feed(worker, trailing_edge_flaps_left_angle_rad=0.095,
                      trailing_edge_flaps_right_angle_rad=0.095)
        self.assertIsNotNone(worker.start_time)
        self.assertEqual(worker.rows, [])
        self.feed(worker, trailing_edge_flaps_left_angle_rad=0.095,
                  trailing_edge_flaps_right_angle_rad=0.095)
        self.assertEqual(worker.rows[0]['TRAILING EDGE FLAPS LEFT ANGLE'], 0.095)
        self.assertEqual(worker.rows[0]['TRAILING EDGE FLAPS RIGHT ANGLE'], 0.095)

    def test_manual_initialization_allows_flap_mismatch_only_for_flap_tests(self):
        import sys
        import threading
        from types import SimpleNamespace
        from backend.initializer import apply_manual_conditions
        cfg = dict(alt_ft=4000.0, ias_kt=90.0, lat=32.235, lon=74.135,
                   heading_deg=48.4, pitch_deg=3.0, bank_deg=0.0,
                   flaps=1.0, throttle=50.0, temperature_c=15.0, on_ground=False)
        telemetry = dict(pressure_alt=4000.0, plane_latitude=32.235, plane_longitude=74.135,
                         measured_flaps_handle_index=0.0, engine_throttle_percent=50.0)
        def make_window(ref):
            return SimpleNamespace(
                p3d=self.sim,
                stop_worker_event=Mock(is_set=Mock(return_value=False), wait=Mock(return_value=False)),
                _session_context={'test_reference': ref},
                _wait_for_initial_altitude=lambda alt: None,
                _refresh_paused_telemetry=lambda: dict(telemetry),
            )
        fake_replay = SimpleNamespace(reference_weather_metar=lambda cond, metar: None)
        with patch.dict(sys.modules, {'replay_support': fake_replay}):
            # Flap test 2.c.(2a) proceeds even if paused telemetry still reports flap index 0
            self.sim.events.clear()
            actual = apply_manual_conditions(make_window('2.c.(2a)'), cfg)
            self.assertEqual(actual['measured_flaps_handle_index'], 0.0)
            self.assertIn(('unpause',), self.sim.events)
            self.assertEqual(self.sim.events[-1], ('pause',))
            # Non-flap test still enforces measured_flaps_handle_index when target != 0
            with patch('backend.initializer.time.monotonic', side_effect=[0.0, 0.0, 4.0]):
                with self.assertRaises(RuntimeError) as ctx:
                    apply_manual_conditions(make_window('1.c.(1)'), cfg)
            self.assertIn('measured_flaps_handle_index', str(ctx.exception))

    def test_manual_selected_report_includes_flap_lever_graphs(self):
        import csv
        from interfaces.brunner_base.src import plot_graphs as brunner_plot
        csv_path = Path(self.tmp.name) / '2_c_2a_manual.csv'
        pdf_path = Path(self.tmp.name) / '2_c_2a_manual_graphs.pdf'
        with csv_path.open('w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=[
                'Time (s)', 'Indicated Airspeed (kts)', 'Flaps Handle Index', 'Flaps Handle Percent (%)'
            ])
            writer.writeheader()
            writer.writerow({'Time (s)': 0.0, 'Indicated Airspeed (kts)': 90.0, 'Flaps Handle Index': 1.0, 'Flaps Handle Percent (%)': 33.0})
            writer.writerow({'Time (s)': 1.0, 'Indicated Airspeed (kts)': 91.0, 'Flaps Handle Index': 0.0, 'Flaps Handle Percent (%)': 0.0})
        brunner_plot._selected_report(
            csv_path, pdf_path, ['flaps_handle_index', 'flaps_handle_percent']
        )
        self.assertTrue(pdf_path.is_file() and pdf_path.read_bytes().startswith(b'%PDF'))

    def test_manual_selected_report_flap_rebases_time_and_uses_altitude_msl(self):
        import csv
        from interfaces.brunner_base.src import plot_graphs as brunner_plot
        flap_csv = Path(self.tmp.name) / '2_c_2a_manual.csv'
        flap_pdf = Path(self.tmp.name) / '2_c_2a_manual_graphs.pdf'
        with flap_csv.open('w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=[
                'Time (s)', 'Indicated Altitude (ft)', 'Altitude MSL (ft)',
                'Flaps Handle Index', 'Flaps Handle Percent (%)',
            ])
            writer.writeheader()
            writer.writerow({'Time (s)': 0.3366, 'Indicated Altitude (ft)': 4403.1, 'Altitude MSL (ft)': 4499.55, 'Flaps Handle Index': 1.0, 'Flaps Handle Percent (%)': 33.0})
            writer.writerow({'Time (s)': 1.3366, 'Indicated Altitude (ft)': 4494.2, 'Altitude MSL (ft)': 4493.31, 'Flaps Handle Index': 0.0, 'Flaps Handle Percent (%)': 0.0})
        recorded = []
        orig_plot_manual = brunner_plot._plot_manual
        def capture(ax, t, y, ylabel, *args, **kwargs):
            recorded.append((ylabel, list(t), list(y)))
            return orig_plot_manual(ax, t, y, ylabel, *args, **kwargs)
        with patch.object(brunner_plot, '_plot_manual', side_effect=capture):
            brunner_plot._selected_report(flap_csv, flap_pdf, ['altitude_msl_ft'])
        self.assertTrue(flap_pdf.is_file() and flap_pdf.read_bytes().startswith(b'%PDF'))
        self.assertEqual(len(recorded), 1)
        ylabel, ts, ys = recorded[0]
        self.assertEqual(ylabel, 'Altitude MSL (ft)')
        self.assertAlmostEqual(ts[0], 0.0, places=6)
        self.assertAlmostEqual(ts[1], 1.0, places=6)
        self.assertAlmostEqual(ys[0], 4499.55, places=2)
        self.assertAlmostEqual(ys[1], 4493.31, places=2)

        # Non-flap tests must keep their original timestamps and Indicated Altitude (ft) column.
        other_csv = Path(self.tmp.name) / '1_c_1_manual.csv'
        other_pdf = Path(self.tmp.name) / '1_c_1_manual_graphs.pdf'
        with other_csv.open('w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=[
                'Time (s)', 'Indicated Altitude (ft)', 'Altitude MSL (ft)',
            ])
            writer.writeheader()
            writer.writerow({'Time (s)': 0.25, 'Indicated Altitude (ft)': 4500.0, 'Altitude MSL (ft)': 4499.5})
            writer.writerow({'Time (s)': 1.25, 'Indicated Altitude (ft)': 4505.0, 'Altitude MSL (ft)': 4504.5})
        recorded.clear()
        with patch.object(brunner_plot, '_plot_manual', side_effect=capture):
            brunner_plot._selected_report(other_csv, other_pdf, ['altitude_msl_ft'])
        self.assertEqual(recorded[0][0], 'Indicated Altitude (ft)')
        self.assertAlmostEqual(recorded[0][1][0], 0.25, places=6)
        self.assertAlmostEqual(recorded[0][2][0], 4500.0, places=2)


if __name__ == '__main__':
    unittest.main()
