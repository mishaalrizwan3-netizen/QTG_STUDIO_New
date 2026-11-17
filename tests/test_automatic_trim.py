"""Offline Pitch Trim worker and SimConnect trim-command coverage."""
import ctypes
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from backend.automatic_trim import AutomaticTrim, TrimEvents, UP, DOWN, selected, run


def context():
    return {'test_reference': '2.a.(4)', 'control_method': 'pitch_trim', 'mode': 'auto',
            'initial_conditions': {'on_ground': False}, 'result_directory': '.'}


class AutomaticTrimTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.socket = Mock()
        self.socket.recv.return_value = b'\x01\x00\x00'
        self.brunner = Mock(sock=self.socket)
        self.sim = Mock()
        self.sim.is_connected.return_value = True
        self.sim.get_telemetry.return_value = {'airspeed_indicated': 80., 'engine_rpm': 2300.}
        self.reader = Mock()
        self.events = Mock()
        self.worker = AutomaticTrim(context(), self.sim, self.brunner, lambda _: None,
                                    self.reader, clock=lambda: self.now, events=self.events)
        self.worker.phase, self.worker.stage = 'READY', 'READY TO RECORD'

    def frame(self, sequence, angle, indicator=0.):
        return dict(sequence=sequence, receipt_time=self.now, trim_rad=angle / 2,
                    indicator=indicator, trim_fraction=indicator, elevator_rad=angle)

    def test_context_and_simconnect_use_only_trim_increment_events(self):
        self.assertTrue(selected(context()))
        reader = Mock()
        reader.handle = ctypes.c_void_p(4)
        reader.dll.SimConnect_MapClientEventToSimEvent.return_value = 0
        reader.dll.SimConnect_TransmitClientEvent.return_value = 0
        events = TrimEvents(reader)
        events.step(UP); events.step(DOWN)
        self.assertEqual(reader.dll.SimConnect_MapClientEventToSimEvent.call_args_list[0].args[2], b'ELEV_TRIM_UP')
        self.assertEqual(reader.dll.SimConnect_MapClientEventToSimEvent.call_args_list[1].args[2], b'ELEV_TRIM_DN')
        self.assertEqual([call.args[2] for call in reader.dll.SimConnect_TransmitClientEvent.call_args_list], [UP, DOWN])
        self.socket.send.assert_not_called()
        self.brunner.write_setting.assert_not_called()

    def test_automatic_sends_p3d_trim_before_first_telemetry_frame(self):
        self.worker.command({'action': 'record'})
        self.reader.poll.return_value = None
        self.now = 10.11
        self.worker.step()
        self.now = 10.24
        self.worker.step()
        self.now = 10.33
        self.worker.step()
        self.assertEqual([call.args[0] for call in self.events.step.call_args_list], [UP, UP])
        self.socket.send.assert_not_called()
        self.brunner.write_setting.assert_not_called()

    def test_aircraft_initialization_reaches_ready_without_actuator_motion(self):
        conditions = {'on_ground': False, 'alt_ft': 755., 'ias_kt': 0., 'lat': 32., 'lon': 74.,
                      'heading_deg': 0., 'throttle': 5.}
        self.sim.get_telemetry_sequence.return_value = 1
        self.sim.queue_position_airborne.return_value = True
        self.sim.set_initial_throttle.return_value = True
        self.sim.wait_for_fresh_telemetry.return_value = {'engine_throttle_percent': 5.,
            'airspeed_indicated': 0., 'indicated_altitude': 755.}
        self.brunner.read_axis.return_value = (0., 0., 0.)
        worker = AutomaticTrim(dict(context(), initial_conditions=conditions), self.sim, self.brunner,
                               lambda _: None, self.reader, clock=lambda: self.now, events=self.events)
        worker.step()
        worker.step()
        self.assertEqual(worker.stage, 'READY TO RECORD')
        self.assertTrue(worker.status()['can_record'])
        self.socket.send.assert_not_called()
        self.brunner.write_setting.assert_not_called()

    def test_two_full_indicator_sweeps_stop_at_second_minus_one(self):
        self.brunner.read_axis.return_value = (.12, .03, 0.)
        self.worker.command({'action': 'record'})
        with patch('reports.automatic_control_report.save', return_value=['test.csv', 'test.pdf']) as save:
            for sequence, indicator in enumerate((.25, .7, 1., .7, 0., -.7, -1., -.7, 0., .7, 1., .7, 0., -.7, -1.), 1):
                self.now += .18
                self.reader.poll.return_value = self.frame(sequence, -.1, indicator)
                self.worker.step()
                self.assertNotEqual(self.worker.result, 'COMPLETE')
            self.assertTrue(self.worker.sweep_complete)
            sent_before_hold = self.events.step.call_count
            self.now = 12.99
            self.reader.poll.return_value = self.frame(16, -.1, -1.)
            self.worker.step()
            self.assertNotEqual(self.worker.result, 'COMPLETE')
            self.assertEqual(self.events.step.call_count, sent_before_hold)
            self.now = 13.05
            self.reader.poll.return_value = self.frame(17, -.1, -1.)
            self.worker.step()
            self.assertEqual(self.worker.result, 'COMPLETE', self.worker.details)
            self.assertGreaterEqual(self.worker.rows[-1]['Time (s)'], 3.)
            self.assertEqual(self.events.step.call_count, sent_before_hold)
            self.assertEqual(self.worker.rows[0]['Elevator Trim (%)'], 25.)
            self.assertEqual(self.worker.rows[-1]['Elevator Trim Indicator (position)'], -1.)
            self.assertEqual(self.worker.outputs, ['test.csv', 'test.pdf'])
            self.assertTrue(all('Joystick Pitch Position (mm)' in row for row in self.worker.rows))
            self.assertTrue(all('Joystick Pitch Force (N)' in row for row in self.worker.rows))
            sent = [call.args[0] for call in self.events.step.call_args_list]
            groups = [sent[0]]
            for direction in sent[1:]:
                if direction != groups[-1]: groups.append(direction)
            self.assertEqual(groups, [UP, DOWN, UP, DOWN])
            self.socket.send.assert_not_called()
            save.assert_called_once()
        self.brunner.write_setting.assert_not_called()

    def test_stale_telemetry_stops_trim_events_and_fails(self):
        self.worker.command({'action': 'record'})
        self.reader.poll.return_value = self.frame(1, 0.)
        self.worker.step()
        self.now += 1.
        with patch('reports.automatic_control_report.save', return_value=[]):
            self.worker.step()
        self.assertEqual(self.worker.result, 'FAILED')
        self.assertIn('telemetry stopped', self.worker.details)
        self.socket.send.assert_not_called()

    def test_emergency_stop_stops_trim_events_without_centering(self):
        self.worker.command({'action': 'record'})
        self.worker.command({'action': 'emergency_stop'})
        self.assertEqual(self.worker.result, 'ABORTED')
        self.socket.send.assert_not_called()
        self.brunner.write_setting.assert_not_called()

    def test_timeout_stops_without_reaching_indicator_endpoint(self):
        self.worker.command({'action': 'record'})
        self.reader.poll.return_value = self.frame(1, 0.)
        self.worker.step()
        self.now += 31.
        self.reader.poll.return_value = self.frame(2, .05)
        with patch('reports.automatic_control_report.save', return_value=[]):
            self.worker.step()
        self.assertEqual(self.worker.result, 'FAILED')
        self.assertIn('timeout', self.worker.details)
        self.socket.send.assert_not_called()

    def test_no_p3d_trim_movement_fails_with_clear_diagnostic(self):
        self.worker.command({'action': 'record'})
        self.now += .2
        self.reader.poll.return_value = self.frame(1, 0.)
        self.worker.step()
        self.now += 3.1
        self.reader.poll.return_value = self.frame(2, 0.)
        with patch('reports.automatic_control_report.save', return_value=[]):
            self.worker.step()
        self.assertEqual(self.worker.result, 'FAILED')
        self.assertIn('P3D elevator trim did not move', self.worker.details)

    def test_real_csv_pdf_contains_trim_graphs(self):
        import fitz
        from reports.automatic_control_report import save
        with tempfile.TemporaryDirectory() as folder:
            data = dict(context(), result_directory=folder, title='Pitch trim', selected_graphs=['trim_position_rad','trim_indicator','trim_elevator_surface_rad'])
            rows = [{'Time (s)': t, 'Elevator Trim Angle (rad)': t / 10,
                     'Elevator Trim Indicator (position)': -.5, 'Elevator Trim (%)': 10.,
                     'Elevator Surface (rad)': .1} for t in (0., 1., 2.)]
            outputs = save(data, rows, 'COMPLETE', 'offline')
            self.assertIn('Elevator Trim Indicator (position)', Path(outputs[0]).read_text())
            with fitz.open(outputs[2]) as doc:
                self.assertEqual(len(doc), 2)
                page = doc[1].get_text()
                self.assertIn('Elevator Trim Indicator', page)
                self.assertIn('Elevator Surface', page)

    def test_gui_dispatches_only_pitch_trim_automatic(self):
        from frontend.main import App, TESTS
        import qtg_common_engine as engine
        spec = next(item for item in TESTS if item.reference == '2.a.(4)')
        values = {'Flt cond': 'Clean', 'Alt[ft]': '2000', 'IAS[kts]': '80',
                  'Latitude[deg]': '32', 'Longitude[deg]': '74', 'Heading[deg]': '0',
                  'Pitch[deg]': '0', 'Bank[deg]': '0', 'Flaps': '0', 'Throttle[%]': '80', 'Temperature[C]': '15'}
        fields = {key: Mock(get=Mock(return_value=value)) for key, value in values.items()}
        host = SimpleNamespace(_common_duration_var=Mock(get=Mock(return_value='75')),
            _test_settings={spec.reference: (fields, [])}, _apply_test_settings=Mock(return_value=True),
            _common_reference_var=Mock(get=Mock(return_value='')), _common_process=None,
            common_record_button=Mock(), common_status=Mock(), after=Mock(), _poll_common_test=Mock())
        with patch.object(engine, 'launch', return_value=(Mock(), {'status_file': 'dummy'})) as launch:
            App._launch_common_test(host, 'auto', spec)
        self.assertEqual(launch.call_args.kwargs['control_method'], 'pitch_trim')
        self.assertEqual(launch.call_args.args[1].reference, '2.a.(4)')

    def test_worker_end_to_end_ready_record_and_shutdown(self):
        brunner = Mock()
        brunner.read_axis.return_value = (0., 0., 0.)
        class Events:
            direction = None
            def step(self, direction): self.direction = direction
        events = Events()
        sim = Mock()
        sim.is_connected.return_value = True
        sim.get_telemetry_sequence.return_value = 1
        sim.queue_position_airborne.return_value = sim.set_initial_throttle.return_value = True
        sim.wait_for_fresh_telemetry.return_value = {'engine_throttle_percent': 5., 'airspeed_indicated': 0., 'indicated_altitude': 755.}
        sim.get_telemetry.return_value = dict(sim.wait_for_fresh_telemetry.return_value, engine_rpm=2200.)
        class Reader:
            angle = 0.
            sequence = 0
            def poll(self):
                self.sequence += 1
                if events.direction == UP: self.angle = min(.4, self.angle + .1)
                if events.direction == DOWN: self.angle = max(-.4, self.angle - .1)
                return dict(sequence=self.sequence, receipt_time=time.monotonic(), elevator_rad=self.angle,
                            trim_rad=self.angle/2, indicator=self.angle/.4, trim_fraction=self.angle/.4)
            def close(self): pass
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            commands = root / 'commands'; commands.mkdir()
            cfg = {'on_ground': False, 'alt_ft': 755., 'ias_kt': 0., 'lat': 32., 'lon': 74., 'heading_deg': 0., 'throttle': 5.}
            ctx = dict(context(), initial_conditions=cfg, title='Pitch trim', result_directory=folder,
                       command_directory=str(commands), stop_file=str(root/'stop'),
                       selected_graphs=['trim_position_rad', 'trim_indicator', 'trim_elevator_surface_rad'])
            states = []
            def publish(state):
                states.append(state)
                if state['stage'] == 'READY TO RECORD' and not any(p.name == 'record.sent' for p in root.iterdir()):
                    (root/'record.sent').touch()
                    (commands/'01.json').write_text('{"action":"record"}')
                if state['stage'] in ('Completed', 'FAILED'):
                    (commands/'02.json').write_text('{"action":"shutdown"}')
            result = run(ctx, publish, sim_factory=lambda: sim, brunner_factory=lambda: brunner,
                         reader_factory=Reader, events_factory=lambda reader: events)
            self.assertEqual(result, 0)
            completed_states = [state for state in states if state['stage'] == 'Completed']
            self.assertTrue(completed_states, [(s['stage'], s.get('details')) for s in states[-5:]])
            completed = completed_states[0]
            self.assertTrue(any(path.endswith('.csv') for path in completed['outputs']))
            self.assertTrue(any(path.endswith('.pdf') for path in completed['outputs']))
            brunner.write_setting.assert_not_called()
