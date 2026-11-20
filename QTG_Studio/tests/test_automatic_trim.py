"""Offline Pitch Trim worker and SimConnect trim-command coverage."""
import ctypes
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from backend.automatic_trim import AutomaticTrim, TrimEvents, UP, DOWN, SET_TRIM, selected, run


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
        self.worker.start_trim_pos = 1.0
        self.worker.verified_nose_up_limit = 1.0

    def frame(self, sequence, angle, indicator=0.):
        return dict(sequence=sequence, receipt_time=self.now, trim_rad=angle / 2,
                    indicator=indicator, trim_fraction=indicator, elevator_rad=angle)

    def test_simconnect_uses_trim_events_and_encodes_signed_targets(self):
        self.assertTrue(selected(context()))
        reader = Mock()
        reader.handle = ctypes.c_void_p(4)
        reader.dll.SimConnect_MapClientEventToSimEvent.return_value = 0
        reader.dll.SimConnect_TransmitClientEvent.return_value = 0
        events = TrimEvents(reader)
        events.step(UP); events.step(DOWN)
        events.set_fraction(-0.001)
        events.set_fraction(0.001)
        self.assertEqual(reader.dll.SimConnect_MapClientEventToSimEvent.call_args_list[0].args[2], b'ELEV_TRIM_UP')
        self.assertEqual(reader.dll.SimConnect_MapClientEventToSimEvent.call_args_list[1].args[2], b'ELEV_TRIM_DN')
        self.assertEqual(reader.dll.SimConnect_MapClientEventToSimEvent.call_args_list[2].args[2], b'AXIS_ELEV_TRIM_SET')
        sent = reader.dll.SimConnect_TransmitClientEvent.call_args_list
        self.assertEqual([call.args[2] for call in sent], [UP, DOWN, SET_TRIM, SET_TRIM])
        self.assertEqual([call.args[3] for call in sent[2:]], [16, (-16) & 0xFFFFFFFF])
        self.socket.send.assert_not_called()
        self.brunner.write_setting.assert_not_called()

    def test_aircraft_initialization_sets_trim_to_full_nose_up_and_waits_1s(self):
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

        # 1. P3D conditions initialization
        worker.step()
        self.assertEqual(worker.stage, 'Initializing aircraft')
        worker.step()
        # Aircraft positioning verified, intercepted into DRIVE_UP
        self.assertEqual(worker.init_trim_phase, 'DRIVE_UP')

        # 2. Drive trim up to verified limit
        self.now += 0.05
        self.reader.poll.return_value = self.frame(1, 0.1, indicator=0.999)
        worker.step()
        # Limit reached, transitions to WAIT_1S
        self.assertEqual(worker.init_trim_phase, 'WAIT_1S')
        self.assertAlmostEqual(worker.verified_nose_up_limit, 0.999)

        # 3. Wait 1 second and confirm starting position
        self.now += 0.5
        worker.step()
        self.assertNotEqual(worker.stage, 'READY TO RECORD')

        self.now += 0.6
        worker.step()
        self.assertEqual(worker.stage, 'READY TO RECORD')
        self.assertTrue(worker.status()['can_record'])
        self.assertAlmostEqual(worker.start_trim_pos, 0.999)
        self.socket.send.assert_not_called()
        self.brunner.write_setting.assert_not_called()

    def test_step_by_step_nose_up_to_nose_down_sequence(self):
        self.brunner.read_axis.return_value = (.12, .03, 0.)
        self.worker.command({'action': 'record'})
        self.assertEqual(self.worker.record_state, 'RECORDING_START')

        with patch('reports.automatic_control_report.save', return_value=['test.csv', 'test.qtg_test.json', 'test.pdf']) as save:
            # Step 1: Initial baseline frame at t=0
            self.reader.poll.return_value = self.frame(1, -.1, indicator=1.0)
            self.worker.step()
            self.events.set_fraction.assert_called_once_with(0.999)
            self.assertEqual(self.worker.record_state, 'WAIT_STEP')
            self.assertEqual(self.worker.step_count, 1)

            # Wait during Step 1 (e.g. 0.5s elapsed)
            self.now += 0.5
            self.reader.poll.return_value = self.frame(2, -.1, indicator=0.999)
            self.worker.step()
            self.assertEqual(self.worker.record_state, 'WAIT_STEP')
            self.assertEqual(self.worker.step_count, 1)

            for step in range(1, 2001):
                self.now += 0.5 if step == 1 else 1.0
                measured = 1.0 - step * 0.001
                self.reader.poll.return_value = self.frame(step + 2, -.1, indicator=measured)
                self.worker.step()
                if step < 2000:
                    self.assertEqual(self.worker.step_count, step + 1)
                    self.assertAlmostEqual(self.events.set_fraction.call_args.args[0], measured - 0.001)
            self.assertEqual(self.events.set_fraction.call_count, 2000)
            self.events.step.assert_not_called()
            self.assertEqual(self.worker.record_state, 'HOLD_FINAL')
            self.assertAlmostEqual(self.worker.verified_nose_down_limit, -1.0)
            self.assertNotEqual(self.worker.result, 'COMPLETE')

            # Holding final position: 0.5s into hold
            self.now += 0.5
            self.reader.poll.return_value = self.frame(2003, -.1, indicator=-1.0)
            self.worker.step()
            self.assertEqual(self.worker.record_state, 'HOLD_FINAL')
            self.assertNotEqual(self.worker.result, 'COMPLETE')

            # 1.0s final hold complete: automatically finishes and generates reports
            self.now += 0.55
            self.reader.poll.return_value = self.frame(2004, -.1, indicator=-1.0)
            self.worker.step()
            self.assertEqual(self.worker.result, 'COMPLETE')
            self.assertEqual(self.worker.outputs, ['test.csv', 'test.qtg_test.json', 'test.pdf'])
            self.assertGreater(self.worker.total_travel_time, 0)
            self.assertGreater(self.worker.average_trim_rate_pct, 0)
            save.assert_called_once()

        self.socket.send.assert_not_called()
        self.brunner.write_setting.assert_not_called()

    def test_stale_telemetry_stops_trim_events_and_fails(self):
        self.worker.command({'action': 'record'})
        self.reader.poll.return_value = self.frame(1, 0.)
        self.worker.step()
        self.now += 4.5
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

    def test_unresponsive_trim_stops_and_fails_safely(self):
        self.worker.command({'action': 'record'})
        self.reader.poll.return_value = self.frame(1, 0., indicator=0.8)
        self.worker.step()
        self.assertEqual(self.worker.step_count, 1)

        # Step 1 expires with no movement (trim remains at 0.8, which is NOT nose-down limit)
        self.now += 1.05
        self.reader.poll.return_value = self.frame(2, 0., indicator=0.8)
        self.worker.step()
        self.assertEqual(self.worker.unresponsive_steps, 1)

        # Retry 1 expires with no movement
        self.now += 1.05
        self.reader.poll.return_value = self.frame(3, 0., indicator=0.8)
        self.worker.step()
        self.assertEqual(self.worker.unresponsive_steps, 2)

        # Retry 2 expires with no movement -> Exceeds UNRESPONSIVE_LIMIT (3) and fails
        self.now += 1.05
        self.reader.poll.return_value = self.frame(4, 0., indicator=0.8)
        with patch('reports.automatic_control_report.save', return_value=[]):
            self.worker.step()
        self.assertEqual(self.worker.result, 'FAILED')
        self.assertIn('did not reach', self.worker.details)

    def test_real_csv_pdf_contains_trim_graphs_and_metrics(self):
        import fitz
        from reports.automatic_control_report import save
        with tempfile.TemporaryDirectory() as folder:
            data = dict(context(), result_directory=folder, title='Pitch trim',
                        selected_graphs=['trim_position_rad', 'trim_indicator', 'trim_value_pct', 'trim_elevator_surface_rad'],
                        pitch_trim_metrics={'total_travel_time_s': 20.0, 'average_trim_rate_pct_s': 10.0, 'average_trim_rate_pos_s': 0.1,
                                            'start_trim_pos': 1.0, 'final_trim_pos': -1.0})
            rows = [{'Time (s)': t, 'Elevator Trim Angle (rad)': (1.0 - t / 10.0) * 0.2,
                     'Elevator Trim Indicator (position)': 1.0 - t / 10.0, 'Elevator Trim (%)': (1.0 - t / 10.0) * 100.,
                     'Elevator Surface (rad)': .1} for t in (0., 1., 2.)]
            outputs = save(data, rows, 'COMPLETE', 'offline')
            self.assertIn('Elevator Trim Indicator (position)', Path(outputs[0]).read_text())
            with fitz.open(outputs[2]) as doc:
                self.assertGreaterEqual(len(doc), 2)
                page = doc[1].get_text()
                self.assertIn('Elevator Trim Indicator', page)
                self.assertIn('Elevator Surface', page)
                self.assertIn('Total Travel Time', page)
                self.assertIn('Average Trim Rate', page)

    def test_gui_dispatches_only_pitch_trim_automatic(self):
        from frontend.main import App, TESTS
        import qtg_common_engine as engine
        spec = next(item for item in TESTS if item.reference in ('2.a.(7)', '2.a.(4)'))
        values = {'Flt cond': 'Clean', 'Alt[ft]': '2000', 'IAS[kts]': '80',
                  'Latitude[deg]': '32', 'Longitude[deg]': '74', 'Heading[deg]': '0',
                  'Pitch[deg]': '0', 'Bank[deg]': '0', 'Flaps': '0', 'Throttle[%]': '80', 'Temperature[C]': '15'}
        fields = {key: Mock(get=Mock(return_value=value)) for key, value in values.items()}
        host = SimpleNamespace(_common_duration_var=Mock(get=Mock(return_value='75')),
            _test_settings={spec.reference: (fields, [])}, _apply_test_settings=Mock(return_value=True),
            _common_reference_var=Mock(get=Mock(return_value='')), _common_process=None,
            common_record_button=Mock(), common_status=Mock(), after=Mock(), _poll_common_test=Mock(),
            _pid_axis_checks={spec.reference: {}})
        with patch.object(engine, 'launch', return_value=(Mock(), {'status_file': 'dummy'})) as launch:
            App._launch_common_test(host, 'auto', spec)
        self.assertEqual(launch.call_args.kwargs['control_method'], 'pitch_trim')
        self.assertEqual(launch.call_args.args[1].reference, spec.reference)

    def test_worker_end_to_end_ready_record_and_shutdown(self):
        brunner = Mock()
        brunner.read_axis.return_value = (0., 0., 0.)
        class Events:
            direction = None
            target = None
            def step(self, direction): self.direction = direction
            def set_fraction(self, target): self.target = target
        events = Events()
        sim = Mock()
        sim.is_connected.return_value = True
        sim.get_telemetry_sequence.return_value = 1
        sim.queue_position_airborne.return_value = sim.set_initial_throttle.return_value = True
        sim.wait_for_fresh_telemetry.return_value = {'engine_throttle_percent': 5., 'airspeed_indicated': 0., 'indicated_altitude': 755.}
        sim.get_telemetry.return_value = dict(sim.wait_for_fresh_telemetry.return_value, engine_rpm=2200.)
        class Reader:
            angle = 0.4
            sequence = 0
            def poll(self):
                self.sequence += 1
                if events.direction == UP: self.angle = min(.4, self.angle + .05)
                if events.target is not None: self.angle = events.target * .4
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
            with patch('backend.automatic_trim.TRIM_STEP_FRACTION', .5):
                result = run(ctx, publish, sim_factory=lambda: sim, brunner_factory=lambda: brunner,
                             reader_factory=Reader, events_factory=lambda reader: events)
            self.assertEqual(result, 0)
            completed_states = [state for state in states if state['stage'] == 'Completed']
            self.assertTrue(completed_states, [(s['stage'], s.get('details')) for s in states[-5:]])
            completed = completed_states[0]
            self.assertTrue(any(path.endswith('.csv') for path in completed['outputs']))
            self.assertTrue(any(path.endswith('.pdf') for path in completed['outputs']))
            brunner.write_setting.assert_not_called()
