import unittest
import tempfile
import csv
from pathlib import Path
import fitz
from unittest.mock import Mock, patch

from backend.automatic_stall import AutomaticStall


def sample(sequence, warning, now):
    return dict(sequence=sequence, receipt_time=now, stall_warning=warning, ias_kt=80,
                altitude_ft=4000, aoa_deg=5)


class AutomaticStallTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.sim = Mock()
        self.sim.is_connected.return_value = True
        self.sim.simulation_running.return_value = True
        self.sim.set_replay_throttle.return_value = True
        self.sim.get_telemetry_sequence.return_value = 3
        self.sim.wait_for_fresh_telemetry.return_value = dict(pitch_angle=1., bank_angle=0.,
            engine_throttle_percent=0., engine_rpm=1800, surface_elevator_rad=.1)
        self.brunner = Mock()
        self.brunner.sock = Mock()
        self.brunner.AXIS_PITCH = 1
        self.brunner.AXIS_ROLL = 2
        self.brunner.AXIS_RUDDER = 4
        self.brunner.read_axis.side_effect = lambda axis: (
            self.controller.position if axis == self.brunner.AXIS_PITCH else 0., .1, .2)
        self.reader = Mock()
        self.context = dict(mode='auto', test_reference='2.c.(8b)', control_method='stall',
            initial_conditions={'on_ground':False}, result_directory='unused')
        self.controller = AutomaticStall(self.context, self.sim, self.brunner, Mock(), self.reader, clock=lambda:self.now)
        self.controller.phase, self.controller.stage = 'READY', 'READY TO RECORD'
        self.controller.latest_sim = dict(pitch_angle=1., bank_angle=0.)
        self.patches = [patch('reports.automatic_control_report.save', return_value=['result.csv','result.pdf'])]
        for p in self.patches: p.start(); self.addCleanup(p.stop)

    def test_warning_on_then_off_completes_and_keeps_roll_yaw_uncommanded(self):
        self.reader.poll.side_effect = lambda: sample(int(round(self.now * 10)),
            1 if 10.15 <= self.now < 10.25 else 0, self.now)
        self.controller.command({'action':'record'})
        self.sim.set_replay_throttle.assert_called_once_with(0.)
        for tick in range(1, 26):
            self.now = 10. + tick * .1
            self.controller.step()
            if tick == 3:
                self.assertEqual(self.controller.stage, 'RECORDING')
            if self.controller.phase == 'DONE':
                break
        self.assertEqual(self.controller.stage, 'Completed')
        self.assertEqual([row['Stall Warning (0/1)'] for row in self.controller.rows[:4]], [0, 0, 1, 0])
        self.assertGreaterEqual(self.controller.rows[-1]['Time (s)'], 2.3)
        self.assertEqual(self.brunner.move_actuators.call_args_list[-1].args, (0., None, None))
        self.brunner.set_overrides.assert_called_with(self.brunner.AXIS_PITCH, False)
        self.brunner.release_controls.assert_not_called()
        self.assertEqual(self.controller.outputs, ['result.csv','result.pdf'])

    def test_initial_row_precedes_idle_throttle_command(self):
        self.controller.latest_sim.update(engine_throttle_percent=50., total_weight=850.)
        self.reader.poll.return_value = sample(1, 0, 10.)
        self.controller.command({'action': 'record'})
        self.assertEqual(self.controller.rows[0]['Time (s)'], 0.)
        self.assertEqual(self.controller.rows[0]['Throttle 1 Position (%)'], 50.)
        self.assertEqual(self.controller.rows[0]['Gross Wt (kg)'], 850.)
        self.sim.set_replay_throttle.assert_called_once_with(0.)

    def test_recording_saves_csv_with_elevator_surface_in_every_row(self):
        with tempfile.TemporaryDirectory() as folder:
            self.controller.context['result_directory'] = folder
            self.reader.poll.side_effect = [sample(1, 0, 10.), sample(2, 0, 10.1)]
            self.controller.command({'action': 'record'})
            self.now = 10.1
            self.controller.step()
            self.patches[0].stop()
            self.controller.finish('COMPLETE', 'offline report check')
            self.assertEqual(self.controller.stage, 'Completed')
            with open(self.controller.outputs[0], newline='', encoding='utf-8') as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 2)
            self.assertIn('Elevator Surface (rad)', rows[0])
            self.assertEqual(rows[1]['Elevator Surface (rad)'], '0.1')

    def test_warning_clear_waits_for_physical_pitch_neutral(self):
        self.reader.poll.side_effect = lambda: sample(int(round(self.now * 10)),
            1 if 10.05 <= self.now < 10.15 else 0, self.now)
        self.brunner.read_axis.side_effect = lambda axis: (
            .2 if self.now < 12.4 else 0., .1, .2)
        self.controller.command({'action': 'record'})
        for tick in range(1, 25):
            self.now = 10. + tick * .1
            self.controller.step()
            if tick == 23:
                self.assertEqual(self.controller.stage, 'RECORDING')
        self.assertEqual(self.controller.stage, 'Completed')
        self.assertEqual(self.brunner.move_actuators.call_args_list[-1].args, (0., None, None))

    def test_warning_already_on_blocks_movement(self):
        self.reader.poll.return_value = sample(1, 1, 10.)
        self.controller.command({'action':'record'})
        self.assertEqual(self.controller.stage, 'FAILED')
        self.brunner.configure_replay_mode.assert_not_called()
        self.sim.set_replay_throttle.assert_not_called()

    def test_stop_releases_pitch_and_does_not_command_center(self):
        self.reader.poll.return_value = sample(1, 0, 10.)
        self.controller.command({'action':'record'})
        self.controller.command({'action':'stop'})
        self.assertEqual(self.controller.stage, 'ABORTED')
        self.brunner.move_actuators.assert_not_called()
        self.brunner.set_overrides.assert_called_with(self.brunner.AXIS_PITCH, False)
        self.brunner.release_controls.assert_not_called()

    def test_stale_warning_does_not_trigger_completion(self):
        self.reader.poll.side_effect = [sample(1, 0, 10.), sample(2, 1, 10.1), sample(3, 0, 9.)]
        self.controller.command({'action':'record'})
        self.now = 10.1
        self.controller.step()
        self.now = 10.2
        self.controller.step()
        self.assertEqual(self.controller.stage, 'FAILED')
        self.assertIn('stale', self.controller.details)
        self.brunner.set_overrides.assert_called_with(self.brunner.AXIS_PITCH, False)

    def test_full_initialization_to_ready_to_record(self):
        self.sim.reset_mock()
        self.sim.is_connected.return_value = True
        self.sim.simulation_running.side_effect = RuntimeError('Simulator pause/running state was not confirmed within 0.25 seconds.')
        self.sim.get_telemetry_sequence.return_value = 1
        self.sim.set_initial_throttle.return_value = True
        self.sim.queue_position_airborne.return_value = True
        self.reader.poll.return_value = sample(1, 0, 10.)

        fresh_telemetry = {
            'airspeed_indicated': 90.0,
            'indicated_altitude': 4000.0,
            'engine_throttle_percent': 80.0,
            'pitch_angle': 2.0,
            'bank_angle': 0.0,
        }
        self.sim.wait_for_fresh_telemetry.return_value = fresh_telemetry

        context = dict(
            mode='auto',
            test_reference='2.c.(8b)',
            control_method='stall',
            initial_conditions={
                'alt_ft': 4000.0,
                'ias_kt': 90.0,
                'lat': 32.233667,
                'lon': 74.1235,
                'heading_deg': 48.4,
                'throttle': 80.0,
                'on_ground': False,
            },
            result_directory='unused'
        )
        ctrl = AutomaticStall(context, self.sim, self.brunner, Mock(), self.reader, clock=lambda: self.now)
        self.assertEqual(ctrl.phase, 'CONNECT')

        # First step: queues airborne position, sets 80% throttle, unpauses sim
        ctrl.step()
        self.reader.poll.assert_not_called()
        self.sim.queue_position_airborne.assert_called_once_with(
            alt_ft=4000.0, speed_kts=90.0, lat=32.233667, lon=74.1235,
            hdg_deg=48.4, pitch_deg=0.0, bank_deg=0.0, on_ground=False
        )
        self.sim.set_initial_throttle.assert_called_once_with(80.0)
        self.sim.unpause_sim.assert_called_once()
        self.assertTrue(ctrl.initialized)

        # Second step: verifies readback and pauses sim
        self.sim.get_telemetry_sequence.return_value = 2
        ctrl.step()
        self.assertEqual(ctrl.phase, 'READY')
        self.assertEqual(ctrl.stage, 'READY TO RECORD')
        self.sim.pause_sim.assert_called_once()
        self.sim.simulation_running.assert_not_called()
        self.assertFalse(ctrl.status()['can_record'])
        ctrl.step()
        self.reader.poll.assert_called_once()
        status = ctrl.status()
        self.assertTrue(status['can_record'])
        self.assertEqual(status['recording_text'], 'READY TO RECORD')
        self.assertIn('stall approach', status['progress_text'])

    def test_long_initialization_does_not_start_stall_measurement_request(self):
        self.controller.phase = 'CONNECT'
        def initialize():
            self.now += 3.0
            self.controller.phase = 'READY'
            self.controller.stage = 'READY TO RECORD'
        with patch('backend.automatic_control.AutomaticControl.step', side_effect=initialize):
            self.controller.step()
        self.reader.poll.assert_not_called()
        self.reader.poll.return_value = sample(1, 0, self.now)
        self.controller.step()
        self.assertTrue(self.controller.status()['can_record'])

    def test_ready_recording_lock_rejects_stale_or_active_warning(self):
        self.controller.ready_sample = sample(1, 0, self.now - 2)
        self.assertFalse(self.controller.status()['can_record'])
        self.controller.ready_sample = sample(2, 1, self.now)
        self.assertFalse(self.controller.status()['can_record'])
        self.controller.ready_sample = sample(3, 0, self.now)
        self.assertTrue(self.controller.status()['can_record'])

    def test_throttle_not_reaching_idle_aborts(self):
        self.reader.poll.side_effect = lambda: sample(int(self.now * 10), 0, self.now)
        # Telemetry reports throttle remaining at 50% instead of 0%
        self.sim.wait_for_fresh_telemetry.return_value = dict(
            pitch_angle=1., bank_angle=0., engine_throttle_percent=50.,
            engine_rpm=2400, surface_elevator_rad=.1
        )
        self.controller.command({'action': 'record'})
        for step_idx in range(35):
            self.now = 10.0 + (step_idx + 1) * 0.1
            self.controller.step()
            if self.controller.stage == 'FAILED':
                break
        self.assertEqual(self.controller.stage, 'FAILED')
        self.assertIn('throttle did not reach idle', self.controller.details)
        self.brunner.set_overrides.assert_called_with(self.brunner.AXIS_PITCH, False)

    def test_overall_stall_timeout_aborts(self):
        self.reader.poll.side_effect = lambda: sample(int(self.now * 10), 0, self.now)
        self.brunner.read_axis.side_effect = lambda *a: (self.controller.position, 0.1, 0.2)
        self.controller.command({'action': 'record'})
        for step_idx in range(920):
            self.now = 10.0 + (step_idx + 1) * 0.1
            self.controller.step()
            if self.controller.stage == 'FAILED':
                break
        self.assertEqual(self.controller.stage, 'FAILED')
        self.assertIn('timed out', self.controller.details)
        self.brunner.set_overrides.assert_called_with(self.brunner.AXIS_PITCH, False)

    def test_pitch_loader_mismatch_aborts(self):
        self.reader.poll.side_effect = lambda: sample(int(self.now * 10), 0, self.now)
        # Motor stuck at 0.0 while commanded pitches up
        self.brunner.read_axis.side_effect = lambda axis: (0.0, 0.1, 0.2)
        self.controller.command({'action': 'record'})
        for step_idx in range(400):
            self.now = 10.0 + (step_idx + 1) * 0.1
            self.controller.step()
            if self.controller.stage == 'FAILED':
                break
        self.assertEqual(self.controller.stage, 'FAILED')
        self.assertIn('did not track', self.controller.details)
        self.brunner.set_overrides.assert_called_with(self.brunner.AXIS_PITCH, False)

    def test_initial_attitude_does_not_block_recording(self):
        self.controller.latest_sim['pitch_angle'] = -30
        self.controller.latest_sim['bank_angle'] = 25
        self.reader.poll.return_value = sample(1, 0, 10.)
        self.controller.command({'action':'record'})
        self.assertEqual(self.controller.stage, 'RECORDING')
        self.brunner.configure_replay_mode.assert_called_once_with(self.brunner.AXIS_PITCH)

    def test_bank_over_twenty_does_not_abort_recording(self):
        self.reader.poll.side_effect = [sample(1, 0, 10.), sample(2, 0, 10.1)]
        self.sim.wait_for_fresh_telemetry.return_value['bank_angle'] = 25.
        self.controller.command({'action':'record'})
        self.now = 10.1
        self.controller.step()
        self.assertEqual(self.controller.stage, 'RECORDING')

    def test_recorded_fields_use_actual_telemetry(self):
        first = sample(1, 0, 10.)
        second = dict(sample(2, 0, 10.1), cg_long_fraction=.24, cg_lateral_fraction=.01)
        self.reader.poll.side_effect = [first, second]
        self.sim.wait_for_fresh_telemetry.return_value.update(
            prop_rpms=[2200.], total_fuel_quantity=112., baro_pressure=1012.,
            ambient_temp=15., accelY=-32.1740485564)
        self.controller.command({'action':'record'})
        self.now = 10.1
        self.controller.step()
        row = self.controller.rows[1]
        self.assertEqual(row['Propeller 1 RPM'], 2200.)
        self.assertEqual(row['Total Fuel Quantity (kg)'], 112.)
        self.assertEqual(row['CG Longitudinal (% reference chord)'], 24.)
        self.assertEqual(row['Y Body Acceleration (g)'], -1.)
        self.assertEqual(row['Joystick Roll Force (N)'], 6.)
        self.assertEqual(row['Rudder Pedal Force (N)'], 8.)

    def test_pitch_pulls_back_to_full_travel_and_holds_until_warning(self):
        self.reader.poll.side_effect = lambda: sample(int(round(self.now * 10)), 0, self.now)
        self.controller.command({'action': 'record'})
        positions = []
        for tick in range(1, 61):
            self.now = 10. + tick * .1
            self.controller.step()
            positions.append(self.brunner.move_actuators.call_args.args[0])
        self.assertGreater(positions[0], 0.)
        self.assertTrue(all(a <= b for a, b in zip(positions, positions[1:])))
        self.assertAlmostEqual(positions[-1], 1.)
        self.assertEqual(self.controller.stage, 'RECORDING')
        self.brunner.configure_replay_mode.assert_called_once_with(self.brunner.AXIS_PITCH)

    def test_warning_reverses_pitch_command_toward_neutral(self):
        self.reader.poll.side_effect = lambda: sample(int(round(self.now * 10)),
            1 if self.now >= 10.4 else 0, self.now)
        self.controller.command({'action': 'record'})
        for tick in range(1, 4):
            self.now = 10. + tick * .1
            self.controller.step()
        pulled_back = self.controller.position
        self.now = 10.4
        self.controller.step()
        self.assertLess(self.controller.position, pulled_back)
        self.assertEqual(self.controller.stage, 'RECORDING')

    def test_emergency_stop_releases_pitch_without_centering(self):
        self.reader.poll.return_value = sample(1, 0, 10.)
        self.controller.command({'action': 'record'})
        self.controller.command({'action': 'emergency_stop'})
        self.assertEqual(self.controller.stage, 'ABORTED')
        self.assertIn('operator', self.controller.details.lower())
        self.brunner.move_actuators.assert_not_called()
        self.brunner.set_overrides.assert_called_with(self.brunner.AXIS_PITCH, False)
        self.brunner.release_controls.assert_not_called()

    def test_gui_launch_stall_dispatch(self):
        from frontend.main import App, TESTS
        from types import SimpleNamespace
        import qtg_common_engine as engine

        spec = next(s for s in TESTS if s.reference == '2.c.(8b)')
        values = {
            'Flt cond': 'Approach', 'Alt[ft]': '4000', 'IAS[kts]': '90',
            'Latitude[deg]': '32.0', 'Longitude[deg]': '74.0',
            'Heading[deg]': '48.0', 'Pitch[deg]': '0', 'Bank[deg]': '0',
            'Flaps': '0', 'Throttle[%]': '80', 'Temperature[C]': '15'
        }
        fields = {k: Mock(get=lambda v=v: v) for k, v in values.items()}
        host = SimpleNamespace(
            _common_duration_var=Mock(get=lambda: '75'),
            _common_duration_vars={'2.c.(8b)': Mock(get=lambda: '75')},
            _test_settings={'2.c.(8b)': (fields, {})},
            _test_defaults={'2.c.(8b)': values},
            _apply_test_settings=Mock(return_value=True),
            _common_reference_var=Mock(get=lambda: ''),
            common_status=Mock(),
            common_record_button=Mock(),
            after=Mock(),
            _poll_common_test=Mock(),
            common_reference_frame=Mock(winfo_exists=lambda: False),
        )

        with patch.object(engine, 'launch', return_value=(Mock(), {})) as mock_launch:
            App._launch_common_test(host, 'auto', spec)

        mock_launch.assert_called_once()
        kwargs = mock_launch.call_args.kwargs
        self.assertEqual(kwargs['control_method'], 'stall')
        self.assertIsNotNone(kwargs['initial_conditions'])
        self.assertEqual(kwargs['initial_conditions']['alt_ft'], 4000.0)
        self.assertEqual(kwargs['initial_conditions']['ias_kt'], 90.0)
        self.assertEqual(kwargs['initial_conditions']['throttle'], 80.0)


class AutomaticStallReportTests(unittest.TestCase):
    def test_missing_validation_is_not_fabricated(self):
        import pandas as pd
        from reports.automatic_control_report import _build_reference_df
        context = {'test_reference':'2.c.(8b)', 'initial_conditions':{'throttle':50, 'ias_kt':90}}
        reference = _build_reference_df(context, pd.DataFrame([{'Engine 1 RPM':2600}]))
        self.assertIsNone(reference)

    def test_real_validation_csv_is_used(self):
        import pandas as pd
        from reports.automatic_control_report import _build_reference_df
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder)/'validation.csv'
            source.write_text('Engine 1 RPM\n2200\n', encoding='utf-8')
            reference = _build_reference_df({'test_reference':'2.c.(8b)', 'validation_csv':str(source)}, pd.DataFrame())
            self.assertEqual(reference.iloc[0]['Engine 1 RPM'], 2200)

    def test_warning_graph_and_metadata_are_written(self):
        from reports.automatic_control_report import save
        with tempfile.TemporaryDirectory() as folder:
            context = {'test_reference':'2.c.(8b)', 'title':'Stall warning',
                       'result_directory':folder, 'selected_graphs':['stall_warning', 'ias_kt']}
            rows = [{'Time (s)':0, 'Stall Warning (0/1)':0, 'Indicated Airspeed (kts)':90},
                    {'Time (s)':1, 'Stall Warning (0/1)':1, 'Indicated Airspeed (kts)':89}]
            outputs = save(context, rows, 'COMPLETE', 'Warning appeared and cleared')
            pdf = next(Path(path) for path in outputs if path.endswith('.pdf'))
            with fitz.open(pdf) as document:
                self.assertIn('Stall Warning (0/1)', ''.join(page.get_text() for page in document))
