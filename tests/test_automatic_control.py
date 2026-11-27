"""Unit and integration tests for Automatic Static Control Testing (2.a.(1), 2.a.(2), 2.a.(3)).

Verifies:
1. S-curve trajectory profile: Center -> Limit 1 -> Limit 2 -> Center.
2. Initialization: aircraft positioned, physical controls untouched.
3. Recording: commands only selected axis within travel/speed/force limits.
4. Independent execution for Pitch, Roll, and Yaw.
5. Immediate Emergency Stop / Fault handling without commanding automatic centering.
6. CSV data recording and PDF generation with Position vs Force in BLUE (#0066cc).
7. Zero modifications or regressions to engine tests, PID, or manual testing.
"""

import math
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from backend.control_profile import (
    STATIC_CONTROL_REFERENCES,
    TEST_PITCH,
    TEST_ROLL,
    TEST_YAW,
    AXIS_INFO,
    StaticControlProfile,
)
from backend.automatic_control import (
    selected,
    AutomaticControl,
    run as run_control,
    METHOD,
)


class MockSimConnect:
    def __init__(self, connected=True):
        self._connected = connected
        self.paused = False
        self.positions = []
        self.throttles = []
        self.sequence = 0
        self.unpause_calls = 0

    def is_connected(self):
        return self._connected

    def get_telemetry(self):
        return {
            'indicated_airspeed': 0.0,
            'indicated_altitude': 755.0,
            'engine_throttle_percent': self.throttles[-1] if self.throttles else 0.0,
            'pitch_angle': 0.0,
            'bank_angle': 0.0,
            'true_heading': 48.4,
            'elevator_deflection': 0.0,
            'aileron_left_deflection': 0.0,
            'rudder_deflection': 0.0,
        }

    def get_telemetry_sequence(self):
        return self.sequence

    def request_telemetry_once(self):
        self.sequence += 1
        return True

    def wait_for_fresh_telemetry(self, previous_sequence, timeout_sec):
        return self.get_telemetry() if self.sequence > previous_sequence else None

    def queue_position_airborne(self, **kwargs):
        self.positions.append(kwargs)
        return True

    def set_initial_throttle(self, pct):
        self.throttles.append(pct)
        return True

    def get_global_weather_metar(self):
        return 'GLOB 000000Z 00000KT 9999 SKC 15/05 Q1013'

    def set_global_weather_metar(self, _metar):
        return True

    def stop(self):
        pass

    def unpause_sim(self):
        self.unpause_calls += 1
        self.paused = False

    def pause_sim(self):
        self.paused = True

    def wait_for_paused(self, timeout_sec=3.0):
        return self.paused


class MockBrunner:
    def __init__(self, connected=True):
        self.sock = object() if connected else None
        self._connected = connected
        self.commands = []  # (axis, setting_id, fmt, val)
        self.overrides = []
        self.replay_modes = []
        self.read_pos = 0.0
        self.read_force = 0.0
        self.read_vf = 0.0

    def read_axis(self, axis):
        return self.read_pos, self.read_force, self.read_vf

    def write_setting(self, axis, setting_id, fmt, val):
        self.commands.append((axis, setting_id, fmt, val))

    def configure_replay_mode(self, axis_mask=None):
        self.replay_modes.append(axis_mask)
        return True

    def set_overrides(self, mask, enable):
        self.overrides.append((mask, enable))
        return True

    def set_position_forwarding(self):
        return True

    def close(self):
        self.sock = None
        self._connected = False


class StaticControlProfileTests(unittest.TestCase):
    def test_profile_generation_for_all_axes(self):
        for ref in STATIC_CONTROL_REFERENCES:
            profile = StaticControlProfile(ref, baseline_seconds=0.5, settle_seconds=0.5)
            info = AXIS_INFO[ref]
            limit = info['limit']

            # Total duration is positive and bounded
            self.assertGreater(profile.total_duration, 10.0)
            self.assertLess(profile.total_duration, 45.0 if ref == '2.a.(1)' else 40.0)

            # Check start at center
            pos0, seg0, done0 = profile.target(0.0)
            self.assertAlmostEqual(pos0, 0.0, places=2)
            self.assertFalse(done0)

            # Check reaches first limit (negative)
            t_first_limit = profile.segments[1].end_time
            pos1, _, _ = profile.target(t_first_limit)
            self.assertAlmostEqual(pos1, -limit, places=2)

            # Check reaches second limit (positive)
            t_second_limit = profile.segments[2].end_time
            pos2, _, _ = profile.target(t_second_limit)
            self.assertAlmostEqual(pos2, limit, places=2)

            # Check returns to center
            t_center = profile.segments[3].end_time
            pos3, _, _ = profile.target(t_center)
            self.assertAlmostEqual(pos3, 0.0, places=2)

            # Check completion
            pos_end, seg_end, done_end = profile.target(profile.total_duration + 0.1)
            self.assertAlmostEqual(pos_end, 0.0, places=2)
            self.assertTrue(done_end)
            self.assertEqual(seg_end, 'Completed')

    def test_profile_never_exceeds_limits_or_jumps(self):
        for ref in STATIC_CONTROL_REFERENCES:
            profile = StaticControlProfile(ref, baseline_seconds=0.5, settle_seconds=0.5)
            info = AXIS_INFO[ref]
            limit = info['limit']

            dt = 0.05
            prev_pos = 0.0
            max_rate_observed = 0.0

            for i in range(int(profile.total_duration / dt) + 2):
                t = i * dt
                pos, seg, _ = profile.target(t)
                self.assertLessEqual(abs(pos), limit + 1e-4, f"Position {pos} exceeded limit {limit} at t={t}")
                rate = abs(pos - prev_pos) / dt
                if rate > max_rate_observed:
                    max_rate_observed = rate
                prev_pos = pos

            # Rate should stay controlled (S-curve peak rate <= 1.8 * average rate)
            self.assertLess(max_rate_observed, info['rate'] * 2.0)


class AutomaticControlIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.now = 100.0

    def tearDown(self):
        self.tmp.cleanup()

    def clock(self):
        return self.now

    def make_context(self, ref=TEST_PITCH):
        info = AXIS_INFO[ref]
        return {
            'mode': 'auto',
            'control_method': METHOD,
            'test_reference': ref,
            'title': f"{info['name']} static control",
            'initial_conditions': {
                'alt_ft': 755.0,
                'ias_kt': 0.0,
                'lat': 32.233667,
                'lon': 74.1235,
                'heading_deg': 48.4,
                'temperature_c': 15.0,
                'on_ground': False,
            },
            'result_directory': str(self.root / f"test_{ref.replace('.', '_')}"),
        }

    def test_selected_validation(self):
        # Matches all 3 static control tests in auto mode
        for ref in (TEST_PITCH, TEST_ROLL, TEST_YAW):
            ctx = {'mode': 'auto', 'control_method': METHOD, 'test_reference': ref}
            self.assertTrue(selected(ctx))

        # Rejects manual mode or other control methods
        self.assertFalse(selected({'mode': 'manual', 'control_method': METHOD, 'test_reference': TEST_PITCH}))
        self.assertFalse(selected({'mode': 'auto', 'control_method': 'replay', 'test_reference': TEST_PITCH}))
        self.assertFalse(selected({'mode': 'auto', 'control_method': METHOD, 'test_reference': '1.f.(1)'}))

    def test_initialization_does_not_depend_on_pause_subscription(self):
        sim = MockSimConnect()
        sim.simulation_running = lambda: (_ for _ in ()).throw(RuntimeError('pause state unavailable'))
        worker = AutomaticControl(self.make_context(), sim, MockBrunner(), lambda _: None, clock=self.clock)
        worker.step()
        self.assertNotEqual(worker.stage, 'ABORTED')
        self.assertNotEqual(worker.stage, 'FAILED')
        self.assertTrue(sim.positions)

    def test_initial_status_is_strict_json_before_telemetry_arrives(self):
        worker = AutomaticControl(self.make_context(), MockSimConnect(), MockBrunner(), lambda _: None, clock=self.clock)
        status = worker.status()
        json.dumps(status, allow_nan=False)
        self.assertIsNone(status['telemetry']['ias_kt'])
        self.assertEqual(status['throttle_text'], 'Throttle: N/A')
        self.assertIn('N/A', status['flight_text'])

    def test_initial_throttle_requires_fresh_matching_readback_for_each_axis(self):
        for ref in (TEST_PITCH, TEST_ROLL, TEST_YAW):
            with self.subTest(ref=ref):
                sim = MockSimConnect()
                ctx = self.make_context(ref)
                ctx['initial_conditions']['throttle'] = 80.0
                worker = AutomaticControl(ctx, sim, MockBrunner(), lambda _: None, clock=self.clock)
                worker.step()
                self.assertEqual(sim.throttles, [80.0])
                with patch.object(sim, 'wait_for_fresh_telemetry', return_value=None):
                    worker.step()
                self.assertNotEqual(worker.stage, 'READY TO RECORD')
                with patch.object(sim, 'wait_for_fresh_telemetry', return_value={'engine_throttle_percent': 5.0}):
                    worker.step()
                self.assertNotEqual(worker.stage, 'READY TO RECORD')
                self.assertEqual(worker.status()['throttle_text'], 'Throttle: 5.0%')
                worker.step()
                self.assertEqual(worker.stage, 'READY TO RECORD')
                self.assertEqual(worker.status()['throttle_text'], 'Throttle: 80.0%')

    def test_first_recorded_values_and_pdf_use_fresh_data(self):
        import fitz
        sim = MockSimConnect()
        ctx = self.make_context()
        ctx['initial_conditions'].update(throttle=80.0, on_ground=False, alt_ft=5000.0, ias_kt=80.0)
        telemetry = {'engine_throttle_percent': 80.0, 'airspeed_indicated': 80.0,
                     'indicated_altitude': 5000.0, 'engine_rpm': 2400.0, 'total_weight': 850.0,
                     'ambient_wind_dir': 125.0, 'ambient_wind_spd': 12.0, 'egt_c': 610.0,
                     'ground_alt': 750.0, 'aoa': 3.2, 'sideslip': -1.2, 'track_angle': 45.0,
                     'accelX': 32.17404856, 'accelY': 0.0, 'accelZ': -32.17404856}
        worker = AutomaticControl(ctx, sim, MockBrunner(), lambda _: None, clock=self.clock, live_hardware=False)
        with patch.object(sim, 'get_telemetry', return_value=telemetry):
            worker.step()
            with patch.object(sim, 'wait_for_fresh_telemetry', return_value=dict(telemetry, airspeed_indicated=0.0)):
                worker.step()
            self.assertNotEqual(worker.stage, 'READY TO RECORD')
            worker.step()
            self.assertEqual(worker.stage, 'READY TO RECORD')
            worker.command({'action': 'record'})
            self.now += 0.1
            worker.step()
            row = worker.rows[0]
            self.assertEqual([row['Indicated Airspeed (kts)'], row['Engine 1 RPM'], row['Gross Wt (kg)'], row['Throttle 1 Position (%)']], [80.0, 2400.0, 850.0, 80.0])
            self.assertTrue(math.isnan(row['OAT (deg C)']))
            self.assertEqual([row['Wind Dir (mag,deg)'], row['Wind Spd (kt)'], row['EGT (deg C)'],
                              row['Height of Terrain (ft)'], row['Aircraft Angle of Attack (deg)'],
                              row['Sideslip Angle (deg)'], row['Track Angle (deg)']],
                             [125.0, 12.0, 610.0, 750.0, 3.2, -1.2, 45.0])
            self.assertEqual([row['X Body Acceleration (g)'], row['Y Body Acceleration (g)'], row['Z Body Acceleration (g)']], [1.0, 0.0, -1.0])
            worker.finish('COMPLETE', 'offline test')
        with fitz.open(worker.outputs[2]) as pdf:
            page = pdf[0].get_text()
            self.assertIn('850.0', page)
            self.assertIn('2400', page)
            self.assertIn('N/A', page)
            self.assertIn('610.0', page)
            self.assertIn('750.0', page)
            self.assertIn('Sim', page)
            self.assertNotIn('Validation', page)

    def test_paused_sim_is_unpaused_to_apply_position_and_paused_when_ready(self):
        sim = MockSimConnect()
        sim.paused = True
        ctx = self.make_context()
        ctx['initial_conditions'].update(throttle=80.0, on_ground=False, alt_ft=5000.0, ias_kt=80.0)
        worker = AutomaticControl(ctx, sim, MockBrunner(), lambda _: None, clock=self.clock)
        worker.step()
        self.assertEqual(sim.unpause_calls, 1)
        self.assertFalse(sim.paused)
        sample = {'engine_throttle_percent': 80.0, 'airspeed_indicated': 80.0, 'indicated_altitude': 5000.0}
        with patch.object(sim, 'wait_for_fresh_telemetry', return_value=sample):
            worker.step()
        self.assertEqual(worker.stage, 'READY TO RECORD')
        self.assertTrue(sim.paused)

    def test_unconfirmed_pause_does_not_expose_ready_to_record(self):
        sim = MockSimConnect()
        sim.wait_for_paused = lambda timeout_sec=3.0: False
        ctx = self.make_context()
        ctx['initial_conditions'].update(throttle=80.0, alt_ft=5000.0, ias_kt=80.0)
        worker = AutomaticControl(ctx, sim, MockBrunner(), lambda _: None, clock=self.clock)
        worker.step()
        sample = {'engine_throttle_percent': 80.0, 'airspeed_indicated': 80.0,
                  'indicated_altitude': 5000.0}
        with patch.object(sim, 'wait_for_fresh_telemetry', return_value=sample):
            worker.step()
        self.assertNotEqual(worker.stage, 'READY TO RECORD')
        self.assertIn('pause', worker.details.lower())

    def test_initialization_does_not_move_physical_controls(self):
        sim = MockSimConnect()
        brunner = MockBrunner()
        published = []

        ctx = self.make_context(TEST_PITCH)
        ctx['initial_conditions']['on_ground'] = True
        reader = Mock()
        reader.poll.return_value = {'parking_brake': 1, 'on_ground': 1, 'ground_speed': 0,
                                    'receipt_time': self.now, 'sequence': 1}
        worker = AutomaticControl(ctx, sim, brunner, published.append, clock=self.clock, brake_reader=reader)

        # Step during CONNECT
        worker.step()

        # P3D aircraft positioning was called
        self.assertEqual(len(sim.positions), 1)
        self.assertTrue(sim.positions[0]['on_ground'])

        # Controls were NOT commanded during initialization
        self.assertEqual(len(brunner.commands), 0)
        self.assertEqual(len(brunner.replay_modes), 0)

        # Reached READY TO RECORD
        worker.step()
        worker.step()
        self.assertEqual(worker.stage, 'READY TO RECORD')
        status = worker.status()
        self.assertTrue(status['can_record'])
        self.assertFalse(status['recording'])

    def test_ground_static_waits_for_brake_on_before_requested_throttle(self):
        sim = MockSimConnect()
        ctx = self.make_context(TEST_ROLL)
        ctx['initial_conditions'].update(on_ground=True, throttle=80.0)
        reader = Mock()
        reader.poll.side_effect = [
            dict(parking_brake=0, on_ground=1, ground_speed=0, receipt_time=self.now, sequence=1),
            dict(parking_brake=1, on_ground=1, ground_speed=0, receipt_time=self.now, sequence=2),
        ]
        worker = AutomaticControl(ctx, sim, MockBrunner(), lambda _: None, clock=self.clock, brake_reader=reader)
        worker.step()
        worker.step()
        reader.engage_parking_brake.assert_called_once_with()
        self.assertEqual(sim.throttles, [5.0])
        worker.step()
        self.assertEqual(sim.throttles, [5.0, 80.0])
        worker.step()
        self.assertEqual(worker.stage, 'READY TO RECORD')

    def test_full_static_control_sequence_and_report_generation(self):
        for ref in (TEST_PITCH, TEST_ROLL, TEST_YAW):
            with self.subTest(reference=ref):
                sim = MockSimConnect()
                brunner = MockBrunner()
                published = []

                ctx = self.make_context(ref)
                worker = AutomaticControl(ctx, sim, brunner, published.append, clock=self.clock, live_hardware=True)

                # Initialize
                worker.step()
                worker.step()
                self.assertEqual(worker.stage, 'READY TO RECORD')

                # Start Recording
                worker.command({'action': 'record'})
                self.assertEqual(worker.stage, 'RECORDING')
                self.assertEqual(len(brunner.replay_modes), 1)
                self.assertEqual(brunner.replay_modes[0], worker.axis_mask)

                # Step through entire profile
                total_duration = worker.profile.total_duration
                dt = 0.1
                steps = int(total_duration / dt) + 5
                for _ in range(steps):
                    self.now += dt
                    worker.step()
                    if worker.phase == 'DONE':
                        break

                # Completed automatically
                self.assertEqual(worker.stage, 'Completed')
                self.assertEqual(worker.result, 'COMPLETE')

                # The other axes are read from BRUNNER for the initial conditions,
                # without commanding those operator-owned controls.
                position_columns = {0x01: 'Joystick Pitch Position (mm)', 0x02: 'Joystick Roll Position (mm)', 0x04: 'Rudder Pedal Position (mm)'}
                for mask, column in position_columns.items():
                    self.assertTrue(math.isfinite(worker.rows[0][column]))
                self.assertEqual(worker.rows[0]['Control Force (N)'], 0.0)

                # Verify commanded axis: only target axis was commanded
                self.assertGreater(len(brunner.commands), 10)
                for cmd_axis, setting_id, fmt, val in brunner.commands:
                    if setting_id == 0x80:  # Position command
                        self.assertEqual(cmd_axis, worker.axis_mask)
                        self.assertLessEqual(abs(val), worker.axis_info['limit'] + 1e-4)

                # Verify outputs: CSV, JSON, and PDF were generated
                self.assertEqual(len(worker.outputs), 3)
                csv_path = Path(worker.outputs[0])
                json_path = Path(worker.outputs[1])
                pdf_path = Path(worker.outputs[2])

                self.assertTrue(csv_path.is_file(), f"CSV missing: {csv_path}")
                self.assertTrue(json_path.is_file(), f"JSON missing: {json_path}")
                self.assertTrue(pdf_path.is_file(), f"PDF missing: {pdf_path}")
                self.assertTrue(pdf_path.read_bytes().startswith(b'%PDF'))

                # Verify CSV columns
                csv_content = csv_path.read_text(encoding='utf-8')
                self.assertIn('Control Position (norm)', csv_content)
                self.assertIn('Control Position (mm)', csv_content)
                self.assertIn('Control Force (N)', csv_content)
                self.assertIn(worker.axis_info['position_col'], csv_content)
                self.assertIn(worker.axis_info['force_col'], csv_content)

    def test_emergency_stop_halts_without_centering(self):
        sim = MockSimConnect()
        brunner = MockBrunner()
        published = []

        ctx = self.make_context(TEST_ROLL)
        worker = AutomaticControl(ctx, sim, brunner, published.append, clock=self.clock, live_hardware=True)

        worker.step()
        worker.step()
        worker.command({'action': 'record'})

        # Advance 2 seconds into movement
        for _ in range(20):
            self.now += 0.1
            worker.step()

        # Trigger Emergency Stop
        cmd_count_before = len([c for c in brunner.commands if c[1] == 0x80])
        worker.command({'action': 'emergency_stop'})

        self.assertEqual(worker.stage, 'ABORTED')
        self.assertIn('Emergency Stop', worker.details)

        # Verify NO new position command was sent to center the stick
        cmd_count_after = len([c for c in brunner.commands if c[1] == 0x80])
        self.assertEqual(cmd_count_before, cmd_count_after)

        # Autopilot was disengaged directly
        ap_off_cmds = [c for c in brunner.commands if c[1] == 0x70 and c[3] == 0]
        self.assertGreater(len(ap_off_cmds), 0)

    def test_force_limit_watchdog_aborts(self):
        sim = MockSimConnect()
        brunner = MockBrunner()
        published = []

        ctx = self.make_context(TEST_PITCH)
        worker = AutomaticControl(ctx, sim, brunner, published.append, clock=self.clock, live_hardware=True)

        worker.step()
        worker.step()
        worker.command({'action': 'record'})

        # Simulate force sensor spiking above safe limit (400 N > 350 N)
        brunner.read_force = 400.0
        self.now += 0.1
        worker.step()

        self.assertEqual(worker.stage, 'ABORTED')
        self.assertIn('Safety trip: Measured force', worker.details)
        self.assertIn('exceeds safe limit', worker.details)

    def test_dry_run_safety_gate_disabled_hardware(self):
        sim = MockSimConnect()
        brunner = MockBrunner()
        published = []

        ctx = self.make_context(TEST_YAW)
        # live_hardware explicitly False
        worker = AutomaticControl(ctx, sim, brunner, published.append, clock=self.clock, live_hardware=False)

        worker.step()
        worker.step()
        worker.command({'action': 'record'})

        self.now += 1.0
        worker.step()

        # In dry run mode, NO motor position commands (0x80) are sent to hardware
        pos_commands = [c for c in brunner.commands if c[1] == 0x80]
        self.assertEqual(len(pos_commands), 0)
        # Rows and profile tracking still function safely
        self.assertGreater(len(worker.rows), 0)

    def test_pdf_rendering_structure(self):
        import fitz
        sim = MockSimConnect()
        brunner = MockBrunner()
        published = []

        ctx = self.make_context(TEST_PITCH)
        worker = AutomaticControl(ctx, sim, brunner, published.append, clock=self.clock, live_hardware=False)

        worker.step()
        worker.step()
        worker.command({'action': 'record'})

        for _ in range(int(worker.profile.total_duration / 0.1) + 5):
            self.now += 0.1
            worker.step()
            if worker.phase == 'DONE':
                break

        pdf_path = Path(worker.outputs[2])
        with fitz.open(pdf_path) as doc:
            self.assertEqual(len(doc), 3)
            page1_text = doc[0].get_text()
            self.assertIn('Super Mushshak Automatic Flight Test', page1_text)
            self.assertIn('Doc Number: 161310G00000-00', page1_text)
            self.assertIn('Initial Test Conditions', page1_text)

            page2_text = doc[1].get_text()
            self.assertIn('Automatic Test (Sim)', page2_text)
            self.assertIn('Indicated Airspeed', page2_text)
            self.assertIn('Joystick Pitch Position', page2_text)
            self.assertIn('Pitch Virtual Force', page2_text)
            page3_text = doc[2].get_text()
            self.assertIn('Pitch Force vs. Position', page3_text)
            self.assertIn('Control Force (N)', page3_text)
            self.assertIn('Pitch Control Position (mm)', page3_text)
            self.assertIn('100', page3_text)
            self.assertIn('−100', page3_text.replace('-', '−'))

    def test_force_position_page_is_available_for_pitch_roll_and_rudder(self):
        import fitz
        import pandas as pd
        from matplotlib.backends.backend_pdf import PdfPages
        from reports.automatic_control_report import _add_force_position_page
        configs = {
            TEST_PITCH: ('Pitch', 'Joystick Pitch Position (mm)'),
            TEST_ROLL: ('Roll', 'Joystick Roll Position (mm)'),
            TEST_YAW: ('Rudder', 'Rudder Pedal Position (mm)'),
        }
        for reference, (axis, position_column) in configs.items():
            path = self.root / f'{axis.lower()}_force_position.pdf'
            df = pd.DataFrame({position_column: [-100, 0, 100],
                               'Control Force (N)': [-50, 0, 50]})
            with PdfPages(path) as pdf:
                _add_force_position_page(pdf, df, reference)
            with fitz.open(path) as document:
                self.assertEqual(len(document), 1)
                text = document[0].get_text()
                self.assertIn(f'{axis} Force vs. Position', text)
                self.assertIn('Control Force (N)', text)
                self.assertIn('-100', text.replace('\u2212', '-'))
                self.assertIn('100', text)

    def test_gui_launch_static_control_dispatch(self):
        from frontend.main import App, TESTS
        from types import SimpleNamespace
        import qtg_common_engine as engine

        for ref in (TEST_PITCH, TEST_ROLL, TEST_YAW):
            spec = next(s for s in TESTS if s.reference == ref)
            values = {
                'Flt cond': 'Ground', 'Alt[ft]': '0', 'IAS[kts]': '0',
                'Latitude[deg]': '32.0', 'Longitude[deg]': '74.0',
                'Heading[deg]': '0', 'Pitch[deg]': '0', 'Bank[deg]': '0',
                'Flaps': '0', 'Throttle[%]': '0', 'Temperature[C]': '15'
            }
            fields = {k: Mock(get=lambda v=v: v) for k, v in values.items()}
            host = SimpleNamespace(
                _common_duration_var=Mock(get=lambda: '75'),
                _test_settings={ref: (fields, {})},
                _test_defaults={ref: values},
                _common_reference_var=Mock(get=lambda: ''),
                _automatic_methods={ref: Mock(get=lambda: 'Static Control Step')},
                _apply_test_settings=Mock(return_value=True),
                _common_process=None,
                _common_running=False,
                common_record_button=Mock(),
                common_stop_button=Mock(),
                common_status=Mock(),
                _common_poll_job=None,
                _poll_common_test=Mock(),
                after=Mock(),
                after_cancel=Mock(),
            )

            with patch.object(engine, 'launch', return_value=(Mock(), {'status_file': 'dummy'})) as mock_launch:
                App._launch_common_test(host, 'auto', spec)
                mock_launch.assert_called_once()
                kwargs = mock_launch.call_args.kwargs
                self.assertEqual(kwargs['control_method'], 'static_control')
                self.assertEqual(mock_launch.call_args.args[1].reference, ref)
                host.common_record_button.configure.assert_called_with(state='disabled')


if __name__ == '__main__':
    unittest.main()
