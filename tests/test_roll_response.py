import csv
import json
from pathlib import Path
import tempfile
import unittest

from backend.automatic_roll_response import (
    APPROACH_REFERENCE,
    CRUISE_REFERENCE,
    LEFT_ROLL_COMMAND,
    METHOD,
    AutomaticRollResponse,
    selected,
)
from backend.catalog import TESTS
from backend.control import validate_method
from backend.initial_condition_settings import catalog_defaults
from frontend.main import ROLL_RESPONSE_REFERENCES, default_channels_for_spec
from reports.power_dynamics import CHANNELS


class _FakeClock:
    def __init__(self, start=100.0):
        self.now = float(start)

    def __call__(self):
        return self.now

    def advance(self, dt):
        self.now += float(dt)
        return self.now


class _FakeSim:
    def __init__(self):
        self.connected = True
        self.paused = True
        self.throttles = []
        self.flaps = []
        self.positions = []
        self.metars = []

    def is_connected(self):
        return self.connected

    def pause_sim(self):
        self.paused = True

    def unpause_sim(self):
        self.paused = False

    def wait_for_paused(self, _timeout):
        return self.paused

    def set_initial_throttle(self, val):
        self.throttles.append(float(val))
        return True

    def queue_position_airborne(self, **kwargs):
        self.positions.append(kwargs)
        return True

    def set_initial_flaps(self, val):
        self.flaps.append(float(val))
        return True

    def get_global_weather_metar(self):
        return 'OPRN 151200Z 00000KT CAVOK 15/05 Q1013'

    def set_global_weather_metar(self, metar):
        self.metars.append(metar)
        return True

    def set_linear_body_acceleration(self, *vector):
        return True

    def get_telemetry(self):
        return {}


class _FakeRollTransport:
    def __init__(self, _brunner=None):
        self.engaged = False
        self.released = False
        self.commands = []
        self.measured_roll = 0.0
        self.watchdog_calls = 0

    def engage(self):
        self.engaged = True
        return (0.0, 0.0, 0.0)

    def refresh_watchdog(self):
        self.watchdog_calls += 1

    def read_positions(self):
        # Track previous commanded roll gradually to emulate physical loader response.
        if self.commands:
            last = self.commands[-1]
            self.measured_roll += (last - self.measured_roll) * 0.6
        return (0.0, self.measured_roll, 0.0)

    def command(self, pitch_pos=None, roll_pos=None, rudder_pos=None):
        assert pitch_pos is None and rudder_pos is None, 'Roll Response must own Roll axis only'
        self.commands.append(float(roll_pos))

    def release(self):
        self.released = True
        return []


def _make_context(result_dir, ref=APPROACH_REFERENCE, *, bank_deg=0.0, flaps=1.0, ias_kt=90.0, throttle=50.0):
    return {
        'test_reference': ref,
        'title': 'Roll response (rate)',
        'mode': 'auto',
        'control_method': METHOD,
        'result_directory': str(result_dir),
        'initial_conditions': {
            'alt_ft': 4000.0 if ref == APPROACH_REFERENCE else 3000.0,
            'ias_kt': ias_kt,
            'lat': 32.235,
            'lon': 74.135,
            'heading_deg': 48.4,
            'pitch_deg': 3.0 if ref == APPROACH_REFERENCE else 0.0,
            'bank_deg': bank_deg,
            'flaps': flaps,
            'throttle': throttle,
            'temperature_c': 15.0,
            'on_ground': False,
        },
    }


def _frame(seq, clock, *, bank_right_pos=0.0, roll_rate=0.0, aileron=0.0, ias_kt=90.0, throttle=50.0, flaps=1.0,
           alt_ft=4000.0, pitch_deg=3.0):
    # Raw P3D SimConnect PLANE BANK DEGREES is left-positive (-bank_right_pos).
    return {
        'sequence': seq,
        'sim_time': clock.now - 90.0,
        'receipt_time': clock.now,
        'rpm': 2200.0,
        'throttle': throttle,
        'ias_kt': ias_kt,
        'alt_ft': alt_ft,
        'height_agl_ft': max(500.0, alt_ft - 800.0),
        'ground_speed': ias_kt,
        'vertical_speed': 0.0,
        'lat': 32.235,
        'lon': 74.135,
        'flaps': flaps,
        'flaps_handle_percent': flaps * 33.3,
        'temperature_c': 15.0,
        'heading_deg': 48.4,
        'pitch_deg': pitch_deg,
        'bank_deg': -bank_right_pos,
        'roll_rate_deg_s': roll_rate,
        'pitch_rate_deg_s': 0.0,
        'yaw_rate_deg_s': 0.0,
        'aoa_deg': 4.0,
        'sideslip_deg': 0.0,
        'prop_rpm': 2200.0,
        'manifold_pressure_psi': 11.0,
        'aileron_input': aileron,
        'elevator_input': 0.0,
        'rudder_input': 0.0,
        'autopilot_active': 0,
        'on_ground': 0,
    }


class RollResponseCatalogTests(unittest.TestCase):
    def test_catalog_split_matches_flap_naming_and_channels(self):
        refs = [spec.reference for spec in TESTS]
        self.assertNotIn('2.d.(2)', refs)
        idx = refs.index('2.d.(2a)')
        self.assertEqual(refs[idx:idx + 2], ['2.d.(2a)', '2.d.(2b)'])

        specs = {spec.reference: spec for spec in TESTS}
        self.assertEqual(specs['2.d.(2a)'].title, 'Roll response (rate) (Approach)')
        self.assertEqual(specs['2.d.(2b)'].title, 'Roll response (rate) (Cruise)')
        self.assertEqual(catalog_defaults(specs['2.d.(2a)'])['Flt cond'], 'Approach')
        self.assertEqual(catalog_defaults(specs['2.d.(2a)'])['Bank[deg]'], '0')
        self.assertEqual(catalog_defaults(specs['2.d.(2b)'])['Flt cond'], 'Cruise')
        self.assertEqual(catalog_defaults(specs['2.d.(2b)'])['Bank[deg]'], '0')

        self.assertEqual(ROLL_RESPONSE_REFERENCES, ('2.d.(2a)', '2.d.(2b)'))
        self.assertIn('roll_rate', CHANNELS)
        for ref in ROLL_RESPONSE_REFERENCES:
            self.assertEqual(validate_method(ref, 'roll_response'), 'roll_response')
            self.assertIn('roll_rate', default_channels_for_spec(ref))
            self.assertIn('bank_deg', default_channels_for_spec(ref))
            self.assertIn('roll_position', default_channels_for_spec(ref))
            self.assertIn('roll_force', default_channels_for_spec(ref))
        with self.assertRaises(ValueError):
            validate_method('1.c.(1)', 'roll_response')


class AutomaticRollResponseTests(unittest.TestCase):
    def test_validation_guards(self):
        with tempfile.TemporaryDirectory() as tmp:
            sim = _FakeSim()
            bad_bank = _make_context(tmp, bank_deg=5.0)
            with self.assertRaisesRegex(ValueError, 'Bank\\[deg\\] = 0'):
                AutomaticRollResponse(bad_bank, sim, lambda _s: None)

            on_ground = _make_context(tmp)
            on_ground['initial_conditions']['on_ground'] = True
            with self.assertRaisesRegex(ValueError, 'airborne'):
                AutomaticRollResponse(on_ground, sim, lambda _s: None)

            with_pid = _make_context(tmp)
            with_pid['active_pids'] = {'airspeed': True, 'bank': False, 'yaw': False}
            with self.assertRaisesRegex(ValueError, 'disable PID'):
                AutomaticRollResponse(with_pid, sim, lambda _s: None)

            self.assertTrue(selected(_make_context(tmp, CRUISE_REFERENCE, flaps=0.0, ias_kt=110.0, throttle=75.0)))

    def _run_sequence(self, ref, *, stabilize=True):
        with tempfile.TemporaryDirectory() as tmp:
            clock = _FakeClock()
            sim = _FakeSim()
            transport = _FakeRollTransport()
            statuses = []
            ctx = _make_context(
                tmp,
                ref,
                flaps=1.0 if ref == APPROACH_REFERENCE else 0.0,
                ias_kt=90.0 if ref == APPROACH_REFERENCE else 110.0,
                throttle=50.0 if ref == APPROACH_REFERENCE else 75.0,
            )
            runner = AutomaticRollResponse(
                ctx,
                sim,
                statuses.append,
                clock=clock,
                transport_factory=lambda _b: transport,
            )
            ic = ctx['initial_conditions']
            frame_cfg = dict(
                ias_kt=ic['ias_kt'],
                throttle=ic['throttle'],
                flaps=ic['flaps'],
                alt_ft=ic['alt_ft'],
                pitch_deg=ic['pitch_deg'],
            )
            seq = 1
            runner.step(_frame(seq, clock, **frame_cfg))
            self.assertEqual(runner.phase, 'PREPARE')
            for _ in range(10):
                if runner.phase == 'READY':
                    break
                clock.advance(0.5); seq += 1
                runner.step(_frame(seq, clock, **frame_cfg))
            self.assertEqual(runner.phase, 'READY')

            runner.command({'action': 'record'})
            self.assertEqual(runner.phase, 'RECORD')
            self.assertTrue(transport.engaged)

            # 1) Baseline: brief excursion resets 2 s timer, then 2.2 s steady at 0 deg bank
            bank = 0.0
            for i in range(60):
                clock.advance(0.05); seq += 1
                if i == 5:
                    # Brief bank disturbance at t=0.25 s resets 2.0 s baseline hold
                    runner.step(_frame(seq, clock, bank_right_pos=3.2, roll_rate=4.0, **frame_cfg))
                    continue
                runner.step(_frame(seq, clock, bank_right_pos=0.1, roll_rate=0.2, **frame_cfg))
                if runner.subphase == 'ROLL_HOLD':
                    break

            self.assertEqual(runner.subphase, 'ROLL_HOLD')
            # Must have held level at least 2.0 s after disturbance
            self.assertGreaterEqual(runner.roll_command_time - runner.start_time, 2.25)
            self.assertEqual(transport.commands[-1], LEFT_ROLL_COMMAND)

            # 2) Roll left at -30%: spin up roll rate until it stabilizes (or safety bank cap)
            rates = [-5.0, -10.0, -14.5, -17.0, -18.0]
            for idx in range(90):
                clock.advance(0.05); seq += 1
                if stabilize:
                    rate = rates[idx] if idx < len(rates) else -18.2
                    bank -= abs(rate) * 0.05
                else:
                    rate = -25.0 - idx * 0.5
                    bank -= abs(rate) * 0.05
                runner.step(_frame(seq, clock, bank_right_pos=bank, roll_rate=rate, aileron=-0.30, **frame_cfg))
                if runner.subphase == 'RETURN_ZERO':
                    break

            self.assertIn(runner.subphase, ('RETURN_ZERO', 'DONE'))
            self.assertEqual(transport.commands[-1], 0.0)

            # 3) Continue steps while roll controller returns to 0 and recording stops
            for _ in range(30):
                if runner.phase == 'DONE':
                    break
                clock.advance(0.05); seq += 1
                runner.step(_frame(seq, clock, bank_right_pos=bank, roll_rate=-5.0, aileron=0.0, **frame_cfg))

            self.assertEqual(runner.phase, 'DONE')
            self.assertEqual(runner.result, 'COMPLETE')
            self.assertTrue(transport.released)
            self.assertTrue(sim.paused)
            self.assertEqual(len(runner.outputs), 3)

            csv_path, meta_path, pdf_path = [Path(p) for p in runner.outputs]
            self.assertTrue(csv_path.is_file() and meta_path.is_file() and pdf_path.is_file())
            self.assertGreater(pdf_path.stat().st_size, 1000)

            meta = json.loads(meta_path.read_text(encoding='utf-8'))
            with csv_path.open(newline='', encoding='utf-8') as f:
                rows = list(csv.DictReader(f))
            self.assertIn('Roll Rate (deg/s)', rows[0])
            self.assertIn('Roll Command (norm)', rows[0])
            self.assertIn('Bank Angle (deg)', rows[0])
            return runner, meta, rows

    def test_approach_and_cruise_complete_workflow(self):
        for ref in (APPROACH_REFERENCE, CRUISE_REFERENCE):
            runner, meta, rows = self._run_sequence(ref, stabilize=True)
            self.assertEqual(meta['stop_trigger'], 'roll_rate_stabilized')
            self.assertAlmostEqual(meta['steady_roll_rate_deg_s'], -18.2, delta=0.5)
            self.assertEqual(runner.roll_cmd, 0.0)
            self.assertGreaterEqual(float(rows[-1]['Time (s)']), meta['roll_return_command_time_s'])

    def test_safety_bank_cap_forces_zero_return(self):
        _runner, meta, _rows = self._run_sequence(APPROACH_REFERENCE, stabilize=False)
        self.assertEqual(meta['stop_trigger'], 'max_safe_bank_reached')


if __name__ == '__main__':
    unittest.main()
