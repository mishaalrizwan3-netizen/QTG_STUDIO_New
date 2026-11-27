import unittest
import json

from backend.automatic_longitudinal_trim import (
    AutomaticLongitudinalTrim,
    METHOD,
    REFERENCES,
    ThrottleIasHold,
    selected,
)
from backend.control import validate_method
from frontend.main import LONGITUDINAL_TRIM_REFERENCES, default_channels_for_spec


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class Sim:
    def __init__(self):
        self.throttles = []
        self.paused = False

    def set_replay_throttle(self, value):
        self.throttles.append(value)
        return True

    def is_connected(self):
        return True

    def unpause_sim(self):
        self.paused = False
        return True

    def pause_sim(self):
        self.paused = True
        return True


class Hold:
    def __init__(self):
        self.steps = 0
        self.engaged = False
        self.released = False
        self.last_command = 0.0

    def engage(self, *_args, **_kwargs):
        self.engaged = True

    def step(self, *_args):
        self.steps += 1

    def release(self):
        self.released = True


class Events:
    def __init__(self):
        self.commands = []

    def step(self, direction):
        self.commands.append(direction)


class Brunner:
    sock = object()
    strict_io = False
    force = 0.0

    def read_axis_measurement(self, _axis):
        return 0.0, self.force


def context(reference='2.c.(5a)'):
    return {
        'test_reference': reference,
        'title': 'Longitudinal Trim',
        'mode': 'auto',
        'control_method': METHOD,
        'duration_s': 10,
        'result_directory': '.',
        'selected_graphs': [],
        'initial_conditions': {
            'alt_ft': 3500, 'ias_kt': 80, 'lat': 32.23, 'lon': 74.13,
            'heading_deg': 48, 'pitch_deg': -2, 'bank_deg': 0,
            'flaps': 0, 'throttle': 55, 'on_ground': False,
            'wind_speed': 0, 'wind_direction': 0, 'temperature_c': 15,
            'atmospheric_pressure_hpa': 1013.25, 'cg_percent': 25,
        },
    }


class AutomaticLongitudinalTrimTests(unittest.TestCase):
    def make_worker(self):
        clock, sim, hold, events, brunner = Clock(), Sim(), Hold(), Events(), Brunner()
        worker = AutomaticLongitudinalTrim(
            dict(context(), brunner=brunner), sim, lambda _state: None,
            clock=clock, vvi_controller=object(), events=events,
        )
        worker.vvi_hold = hold
        worker.brunner = brunner
        self.assertNotIn('brunner', worker.status())
        json.dumps(worker.status(), allow_nan=False)
        worker.phase = 'TRIM_STABILIZE'
        worker.stage = 'AUTOMATIC TRIM STABILIZATION'
        worker.trim_started = 0.0
        worker.last_sequence = -1
        worker._ensure_live = lambda _frame, _now: None
        return worker, clock, sim, hold, events, brunner

    @staticmethod
    def frame(sequence, trim=0.0):
        return {
            'sequence': sequence, 'sim_time': sequence * 0.1, 'receipt_time': 0.0,
            'rpm': 2500.0, 'throttle': 55.0,
            'ias_kt': 80.0, 'alt_ft': 3500.0, 'vertical_speed': 0.0,
            'pitch_deg': -2.0, 'bank_deg': 0.0, 'trim_indicator': trim,
            'throttle_pct': 55.0,
        }

    def test_method_is_strictly_isolated_to_two_automatic_tests(self):
        self.assertEqual(REFERENCES, LONGITUDINAL_TRIM_REFERENCES)
        for reference in REFERENCES:
            self.assertTrue(selected(context(reference)))
            self.assertEqual(validate_method(reference, METHOD), METHOD)
        self.assertFalse(selected(dict(context(), mode='manual')))
        with self.assertRaises(ValueError):
            validate_method('2.a.(4)', METHOD)

    def test_default_graphs_include_required_trim_and_control_measurements(self):
        keys = default_channels_for_spec('2.c.(5a)')
        self.assertTrue({'ias_kt', 'altitude_msl_ft', 'vertical_speed_fpm',
                         'throttle_pct', 'pitch_force', 'trim_indicator'} <= keys)

    def test_force_causes_gradual_trim_event_and_blocks_recording(self):
        worker, clock, sim, hold, events, brunner = self.make_worker()
        brunner.force = 0.05  # Existing interface reports normalized force; worker converts to 5 N.
        clock.now = 0.6
        worker.step(self.frame(1))
        self.assertEqual(worker.phase, 'TRIM_STABILIZE')
        self.assertEqual(len(events.commands), 1)
        self.assertEqual(hold.steps, 1)
        self.assertTrue(sim.throttles)

    def test_recording_starts_only_after_five_continuous_stable_seconds(self):
        worker, clock, sim, hold, events, brunner = self.make_worker()
        brunner.force = 0.0
        for sequence, now in enumerate((0.1, 2.0, 4.9, 5.2), start=1):
            clock.now = now
            worker.step(self.frame(sequence))
        self.assertEqual(worker.phase, 'RECORD')
        pulses = len(events.commands)
        clock.now = 5.5
        worker.step(self.frame(5))
        self.assertEqual(len(events.commands), pulses)
        self.assertGreaterEqual(hold.steps, 5)
        self.assertTrue(sim.throttles)

    def test_throttle_hold_moves_in_correct_direction_and_is_bounded(self):
        hold = ThrottleIasHold(80, 50)
        up = hold.update(70, 0.0)
        down = hold.update(90, 1.0)
        self.assertGreater(up, 50)
        self.assertLess(down, up)
        self.assertGreaterEqual(down, 0)
        self.assertLessEqual(up, 100)


if __name__ == '__main__':
    unittest.main()
