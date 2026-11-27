import csv
import json
from pathlib import Path
import tempfile
import unittest

from backend.automatic_flap_change import (
    AutomaticFlapDynamics,
    EXTENSION_FORCE_REFERENCE,
    EXTENSION_REFERENCE,
    RETRACTION_FORCE_REFERENCE,
    RETRACTION_REFERENCE,
)
from backend.automatic_power_change import (
    AutomaticPowerDynamics,
    FORCE_REFERENCE as POWER_FORCE_REFERENCE,
    REFERENCE as POWER_REFERENCE,
)


class Clock:
    def __init__(self):
        self.value = 100.0

    def __call__(self):
        return self.value

    def advance(self, dt):
        self.value += dt


class Sim:
    def __init__(self, throttle=50.0, flaps=1.0):
        self.connected = True
        self.paused = False
        self.throttle = float(throttle)
        self.flaps = float(flaps)
        self.events = []

    def is_connected(self):
        return self.connected

    def pause_sim(self):
        self.paused = True
        self.events.append(('pause',))

    def wait_for_paused(self, timeout):
        return self.paused

    def unpause_sim(self):
        self.paused = False
        self.events.append(('unpause',))

    def set_initial_throttle(self, value):
        self.throttle = float(value)
        return True

    def set_replay_throttle(self, value):
        self.throttle = float(value)
        self.events.append(('throttle_step', float(value)))
        return True

    def queue_position_airborne(self, **kwargs):
        return True

    def set_initial_flaps(self, value):
        self.flaps = float(value)
        return True

    def set_flaps_handle_index(self, value):
        self.flaps = float(value)
        self.events.append(('flap_step', float(value)))
        return True

    def get_global_weather_metar(self):
        return 'METAR OPRW 171000Z 00000KT CAVOK 15/10 Q1013'

    def set_global_weather_metar(self, value):
        return True


class FakeBrunner:
    def __init__(self):
        self.sock = object()

    def read_axis_measurement(self, axis):
        return (0.12, -14.0)

    def read_axis(self, axis):
        return (0.12, -14.0, -21.5)


class FakeTransport:
    def __init__(self, brunner, *, pitch_enabled=True, bank_enabled=False, yaw_enabled=False):
        self.brunner = brunner
        self.pitch_enabled = pitch_enabled
        self.bank_enabled = bank_enabled
        self.yaw_enabled = yaw_enabled
        self.engaged = False
        self.released = False
        self.commands = []

    def engage(self):
        self.engaged = True
        return (0.0, 0.0, 0.0)

    def refresh_watchdog(self):
        pass

    def read_positions(self):
        return (0.0, None, None)

    def command(self, pitch_pos=None, roll_pos=None, rudder_pos=None):
        self.commands.append((pitch_pos, roll_pos, rudder_pos))

    def release(self):
        self.released = True
        return []


class ForceTestsAltitudeHoldSuite(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.count = 0

    def _feed(self, worker, sim, dt=0.5, **overrides):
        self.clock.advance(dt)
        self.count += 1
        frame = dict(
            sequence=self.count,
            receipt_time=self.clock(),
            sim_time=self.clock(),
            rpm=2100.0,
            throttle=sim.throttle,
            flaps=sim.flaps,
            ias_kt=90.0,
            alt_ft=4000.0,
            lat=32.235,
            lon=74.135,
            height_agl_ft=3000.0,
            on_ground=0.0,
            autopilot_active=0.0,
            temperature_c=15.0,
            pitch_deg=3.0,
            bank_deg=0.0,
            heading_deg=48.4,
            prop_rpm=2100.0,
            manifold_pressure_psi=12.0,
            vertical_speed=0.0,
            elevator_input=0.0,
            elevator_trim_rad=0.0,
        )
        frame.update(overrides)
        worker.step(frame)

    def _prepare(self, worker, sim):
        for _ in range(10):
            self._feed(worker, sim)
            if worker.phase == 'READY':
                break
        self.assertEqual(worker.phase, 'READY', worker.details)

    def test_power_change_force_keeps_dynamics_sequence_and_holds_altitude_with_vvi_pid(self):
        sim = Sim(throttle=50.0, flaps=1.0)
        brunner = FakeBrunner()
        transports = []
        ctx = dict(
            mode='auto',
            control_method='power_dynamics',
            test_reference=POWER_FORCE_REFERENCE,
            title='Power change force',
            result_directory=self.tmp.name,
            initial_conditions=dict(
                alt_ft=4000.0, ias_kt=90.0, lat=32.235, lon=74.135,
                heading_deg=48.4, pitch_deg=3.0, bank_deg=0.0, flaps=1.0,
                throttle=50.0, temperature_c=15.0, on_ground=False,
            ),
            brunner=brunner,
        )
        worker = AutomaticPowerDynamics(
            ctx, sim, lambda _s: None, clock=self.clock,
            transport_factory=lambda b: transports.append(FakeTransport(b)) or transports[-1],
        )
        self.assertIsNotNone(worker.vvi_hold)
        self._prepare(worker, sim)
        worker.command({'action': 'record'})
        self.assertEqual(len(transports), 1)
        self.assertTrue(transports[0].engaged)

        for _ in range(12):
            self._feed(worker, sim, alt_ft=3990.0, vertical_speed=-50.0)
            if worker.power_command_time is not None:
                break
        self.assertEqual([e for e in sim.events if e[0] == 'throttle_step'], [('throttle_step', 100.0)])
        for _ in range(35):
            if worker.phase == 'DONE':
                break
            self._feed(worker, sim, alt_ft=4010.0, vertical_speed=40.0)

        self.assertEqual(worker.result, 'COMPLETE', worker.details)
        self.assertTrue(transports[0].released)
        self.assertTrue(all(p is not None and r is None and y is None for p, r, y in transports[0].commands))
        with Path(worker.outputs[0]).open(newline='', encoding='utf-8') as f:
            rows = list(csv.DictReader(f))
        self.assertIn('Pitch Virtual Force 0x51 (N)', rows[0])
        self.assertIn('Pitch VVI PID Command (norm)', rows[0])
        meta = json.loads(Path(worker.outputs[1]).read_text(encoding='utf-8'))
        self.assertEqual(meta['kind'], 'Power Change Force')
        self.assertIn('pitch_virtual_force', meta['graphed_channels'])
        self.assertIn('pitch_force', meta['graphed_channels'])
        self.assertTrue(Path(worker.outputs[2]).is_file())

    def test_flap_change_force_retraction_and_extension_keep_sequence_and_hold_altitude(self):
        for ref, expected_target in ((RETRACTION_FORCE_REFERENCE, 0.0), (EXTENSION_FORCE_REFERENCE, 3.0)):
            with self.subTest(ref=ref):
                sim = Sim(throttle=50.0, flaps=1.0)
                brunner = FakeBrunner()
                transports = []
                ctx = dict(
                    mode='auto',
                    control_method='flap_dynamics',
                    test_reference=ref,
                    title=f'Flap change force {ref}',
                    result_directory=self.tmp.name,
                    initial_conditions=dict(
                        alt_ft=4000.0, ias_kt=90.0, lat=32.235, lon=74.135,
                        heading_deg=48.4, pitch_deg=3.0, bank_deg=0.0, flaps=1.0,
                        throttle=50.0, temperature_c=15.0, on_ground=False,
                    ),
                    brunner=brunner,
                )
                worker = AutomaticFlapDynamics(
                    ctx, sim, lambda _s: None, clock=self.clock,
                    transport_factory=lambda b: transports.append(FakeTransport(b)) or transports[-1],
                )
                self.assertIsNotNone(worker.vvi_hold)
                self._prepare(worker, sim)
                worker.command({'action': 'record'})
                self.assertEqual(len(transports), 1)
                self.assertTrue(transports[0].engaged)
                for _ in range(12):
                    self._feed(worker, sim, alt_ft=3995.0, vertical_speed=-30.0)
                    if worker.flap_command_time is not None:
                        break
                self.assertEqual([e for e in sim.events if e[0] == 'flap_step'], [('flap_step', expected_target)])
                for _ in range(35):
                    if worker.phase == 'DONE':
                        break
                    self._feed(worker, sim, alt_ft=4015.0, vertical_speed=60.0)
                self.assertEqual(worker.result, 'COMPLETE', worker.details)
                self.assertTrue(transports[0].released)
                self.assertTrue(all(p is not None and r is None and y is None for p, r, y in transports[0].commands))
                meta = json.loads(Path(worker.outputs[1]).read_text(encoding='utf-8'))
                self.assertEqual(meta['kind'], 'Flap Change Force')
                self.assertIn('pitch_virtual_force', meta['graphed_channels'])

    def test_dynamics_tests_do_not_engage_vvi_pid(self):
        sim = Sim(throttle=50.0, flaps=1.0)
        p_worker = AutomaticPowerDynamics(
            dict(
                mode='auto', control_method='power_dynamics', test_reference=POWER_REFERENCE,
                title='Power change dynamics', result_directory=self.tmp.name,
                initial_conditions=dict(
                    alt_ft=4000.0, ias_kt=90.0, lat=32.235, lon=74.135,
                    heading_deg=48.4, pitch_deg=3.0, bank_deg=0.0, flaps=1.0,
                    throttle=50.0, temperature_c=15.0, on_ground=False,
                ),
            ),
            sim, lambda _s: None, clock=self.clock,
        )
        self.assertIsNone(p_worker.vvi_hold)
        for ref in (RETRACTION_REFERENCE, EXTENSION_REFERENCE):
            initial_flap = 0.0 if ref == EXTENSION_REFERENCE else 1.0
            f_worker = AutomaticFlapDynamics(
                dict(
                    mode='auto', control_method='flap_dynamics', test_reference=ref,
                    title='Flap change dynamics', result_directory=self.tmp.name,
                    initial_conditions=dict(
                        alt_ft=4000.0, ias_kt=90.0, lat=32.235, lon=74.135,
                        heading_deg=48.4, pitch_deg=3.0, bank_deg=0.0, flaps=initial_flap,
                        throttle=50.0, temperature_c=15.0, on_ground=False,
                    ),
                ),
                sim, lambda _s: None, clock=self.clock,
            )
            self.assertIsNone(f_worker.vvi_hold)
