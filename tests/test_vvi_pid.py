import unittest

from backend.vvi_pid import ForceAltitudeHold, VviPidController


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
        return (0.05, 0.0, 0.0)

    def refresh_watchdog(self):
        pass

    def read_positions(self):
        return (0.05, None, None)

    def command(self, pitch_pos=None, roll_pos=None, rudder_pos=None):
        self.commands.append((pitch_pos, roll_pos, rudder_pos))

    def release(self):
        self.released = True
        return []


class VviPidTests(unittest.TestCase):
    def test_vvi_pid_commands_nose_up_when_below_target_alt_or_descending(self):
        ctrl = VviPidController(target_alt_ft=4000.0, target_vvi_fpm=0.0)
        ctrl.reset(target_alt_ft=4000.0, initial_stick_pos=0.0, initial_pitch_deg=0.0)
        cmd_climb = ctrl.update(alt_ft=3950.0, vertical_speed_fpm=-200.0, raw_pitch_deg=0.0, dt=0.1, measured_stick_pos=0.0)
        self.assertGreater(cmd_climb, 0.0)

        ctrl2 = VviPidController(target_alt_ft=4000.0, target_vvi_fpm=0.0)
        ctrl2.reset(target_alt_ft=4000.0, initial_stick_pos=0.0, initial_pitch_deg=0.0)
        cmd_descend = ctrl2.update(alt_ft=4050.0, vertical_speed_fpm=300.0, raw_pitch_deg=0.0, dt=0.1, measured_stick_pos=0.0)
        self.assertLess(cmd_descend, 0.0)

    def test_custom_vvi_updater_plugs_in_cleanly(self):
        calls = []

        def custom_stick(target_vvi, actual_vvi, alt_ft, target_alt, pitch_deg, dt):
            calls.append((target_vvi, actual_vvi, alt_ft, target_alt, pitch_deg, dt))
            return 0.25

        ctrl = VviPidController(target_alt_ft=4000.0, custom_updater=custom_stick, output_mode='stick')
        ctrl.reset(initial_stick_pos=0.0, initial_pitch_deg=0.0)
        out = ctrl.update(alt_ft=4000.0, vertical_speed_fpm=-100.0, raw_pitch_deg=0.0, dt=0.2, measured_stick_pos=0.0)
        self.assertEqual(len(calls), 1)
        self.assertGreater(out, 0.0)
        self.assertLessEqual(out, 0.25)

    def test_force_altitude_hold_engages_pitch_only_and_releases(self):
        brunner = type('B', (), {'sock': object()})()
        hold = ForceAltitudeHold(
            4500.0,
            transport_factory=lambda b: FakeTransport(b, pitch_enabled=True, bank_enabled=False, yaw_enabled=False),
        )
        hold.engage(brunner, 100.0, alt_ft=4500.0, pitch_deg=0.0)
        self.assertTrue(hold.transport.engaged)
        cmd = hold.step({'alt_ft': 4460.0, 'vertical_speed': -150.0, 'pitch_deg': 0.0}, 100.2)
        self.assertGreater(cmd, 0.05)
        self.assertEqual(len(hold.transport.commands), 1)
        p, r, y = hold.transport.commands[0]
        self.assertIsNotNone(p)
        self.assertIsNone(r)
        self.assertIsNone(y)
        hold.release()
        self.assertTrue(hold.transport is None)
