import unittest

from backend.control_profile import StaticControlProfile
from backend.pid_assist import PidAssist


class PitchSweepPidHandoffTests(unittest.TestCase):
    def test_profile_begins_and_ends_at_captured_pitch_position(self):
        profile = StaticControlProfile('2.a.(1)', start_position=0.23)
        self.assertAlmostEqual(profile.target(0.0)[0], 0.23)
        self.assertAlmostEqual(profile.segments[1].start_pos, 0.23)
        self.assertEqual(profile.segments[1].end_pos, -1.0)
        self.assertEqual(profile.segments[2].start_pos, -1.0)
        self.assertEqual(profile.segments[2].end_pos, 1.0)
        self.assertAlmostEqual(profile.target(profile.total_duration)[0], 0.23)

    def test_handoff_releases_pid_and_returns_measured_position(self):
        context = {'active_pids': {'airspeed': True, 'bank': False, 'yaw': False}}
        assist = PidAssist(context, owned_axis_mask=0)
        class Transport:
            def __init__(self): self.released = False
            def release(self): self.released = True; return []
        transport = Transport()
        assist.transport = transport
        assist.measured_pitch = -0.31
        position = assist.release_for_test_handoff()
        self.assertAlmostEqual(position, -0.31)
        self.assertTrue(transport.released)
        self.assertFalse(assist.pitch_enabled)
        self.assertEqual(assist.pid_axis_mask & 0x01, 0)
        self.assertTrue(context['pid_runtime']['handoff_complete'])

    def test_handoff_rejects_other_pid_axes(self):
        assist = PidAssist({'active_pids': {'airspeed': True, 'bank': True, 'yaw': False}},
                           owned_axis_mask=0)
        with self.assertRaisesRegex(RuntimeError, 'Airspeed PID only'):
            assist.release_for_test_handoff()

    def test_stabilization_cannot_complete_below_one_knot_lower_limit(self):
        context = {'active_pids': {'airspeed': True, 'bank': False, 'yaw': False},
                   'pid_conditions': {'target_ias': 110}, 'pid_minimum_airspeed': 109}
        assist = PidAssist(context, owned_axis_mask=0)
        assist.stabilization_started = 0.0
        assist.update = lambda *_args, **_kwargs: {'pitch': 0.1}
        telemetry = {'airspeed_indicated': 108.9, 'pitch_angle': 0,
                     'bank_angle': 0, 'true_heading': 0}
        self.assertFalse(assist.update_stabilization(0.0, telemetry, None))
        self.assertFalse(assist.update_stabilization(10.0, telemetry, None))
        self.assertEqual(context['pid_runtime']['axes']['airspeed']['minimum'], 109)
        telemetry['airspeed_indicated'] = 109.0
        self.assertFalse(assist.update_stabilization(11.0, telemetry, None))
        self.assertTrue(assist.update_stabilization(16.0, telemetry, None))

    def test_pitch_capture_uses_faster_damped_controller_only_in_this_mode(self):
        active = {'airspeed': True, 'bank': False, 'yaw': False}
        generic = PidAssist({'active_pids': active}, owned_axis_mask=0)
        capture = PidAssist({'active_pids': active,
                             'pitch_sweep_airspeed_capture': True}, owned_axis_mask=0)
        self.assertEqual(generic.airspeed_ctrl.pitch_slew_rate, 1.25)
        self.assertEqual(generic.airspeed_ctrl.rate_filter_tau, 0.60)
        self.assertEqual((generic.airspeed_ctrl.kp, generic.airspeed_ctrl.ki,
                          generic.airspeed_ctrl.kd), (0.24, 0.008, 0.32))
        self.assertEqual(capture.airspeed_ctrl.pitch_slew_rate, 1.25)
        self.assertEqual(capture.airspeed_ctrl.capture_slew_rate, 2.50)
        self.assertEqual(capture.airspeed_ctrl.rate_filter_tau, 0.60)
        self.assertEqual((capture.airspeed_ctrl.kp, capture.airspeed_ctrl.ki,
                          capture.airspeed_ctrl.kd), (0.24, 0.008, 0.32))
        self.assertEqual(capture.airspeed_ctrl.capture_band, 4.0)

    def test_capture_preserves_normal_approach_then_freezes_integral_near_target(self):
        active = {'airspeed': True, 'bank': False, 'yaw': False}
        assist = PidAssist({'active_pids': active,
                            'pitch_sweep_airspeed_capture': True}, owned_axis_mask=0)
        controller = assist.airspeed_ctrl
        controller.reset(initial_pitch=0.0)
        controller.update(80.0, 60.0, 0.5, 0.0)
        self.assertLess(controller.integral, 0.0)
        controller.update(80.0, 75.0, 0.5, 0.0)
        before_capture = controller.integral
        controller.update(80.0, 78.0, 0.5, 0.0)
        self.assertEqual(controller.integral, before_capture)

    def test_capture_matches_original_far_from_target_and_brakes_only_near_target(self):
        from backend.multi_pid_runner import AirspeedController
        generic = AirspeedController()
        capture = AirspeedController(capture_band=4.0, capture_brake_gain=0.55,
                                     capture_slew_rate=2.50)
        generic.reset(0.0); capture.reset(0.0)
        for ias in (60.0, 65.0, 70.0, 75.0):
            self.assertAlmostEqual(generic.update(80.0, ias, 0.5, 0.0),
                                   capture.update(80.0, ias, 0.5, 0.0))
        generic_near = generic.update(80.0, 78.0, 0.5, 0.0)
        capture_near = capture.update(80.0, 78.0, 0.5, 0.0)
        self.assertGreater(capture_near, generic_near)


if __name__ == '__main__':
    unittest.main()
