"""Offline tests for ground-only brake engagement and readback."""
import unittest
from unittest.mock import Mock

from backend.parking_brake import GroundParkingBrake


class GroundParkingBrakeTests(unittest.TestCase):
    def frame(self, **overrides):
        return dict(dict(receipt_time=10, sequence=1, ground_speed=0,
                         on_ground=1, parking_brake=0), **overrides)

    def test_off_brake_is_commanded_once_and_new_on_readback_is_required(self):
        send = Mock()
        brake = GroundParkingBrake(send)
        self.assertFalse(brake.verify(self.frame(), 10))
        self.assertFalse(brake.verify(self.frame(), 10))
        self.assertFalse(brake.verify(self.frame(parking_brake=1), 10))
        self.assertTrue(brake.verify(self.frame(parking_brake=1, sequence=2), 10))
        send.assert_called_once_with()

    def test_never_toggles_when_airborne_moving_stale_or_unknown(self):
        for frame in (self.frame(on_ground=0), self.frame(ground_speed=2),
                      self.frame(receipt_time=8), self.frame(parking_brake=None)):
            with self.subTest(frame=frame):
                send = Mock()
                self.assertFalse(GroundParkingBrake(send).verify(frame, 10))
                send.assert_not_called()

    def test_already_engaged_does_not_send_toggle(self):
        send = Mock()
        self.assertTrue(GroundParkingBrake(send).verify(self.frame(parking_brake=1), 10))
        send.assert_not_called()
