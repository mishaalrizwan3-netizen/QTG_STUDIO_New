"""Engage a ground parking brake once and require newer measured readback."""
import math


class GroundParkingBrake:
    def __init__(self, send, *, require_stationary=True):
        self.send = send
        self.require_stationary = require_stationary
        self.command_sequence = None

    def verify(self, frame, now):
        if not frame:
            return False
        try:
            age = now - float(frame['receipt_time'])
            sequence = int(frame['sequence'])
            speed = float(frame['ground_speed'])
            brake = frame['parking_brake']
            ground = frame['on_ground']
        except (KeyError, TypeError, ValueError, OverflowError):
            return False
        if not 0 <= age <= 1 or ground != 1 or not math.isfinite(speed) or speed < 0:
            return False
        if self.require_stationary and speed > 1:
            return False
        if self.command_sequence is not None:
            return sequence > self.command_sequence and brake == 1
        if brake == 1:
            return True
        if brake == 0:
            self.send()
            self.command_sequence = sequence
        return False
