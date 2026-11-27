"""Track a single stall-warning cycle from fresh simulator measurements.

This module does not command aircraft or hardware controls.
"""
import math


class StallWarningCycle:
    def __init__(self, *, max_age_s=0.5):
        self.max_age_s = max_age_s
        self.last_sequence = None
        self.warning_seen = False
        self.completed = False

    def update(self, sample, now):
        if self.completed:
            return 'COMPLETE'
        try:
            sequence = sample['sequence']
            age = float(now) - float(sample['receipt_time'])
            warning = float(sample['stall_warning'])
        except (KeyError, TypeError, ValueError, OverflowError):
            raise ValueError('Stall warning telemetry unavailable') from None
        if -0.1 <= age < 0:
            age = 0.0
        if not math.isfinite(age) or not 0 <= age <= self.max_age_s:
            raise ValueError('Stall warning telemetry is stale')
        if warning not in (0., 1.):
            raise ValueError('Invalid stall warning telemetry')
        if self.last_sequence is None and warning == 1.:
            raise ValueError('Stall warning is already active before the approach')
        if self.last_sequence is not None and sequence <= self.last_sequence:
            raise ValueError('Stall warning telemetry did not advance')
        self.last_sequence = sequence
        if not self.warning_seen and warning == 1.:
            self.warning_seen = True
            return 'WARNING'
        if self.warning_seen and warning == 0.:
            self.completed = True
            return 'COMPLETE'
        return 'WARNING' if self.warning_seen else 'APPROACH'
