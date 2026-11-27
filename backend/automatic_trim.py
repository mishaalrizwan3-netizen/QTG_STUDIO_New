"""Automatic Pitch Trim test using Prepar3D trim events."""
import ctypes
import math
from pathlib import Path
import sys
import threading
import time
import json

from backend.automatic_control import AutomaticControl, measured_control_fields, recorded_flight_fields
from backend.initializer import apply_body_acceleration

REF = '2.a.(7)'
LEGACY_REF = '2.a.(4)'
METHOD = 'pitch_trim'
UP, DOWN = 910, 911
SET_TRIM = 912
TRIM_STEP_INTERVAL = 1.0
TRIM_STEP_FRACTION = 0.001
TRIM_TARGET_TOLERANCE = 0.00025
CALIBRATION_STEP_FRACTION = 0.04
HOLD_FINAL_SECONDS = 1.0       # Hold final position for 1 second
MIN_RECORDING_SECONDS = 3.0
START_HOLD_SECONDS = 1.0       # Wait 1 second and confirm starting position
UNRESPONSIVE_LIMIT = 3         # Fail if trim does not respond for 3 consecutive attempts when not at limit
STAGED_TRIM_PERCENTAGES = (100.0, 50.0, 25.0, 0.0, -25.0, -50.0, -100.0)
STAGED_TARGET_TIMEOUT = 30.0
STAGED_HOLD_SECONDS = 2.0
STAGED_PULSE_INTERVAL = 0.04
STAGED_PULSE_RETRY_SECONDS = 1.0
STAGED_FEEDBACK_CHANGE_PCT = 0.25
STAGED_TARGET_TOLERANCE_PCT = 2.5
STAGED_HOLD_DRIFT_TOLERANCE_PCT = 1.0
CHANNELS = (('trim_rad', 'ELEVATOR TRIM POSITION', 'radians'),
            ('indicator', 'ELEVATOR TRIM INDICATOR', 'position'),
            ('trim_fraction', 'ELEVATOR TRIM PCT', 'percent over 100'),
            ('elevator_rad', 'ELEVATOR DEFLECTION', 'radians'))


def selected(context):
    ref = context.get('test_reference')
    return context.get('mode') == 'auto' and (ref == REF or ref == LEGACY_REF) and context.get('control_method') == METHOD


class TrimEvents:
    """Send trim button increments on the trim measurement's SimConnect session."""
    def __init__(self, reader):
        self.dll, self.handle = reader.dll, reader.handle
        u, p = ctypes.c_uint32, ctypes.c_void_p
        self.mapping = self.dll.SimConnect_MapClientEventToSimEvent
        self.mapping.argtypes, self.mapping.restype = [p, u, ctypes.c_char_p], ctypes.c_int32
        self.transmit = self.dll.SimConnect_TransmitClientEvent
        self.transmit.argtypes, self.transmit.restype = [p, u, u, u, u, u], ctypes.c_int32
        self.target_mapped = False
        for identifier, name in ((UP, b'ELEV_TRIM_UP'), (DOWN, b'ELEV_TRIM_DN')):
            self._check(self.mapping(self.handle, identifier, name))

    @staticmethod
    def _check(result):
        if result < 0:
            raise RuntimeError('Pitch trim SimConnect event failed: ' + hex(result & 0xFFFFFFFF))

    def step(self, direction):
        if direction not in (UP, DOWN):
            raise ValueError('Invalid trim direction')
        self._check(self.transmit(self.handle, 0, direction, 0, 1, 0x10))

    def set_fraction(self, fraction):
        if not math.isfinite(fraction) or not -1.0 <= fraction <= 1.0:
            raise ValueError('Invalid elevator trim target')
        if not self.target_mapped:
            self._check(self.mapping(self.handle, SET_TRIM, b'AXIS_ELEV_TRIM_SET'))
            self.target_mapped = True
        # Prepar3D's trim-axis event uses the opposite sign to ELEVATOR TRIM PCT.
        axis = round(-fraction * 16383)
        self._check(self.transmit(self.handle, 0, SET_TRIM, axis & 0xFFFFFFFF, 1, 0x10))


class AutomaticTrim(AutomaticControl):
    def __init__(self, context, sim, brunner, publish, reader, *, clock=time.monotonic, brake_reader=None, events=None):
        if not selected(context):
            raise ValueError('Pitch trim Automatic context required')
        synthetic = dict(context, test_reference='2.a.(1)', control_method='static_control')
        super().__init__(synthetic, sim, brunner, publish, clock=clock, live_hardware=False, brake_reader=brake_reader)
        self.context = dict(context)
        from backend.pid_assist import PidAssist
        self.pid_assist = PidAssist(self.context, owned_axis_mask=0)
        self.test_ref = REF
        self.trim_step_fraction = CALIBRATION_STEP_FRACTION if context['test_reference'] == REF else TRIM_STEP_FRACTION
        self.axis_name = 'Pitch trim'
        self.reader = reader
        self.events = events if events is not None else TrimEvents(reader)
        self.sequence_number = None
        self.last_frame_time = None

        # Initialization state machine:
        # Step 1: P3D condition initialization (handled by super().step())
        # Step 2: Drive trim to verified full nose-up limit via SimConnect
        # Step 3: Wait 1 second, confirm starting position, display READY TO RECORD
        self.init_trim_phase = 'AIRCRAFT_INIT'
        self.init_trim_start = None
        self.init_last_up_step = 0.0
        self.init_last_pos = None
        self.init_stable_count = 0
        self.init_max_pos = None
        self.init_last_increase_time = None
        self.init_hold_start = None
        self.verified_nose_up_limit = None
        self.start_trim_pos = None
        self.unpausing_until = 0.0

        # Recording state machine:
        # Step 3: Click Start Recording -> Begin recording actual trim position
        # Step 4: Command a decrease of 0.1 percentage points
        # Step 5: Wait 1 second while recording actual trim position
        # Step 6: Repeat one trim step + 1s wait until verified full nose-down limit
        # Step 7: Hold final position for 1 second
        # Step 8: Automatically stop recording and generate reports
        self.record_state = 'IDLE'
        self.step_count = 0
        self.step_sent_time = None
        self.trim_start_fraction = None
        self.trim_target = None
        self.unresponsive_steps = 0
        self.verified_nose_down_limit = None
        self.final_trim_pos = None
        self.travel_start_time = None
        self.travel_end_time = None
        self.total_travel_time = 0.0
        self.average_trim_rate_pct = 0.0
        self.average_trim_rate_pos = 0.0
        self.hold_start_time = None
        # Test 2.a.(4) only: measured 0 -> 25 -> 50 -> 100 -> 50 -> 25 -> 0 cycle.
        self.staged_cycle = context['test_reference'] in (REF, LEGACY_REF)
        self.staged_target_index = 0
        self.staged_target_started = None
        self.staged_hold_started = None
        self.staged_pending_timing = None
        self.staged_last_pulse = None
        self.staged_direction = None
        self.staged_hold_anchor_pct = None
        self.staged_last_pulse_feedback_pct = None
        self.staged_timings = []

    def status(self):
        state = super().status()
        state['duration_s'] = (math.ceil(len(STAGED_TRIM_PERCENTAGES) * (STAGED_TARGET_TIMEOUT + STAGED_HOLD_SECONDS) + HOLD_FINAL_SECONDS)
                               if self.staged_cycle else
                               math.ceil(math.ceil(2.0 / self.trim_step_fraction) * TRIM_STEP_INTERVAL + HOLD_FINAL_SECONDS))
        state['engine_text'] = 'Engine: Pitch Trim Test'
        if self.init_trim_phase != 'DONE':
            state['can_record'] = False
            if state.get('stage') == 'READY TO RECORD':
                state['stage'] = 'Initializing aircraft'
                state['recording_text'] = 'Initializing aircraft'
        else:
            if self.stage == 'READY TO RECORD':
                limit_str = f"{self.start_trim_pos:+.3f}" if self.start_trim_pos is not None else "full nose-up"
                state['details'] = f"Trim at verified nose-up limit ({limit_str}). Ready to record."
                state['recording_text'] = 'READY TO RECORD [P3D pitch trim]'
                state['progress_text'] = ('Click Start Recording for +100/+50/+25/0/-25/-50/-100% trim steps'
                                          if self.staged_cycle else
                                          'Click Start Recording to begin pitch trim travel to nose-down limit')
                state['can_record'] = True
        if self.phase == 'RECORD':
            state['progress_text'] = self.details
        state['controls_text'] = 'P3D pitch trim events ready (SimConnect)'
        return state

    def _release(self):
        # Increment events have no held state to release.
        return None

    def command(self, cmd):
        action = cmd.get('action')
        if action in ('stop', 'shutdown', 'emergency_stop'):
            self.finish('ABORTED', 'Operator stopped the pitch trim test')
            if action == 'shutdown':
                self.closed = True
            return
        if action == 'record' and self.stage == 'READY TO RECORD':
            try:
                if hasattr(self.sim, 'unpause_sim'):
                    self.sim.unpause_sim()
                if hasattr(self.reader, 'discard_pending'):
                    self.reader.discard_pending()
                now = self.clock()
                self.t_start = now
                self.travel_start_time = now
                self.last_frame_time = now
                self.unpausing_until = now + 4.0
                self.sequence_number = None
                if (getattr(self, 'pid_assist', None) and self.pid_assist.has_active_pids
                        and self.brunner and getattr(self.brunner, 'sock', None)
                        and self.pid_assist.transport is None):
                    try:
                        self.brunner.configure_replay_mode(axis_mask=self.pid_assist.pid_axis_mask)
                        self.brunner.write_setting(self.pid_assist.pid_axis_mask, 0x70, "B", 1)
                        self.brunner.set_overrides(self.pid_assist.pid_axis_mask, True)
                    except Exception:
                        pass
                self.phase, self.stage = 'RECORD', 'RECORDING'
                self.record_state = 'RECORDING_START'
                self.rows = []
                self.step_count = 0
                self.step_sent_time = None
                self.trim_start_fraction = None
                self.trim_target = None
                self.unresponsive_steps = 0
                self.staged_target_index = 0
                self.staged_target_started = None
                self.staged_hold_started = None
                self.staged_pending_timing = None
                self.staged_last_pulse = None
                self.staged_direction = None
                self.staged_hold_anchor_pct = None
                self.staged_last_pulse_feedback_pct = None
                self.staged_timings = []
                self.details = 'Starting pitch trim recording...'
            except Exception as exc:
                self.finish('FAILED', str(exc))
        self.emit()

    def step(self):
        now = self.clock()
        try:
            if self.phase != 'RECORD':
                if self.phase in ('DONE', 'CLOSED'):
                    return

                if self.phase == 'PID_STABILIZE':
                    self.latest_sim = self.sim.get_telemetry() or {}
                    if self.pid_assist.update_stabilization(now, self.latest_sim, self.brunner):
                        self.phase, self.stage = 'READY', 'READY TO RECORD'
                        self.details = f'PID stabilization achieved for 5.0 s. Starting recording automatically from {self.start_trim_pos:+.3f}.'
                        self.command({'action': 'record'})
                    else:
                        runtime = self.context.get('pid_runtime') or {}
                        self.details = f"All selected PIDs within tolerance: {runtime.get('stabilization_remaining_s', 5.0):.1f} s remaining"
                    self.emit()
                    return

                # Stage 1: Standard aircraft condition initialization (position, altitude, throttle, brakes)
                if self.init_trim_phase == 'AIRCRAFT_INIT':
                    super().step()
                    # When super() declares READY, intercept it to initialize elevator trim
                    if self.phase == 'READY':
                        self.phase = 'CONNECT'
                        self.stage = 'Initializing aircraft'
                        self.details = 'Setting elevator trim to verified full nose-up limit...'
                        if hasattr(self.sim, 'unpause_sim'):
                            self.sim.unpause_sim()
                        self.init_trim_phase = 'DRIVE_UP'
                        self.init_trim_start = now
                        self.init_last_up_step = 0.0
                        self.init_max_pos = None
                        self.init_last_increase_time = None
                        self.emit()
                    return

                if not self.sim.is_connected():
                    raise RuntimeError('Sim Flight Model disconnected during initialization')

                # Stage 2: Drive elevator trim to verified full nose-up limit using SimConnect
                if self.init_trim_phase == 'DRIVE_UP':
                    frame = self.reader.poll()
                    if frame:
                        self.sequence_number = frame.get('sequence')
                        self.last_frame_time = now
                        pos = float(frame.get('indicator', frame.get('trim_fraction', 0.0)))

                        if self.init_max_pos is None or pos > self.init_max_pos + 0.0005:
                            self.init_max_pos = pos
                            self.init_last_increase_time = now

                        # Verified nose-up limit reached when position no longer advances upward or hits max limit (+0.999)
                        limit_reached = (pos >= 0.999) or (
                            (now - self.init_trim_start >= 1.0) and
                            (self.init_last_increase_time is not None) and
                            (now - self.init_last_increase_time >= 0.6)
                        )
                        if limit_reached:
                            self.verified_nose_up_limit = pos
                            self.init_trim_phase = 'WAIT_1S'
                            self.init_hold_start = now
                            self.details = f"Trim at verified nose-up limit ({pos:+.3f}). Confirming starting position..."
                            self.emit()
                            return

                    if self.init_trim_start is not None and now - self.init_trim_start > 25.0:
                        raise RuntimeError('Timed out setting elevator trim to full nose-up limit')

                    # Pulse ELEV_TRIM_UP
                    if now - self.init_last_up_step >= 0.04:
                        self.events.step(UP)
                        self.init_last_up_step = now
                    return

                # Stage 3: Wait 1 second, confirm the starting position, then display READY TO RECORD
                if self.init_trim_phase == 'WAIT_1S':
                    frame = self.reader.poll()
                    if frame:
                        self.sequence_number = frame.get('sequence')
                        self.last_frame_time = now
                        pos = float(frame.get('indicator', frame.get('trim_fraction', 0.0)))
                        if self.verified_nose_up_limit is None:
                            self.verified_nose_up_limit = pos
                        elif abs(pos - self.verified_nose_up_limit) > 0.05:
                            self.verified_nose_up_limit = pos
                            self.init_hold_start = now

                    if self.init_hold_start is not None and now - self.init_hold_start >= START_HOLD_SECONDS:
                        self.start_trim_pos = self.verified_nose_up_limit
                        self._pause_for_ready()
                        apply_body_acceleration(self.sim, self.context)
                        if (self.pid_assist.has_active_pids
                                and self.context.get('test_reference') != '2.a.(4)'):
                            self.sim.unpause_sim()
                            self.pid_assist.begin_stabilization(now, self.brunner)
                            self.phase, self.stage = 'PID_STABILIZE', 'PID STABILIZATION'
                            self.details = 'Starting selected PIDs; stabilization requires 5.0 continuous seconds'
                            self.init_trim_phase = 'DONE'
                            self.emit()
                            return
                        self.phase = 'READY'
                        self.stage = 'READY TO RECORD'
                        self.details = f"Verified nose-up limit: {self.start_trim_pos:+.3f}. Ready to record."
                        self.init_trim_phase = 'DONE'
                        self.emit()
                    return

                return

            # Phase == 'RECORD'
            if not self.sim.is_connected():
                raise RuntimeError('Sim Flight Model disconnected during recording')

            frame = self.reader.poll()
            now = self.clock()
            if frame and frame.get('sequence') != self.sequence_number:
                if not -0.1 <= now - frame['receipt_time'] <= 2.0:
                    raise RuntimeError('Trim telemetry is stale')
                if not all(math.isfinite(float(frame[k])) for k in ('trim_rad', 'indicator', 'trim_fraction', 'elevator_rad')):
                    raise RuntimeError('Trim or elevator telemetry unavailable')

                self.sequence_number = frame['sequence']
                self.last_frame_time = now
                self.latest_sim = self.sim.get_telemetry() or {}

                # Continuously record actual P3D elevator-trim position and parameters
                pos = float(frame['indicator'])
                angle = float(frame['elevator_rad'])
                trim_pct = float(frame['trim_fraction']) * 100.0

                row = {
                    'Time (s)': now - self.t_start,
                    'Elevator Trim Angle (rad)': frame['trim_rad'],
                    'Elevator Trim Indicator (position)': pos,
                    'Elevator Trim (%)': trim_pct,
                    'Elevator Surface (rad)': angle,
                    **recorded_flight_fields(self.latest_sim)
                }
                row.update(measured_control_fields(self.brunner))
                self.rows.append(row)

                if getattr(self, 'pid_assist', None) and self.pid_assist.has_active_pids:
                    self.pid_assist.update(now, self.latest_sim, self.brunner)

                if self.start_trim_pos is None:
                    self.start_trim_pos = pos

                # Start from the measured percentage and command one step per second.
                if self.record_state == 'RECORDING_START':
                    self.trim_start_fraction = min(1.0, max(-1.0, float(frame['trim_fraction'])))
                    if self.staged_cycle:
                        self._send_staged_target(now)
                        self.record_state = 'WAIT_STAGE'
                    else:
                        self._send_trim_target(now)
                        self.record_state = 'WAIT_STEP'
                    return

                # Test 2.a.(4): command each requested percentage and use actual
                # SimConnect feedback (not the command time) to stop its timer.
                elif self.record_state == 'WAIT_STAGE':
                    requested_pct = STAGED_TRIM_PERCENTAGES[self.staged_target_index]
                    elapsed = now - self.staged_target_started
                    measured_position_pct = float(frame['trim_fraction']) * 100.0
                    self.details = (f'Trim target {requested_pct:.0f}% | actual {measured_position_pct:.1f}% | '
                                    f'elapsed {elapsed:.2f} s')
                    reached = ((self.staged_direction == DOWN and
                                measured_position_pct <= requested_pct + STAGED_TARGET_TOLERANCE_PCT)
                               or (self.staged_direction == UP and
                                   measured_position_pct >= requested_pct - STAGED_TARGET_TOLERANCE_PCT)
                               or self.staged_direction is None)
                    if reached:
                        self.staged_pending_timing = {
                            'sequence': self.staged_target_index + 1,
                            'target_percent': requested_pct,
                            'direction': 'descending',
                            'movement_time_s': elapsed,
                            'elapsed_s': elapsed,
                            'reached_at_s': now - self.t_start,
                            'measured_percent': measured_position_pct,
                            'hold_time_s': STAGED_HOLD_SECONDS,
                        }
                        self.staged_hold_started = now
                        self.staged_hold_anchor_pct = measured_position_pct
                        self.record_state = 'HOLD_STAGE'
                        self.details = (f'Trim {requested_pct:.0f}% reached in {elapsed:.2f} s; '
                                        f'holding {STAGED_HOLD_SECONDS:.1f} s')
                        return
                    if elapsed >= STAGED_TARGET_TIMEOUT:
                        raise RuntimeError(f'Pitch trim failed to reach {requested_pct:.0f}% within {STAGED_TARGET_TIMEOUT:.0f} s '
                                           f'(actual {measured_position_pct:.1f}%)')
                    feedback_changed = (self.staged_last_pulse_feedback_pct is not None and
                                        abs(measured_position_pct - self.staged_last_pulse_feedback_pct) >= STAGED_FEEDBACK_CHANGE_PCT)
                    retry_due = (self.staged_last_pulse is not None and
                                 now - self.staged_last_pulse >= STAGED_PULSE_RETRY_SECONDS)
                    if self.staged_last_pulse is None or feedback_changed or retry_due:
                        self.events.step(self.staged_direction)
                        self.staged_last_pulse = now
                        self.staged_last_pulse_feedback_pct = measured_position_pct
                    return

                elif self.record_state == 'HOLD_STAGE':
                    requested_pct = STAGED_TRIM_PERCENTAGES[self.staged_target_index]
                    measured_position_pct = float(frame['trim_fraction']) * 100.0
                    if abs(measured_position_pct - self.staged_hold_anchor_pct) > STAGED_HOLD_DRIFT_TOLERANCE_PCT:
                        raise RuntimeError(f'Pitch trim left the {requested_pct:.0f}% position during its '
                                           f'{STAGED_HOLD_SECONDS:g} s hold '
                                           f'(actual {measured_position_pct:.1f}%)')
                    hold_elapsed = now - self.staged_hold_started
                    self.details = (f'Holding trim {requested_pct:.0f}% | actual {measured_position_pct:.1f}% | '
                                    f'{min(hold_elapsed, STAGED_HOLD_SECONDS):.1f}/{STAGED_HOLD_SECONDS:.1f} s')
                    if hold_elapsed < STAGED_HOLD_SECONDS:
                        return
                    self.staged_pending_timing['hold_completed_at_s'] = now - self.t_start
                    self.staged_timings.append(self.staged_pending_timing)
                    self.staged_pending_timing = None
                    self.staged_target_index += 1
                    if self.staged_target_index < len(STAGED_TRIM_PERCENTAGES):
                        self._send_staged_target(now)
                        self.record_state = 'WAIT_STAGE'
                        return
                    self.final_trim_pos = pos
                    self.travel_end_time = now
                    self.total_travel_time = max(0.001, now - self.travel_start_time)
                    self.total_travel_amount = 2.0
                    movement_total = sum(item['movement_time_s'] for item in self.staged_timings)
                    self.average_trim_rate_pos = self.total_travel_amount / max(0.001, movement_total)
                    self.average_trim_rate_pct = 200.0 / max(0.001, movement_total)
                    self.record_state = 'HOLD_FINAL'
                    self.hold_start_time = now
                    self.details = 'Pitch-trim positive-to-negative step sequence complete.'
                    return

                # 5 & 6. Wait 1 second while recording actual trim position; repeat until verified full nose-down limit
                elif self.record_state == 'WAIT_STEP':
                    step_elapsed = now - self.step_sent_time
                    if step_elapsed < TRIM_STEP_INTERVAL:
                        self.details = f"Step {self.step_count}: Recording actual trim ({step_elapsed:.1f}/1.0 s) | Pos: {pos:+.3f}"
                        return

                    if abs(frame['trim_fraction'] - self.trim_target) > TRIM_TARGET_TOLERANCE:
                        self.unresponsive_steps += 1
                        if self.unresponsive_steps >= UNRESPONSIVE_LIMIT:
                            raise RuntimeError(f"P3D elevator trim did not reach {self.trim_target * 100:+.1f}% (measured {trim_pct:+.2f}%)")
                        self.events.set_fraction(self.trim_target)
                        self.step_sent_time = now
                        self.details = f"Waiting for P3D to reach trim target {self.trim_target * 100:+.1f}%"
                        return
                    self.unresponsive_steps = 0
                    if self.trim_target > -1.0:
                        self._send_trim_target(now)
                        return

                    self.verified_nose_down_limit = self.final_trim_pos = pos
                    self.travel_end_time = now
                    self.total_travel_time = max(0.1, now - self.travel_start_time)
                    self.total_travel_amount = abs(self.start_trim_pos - pos)
                    self.average_trim_rate_pos = self.total_travel_amount / self.total_travel_time
                    self.average_trim_rate_pct = abs(self.rows[0]['Elevator Trim (%)'] - trim_pct) / self.total_travel_time
                    self.record_state = 'HOLD_FINAL'
                    self.hold_start_time = now
                    self.details = f"Nose-down limit reached ({pos:+.3f}). Holding final position for 1.0 s..."
                    return

                # 7. Hold final position for 1 second
                elif self.record_state == 'HOLD_FINAL':
                    hold_elapsed = now - self.hold_start_time
                    expected_final = self.trim_target if self.staged_cycle else -1.0
                    if abs(frame['trim_fraction'] - expected_final) > TRIM_TARGET_TOLERANCE:
                        raise RuntimeError('P3D elevator trim left the final position during the final hold')
                    if hold_elapsed < HOLD_FINAL_SECONDS or now - self.t_start < MIN_RECORDING_SECONDS:
                        self.details = f"Holding final position ({hold_elapsed:.1f}/1.0 s) | Pos: {pos:+.3f}"
                        return

                    # 8. Automatically stop recording and generate CSV/PDF graphs
                    if self.staged_cycle:
                        msg = (f'Pitch trim +100/+50/+25/0/-25/-50/-100% step sequence complete. '
                               f'Total cycle time: {self.total_travel_time:.2f} s')
                    else:
                        msg = (f"Pitch trim nose-down limit reached ({self.verified_nose_down_limit:+.3f}). "
                               f"Travel time: {self.total_travel_time:.1f} s, Avg rate: {self.average_trim_rate_pct:.2f} %/s")
                    self.finish('COMPLETE', msg)
                    return

            # Telemetry watchdog
            watchdog_limit = 4.0 if now <= getattr(self, 'unpausing_until', 0.0) else 2.0
            if self.last_frame_time is not None and (now - self.last_frame_time > watchdog_limit):
                raise RuntimeError('Trim telemetry stopped advancing')

        except Exception as exc:
            self.finish('FAILED', str(exc))
        finally:
            self._maybe_emit(now)

    def _send_trim_target(self, now):
        self.step_count += 1
        self.trim_target = max(-1.0, self.trim_start_fraction - self.step_count * self.trim_step_fraction)
        self.events.set_fraction(self.trim_target)
        self.step_sent_time = now
        self.details = f"Step {self.step_count}: Target trim {self.trim_target * 100:+.1f}%"

    def _send_staged_target(self, now):
        requested_pct = STAGED_TRIM_PERCENTAGES[self.staged_target_index]
        self.trim_target = min(1.0, max(-1.0, requested_pct / 100.0))
        self.staged_target_started = now
        current_pct = (self.rows[-1]['Elevator Trim (%)'] if self.rows else
                       self.trim_start_fraction * 100.0)
        if abs(current_pct - requested_pct) <= STAGED_TARGET_TOLERANCE_PCT:
            self.staged_direction = None
        else:
            self.staged_direction = DOWN if requested_pct < current_pct else UP
        self.staged_last_pulse = None
        self.staged_last_pulse_feedback_pct = None
        self.details = f'Moving pitch trim toward {requested_pct:+.0f}%; measuring actual travel time'

    def finish(self, result, reason):
        if self.phase == 'DONE':
            return
        try:
            self.sim.pause_sim()
        except Exception:
            pass
        self.phase, self.result, self.stage, self.details = 'DONE', result, result, reason
        if self.rows:
            try:
                # Add calculated travel metrics to context for reports
                self.context['pitch_trim_metrics'] = {
                    'total_travel_time_s': getattr(self, 'total_travel_time', 0.0),
                    'average_trim_rate_pct_s': getattr(self, 'average_trim_rate_pct', 0.0),
                    'average_trim_rate_pos_s': getattr(self, 'average_trim_rate_pos', 0.0),
                    'start_trim_pos': getattr(self, 'start_trim_pos', 0.0),
                    'final_trim_pos': getattr(self, 'final_trim_pos', 0.0),
                    'staged_cycle': bool(getattr(self, 'staged_cycle', False)),
                    'stage_timings': list(getattr(self, 'staged_timings', [])),
                }
                from reports.automatic_control_report import save
                self.outputs = save(self.context, self.rows, result, reason)
                if result == 'COMPLETE':
                    self.stage = 'Completed'
            except Exception as exc:
                self.stage, self.details = 'FAILED', reason + '; report error: ' + str(exc)
        self.emit()


def run(context, publish, *, sim_factory=None, brunner_factory=None, reader_factory=None, events_factory=None, sleep=time.sleep):
    from interfaces.brunner_base.src.p3d_simconnect_v15 import SimConnectClient
    from interfaces.brunner_base.src.brunner_interface import BrunnerInterface
    from interfaces.engine_measurements import Measurements
    sim = reader = brake_reader = brunner = thread = controller = None
    try:
        sim = (sim_factory or SimConnectClient)()
        thread = threading.Thread(target=sim.connect_and_run, kwargs={'place_at_rahwali': False, 'place_airborne': False}, daemon=True)
        thread.start()
        brunner = (brunner_factory or (lambda: BrunnerInterface(timeout=.02)))()
        if not getattr(brunner, 'sock', None) and not brunner.connect():
            raise RuntimeError('CLS2Sim connection unavailable')
        root = Path(sys._MEIPASS) if getattr(sys, 'frozen', False) else Path(__file__).resolve().parents[1]
        dll = root / 'bridge/bin/qtg_simconnect.dll'
        reader = (reader_factory or (lambda: Measurements(dll, channels=CHANNELS)))()
        if context.get('initial_conditions', {}).get('on_ground', True):
            brake_reader = Measurements(dll)
        events = (events_factory or TrimEvents)(reader)
        controller = AutomaticTrim(context, sim, brunner, publish, reader, brake_reader=brake_reader, events=events)
        controller.emit()
        directory = Path(context['command_directory'])
        stop = Path(context['stop_file'])
        while not controller.closed:
            commands = []
            if directory.exists():
                for path in sorted(directory.glob('*.json')):
                    commands.append(json.loads(path.read_text(encoding='utf-8')))
                    path.unlink()
            if stop.exists():
                stop.unlink()
                commands.insert(0, {'action': 'stop'})
            stopping = next((x for x in commands if x.get('action') in ('stop', 'shutdown', 'emergency_stop')), None)
            for command in ([stopping] if stopping else commands):
                controller.command(command)
            if not controller.closed:
                controller.step()
                sleep(.025)
        return 0
    except Exception as exc:
        import traceback
        traceback.print_exc()
        if controller:
            controller.finish('FAILED', str(exc))
        else:
            publish(dict(context, stage='FAILED', details=str(exc), can_record=False, outputs=[]))
        return 1
    finally:
        if reader: reader.close()
        if brake_reader: brake_reader.close()
        if brunner and getattr(brunner, 'sock', None):
            try: brunner.sock.close()
            except Exception: pass
            brunner.sock = None
        if sim: sim.stop()
        if thread: thread.join(timeout=3)
