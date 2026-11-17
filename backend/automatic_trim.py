"""Automatic Pitch Trim test using Prepar3D trim-increment events."""
import ctypes
import math
from pathlib import Path
import sys
import threading
import time
import json

from backend.automatic_control import AutomaticControl, measured_control_fields, recorded_flight_fields

REF = '2.a.(4)'
METHOD = 'pitch_trim'
UP, DOWN = 910, 911
TRIM_STEP_INTERVAL = 1.0       # 1 second wait between trim steps
HOLD_FINAL_SECONDS = 1.0       # Hold final position for 1 second
START_HOLD_SECONDS = 1.0       # Wait 1 second and confirm starting position
UNRESPONSIVE_LIMIT = 3         # Fail if trim does not respond for 3 consecutive attempts when not at limit
CHANNELS = (('trim_rad', 'ELEVATOR TRIM POSITION', 'radians'),
            ('indicator', 'ELEVATOR TRIM INDICATOR', 'position'),
            ('trim_fraction', 'ELEVATOR TRIM PCT', 'percent over 100'),
            ('elevator_rad', 'ELEVATOR DEFLECTION', 'radians'))


def selected(context):
    return context.get('mode') == 'auto' and context.get('test_reference') == REF and context.get('control_method') == METHOD


class TrimEvents:
    """Send trim button increments on the trim measurement's SimConnect session."""
    def __init__(self, reader):
        self.dll, self.handle = reader.dll, reader.handle
        u, p = ctypes.c_uint32, ctypes.c_void_p
        self.mapping = self.dll.SimConnect_MapClientEventToSimEvent
        self.mapping.argtypes, self.mapping.restype = [p, u, ctypes.c_char_p], ctypes.c_int32
        self.transmit = self.dll.SimConnect_TransmitClientEvent
        self.transmit.argtypes, self.transmit.restype = [p, u, u, u, u, u], ctypes.c_int32
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


class AutomaticTrim(AutomaticControl):
    def __init__(self, context, sim, brunner, publish, reader, *, clock=time.monotonic, brake_reader=None, events=None):
        if not selected(context):
            raise ValueError('Pitch trim Automatic context required')
        synthetic = dict(context, test_reference='2.a.(1)', control_method='static_control')
        super().__init__(synthetic, sim, brunner, publish, clock=clock, live_hardware=False, brake_reader=brake_reader)
        self.context = dict(context)
        self.test_ref = REF
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
        self.init_hold_start = None
        self.verified_nose_up_limit = None
        self.start_trim_pos = None

        # Recording state machine:
        # Step 3: Click Start Recording -> Begin recording actual trim position
        # Step 4: Send one ELEV_TRIM_DN SimConnect event
        # Step 5: Wait 1 second while recording actual trim position
        # Step 6: Repeat one trim step + 1s wait until verified full nose-down limit
        # Step 7: Hold final position for 1 second
        # Step 8: Automatically stop recording and generate reports
        self.record_state = 'IDLE'
        self.step_count = 0
        self.step_sent_time = None
        self.step_start_pos = None
        self.unresponsive_steps = 0
        self.verified_nose_down_limit = None
        self.final_trim_pos = None
        self.travel_start_time = None
        self.travel_end_time = None
        self.total_travel_time = 0.0
        self.average_trim_rate_pct = 0.0
        self.average_trim_rate_pos = 0.0
        self.hold_start_time = None

    def status(self):
        state = super().status()
        state['duration_s'] = 62
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
                state['progress_text'] = 'Click Start Recording to begin pitch trim travel to nose-down limit'
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
                self.sim.unpause_sim()
                now = self.clock()
                self.t_start = now
                self.travel_start_time = now
                self.phase, self.stage = 'RECORD', 'RECORDING'
                self.record_state = 'RECORDING_START'
                self.rows = []
                self.step_count = 0
                self.step_sent_time = None
                self.step_start_pos = None
                self.unresponsive_steps = 0
                self.details = 'Starting pitch trim recording...'
            except Exception as exc:
                self.finish('FAILED', str(exc))
        self.emit()

    def step(self):
        now = self.clock()
        try:
            if not self.sim.is_connected():
                raise RuntimeError('P3D disconnected')

            if self.phase != 'RECORD':
                if self.phase in ('DONE', 'CLOSED'):
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
                        self.init_last_pos = None
                        self.init_stable_count = 0
                        self.emit()
                    return

                # Stage 2: Drive elevator trim to verified full nose-up limit using SimConnect
                if self.init_trim_phase == 'DRIVE_UP':
                    frame = self.reader.poll()
                    if frame:
                        self.sequence_number = frame.get('sequence')
                        self.last_frame_time = now
                        pos = float(frame.get('indicator', frame.get('trim_fraction', 0.0)))

                        if self.init_last_pos is not None:
                            # Check if trim stopped moving up
                            if pos - self.init_last_pos <= 0.0002:
                                self.init_stable_count += 1
                            else:
                                self.init_stable_count = 0
                        self.init_last_pos = pos

                        # Verified nose-up limit reached when position no longer advances upward or hits max limit
                        if self.init_stable_count >= 5 or (self.init_last_pos is not None and pos >= 0.999):
                            self.verified_nose_up_limit = pos
                            self.init_trim_phase = 'WAIT_1S'
                            self.init_hold_start = now
                            self.details = f"Trim at verified nose-up limit ({pos:+.3f}). Confirming starting position..."
                            self.emit()
                            return

                    if self.init_trim_start is not None and now - self.init_trim_start > 15.0:
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
                        self.phase = 'READY'
                        self.stage = 'READY TO RECORD'
                        self.details = f"Verified nose-up limit: {self.start_trim_pos:+.3f}. Ready to record."
                        self.init_trim_phase = 'DONE'
                        self.emit()
                    return

                return

            # Phase == 'RECORD'
            frame = self.reader.poll()
            now = self.clock()
            if frame and frame.get('sequence') != self.sequence_number:
                if not -0.05 <= now - frame['receipt_time'] <= 0.5:
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

                if self.start_trim_pos is None:
                    self.start_trim_pos = pos

                # 4. Send one ELEV_TRIM_DN SimConnect event
                if self.record_state == 'RECORDING_START':
                    self.events.step(DOWN)
                    self.step_count = 1
                    self.step_sent_time = now
                    self.step_start_pos = pos
                    self.record_state = 'WAIT_STEP'
                    self.details = f"Step 1: Trim DOWN sent. Waiting 1.0 s... (Trim: {pos:+.3f})"
                    return

                # 5 & 6. Wait 1 second while recording actual trim position; repeat until verified full nose-down limit
                elif self.record_state == 'WAIT_STEP':
                    step_elapsed = now - self.step_sent_time
                    if step_elapsed < TRIM_STEP_INTERVAL:
                        self.details = f"Step {self.step_count}: Recording actual trim ({step_elapsed:.1f}/1.0 s) | Pos: {pos:+.3f}"
                        return

                    # 1 second wait complete: verify actual trim position after step
                    delta_down = self.step_start_pos - pos

                    if delta_down > 0.0005:
                        # Trim moved down as commanded!
                        self.unresponsive_steps = 0
                        # Send next ELEV_TRIM_DN event
                        self.events.step(DOWN)
                        self.step_count += 1
                        self.step_sent_time = now
                        self.step_start_pos = pos
                        self.details = f"Step {self.step_count}: Trim DOWN sent. Waiting 1.0 s... (Trim: {pos:+.3f})"
                        return
                    else:
                        # Trim did not move further down.
                        # Detect if verified full nose-down limit has been reached:
                        total_down = self.start_trim_pos - pos
                        if total_down >= 0.5 or pos < 0.0:
                            # Verified full nose-down limit reached!
                            self.verified_nose_down_limit = pos
                            self.final_trim_pos = pos
                            self.travel_end_time = now
                            self.total_travel_time = max(0.1, self.travel_end_time - self.travel_start_time)
                            self.total_travel_amount = abs(self.start_trim_pos - self.final_trim_pos)
                            self.average_trim_rate_pos = self.total_travel_amount / self.total_travel_time
                            self.average_trim_rate_pct = (abs(self.rows[0]['Elevator Trim (%)'] - trim_pct)) / self.total_travel_time

                            # 7. Hold the final position for 1 second
                            self.record_state = 'HOLD_FINAL'
                            self.hold_start_time = now
                            self.details = f"Nose-down limit reached ({pos:+.3f}). Holding final position for 1.0 s..."
                            return
                        else:
                            # Trim did not respond and is not at nose-down limit
                            self.unresponsive_steps += 1
                            if self.unresponsive_steps >= UNRESPONSIVE_LIMIT:
                                raise RuntimeError(f"P3D elevator trim stopped responding at position {pos:+.3f} before reaching nose-down limit")
                            # Retry sending one trim down event
                            self.events.step(DOWN)
                            self.step_sent_time = now
                            self.step_start_pos = pos
                            self.details = f"Trim unresponsive ({self.unresponsive_steps}/{UNRESPONSIVE_LIMIT}), retrying DOWN..."
                            return

                # 7. Hold final position for 1 second
                elif self.record_state == 'HOLD_FINAL':
                    hold_elapsed = now - self.hold_start_time
                    if hold_elapsed < HOLD_FINAL_SECONDS:
                        self.details = f"Holding final position ({hold_elapsed:.1f}/1.0 s) | Pos: {pos:+.3f}"
                        return

                    # 8. Automatically stop recording and generate CSV/PDF graphs
                    msg = (f"Pitch trim nose-down limit reached ({self.verified_nose_down_limit:+.3f}). "
                           f"Travel time: {self.total_travel_time:.1f} s, Avg rate: {self.average_trim_rate_pct:.2f} %/s")
                    self.finish('COMPLETE', msg)
                    return

            if self.last_frame_time is None:
                if now - self.t_start > 1.0:
                    raise RuntimeError('No fresh trim telemetry')
                # If no frame yet right after record start, send initial step if not sent
                if self.record_state == 'RECORDING_START':
                    self.events.step(DOWN)
                    self.step_count = 1
                    self.step_sent_time = now
                    self.record_state = 'WAIT_STEP'
                return

            if now - self.last_frame_time > 0.5:
                raise RuntimeError('Trim telemetry stopped advancing')

            if now - self.t_start > 120.0:
                raise RuntimeError('Pitch trim recording timed out after 120 s')

        except Exception as exc:
            self.finish('FAILED', str(exc))
        finally:
            self._maybe_emit(now)

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
