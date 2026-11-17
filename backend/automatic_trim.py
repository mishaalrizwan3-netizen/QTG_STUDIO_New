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
TRIM_STEP_SECONDS = 0.1
MOVEMENT_TIMEOUT_SECONDS = 3.0
TRIM_TARGETS = (1.0, -1.0, 1.0, -1.0)
ENDPOINT_TOLERANCE = 0.001
MIN_RECORD_SECONDS = 3.0
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
        self.next_action = None
        self.target_index = 0
        self.sweep_complete = False
        self.leg_pressed = False
        self.leg_started = None
        self.leg_timeout = 30.0
        self.trim_baseline = None
        self.trim_moved = False
        self.first_action_time = None

    def status(self):
        state = super().status()
        state['duration_s'] = int(self.leg_timeout * len(TRIM_TARGETS))
        state['engine_text'] = 'Engine: Pitch Trim Test'
        if self.stage == 'READY TO RECORD':
            state['details'] = 'Ready to sweep trim +1 to -1 twice'
            state['recording_text'] = 'READY TO RECORD [P3D pitch trim]'
            state['progress_text'] = 'Click Start Recording to sweep +1, -1, +1, -1'
        state['controls_text'] = 'P3D pitch trim events ready'
        return state

    def _release(self):
        # Increment events have no held state to release.
        return None

    def _advance_trim_button(self, now):
        """Issue the same incremental trim event as a pilot button press."""
        if self.next_action is not None and now < self.next_action:
            return
        self.events.step(UP if TRIM_TARGETS[self.target_index] > 0 else DOWN)
        self.leg_pressed = True
        if self.first_action_time is None:
            self.first_action_time = now
        self.next_action = now + TRIM_STEP_SECONDS

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
                self.t_start = self.clock()
                self.leg_started = self.t_start
                self.next_action = self.t_start + .1
                self.phase, self.stage = 'RECORD', 'RECORDING'
                self.details = 'Waiting for fresh trim and elevator telemetry'
            except Exception as exc:
                self.finish('FAILED', str(exc))
        self.emit()

    def step(self):
        if self.phase != 'RECORD':
            return super().step()
        now = self.clock()
        try:
            if not self.sim.is_connected():
                raise RuntimeError('P3D disconnected')
            frame = self.reader.poll()
            now = self.clock()
            if frame and frame.get('sequence') != self.sequence_number:
                if not 0 <= now - frame['receipt_time'] <= .5:
                    raise RuntimeError('Trim telemetry is stale')
                if not all(math.isfinite(float(frame[k])) for k in ('trim_rad', 'indicator', 'trim_fraction', 'elevator_rad')):
                    raise RuntimeError('Trim or elevator telemetry unavailable')
                self.sequence_number = frame['sequence']
                self.last_frame_time = now
                self.latest_sim = self.sim.get_telemetry() or {}
                angle = frame['elevator_rad']
                if self.trim_baseline is None:
                    self.trim_baseline = frame['trim_fraction']
                elif abs(frame['trim_fraction'] - self.trim_baseline) >= 0.0001:
                    self.trim_moved = True
                row = {'Time (s)': now - self.t_start,
                    'Elevator Trim Angle (rad)': frame['trim_rad'],
                    'Elevator Trim Indicator (position)': frame['indicator'],
                    'Elevator Trim (%)': frame['trim_fraction'] * 100.,
                    'Elevator Surface (rad)': angle,
                    **recorded_flight_fields(self.latest_sim)}
                row.update(measured_control_fields(self.brunner))
                self.rows.append(row)
                target = TRIM_TARGETS[self.target_index]
                reached = (not self.sweep_complete and self.leg_pressed
                           and target * frame['indicator'] >= 1.0 - ENDPOINT_TOLERANCE)
                if reached:
                    if self.target_index == len(TRIM_TARGETS) - 1:
                        self.sweep_complete = True
                    else:
                        self.target_index += 1
                        self.leg_pressed = False
                        self.leg_started = now
                        self.next_action = now + .1
                        self.trim_baseline = frame['trim_fraction']
                        self.trim_moved = False
                        self.first_action_time = None
                if self.sweep_complete and now - self.t_start >= MIN_RECORD_SECONDS:
                    self.finish('COMPLETE', 'Pitch trim reached +1 and -1 twice')
                    return
            if self.last_frame_time is None:
                if now - self.t_start > 1:
                    raise RuntimeError('No fresh trim telemetry')
                self._advance_trim_button(now)
                return
            if now - self.last_frame_time > .5:
                raise RuntimeError('Trim telemetry stopped advancing')
            if self.sweep_complete:
                self.details = f'Trim sweeps complete; recording until {MIN_RECORD_SECONDS:g} seconds'
                return
            if now - self.leg_started >= self.leg_timeout:
                raise RuntimeError(f'Trim indicator did not reach {TRIM_TARGETS[self.target_index]:+g} before sweep timeout')
            if (self.first_action_time is not None and not self.trim_moved
                    and now - self.first_action_time >= MOVEMENT_TIMEOUT_SECONDS):
                raise RuntimeError('P3D elevator trim did not move after trim-up/down events; check simulator pause state and trim assignments')
            self._advance_trim_button(now)
            self.details = f'Sweeping trim toward {TRIM_TARGETS[self.target_index]:+g} ({self.target_index + 1}/4)'
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
