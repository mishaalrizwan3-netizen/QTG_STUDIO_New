"""Dedicated Automatic Flap Change Dynamics worker for 2.c.(2a)/(2b)."""
import csv
from datetime import datetime
import importlib.util
import json
from pathlib import Path
import sys
import threading
import time

from backend.automatic_power_change import AutomaticPowerDynamics, finite
from backend.verifier import validate_conditions
from backend.vvi_pid import ForceAltitudeHold
from interfaces.engine_measurements import BODY_ACCELERATION_CHANNELS

RETRACTION_REFERENCE = '2.c.(2a)'
EXTENSION_REFERENCE = '2.c.(2b)'
RETRACTION_FORCE_REFERENCE = '2.c.(2a)-force'
EXTENSION_FORCE_REFERENCE = '2.c.(2b)-force'
METHOD = 'flap_dynamics'
POSITIONS = {
    RETRACTION_REFERENCE: (1.0, 0.0),
    RETRACTION_FORCE_REFERENCE: (1.0, 0.0),
    EXTENSION_REFERENCE: (0.0, 1.0),
    EXTENSION_FORCE_REFERENCE: (1.0, 3.0),
}
BASELINE_S = 5.0
POST_FLAP_S = 15.0
FLAP_CONFIRM_TIMEOUT_S = 8.0
MAX_RECORDING_S = 32.0
FLAP_TOLERANCE = 0.1
INITIAL_ANGLE_SETTLE_S = 1.0
INITIAL_ANGLE_CHANGE_TOLERANCE_RAD = 0.001
PITCH_SETTLE_S = 2.0
PITCH_CHANGE_TOLERANCE_DEG = 0.25


def selected(context):
    return (context.get('mode') == 'auto' and context.get('control_method') == METHOD
            and context.get('test_reference') in POSITIONS)


class AutomaticFlapDynamics(AutomaticPowerDynamics):
    """Reuse the proven aircraft initialization, with a flap-only recording step."""

    def __init__(self, context, sim, publish, *, clock=time.monotonic, vvi_controller=None, transport_factory=None):
        if not selected(context):
            raise ValueError('Dedicated Automatic Flap Change Dynamics session required')
        self.context = dict(context)
        self.is_force_test = self.context['test_reference'] in (
            RETRACTION_FORCE_REFERENCE,
            EXTENSION_FORCE_REFERENCE,
        )
        self.cfg = validate_conditions(context['initial_conditions'])
        if not self.cfg or self.cfg['on_ground'] or self.cfg['ias_kt'] < 40.0:
            raise ValueError('Flap Change Dynamics requires an airborne condition')
        self.initial_flap, self.target = POSITIONS[context['test_reference']]
        if self.cfg['flaps'] != self.initial_flap:
            raise ValueError(f'Flap Change Dynamics must start at flap Index {self.initial_flap:g}')
        if any((context.get('active_pids') or {}).values()):
            raise ValueError('Automatic Flap Change Dynamics commands flaps only; disable PID assistance')
        self.sim, self.publish, self.clock = sim, publish, clock
        self.phase, self.stage = 'CONNECT', 'Connecting'
        self.details = 'Connecting to P3D for the flap-only automatic test'
        self.deadline = clock() + 90.0
        self.latest = self.last_sequence = None
        self.last_sim_time = self.last_sim_advance = None
        self.initialization_running = False
        self.initial_throttle_corrected = False
        self.rows, self.outputs = [], []
        self.start_time = None
        self.flap_command_time = self.flap_onset_time = self.flap_completed_time = None
        # Retained for the inherited recording helper's command marker.
        self.power_command_time = None
        self.result, self.closed = None, False
        self.last_sample_time = None
        self.baseline_anchor = None
        self.recording_settle_started = None
        self.initial_angle_stable_since = None
        self.previous_initial_angles = None
        self.pitch_stable_since = None
        self.previous_pitch = None
        self._control_sample = {}
        self._next_control_sample = 0.0
        self.vvi_hold = (
            ForceAltitudeHold(
                self.cfg['alt_ft'],
                controller=vvi_controller,
                transport_factory=transport_factory,
            )
            if self.is_force_test else None
        )
        from backend.pid_assist import PidAssist
        self.pid_assist = PidAssist(self.context, owned_axis_mask=0)
        self.brunner = self.context.pop('brunner', None)

    def status(self):
        frame = self.latest or {}
        elapsed = max(0.0, self.clock() - self.start_time) if self.start_time is not None else 0.0
        can_record = self.phase == 'READY'
        return dict(self.context, stage=self.stage, details=self.details, updated=time.time(),
                    connected=bool(self.sim.is_connected()),
                    brunner_connected=bool(self.brunner and getattr(self.brunner, 'sock', None)),
                    can_record=can_record, controlling=self.phase in ('PREPARE', 'RECORD'),
                    recording=self.phase == 'RECORD' and self.start_time is not None, samples=len(self.rows),
                    elapsed=elapsed, duration_s=BASELINE_S + POST_FLAP_S,
                    outputs=list(self.outputs), data_directory=self.context['result_directory'],
                    engine_text=(f"Engine RPM: {finite(frame, 'rpm') or 0:.0f}" if frame else 'Engine RPM: N/A'),
                    throttle_text=(
                        f"Flaps: Index {finite(frame, 'flaps') or 0:g}"
                        + (f" ({finite(frame, 'flaps_handle_percent'):.0f}%)"
                           if finite(frame, 'flaps_handle_percent') is not None else '')
                        if frame else 'Flaps: N/A'
                    ),
                    flight_text=(f"Airspeed / Altitude: {frame.get('ias_kt', 0):.1f} kt / {frame.get('alt_ft', 0):.0f} ft" if frame else 'Airspeed / Altitude: N/A'),
                    attitude_text=(f"Pitch: {frame.get('pitch_deg', 0):+.1f} deg | Bank: {frame.get('bank_deg', 0):+.1f} deg" if frame else 'Pitch / Bank: N/A'),
                    controls_text=(
                        f'P3D flap step ({self.initial_flap:g} to {self.target:g}) + Pitch VVI PID altitude hold'
                        if self.is_force_test else f'P3D flap handle only: Index {self.initial_flap:g} to Index {self.target:g}'
                    ),
                    recording_text=('READY TO RECORD (00:00)' if can_record else
                                    f'RECORDING ({int(elapsed//60):02}:{int(elapsed%60):02}) - {self.stage}' if self.phase == 'RECORD' else self.stage),
                    progress_text=f'{len(self.rows)} recorded frames | {self.details}',
                    telemetry=dict(ias_kt=frame.get('ias_kt'), alt_ft=frame.get('alt_ft'),
                                   vertical_speed=frame.get('vertical_speed'),
                                   pitch_deg=frame.get('pitch_deg'), bank_deg=frame.get('bank_deg'),
                                   flaps_handle_index=finite(frame, 'flaps'),
                                   flaps_handle_percent=finite(frame, 'flaps_handle_percent'),
                                   trailing_edge_flaps_left_angle=finite(frame, 'flaps_left_angle'),
                                   trailing_edge_flaps_right_angle=finite(frame, 'flaps_right_angle')))

    def command(self, command):
        action = command['action']
        if action in ('stop', 'shutdown', 'invalidate_initialization'):
            if self.phase != 'DONE':
                self.finish('ABORTED', 'Operator stopped Flap Change Dynamics')
            if action == 'shutdown':
                self.closed = True
        elif action == 'record' and self.phase == 'READY':
            if not self.latest or not self.sim.is_connected() or self.clock() - self.latest['receipt_time'] > 1.0:
                self.finish('FAILED', 'Fresh simulator measurements are required to start recording')
            else:
                if self.vvi_hold is not None:
                    self.vvi_hold.engage(
                        self.brunner,
                        self.clock(),
                        alt_ft=self.cfg['alt_ft'],
                        pitch_deg=float(self.latest.get('pitch_deg', self.cfg['pitch_deg'])),
                    )
                self.sim.unpause_sim()
                self.recording_settle_started = self.clock()
                self.start_time = None
                self.initial_angle_stable_since = None
                self.previous_initial_angles = None
                self.pitch_stable_since = None
                self.previous_pitch = None
                self.last_sim_advance = self.recording_settle_started
                self.last_sim_time = self.latest.get('sim_time')
                self._last_unpause_retry = self.recording_settle_started
                self.phase, self.stage = 'RECORD', 'SETTLING INITIAL FLAP POSITION'
                self.details = f'Waiting for both flap angles to settle at initial Index {self.initial_flap:g} before recording'
        self.emit()

    def _record_frame(self, frame, now):
        super()._record_frame(frame, now)
        row = self.rows[-1]
        row['Flap Step Command (0/1)'] = int(self.flap_command_time is not None)
        row.pop('Power Step Command (0/1)', None)

    def step(self, frame):
        if self.phase != 'RECORD':
            previous = self.phase
            super().step(frame)
            if self.phase == 'READY' and previous != 'READY':
                self.details = f'Initial flap Index {self.initial_flap:g} verified. Ready for 5 s baseline, then automatic step to Index {self.target:g}.'
                self.emit()
            return
        if self.phase == 'DONE':
            return
        now = self.clock()
        try:
            if frame is None or not (-0.2 <= now - frame.get('receipt_time', -1.e6) <= 1.0):
                if not self.sim.is_connected():
                    raise RuntimeError('P3D disconnected')
                if self.last_sample_time is not None and now - self.last_sample_time > 1.5:
                    raise RuntimeError('P3D measurement stream stopped')
                return
            self._ensure_live(frame, now)
            if frame['sequence'] == self.last_sequence:
                if self.last_sample_time is not None and now - self.last_sample_time > 1.5:
                    raise RuntimeError('P3D measurement sequence stalled during recording')
                return
            if self.last_sequence is not None and frame['sequence'] < self.last_sequence:
                raise RuntimeError('P3D measurement sequence went backwards')
            self.last_sequence, self.latest, self.last_sample_time = frame['sequence'], frame, now
            if frame.get('autopilot_active') == 1:
                raise RuntimeError('Autopilot must be OFF for an uncontrolled free-response test')
            if frame.get('on_ground') == 1:
                raise RuntimeError('Aircraft is on the ground; airborne flap-response test cannot continue')
            agl = finite(frame, 'height_agl_ft')
            if agl is not None and agl < 300.0:
                raise RuntimeError('Aircraft too low to safely continue; recording stopped')
            if self.last_sim_time is not None and frame['sim_time'] > self.last_sim_time:
                self.last_sim_advance, self.last_sim_time = now, frame['sim_time']
            elif self.last_sim_advance is not None and now - self.last_sim_advance > 8.0:
                raise RuntimeError('P3D is paused or simulation time stopped advancing')
            measured_flap = finite(frame, 'flaps')
            if measured_flap is None:
                raise RuntimeError('Measured flap handle index is unavailable')
            if self.start_time is None:
                if (self.recording_settle_started is not None
                        and now - self.recording_settle_started > FLAP_CONFIRM_TIMEOUT_S):
                    raise RuntimeError(
                        f'Measured flap angles did not settle at initial Index {self.initial_flap:g}'
                    )
                left = finite(frame, 'flaps_left_angle')
                right = finite(frame, 'flaps_right_angle')
                if left is None or right is None:
                    raise RuntimeError('Both measured trailing-edge flap angles are required before recording')
                if abs(measured_flap - self.initial_flap) > FLAP_TOLERANCE:
                    self.initial_angle_stable_since = None
                    self.details = f'Waiting for measured flap handle Index {self.initial_flap:g}'
                    return
                previous = self.previous_initial_angles
                self.previous_initial_angles = (left, right)
                moving = (previous is None or abs(left - previous[0]) > INITIAL_ANGLE_CHANGE_TOLERANCE_RAD
                          or abs(right - previous[1]) > INITIAL_ANGLE_CHANGE_TOLERANCE_RAD)
                if moving:
                    self.initial_angle_stable_since = None
                    self.details = (f'Waiting for initial flap angles to stop moving: '
                                    f'left {left:.4f} rad | right {right:.4f} rad')
                    return
                if self.initial_angle_stable_since is None:
                    self.initial_angle_stable_since = now
                    return
                settled = now - self.initial_angle_stable_since
                if settled < INITIAL_ANGLE_SETTLE_S:
                    self.details = f'Initial flap angles stable: {settled:.1f}/{INITIAL_ANGLE_SETTLE_S:.1f} s'
                    return
                self.start_time = now
                self.rows = []
                self.stage = 'RECORDING BASELINE'
                self.details = (
                    f'Holding settled flap Index {self.initial_flap:g} for the measured 5 second baseline; VVI PID holding altitude'
                    if self.vvi_hold is not None else
                    f'Holding settled flap Index {self.initial_flap:g} for the measured 5 second baseline'
                )
                return
            if now - self.start_time > MAX_RECORDING_S:
                raise RuntimeError('Flap Change Dynamics exceeded its bounded recording window')
            if self.flap_command_time is None and abs(measured_flap - self.initial_flap) > FLAP_TOLERANCE:
                raise RuntimeError(f'Flaps did not remain at Index {self.initial_flap:g} during the 5 second baseline')
            if self.flap_completed_time is not None and abs(measured_flap - self.target) > FLAP_TOLERANCE:
                raise RuntimeError(f'Flaps did not remain at final Index {self.target:g} during the 15 second response')
            if self.vvi_hold is not None:
                self.vvi_hold.step(frame, now)
            self._record_frame(frame, now)
            elapsed = now - self.start_time
            if self.flap_command_time is None:
                if (elapsed >= BASELINE_S and len(self.rows) >= 2 and
                        self.rows[-1]['Time (s)'] - self.rows[0]['Time (s)'] >= BASELINE_S):
                    pitch = finite(frame, 'pitch_deg')
                    if pitch is None:
                        raise RuntimeError('Measured pitch angle is unavailable before flap change')
                    if (self.previous_pitch is None or
                            abs(pitch - self.previous_pitch) > PITCH_CHANGE_TOLERANCE_DEG):
                        self.pitch_stable_since = None
                    elif self.pitch_stable_since is None:
                        self.pitch_stable_since = now
                    self.previous_pitch = pitch
                    if (self.pitch_stable_since is None or
                            now - self.pitch_stable_since < PITCH_SETTLE_S):
                        self.details = f'Waiting for pitch to settle before flap change: {pitch:+.2f} deg'
                        return
                    if not self.sim.set_flaps_handle_index(self.target):
                        raise RuntimeError('P3D flap-change command rejected')
                    self.flap_command_time = self.power_command_time = now
                    self.stage = 'FLAP CHANGE'
                    self.details = f'Automatic flap command: Index {self.initial_flap:g} to Index {self.target:g}'
            elif self.flap_completed_time is None:
                if self.flap_onset_time is None and abs(measured_flap - self.initial_flap) > FLAP_TOLERANCE:
                    self.flap_onset_time = now
                self.stage = 'VERIFYING FLAP CHANGE'
                if abs(measured_flap - self.target) <= FLAP_TOLERANCE:
                    self.flap_completed_time = now
                    self.stage = 'FREE RESPONSE'
                    self.details = f'Recording 15 seconds after measured flap Index {self.target:g} was reached'
                elif now - self.flap_command_time > FLAP_CONFIRM_TIMEOUT_S:
                    raise RuntimeError(f'Measured flaps did not reach final Index {self.target:g}')
            elif now - self.flap_completed_time >= POST_FLAP_S:
                self.finish('COMPLETE', f'Flap response Index {self.initial_flap:g} to Index {self.target:g} captured; reference comparison not performed')
        except Exception as exc:
            self.finish('FAILED', str(exc))
        finally:
            self.emit()

    def finish(self, result, reason):
        if self.phase == 'DONE':
            return
        self.phase, self.result, self.stage, self.details = 'DONE', result, result, reason
        try:
            if self.sim.is_connected():
                self.sim.pause_sim()
        except Exception as exc:
            self.details += f'; pause failed: {exc}'
        if getattr(self, 'vvi_hold', None) is not None:
            try:
                self.vvi_hold.release()
            except Exception:
                pass
        if self.rows:
            self.stage = 'Generating reports'
            self.emit()
            try:
                self.outputs = save_report(self.context, self.rows, result, reason,
                                           flap_command_time=self.flap_command_time,
                                           flap_onset_time=self.flap_onset_time,
                                           flap_completed_time=self.flap_completed_time,
                                           recording_start=self.start_time)
                self.stage = 'Completed' if result == 'COMPLETE' else result
            except Exception as exc:
                self.outputs = list(getattr(exc, 'outputs', []))
                self.stage = 'FAILED'
                self.details += '; reporting error: ' + str(exc)
        self.emit()


def save_report(context, rows, result, reason, *, flap_command_time, flap_onset_time,
                flap_completed_time, recording_start):
    from reports.power_dynamics import DEFAULT_GRAPHS, graph_availability
    folder = Path(context['result_directory'])
    folder.mkdir(parents=True, exist_ok=True)
    reference = context['test_reference']
    direction = 'retraction' if reference in (RETRACTION_REFERENCE, RETRACTION_FORCE_REFERENCE) else 'extension'
    force_test = reference in (RETRACTION_FORCE_REFERENCE, EXTENSION_FORCE_REFERENCE)
    report_kind = 'flap_change_force' if force_test else 'flap_change'
    stem = folder / (f'2_c_2_{report_kind}_{direction}_auto_' + datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
                     + '_' + str(time.time_ns()))
    csv_path = stem.with_suffix('.csv')
    meta_path = stem.with_suffix('.qtg_test.json')
    pdf_path = stem.with_name(stem.name + '_graphs.pdf')
    outputs = []
    try:
        with csv_path.open('x', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader(); writer.writerows(rows)
        outputs.append(str(csv_path))
        relative = lambda stamp: stamp - recording_start if stamp is not None else None
        selected_graphs = context.get('selected_graphs')
        default_graphs = (
            list(DEFAULT_GRAPHS) + ['flaps_handle_index', 'flaps_handle_percent', 'flaps_left_angle', 'flaps_right_angle',
                                    'pitch_position', 'pitch_force', 'pitch_virtual_force']
            if force_test else list(DEFAULT_GRAPHS) + ['flaps_handle_index', 'flaps_handle_percent',
                                                       'flaps_left_angle', 'flaps_right_angle']
        )
        requested = default_graphs if selected_graphs is None else list(selected_graphs)
        graphed, unavailable = graph_availability(rows, requested)
        initial_flap, final_flap = POSITIONS[context['test_reference']]
        metadata = dict(context, mode='auto', control_method=METHOD,
                        kind='Flap Change Force' if force_test else 'Flap Change Dynamics',
                        flap_direction=direction, initial_flap_index=initial_flap,
                        final_flap_index=final_flap, result=result, reason=reason,
                        status=result.lower(), event_time_s=relative(flap_onset_time),
                        flap_command_time_s=relative(flap_command_time),
                        flap_measured_onset_s=relative(flap_onset_time),
                        flap_completed_time_s=relative(flap_completed_time),
                        baseline_required_s=BASELINE_S, post_flap_required_s=POST_FLAP_S,
                        initial_recorded_at_start=True, graphed_channels=graphed,
                        unavailable_selected_channels=unavailable,
                        aircraft_conformity='NOT EVALUATED - approved aircraft reference data not supplied',
                        measurement_source=('Correlated P3D telemetry; automatic P3D flap step + Pitch VVI PID altitude hold via CLS2Sim 0x01'
                                            if force_test else
                                            'Correlated P3D telemetry; automatic P3D flap handle command only'))
        meta_path.write_text(json.dumps(metadata, indent=2, allow_nan=False), encoding='utf-8')
        outputs.append(str(meta_path))
        if flap_onset_time is None:
            return outputs
        from reports.plot_graphs import _render_report
        _render_report(sim_path=csv_path, ref_path=None, out_path=pdf_path,
                       titt_results=None, metadata=metadata, selected_graphs=requested)
        outputs.append(str(pdf_path))
        return outputs
    except Exception as exc:
        exc.outputs = outputs
        raise


def run(context, publish, *, factory=None, reader_factory=None, sleep=time.sleep):
    sim = reader = thread = controller = control_reader = None
    try:
        if factory is None:
            root = Path(sys._MEIPASS) if getattr(sys, 'frozen', False) else Path(__file__).resolve().parents[1]
            source = root / 'interfaces/brunner_base/src/p3d_simconnect_v15.py'
            spec = importlib.util.spec_from_file_location('qtg_auto_flap_p3d', source)
            module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
            factory = module.SimConnectClient
        sim = factory()
        controller = AutomaticFlapDynamics(context, sim, publish)
        from interfaces.brunner_base.src.brunner_interface import BrunnerInterface
        control_reader = BrunnerInterface(timeout=0.05)
        if control_reader.connect():
            controller.brunner = control_reader
        controller.emit()
        thread = threading.Thread(target=sim.connect_and_run,
                                  kwargs=dict(place_at_rahwali=False, place_airborne=False), daemon=True)
        thread.start()
        if reader_factory is None:
            from interfaces.engine_measurements import Measurements, CHANNELS
            root = Path(sys._MEIPASS) if getattr(sys, 'frozen', False) else Path(__file__).resolve().parents[1]
            dll = root / 'bridge/bin/qtg_simconnect.dll'
            extra = (('autopilot_active', 'AUTOPILOT MASTER', 'bool'),)
            reader_factory = lambda: Measurements(dll, channels=CHANNELS + extra + BODY_ACCELERATION_CHANNELS)
        reader = reader_factory()
        directory = Path(context['command_directory'])
        while not controller.closed:
            commands = []
            for path in sorted(directory.glob('*.json')):
                commands.append(json.loads(path.read_text(encoding='utf-8'))); path.unlink()
            stop = Path(context['stop_file'])
            if stop.exists():
                stop.unlink(); commands.insert(0, dict(action='stop'))
            stopping = next((c for c in commands if c['action'] == 'shutdown'), None) or next(
                (c for c in commands if c['action'] in ('stop', 'invalidate_initialization')), None)
            if stopping:
                controller.command(stopping)
            else:
                for command in commands:
                    controller.command(command)
            if not controller.closed and controller.phase != 'DONE':
                before = controller.phase; controller.step(reader.poll())
                if before == 'CONNECT' and controller.phase == 'PREPARE' and hasattr(reader, 'discard_pending'):
                    reader.discard_pending()
                sleep(.04)
            elif not controller.closed:
                sleep(.04)
        return 0
    except Exception as exc:
        if controller:
            controller.finish('FAILED', str(exc))
        else:
            publish(dict(context, stage='FAILED', details=str(exc), can_record=False, outputs=[]))
        return 1
    finally:
        if control_reader: control_reader.close()
        if reader: reader.close()
        if sim: sim.stop()
        if thread: thread.join(timeout=5)
