"""Automatic longitudinal trim for the independent Cruise/Approach tests."""
import csv
from datetime import datetime
import importlib.util
import json
import math
from pathlib import Path
import sys
import threading
import time

from backend.automatic_power_change import AutomaticPowerDynamics, finite
from backend.automatic_trim import TrimEvents, UP, DOWN
from backend.vvi_pid import ForceAltitudeHold
from backend.pid_assist import PidAssist
from backend.verifier import validate_conditions
from interfaces.engine_measurements import BODY_ACCELERATION_CHANNELS

REFERENCES = ('2.c.(5a)', '2.c.(5b)')
METHOD = 'longitudinal_trim'
FORCE_TOLERANCE_N = 2.0
IAS_TOLERANCE_KT = 1.5
ALTITUDE_TOLERANCE_FT = 30.0
VVI_TOLERANCE_FPM = 100.0
STABLE_SECONDS = 5.0
TRIM_PULSE_INTERVAL_S = 0.5
TRIM_RESPONSE_TIMEOUT_S = 45.0
THROTTLE_COMMAND_INTERVAL_S = 0.2


def selected(context):
    return (context.get('mode') == 'auto' and context.get('control_method') == METHOD
            and context.get('test_reference') in REFERENCES)


class ThrottleIasHold:
    """Small bounded PI loop that owns P3D throttle only."""
    def __init__(self, target_ias, initial_throttle):
        self.target = float(target_ias)
        self.command = float(initial_throttle)
        self.baseline = self.command
        self.integral = 0.0
        self.last_time = None
        self.last_sent = None

    def update(self, ias, now):
        dt = 0.04 if self.last_time is None else max(0.01, min(0.5, now - self.last_time))
        self.last_time = now
        error = self.target - float(ias)
        trial = max(-20.0, min(20.0, self.integral + 0.10 * error * dt))
        raw = self.baseline + 1.2 * error + trial
        desired = max(0.0, min(100.0, raw))
        if not ((desired >= 100.0 and error > 0) or (desired <= 0.0 and error < 0)):
            self.integral = trial
        max_step = 8.0 * dt
        self.command += max(-max_step, min(max_step, desired - self.command))
        return max(0.0, min(100.0, self.command))


class AutomaticLongitudinalTrim(AutomaticPowerDynamics):
    def __init__(self, context, sim, publish, *, clock=time.monotonic,
                 vvi_controller=None, transport_factory=None, events=None):
        if not selected(context):
            raise ValueError('Automatic longitudinal trim context required')
        cfg = validate_conditions(context['initial_conditions'])
        if not cfg or cfg['on_ground']:
            raise ValueError('Longitudinal trim requires an airborne condition')
        synthetic_cfg = dict(cfg, throttle=min(90.0, cfg['throttle']))
        synthetic = dict(context, test_reference='2.c.(1)', control_method='power_dynamics',
                         initial_conditions=synthetic_cfg, active_pids={})
        super().__init__(synthetic, sim, publish, clock=clock)
        self.context = dict(context)
        self.context.pop('brunner', None)
        selected_pids = self.context.get('active_pids') or {}
        if selected_pids.get('airspeed'):
            raise ValueError('Longitudinal Trim owns pitch for VVI altitude hold; disable Airspeed pitch PID')
        self.pid_assist = PidAssist(self.context, owned_axis_mask=0x01)
        # In this wings-level trim test the selected Bank target owns roll.
        # Yaw damps sideslip on the rudder; heading capture must not replace
        # the operator's bank target with a turn demand.
        if selected_pids.get('bank'):
            self.pid_assist.heading_ctrl = None
        self.cfg = cfg
        self.vvi_hold = ForceAltitudeHold(cfg['alt_ft'], controller=vvi_controller,
                                          transport_factory=transport_factory)
        self.ias_hold = ThrottleIasHold(cfg['ias_kt'], cfg['throttle'])
        self.events = events
        self.trim_started = None
        self.last_trim_pulse = None
        self.trim_direction = None
        self.force_before_pulse = None
        self.stable_since = None
        self.force_n = float('nan')
        self.trim_fraction = float('nan')
        self.recording_started = None
        self._last_throttle_send = None

    def status(self):
        state = super().status()
        state.update(self.context)
        state['can_record'] = False
        state['controlling'] = self.phase in ('PREPARE', 'TRIM_STABILIZE', 'RECORD')
        state['recording'] = self.phase == 'RECORD'
        state['controls_text'] = 'VVI altitude hold + throttle IAS hold + automatic elevator trim' + (' + selected Bank/Yaw PIDs' if self.pid_assist.has_active_pids else '')
        state['recording_text'] = ('RECORDING' if self.phase == 'RECORD' else self.stage)
        state['progress_text'] = self.details
        state['telemetry'] = dict(state.get('telemetry') or {},
                                  pitch_force_n=(self.force_n if math.isfinite(self.force_n) else None),
                                  elevator_trim_pct=(self.trim_fraction * 100.0
                                                     if math.isfinite(self.trim_fraction) else None))
        return state

    def command(self, command):
        action = command.get('action')
        if action in ('stop', 'shutdown', 'invalidate_initialization'):
            self.finish('ABORTED', 'Operator stopped Longitudinal Trim')
            if action == 'shutdown':
                self.closed = True
            return self.emit()
        # Stabilization and recording start automatically after READY.
        if action == 'record':
            return self.emit()

    def _begin_trim(self, now):
        if self.events is None:
            raise RuntimeError('Pitch-trim event interface is unavailable')
        if not self.brunner or not getattr(self.brunner, 'sock', None):
            raise RuntimeError('Longitudinal Trim requires a BRUNNER pitch-force connection')
        self.vvi_hold.engage(self.brunner, now, alt_ft=self.cfg['alt_ft'],
                             pitch_deg=float(self.latest.get('pitch_deg', self.cfg['pitch_deg'])))
        self.pid_assist.reset()
        if self.pid_assist.has_active_pids:
            self.pid_assist.begin_stabilization(now, self.brunner)
        self.sim.unpause_sim()
        self.phase, self.stage = 'TRIM_STABILIZE', 'AUTOMATIC TRIM STABILIZATION'
        self.trim_started = now
        self.last_sim_time = self.latest.get('sim_time')
        self.last_sim_advance = now
        self.details = 'Holding altitude and IAS; unloading sustained pitch force with elevator trim'

    def _read_pitch(self):
        old = getattr(self.brunner, 'strict_io', False)
        try:
            self.brunner.strict_io = True
            reader = getattr(type(self.brunner), 'read_axis_measurement', None)
            if callable(reader):
                _position, force = reader(self.brunner, 0x01)
            else:
                _position, force, _virtual = self.brunner.read_axis(0x01)
            force = float(force)
            if math.isfinite(force):
                return force * 100.0 if abs(force) <= 5.0 else force
        finally:
            self.brunner.strict_io = old
        return float('nan')

    def _control(self, frame, now):
        self.vvi_hold.step(frame, now)
        if self.pid_assist.has_active_pids:
            # Correlated P3D bank is left-positive; the shared Bank PID uses
            # the right-positive convention used by the roll response worker.
            pid_frame = dict(frame, bank_deg=-frame['bank_deg'])
            self.pid_assist.update_stabilization(now, pid_frame, self.brunner)
        throttle = self.ias_hold.update(frame['ias_kt'], now)
        if (self._last_throttle_send is None or now - self._last_throttle_send >= THROTTLE_COMMAND_INTERVAL_S):
            if not self.sim.set_replay_throttle(throttle):
                raise RuntimeError('P3D rejected IAS-hold throttle command')
            self._last_throttle_send = now
        self.force_n = self._read_pitch()
        trim = finite(frame, 'trim_indicator')
        if trim is None:
            trim = finite(frame, 'elevator_trim_pct')
        self.trim_fraction = float(trim) if trim is not None else float('nan')

    def _within_stability(self, frame):
        return (math.isfinite(self.force_n) and abs(self.force_n) <= FORCE_TOLERANCE_N
                and abs(frame['ias_kt'] - self.cfg['ias_kt']) <= IAS_TOLERANCE_KT
                and abs(frame['alt_ft'] - self.cfg['alt_ft']) <= ALTITUDE_TOLERANCE_FT
                and abs(finite(frame, 'vertical_speed') or 0.0) <= VVI_TOLERANCE_FPM)

    def step(self, frame):
        if self.phase not in ('TRIM_STABILIZE', 'RECORD'):
            if self.phase == 'CONNECT' and (not frame or
                    not (-0.2 <= self.clock() - frame.get('receipt_time', -1.e6) <= 1.0)):
                if self.clock() >= self.deadline:
                    self.finish('FAILED', 'Timed out waiting for fresh P3D telemetry after loading')
                else:
                    self.details = 'Waiting for P3D to finish loading and publish fresh flight measurements'
                    self.emit()
                return
            previous = self.phase
            super().step(frame)
            if self.phase == 'READY' and previous != 'READY':
                self._begin_trim(self.clock())
                self.emit()
            return
        if self.phase == 'DONE':
            return
        now = self.clock()
        try:
            self._ensure_live(frame, now)
            if frame['sequence'] == self.last_sequence:
                return
            self.last_sequence, self.latest, self.last_sample_time = frame['sequence'], frame, now
            if self.pid_assist.bank_requested and abs(frame['bank_deg']) > 20.0:
                raise RuntimeError('Bank exceeded 20 deg during trim stabilization; roll control stopped')
            self._control(frame, now)
            if self.phase == 'TRIM_STABILIZE':
                if now - self.trim_started > TRIM_RESPONSE_TIMEOUT_S:
                    raise RuntimeError('Longitudinal trim did not unload pitch force and stabilize within 45 s')
                if not math.isfinite(self.force_n):
                    raise RuntimeError('Measured pitch-controller force is unavailable')
                if abs(self.force_n) > FORCE_TOLERANCE_N:
                    self.stable_since = None
                    if self.last_trim_pulse is None or now - self.last_trim_pulse >= TRIM_PULSE_INTERVAL_S:
                        if self.trim_direction is None:
                            self.trim_direction = UP if self.force_n > 0.0 else DOWN
                        elif (self.force_before_pulse is not None and
                              abs(self.force_n) > abs(self.force_before_pulse) + 0.5):
                            self.trim_direction = DOWN if self.trim_direction == UP else UP
                        self.events.step(self.trim_direction)
                        self.force_before_pulse = self.force_n
                        self.last_trim_pulse = now
                    self.details = (f'Unloading pitch force {self.force_n:+.2f} N | '
                                    f'IAS {frame["ias_kt"]:.1f}/{self.cfg["ias_kt"]:.1f} kt | '
                                    f'Altitude {frame["alt_ft"]:.0f}/{self.cfg["alt_ft"]:.0f} ft')
                    return
                if self._within_stability(frame):
                    if self.stable_since is None:
                        self.stable_since = now
                    remaining = max(0.0, STABLE_SECONDS - (now - self.stable_since))
                    self.details = f'Trimmed force {self.force_n:+.2f} N; stability countdown {remaining:.1f} s'
                    if remaining <= 0.0:
                        self.phase, self.stage = 'RECORD', 'RECORDING'
                        self.start_time = self.recording_started = now
                        self.rows = []
                        self.details = 'Trim fixed; altitude and IAS controllers remain active during recording'
                else:
                    self.stable_since = None
                return

            self._record_frame(frame, now)
            row = self.rows[-1]
            row['Elevator Trim Indicator (position)'] = self.trim_fraction
            row['Elevator Trim (%)'] = self.trim_fraction * 100.0 if math.isfinite(self.trim_fraction) else float('nan')
            row['Longitudinal Trim Pitch Force (N)'] = self.force_n
            if now - self.start_time >= float(self.context.get('duration_s', 75.0)):
                self.finish('COMPLETE', 'Longitudinal trim recorded with altitude, IAS and unloaded pitch force maintained')
        except Exception as exc:
            self.finish('FAILED', str(exc))
        finally:
            self.emit()

    def finish(self, result, reason):
        if self.phase == 'DONE':
            return
        try:
            self.vvi_hold.release()
        except Exception:
            pass
        if self.pid_assist.transport is not None:
            try:
                self.pid_assist.transport.release()
            except Exception:
                pass
        try:
            self.sim.pause_sim()
        except Exception:
            pass
        self.phase, self.result, self.stage, self.details = 'DONE', result, result, reason
        if self.rows:
            try:
                self.outputs = save_report(self.context, self.rows, result, reason)
            except Exception as exc:
                self.stage, self.details = 'FAILED', reason + '; report error: ' + str(exc)
        self.emit()


def save_report(context, rows, result, reason):
    folder = Path(context['result_directory']); folder.mkdir(parents=True, exist_ok=True)
    variant = 'cruise' if context['test_reference'].endswith('(5a)') else 'approach'
    stem = folder / (f'2_c_5_longitudinal_trim_{variant}_auto_' + datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
                     + '_' + str(time.time_ns()))
    csv_path, meta_path = stem.with_suffix('.csv'), stem.with_suffix('.qtg_test.json')
    pdf_path = stem.with_name(stem.name + '_graphs.pdf')
    with csv_path.open('x', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    metadata = dict(context, result=result, reason=reason, status=result.lower(), mode='auto',
                    control_method=METHOD, kind='Longitudinal Trim',
                    measurement_source='P3D telemetry + BRUNNER pitch force; VVI PID and throttle IAS hold')
    meta_path.write_text(json.dumps(metadata, indent=2, allow_nan=False), encoding='utf-8')
    from reports.plot_graphs import _render_report
    _render_report(sim_path=csv_path, ref_path=None, out_path=pdf_path, titt_results=None,
                   metadata=metadata, selected_graphs=context.get('selected_graphs'))
    return [str(csv_path), str(meta_path), str(pdf_path)]


def run(context, publish, *, factory=None, reader_factory=None, sleep=time.sleep):
    sim = reader = controller = brunner = None
    try:
        if factory is None:
            root = Path(sys._MEIPASS) if getattr(sys, 'frozen', False) else Path(__file__).resolve().parents[1]
            source = root / 'interfaces/brunner_base/src/p3d_simconnect_v15.py'
            spec = importlib.util.spec_from_file_location('qtg_long_trim_p3d', source)
            module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
            factory = module.SimConnectClient
        sim = factory()
        from interfaces.brunner_base.src.brunner_interface import BrunnerInterface
        brunner = BrunnerInterface(timeout=0.05); brunner.connect()
        controller = AutomaticLongitudinalTrim(dict(context, brunner=brunner), sim, publish)
        controller.brunner = brunner
        controller.emit()
        thread = threading.Thread(target=sim.connect_and_run,
                                  kwargs=dict(place_at_rahwali=False, place_airborne=False), daemon=True)
        thread.start()
        if reader_factory is None:
            from interfaces.engine_measurements import Measurements, CHANNELS
            root = Path(sys._MEIPASS) if getattr(sys, 'frozen', False) else Path(__file__).resolve().parents[1]
            reader_factory = lambda: Measurements(root/'bridge/bin/qtg_simconnect.dll',
                                                   channels=CHANNELS + BODY_ACCELERATION_CHANNELS)
        reader = reader_factory(); controller.events = TrimEvents(reader)
        directory = Path(context['command_directory'])
        while not controller.closed:
            for path in sorted(directory.glob('*.json')):
                command = json.loads(path.read_text(encoding='utf-8')); path.unlink(); controller.command(command)
            stop = Path(context['stop_file'])
            if stop.exists(): stop.unlink(); controller.command({'action': 'stop'})
            if controller.phase != 'DONE': controller.step(reader.poll())
            sleep(0.04)
        return 0
    except Exception as exc:
        if controller: controller.finish('FAILED', str(exc))
        else: publish(dict(context, stage='FAILED', details=str(exc), can_record=False, outputs=[]))
        return 1
    finally:
        if reader:
            try: reader.close()
            except Exception: pass
        if brunner:
            try: brunner.close()
            except Exception: pass
        if sim:
            try: sim.stop()
            except Exception: pass
