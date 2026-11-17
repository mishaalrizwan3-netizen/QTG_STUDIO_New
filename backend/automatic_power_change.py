"""Power Change Dynamics: P3D throttle-only step and *uncontrolled* flight response.

Dedicated Automatic worker. No BRUNNER, autopilot, trim, elevator, or PID writes.
The completed recording is *not* an EASA validation without reference data.
"""
import csv
from datetime import datetime
import importlib.util
import json
import math
from pathlib import Path
import sys
import threading
import time

from backend.verifier import validate_conditions

REFERENCE = '2.b.(11)'  # Internal: catalog's 2.c.(1) is already Roll Response.
METHOD = 'power_dynamics'
BASELINE_S = 5.0
POST_POWER_S = 15.0
SETTLE_S = 3.0
PREPARATION_TIMEOUT_S = 40.0
POWER_CONFIRM_TIMEOUT_S = 5.0
MAX_RECORDING_S = 28.0


def selected(context):
    return (context.get('mode') == 'auto' and context.get('control_method') == METHOD
            and context.get('test_reference') == REFERENCE)


def finite(frame, key):
    try:
        value = float(frame[key])
    except (KeyError, ValueError, TypeError):
        return None
    return value if math.isfinite(value) else None


def stable(samples, cfg):
    """Readiness gate, not an aircraft-reference tolerance or a PID controller."""
    if len(samples) < 4 or samples[-1][0] - samples[0][0] < SETTLE_S:
        return False, 'Waiting for 3 s of stable flight measurements'
    frames = [frame for _, frame in samples]
    for name, span in (('ias_kt', 2.), ('alt_ft', 35.), ('pitch_deg', 1.5), ('bank_deg', 3.)):
        values = [finite(row, name) for row in frames]
        if None in values or max(values) - min(values) > span:
            return False, f'Unstable {name} (please trim/stabilize the aircraft before testing)'
    latest = frames[-1]
    if abs(latest['ias_kt'] - cfg['ias_kt']) > 3.:
        return False, 'Airspeed differs from configured approach target'
    if abs(latest['alt_ft'] - cfg['alt_ft']) > 70.:
        return False, 'Altitude differs from configured approach target'
    if abs(latest['bank_deg']) > 3.:
        return False, 'Wings not level'
    vs = finite(latest, 'vertical_speed')
    if vs is None or abs(vs) > 250.:
        return False, 'Aircraft is climbing or descending too fast for steady baseline'
    return True, ''


class AutomaticPowerDynamics:
    def __init__(self, context, sim, publish, *, clock=time.monotonic):
        if not selected(context):
            raise ValueError('Dedicated Power Change Dynamics Automatic session required')
        self.context = dict(context)
        self.cfg = validate_conditions(context['initial_conditions'])
        if not self.cfg or self.cfg['on_ground'] or self.cfg['ias_kt'] < 50.:
            raise ValueError('Power Change Dynamics requires an airborne approach condition')
        if self.cfg['throttle'] > 94.:
            raise ValueError('Starting approach power must be below go-around power')
        self.target = float(context.get('go_around_throttle', 100.))
        if not (self.cfg['throttle'] + 5 <= self.target <= 100.):
            raise ValueError('Invalid go-around power selection')
        self.sim, self.publish, self.clock = sim, publish, clock
        self.phase, self.stage = 'CONNECT', 'Connecting'
        self.details = 'Connecting to P3D; no flight-control or BRUNNER commands'
        self.deadline = clock() + 90.
        self.latest, self.last_sequence = None, None
        self.last_sim_time, self.last_sim_advance = None, None
        self.stable_samples = []
        self.rows, self.outputs = [], []
        self.start_time = self.power_command_time = self.power_onset_time = self.power_completed_time = None
        self.result, self.closed = None, False
        self.last_sample_time = None
        self.baseline_anchor = None

    def status(self):
        frame = self.latest or {}
        elapsed = max(0., self.clock() - self.start_time) if self.start_time is not None else 0.
        can_record = self.phase == 'READY'
        detail = self.details
        return dict(self.context, stage=self.stage, details=detail, updated=time.time(),
                    connected=bool(self.sim.is_connected()), brunner_connected=False,
                    can_record=can_record, controlling=self.phase in ('PREPARE', 'RECORD'),
                    recording=self.phase == 'RECORD', samples=len(self.rows),
                    elapsed=elapsed, duration_s=BASELINE_S + POST_POWER_S,
                    outputs=list(self.outputs), data_directory=self.context['result_directory'],
                    engine_text=(f"Engine RPM: {finite(frame, 'rpm') or 0:.0f}" if frame else 'Engine RPM: N/A'),
                    throttle_text=(f"Throttle: {finite(frame, 'throttle') or 0:.1f}%" if frame else 'Throttle: N/A'),
                    flight_text=(f"Airspeed / Altitude: {frame.get('ias_kt', 0):.1f} kt / {frame.get('alt_ft', 0):.0f} ft" if frame else 'Airspeed / Altitude: N/A'),
                    attitude_text=(f"Pitch: {frame.get('pitch_deg', 0):+.1f}° | Bank: {frame.get('bank_deg', 0):+.1f}°" if frame else 'Pitch / Bank: N/A'),
                    controls_text='No BRUNNER or P3D flight-control movement commanded',
                    recording_text=('READY TO RECORD (00:00)' if can_record else
                                    f'RECORDING ({int(elapsed//60):02}:{int(elapsed%60):02}) - {self.stage}' if self.phase == 'RECORD' else self.stage),
                    progress_text=f'{len(self.rows)} recorded frames | {detail}',
                    telemetry=dict(ias_kt=frame.get('ias_kt'), alt_ft=frame.get('alt_ft'),
                                   vertical_speed=frame.get('vertical_speed'),
                                   pitch_deg=frame.get('pitch_deg'), bank_deg=frame.get('bank_deg'),
                                   engine_rpm=frame.get('rpm'), engine_throttle_percent=frame.get('throttle')))

    def emit(self):
        self.publish(self.status())

    def command(self, command):
        action = command['action']
        if action in ('stop', 'shutdown', 'invalidate_initialization'):
            if self.phase != 'DONE':
                self.finish('ABORTED', 'Operator stopped Power Change Dynamics')
            if action == 'shutdown':
                self.closed = True
        elif action == 'record' and self.phase == 'READY':
            if not self.latest or not self.sim.is_connected() or self.clock() - self.latest['receipt_time'] > 1.:
                self.finish('FAILED', 'Fresh simulator measurements are required to start recording')
            else:
                self.sim.unpause_sim()
                self.start_time = self.clock()
                self.last_sim_advance = self.start_time
                self.last_sim_time = self.latest.get('sim_time')
                self.phase, self.stage = 'RECORD', 'RECORDING BASELINE'
                self.details = '5 s baseline before the power step; no pitch/IAS/altitude PID'
        self.emit()

    def _ensure_live(self, frame, now):
        if not self.sim.is_connected():
            raise RuntimeError('P3D disconnected')
        if not frame or not (0 <= now - frame['receipt_time'] <= 1.):
            raise RuntimeError('Fresh correlated P3D telemetry unavailable')
        for key in ('sequence', 'sim_time', 'rpm', 'throttle', 'ias_kt', 'alt_ft', 'pitch_deg', 'bank_deg'):
            if finite(frame, key) is None:
                raise RuntimeError(f'Invalid P3D telemetry: {key}')

    def _initialize(self, frame, now):
        self.sim.pause_sim()
        if not self.sim.wait_for_paused(3.):
            raise RuntimeError('P3D did not confirm pause')
        cfg = self.cfg
        if not self.sim.queue_position_airborne(alt_ft=cfg['alt_ft'], speed_kts=cfg['ias_kt'],
                                                lat=cfg['lat'], lon=cfg['lon'],
                                                hdg_deg=cfg['heading_deg'], pitch_deg=cfg['pitch_deg'],
                                                bank_deg=-cfg['bank_deg'], on_ground=False):
            raise RuntimeError('P3D initial approach position rejected')
        if not self.sim.set_initial_flaps(cfg['flaps']):
            raise RuntimeError('P3D approach flaps rejected')
        if not self.sim.set_initial_throttle(cfg['throttle']):
            raise RuntimeError('P3D approach power rejected')
        # Follow the existing Engine worker's weather/temperature initialization.
        from interfaces.brunner_base.src.replay_support import reference_weather_metar
        metar = reference_weather_metar(
            dict(altitude_msl=cfg['alt_ft'], oat=cfg['temperature_c']),
            self.sim.get_global_weather_metar())
        if not metar or not self.sim.set_global_weather_metar(metar):
            raise RuntimeError('Approach temperature/weather initialization rejected')
        self.sim.unpause_sim()
        self.phase, self.stage = 'PREPARE', 'Initializing aircraft'
        self.details = 'Waiting for stabilized approach flight; no PID adjustments'
        self.deadline = now + PREPARATION_TIMEOUT_S
        self.last_sim_time = frame['sim_time']
        self.last_sim_advance = now

    def _record_frame(self, frame, now):
        row = {
            'Time (s)': round(now - self.start_time, 4),
            'Simulator Time (s)': frame['sim_time'],
            'Native Sequence': frame['sequence'],
            'Indicated Airspeed (kts)': frame['ias_kt'],
            'Altitude MSL (ft)': frame['alt_ft'],
            'Height AGL (ft)': finite(frame, 'height_agl_ft'),
            'Rate of Climb/Descent (fpm)': finite(frame, 'vertical_speed'),
            'Pitch Angle (deg)': frame['pitch_deg'],
            'Bank Angle (deg)': frame['bank_deg'],
            'Heading (deg)': finite(frame, 'heading_deg'),
            'Throttle 1 Position (%)': frame['throttle'],
            'Engine 1 RPM': frame['rpm'],
            'Propeller 1 RPM': finite(frame, 'prop_rpm'),
            'Manifold Pressure (inHg)': (frame['manifold_pressure_psi'] * 2.0360206576
                                            if finite(frame, 'manifold_pressure_psi') is not None else None),
            'Elevator Input (normalised)': finite(frame, 'elevator_input'),
            'Elevator Trim Angle (rad)': finite(frame, 'elevator_trim_rad'),
            'Autopilot Master (0/1)': finite(frame, 'autopilot_active'),
            'Power Step Command (0/1)': int(self.power_command_time is not None),
        }
        self.rows.append(row)

    def step(self, frame):
        if self.phase == 'DONE':
            return
        now = self.clock()
        try:
            if self.phase == 'CONNECT':
                if now >= self.deadline:
                    raise RuntimeError('Timed out connecting to P3D')
                if not self.sim.is_connected() or not frame:
                    return
                self._ensure_live(frame, now)
                self.latest, self.last_sequence = frame, frame['sequence']
                self._initialize(frame, now)
                return
            if frame is None or not (0. <= now - frame.get('receipt_time', -1.e6) <= 1.):
                if not self.sim.is_connected():
                    raise RuntimeError('P3D disconnected')
                if self.phase == 'RECORD' and self.last_sample_time is not None and now - self.last_sample_time > 1.5:
                    raise RuntimeError('P3D measurement stream stopped')
                if self.phase == 'PREPARE' and now >= self.deadline:
                    raise RuntimeError('Timed out waiting for fresh approach telemetry')
                return
            self._ensure_live(frame, now)
            if frame['sequence'] == self.last_sequence:
                if self.phase == 'RECORD' and self.last_sample_time is not None and now - self.last_sample_time > 1.5:
                    raise RuntimeError('P3D measurement sequence stalled during recording')
                return
            if self.last_sequence is not None and frame['sequence'] < self.last_sequence:
                raise RuntimeError('P3D measurement sequence went backwards')
            self.last_sequence, self.latest, self.last_sample_time = frame['sequence'], frame, now
            if self.phase in ('PREPARE', 'RECORD'):
                if self.last_sim_time is not None and frame['sim_time'] > self.last_sim_time:
                    self.last_sim_advance = now
                    self.last_sim_time = frame['sim_time']
                elif now - (self.last_sim_advance or now) > 1.5:
                    raise RuntimeError('P3D is paused or simulation time stopped advancing')
            if self.phase in ('PREPARE', 'RECORD') and frame.get('on_ground') == 1:
                raise RuntimeError('Aircraft is on the ground; airborne free-response test cannot continue')
            agl = finite(frame, 'height_agl_ft')
            if self.phase in ('PREPARE', 'RECORD') and agl is not None and agl < 300.:
                raise RuntimeError('Aircraft too low to safely continue; recording stopped')
            if self.phase in ('PREPARE', 'RECORD') and frame.get('autopilot_active') == 1:
                raise RuntimeError('Autopilot must be OFF for an uncontrolled free-response test')
            if self.phase == 'PREPARE':
                if now >= self.deadline:
                    raise RuntimeError('Approach could not stabilize within 40 s; do not record an invalid free response')
                if frame['rpm'] < 300. or abs(frame['throttle'] - self.cfg['throttle']) > 2.5:
                    self.stable_samples = []
                    self.details = 'Waiting for engine power to match approach setting'
                    return
                if finite(frame, 'flaps') is None or abs(frame['flaps'] - self.cfg['flaps']) > 0.1:
                    self.stable_samples = []
                    self.details = 'Waiting for configured flap position'
                    return
                ambient = finite(frame, 'temperature_c')
                if ambient is None or abs(ambient - self.cfg['temperature_c']) > 3.:
                    self.stable_samples = []
                    self.details = 'Waiting for configured approach temperature'
                    return
                self.stable_samples.append((now, dict(frame)))
                self.stable_samples = [(t, f) for t, f in self.stable_samples if t >= now - (SETTLE_S + .3)]
                ok, reason = stable(self.stable_samples, self.cfg)
                self.details = reason or 'Approach stabilized; 5 s baseline available after Start Recording'
                if ok:
                    self.sim.pause_sim()
                    if not self.sim.wait_for_paused(3.):
                        raise RuntimeError('P3D did not confirm pause at READY TO RECORD')
                    self.phase, self.stage = 'READY', 'READY TO RECORD'
                return
            if self.phase == 'READY':
                if abs(frame['throttle'] - self.cfg['throttle']) > 2.5:
                    raise RuntimeError('Starting approach throttle changed; initialize again')
                return
            if self.phase == 'RECORD':
                if now - self.start_time > MAX_RECORDING_S:
                    raise RuntimeError('Power Change Dynamics exceeded its bounded recording window')
                self._record_frame(frame, now)
                elapsed = now - self.start_time
                if self.power_command_time is None:
                    # EASA requires at least 5 s of *measured* pre-step samples,
                    # not merely five seconds since the button was clicked.
                    if (elapsed >= BASELINE_S and len(self.rows) >= 2 and
                            self.rows[-1]['Time (s)'] - self.rows[0]['Time (s)'] >= BASELINE_S):
                        baseline = [(self.start_time + row['Time (s)'], dict(
                            ias_kt=row['Indicated Airspeed (kts)'], alt_ft=row['Altitude MSL (ft)'],
                            pitch_deg=row['Pitch Angle (deg)'], bank_deg=row['Bank Angle (deg)'],
                            vertical_speed=row['Rate of Climb/Descent (fpm)']))
                                    for row in self.rows if row['Time (s)'] >= elapsed-SETTLE_S]
                        ready, why = stable(baseline, self.cfg)
                        if not ready:
                            raise RuntimeError('Baseline not stable: ' + why)
                        if not self.sim.set_replay_throttle(self.target):
                            raise RuntimeError('P3D power-change command rejected')
                        self.power_command_time = now
                        self.stage = 'POWER CHANGE'
                        self.details = f'Step commanded: approach {self.cfg["throttle"]:.1f}% to {self.target:.1f}%'
                elif self.power_completed_time is None:
                    if self.power_onset_time is None and frame['throttle'] - self.cfg['throttle'] > 2.:
                        self.power_onset_time = now
                    self.stage = 'VERIFYING POWER CHANGE'
                    if abs(frame['throttle'] - self.target) <= 2.0:
                        self.power_completed_time = now
                        self.stage = 'FREE RESPONSE'
                        self.details = 'Recording 15 seconds after actual go-around throttle was reached'
                    elif now - self.power_command_time > POWER_CONFIRM_TIMEOUT_S:
                        raise RuntimeError('Actual P3D throttle did not reach go-around power')
                elif now - self.power_completed_time >= POST_POWER_S:
                    self.finish('COMPLETE', 'Uncontrolled power-change response captured; reference comparison not performed')
        except Exception as exc:
            self.finish('FAILED', str(exc))
        finally:
            self.emit()

    def finish(self, result, reason):
        if self.phase == 'DONE':
            return
        self.phase, self.result = 'DONE', result
        self.stage = result
        self.details = reason
        try:
            if self.sim.is_connected():
                self.sim.pause_sim()
        except Exception as exc:
            self.details += f'; pause failed: {exc}'
        if self.rows:
            self.stage = 'Generating reports'
            self.emit()
            try:
                self.outputs = save_report(self.context, self.rows, result, reason,
                                           power_command_time=self.power_command_time,
                                           power_onset_time=self.power_onset_time,
                                           power_completed_time=self.power_completed_time,
                                           recording_start=self.start_time)
                self.stage = 'Completed' if result == 'COMPLETE' else result
            except Exception as exc:
                self.outputs = list(getattr(exc, 'outputs', []))
                self.stage = 'FAILED'
                self.details += '; reporting error: ' + str(exc)
        self.emit()


def save_report(context, rows, result, reason, *, power_command_time, power_onset_time, power_completed_time, recording_start):
    folder = Path(context['result_directory'])
    folder.mkdir(parents=True, exist_ok=True)
    stem = folder / ('2_b_11_power_change_dynamics_auto_' + datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
                     + '_' + str(time.time_ns()))
    csv_path = stem.with_suffix('.csv')
    meta_path = stem.with_suffix('.qtg_test.json')
    pdf_path = stem.with_name(stem.name + '_graphs.pdf')
    outputs = []
    try:
        with csv_path.open('x', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        outputs.append(str(csv_path))
        command_s = (power_command_time-recording_start if power_command_time is not None else None)
        onset_s = (power_onset_time-recording_start if power_onset_time is not None else None)
        complete_s = (power_completed_time-recording_start if power_completed_time is not None else None)
        metadata = dict(context, mode='auto', control_method=METHOD, kind='Power Change Dynamics',
                        result=result, reason=reason, status=result.lower(),
                        event_time_s=onset_s, throttle_command_time_s=command_s, throttle_measured_onset_s=onset_s,
                        throttle_completed_time_s=complete_s,
                        baseline_required_s=BASELINE_S, post_power_required_s=POST_POWER_S,
                        aircraft_conformity='NOT EVALUATED - approved aircraft reference data not supplied',
                        test_code_note='2.b.(11) is an internal identifier; CS-FSTD(A) longitudinal c.(1) is the source requirement',
                        measurement_source='Correlated P3D SimConnect telemetry; no BRUNNER or PID commands')
        meta_path.write_text(json.dumps(metadata, indent=2, allow_nan=False), encoding='utf-8')
        outputs.append(str(meta_path))
        from reports.plot_graphs import _render_report
        chosen = context.get('selected_graphs') or [
            'ias_kt', 'altitude_msl_ft', 'height_agl_ft', 'pitch_deg', 'bank_deg',
            'vertical_speed_fpm', 'throttle_pct', 'engine_rpm', 'manifold_inhg']
        _render_report(sim_path=csv_path, ref_path=None, out_path=pdf_path,
                       titt_results=None, metadata=metadata, selected_graphs=chosen)
        outputs.append(str(pdf_path))
        return outputs
    except Exception as exc:
        exc.outputs = outputs
        raise


def run(context, publish, *, factory=None, reader_factory=None, sleep=time.sleep):
    sim = reader = thread = controller = None
    try:
        if factory is None:
            root = Path(sys._MEIPASS) if getattr(sys, 'frozen', False) else Path(__file__).resolve().parents[1]
            candidates = (root/'interfaces/brunner_base/src/p3d_simconnect_v15.py',
                          root/'brunner_base/src/p3d_simconnect_v15.py')
            source = next((p for p in candidates if p.is_file()), candidates[0])
            spec = importlib.util.spec_from_file_location('qtg_auto_power_p3d', source)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            factory = module.SimConnectClient
        sim = factory()
        controller = AutomaticPowerDynamics(context, sim, publish)
        controller.emit()
        thread = threading.Thread(target=sim.connect_and_run,
                                  kwargs=dict(place_at_rahwali=False, place_airborne=False), daemon=True)
        thread.start()
        if reader_factory is None:
            from interfaces.engine_measurements import Measurements, CHANNELS
            root = Path(sys._MEIPASS) if getattr(sys, 'frozen', False) else Path(__file__).resolve().parents[1]
            candidates = (root/'bridge/bin/qtg_simconnect.dll', root/'qtg_simconnect.dll',
                          Path(sys.executable).parent/'bridge/bin/qtg_simconnect.dll')
            dll = next((p for p in candidates if p.is_file()), candidates[0])
            # Additional read-only channels for evidence that trim/controls/AP were not commanded.
            extra = (('autopilot_active', 'AUTOPILOT MASTER', 'bool'),
                     ('elevator_input', 'ELEVATOR POSITION', 'position'),
                     ('elevator_trim_rad', 'ELEVATOR TRIM POSITION', 'radians'))
            reader_factory = lambda: Measurements(dll, channels=CHANNELS + extra)
        reader = reader_factory()
        directory = Path(context['command_directory'])
        while not controller.closed:
            commands = []
            for path in sorted(directory.glob('*.json')):
                commands.append(json.loads(path.read_text(encoding='utf-8')))
                path.unlink()
            stop = Path(context['stop_file'])
            if stop.exists():
                stop.unlink()
                commands.insert(0, dict(action='stop'))
            stopping = next((c for c in commands if c['action'] == 'shutdown'), None) or next(
                (c for c in commands if c['action'] in ('stop', 'invalidate_initialization')), None)
            if stopping:
                controller.command(stopping)
            else:
                for command in commands:
                    controller.command(command)
            if not controller.closed and controller.phase != 'DONE':
                before = controller.phase
                controller.step(reader.poll())
                if before == 'CONNECT' and controller.phase == 'PREPARE' and hasattr(reader, 'discard_pending'):
                    reader.discard_pending()
                sleep(.04)
            elif not controller.closed:
                sleep(.04)
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
        if reader:
            reader.close()
        if sim:
            sim.stop()
        if thread:
            thread.join(timeout=5)
