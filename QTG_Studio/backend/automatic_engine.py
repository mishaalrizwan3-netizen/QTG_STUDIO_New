"""Live GUI engine-step workflow; transport and clock are injected for offline tests."""
import importlib.util
import json
import math
from pathlib import Path
import sys
import threading
import time

from backend.engine_step_sequence import EngineStepSequence, Settings
from backend.engine_conditions import enforce
from backend.initializer import apply_body_acceleration
from interfaces.engine_measurements import BODY_ACCELERATION_CHANNELS, body_acceleration_fields

REFERENCES = ('1.f.(1)', '1.f.(2)')
METHOD = 'engine_step'
# The engine test definition and existing idle recordings use 5% lever position.
IDLE_THROTTLE = 5.0


def selected(context):
    return (context.get('mode') == 'auto' and context.get('test_reference') in REFERENCES
            and context.get('control_method') == METHOD)


class AutomaticEngine:
    def __init__(self, context, sim, publish, *, clock=time.monotonic, brunner=None):
        if not selected(context):
            raise ValueError('Automatic engine workflow requires an engine Automatic session')
        self.context, self.sim, self.publish, self.clock = context, sim, publish, clock
        self.deceleration = context['test_reference'] == REFERENCES[1]
        self.cfg = enforce(context['test_reference'], context['initial_conditions'])
        if self.cfg is None:
            raise ValueError('Initial aircraft conditions are required')
        self.cfg['throttle'] = 100. if self.deceleration else IDLE_THROTTLE
        if self.deceleration and (not self.cfg['on_ground'] or self.cfg['ias_kt'] != 0):
            raise ValueError('Deceleration requires Ground with zero airspeed')
        self.context['initial_conditions'] = dict(self.cfg)
        self.stage = 'Connecting'
        self.details = 'Connecting to Sim Flight Model'
        self.phase = 'CONNECT'
        self.startup_timeout = float(context.get('startup_timeout', 120.0))
        self.deadline = clock() + self.startup_timeout
        self.latest = None
        self.last_sequence = None
        self.last_sim_time = None
        self.sim_advanced_at = None
        self.sequence = None
        self.rows, self.outputs = [], []
        self.stable_since = None
        self.idle_reapplied = False
        self.initial_idle_corrected = False
        self.initialization_running = False
        from backend.parking_brake import GroundParkingBrake
        def brake_command_unavailable():
            raise RuntimeError('parking brake command is unavailable')
        self.ground_brake = GroundParkingBrake(brake_command_unavailable, require_stationary=False)
        self.closed = False
        self.result = None
        self.diagnostics = {}
        if brunner is not None:
            self.brunner = brunner
        else:
            self.brunner = None
            try:
                from interfaces.brunner_base.src.brunner_interface import BrunnerInterface
                self.brunner = BrunnerInterface(timeout=0.01)
                self.brunner.connect()
            except Exception:
                self.brunner = None
        self._last_brunner_data = None
        self._next_brunner_poll = 0.0
        from backend.pid_assist import PidAssist
        self.pid_assist = PidAssist(self.context, owned_axis_mask=0)

    def _read_brunner(self):
        if not self.brunner or not getattr(self.brunner, 'sock', None):
            return None
        now = self.clock()
        if now < self._next_brunner_poll:
            return self._last_brunner_data
        try:
            p_pos, p_force, _ = self.brunner.read_axis(0x01)
            r_pos, r_force, _ = self.brunner.read_axis(0x02)
            y_pos, y_force, _ = self.brunner.read_axis(0x04)
            data = {
                'pitch_pos': p_pos if math.isfinite(p_pos) else None,
                'pitch_force': p_force if math.isfinite(p_force) else None,
                'roll_pos': r_pos if math.isfinite(r_pos) else None,
                'roll_force': r_force if math.isfinite(r_force) else None,
                'yaw_pos': y_pos if math.isfinite(y_pos) else None,
                'yaw_force': y_force if math.isfinite(y_force) else None,
            }
            self._last_brunner_data = data
            self._next_brunner_poll = now + 0.1
            return data
        except Exception:
            self._next_brunner_poll = now + 2.0
            return None

    def status(self):
        frame = self.latest or {}
        now = self.clock()
        sim_tel = self.sim.get_telemetry() if hasattr(self.sim, 'get_telemetry') else {}

        ias = frame.get('ias_kt')
        if ias is None and sim_tel:
            ias = sim_tel.get('indicated_airspeed')
        alt = frame.get('alt_ft')
        if alt is None and sim_tel:
            alt = sim_tel.get('indicated_altitude')
        h_agl = frame.get('height_agl_ft')
        if h_agl is None and sim_tel:
            h_agl = sim_tel.get('radar_alt')

        if ias is not None and math.isfinite(ias) and alt is not None and math.isfinite(alt):
            flight_text = f"Airspeed / Altitude: {ias:.1f} kt / {alt:.0f} ft"
        else:
            flight_text = "Airspeed / Altitude: N/A"

        pitch = frame.get('pitch_deg')
        if pitch is None and sim_tel:
            pitch = sim_tel.get('pitch_angle')
        bank = frame.get('bank_deg')
        if bank is None and sim_tel:
            bank = sim_tel.get('bank_angle')
        hdg = frame.get('heading_deg')
        if hdg is None and sim_tel:
            hdg = sim_tel.get('true_heading')

        if (pitch is not None and math.isfinite(pitch) and
                bank is not None and math.isfinite(bank) and
                hdg is not None and math.isfinite(hdg)):
            attitude_text = f"Pitch: {pitch:+.1f}° | Bank: {bank:+.1f}° | Heading: {hdg:.1f}°"
        else:
            attitude_text = "Pitch / Bank / Heading: N/A"

        brunner_data = self._read_brunner()
        brunner_connected = bool(self.brunner and getattr(self.brunner, 'sock', None) and getattr(self.brunner, '_connected', False))
        if brunner_data:
            def _fmt_axis(name, pos, force):
                if pos is None:
                    return f"{name}: N/A"
                if force is not None and abs(force) > 0.05:
                    return f"{name}: {pos:+.2f} ({force:+.1f} N)"
                return f"{name}: {pos:+.2f}"
            p_s = _fmt_axis('Pitch', brunner_data.get('pitch_pos'), brunner_data.get('pitch_force'))
            r_s = _fmt_axis('Roll', brunner_data.get('roll_pos'), brunner_data.get('roll_force'))
            y_s = _fmt_axis('Rudder', brunner_data.get('yaw_pos'), brunner_data.get('yaw_force'))
            controls_text = f"CLS2sim {p_s} | {r_s} | {y_s}"
        else:
            controls_text = "CLS2sim Pitch / Roll / Rudder: N/A"

        vs = frame.get('vertical_speed')
        if vs is None and sim_tel:
            vs = sim_tel.get('vertical_speed')
        wt = frame.get('total_weight')
        if wt is None and sim_tel:
            wt = sim_tel.get('total_weight')
        cg = frame.get('cg_percent')
        if cg is None and sim_tel:
            cg = sim_tel.get('cg_percent')

        rpm = frame.get('rpm')
        if rpm is None and sim_tel and 'eng_rpms' in sim_tel and len(sim_tel['eng_rpms']) > 0:
            rpm = sim_tel['eng_rpms'][0]
        rpm_str = f"{rpm:.1f}" if isinstance(rpm, (int, float)) and math.isfinite(rpm) else "N/A"

        prop = frame.get('prop_rpm')
        if prop is None and sim_tel and 'prop_rpms' in sim_tel and len(sim_tel['prop_rpms']) > 0:
            prop = sim_tel['prop_rpms'][0]
        prop_str = f"{prop:.1f}" if isinstance(prop, (int, float)) and math.isfinite(prop) else None

        engine_text = f"Engine RPM: {rpm_str}" + (f" | Prop RPM: {prop_str}" if prop_str else "")

        throttle = frame.get('throttle')
        if throttle is None and sim_tel and 'max_throttle' in sim_tel:
            throttle = sim_tel['max_throttle']
        throttle_str = f"{throttle:.1f}%" if isinstance(throttle, (int, float)) and math.isfinite(throttle) else "N/A"
        throttle_text = f"Throttle: {throttle_str}"

        elapsed_s = 0.0
        if self.phase == 'RECORD':
            if self.sequence and 'recording_start' in self.sequence.events:
                elapsed_s = max(0.0, now - self.sequence.events['recording_start'])
            elif self.rows:
                elapsed_s = self.rows[-1].get('Time (s)', 0.0)
            mins = int(elapsed_s // 60)
            secs = int(elapsed_s % 60)
            timer_str = f"{mins:02d}:{secs:02d}"
            recording_text = f"RECORDING ({timer_str}) - {self.stage}"
            progress_text = f"Elapsed: {timer_str} ({elapsed_s:.1f} s) / {len(self.rows)} samples | {self.details}"
        elif self.phase == 'DONE':
            if self.rows:
                elapsed_s = self.rows[-1].get('Time (s)', 0.0)
            elif self.sequence and 'recording_start' in self.sequence.events:
                stop_t = self.sequence.events.get('recording_stop', now)
                elapsed_s = max(0.0, stop_t - self.sequence.events['recording_start'])
            mins = int(elapsed_s // 60)
            secs = int(elapsed_s % 60)
            final_timer_str = f"{mins:02d}:{secs:02d} ({elapsed_s:.1f} s)"
            recording_text = f"Completed in {final_timer_str} - {self.stage}"
            progress_text = f"Duration: {final_timer_str} / {len(self.rows)} samples | {self.details}"
        elif self.stage == 'READY TO RECORD':
            recording_text = "READY TO RECORD (Timer: 00:00)"
            progress_text = "00:00 / 0 samples | Click Start Recording"
        else:
            recording_text = self.stage
            progress_text = self.details

        telemetry = dict(
            engine_rpm=rpm,
            engine_throttle_percent=throttle,
            prop_rpm=prop,
            vertical_speed=vs,
            total_weight=wt,
            cg_percent=cg,
            ias_kt=ias,
            alt_ft=alt,
            height_agl_ft=h_agl,
            pitch_deg=pitch,
            bank_deg=bank,
            heading_deg=hdg
        )

        return dict(self.context, stage=self.stage, details=self.details, updated=time.time(),
                    connected=bool(self.sim.is_connected()), brunner_connected=brunner_connected,
                    can_record=self.stage == 'READY TO RECORD',
                    controlling=self.phase in ('PREPARE', 'POWER', 'SETTLE', 'RECORD'),
                    recording=self.phase == 'RECORD', samples=len(self.rows),
                    elapsed=elapsed_s, duration_s=15, outputs=list(self.outputs),
                    data_directory=self.context['result_directory'],
                    engine_text=engine_text, throttle_text=throttle_text,
                    flight_text=flight_text, attitude_text=attitude_text, controls_text=controls_text,
                    recording_text=recording_text, progress_text=progress_text,
                    telemetry=telemetry)

    def emit(self):
        self.publish(self.status())

    def command(self, command):
        action = command['action']
        if action in ('stop', 'shutdown', 'invalidate_initialization'):
            if self.sequence and self.phase == 'RECORD':
                self.sequence.abort(self.clock())
            self.finish('ABORTED', 'Operator stopped the test')
            if action == 'shutdown':
                self.closed = True
                self.stage = 'Closed'
                if self.brunner and getattr(self.brunner, 'sock', None):
                    try:
                        self.brunner.sock.close()
                    except Exception:
                        pass
        elif action == 'record' and self.stage == 'READY TO RECORD':
            now = self.clock()
            if (not self.latest or now - self.latest['receipt_time'] > 1
                    or not self.sim.is_connected()):
                self.finish('FAILED', 'Fresh connected telemetry required to record')
                return
            # Resume P3D when starting recording
            self.sim.unpause_sim()
            self.sequence = EngineStepSequence(Settings(
                target_rpm=970. if self.deceleration else 2700.,
                initial_throttle=self.cfg['throttle'], final_throttle=IDLE_THROTTLE if self.deceleration else 100.,
                starting_throttle_tolerance=2., throttle_event_delta=2.,
                engine_response_delta=100., max_receipt_age=1.5,
                baseline_seconds=5.0, min_total_seconds=15.0, stable_seconds=3.0, rpm_tolerance=50.0))
            self.sequence.start(now)
            if getattr(self, 'pid_assist', None) and self.pid_assist.has_active_pids and self.brunner and getattr(self.brunner, 'sock', None):
                try:
                    self.brunner.configure_replay_mode(axis_mask=self.pid_assist.pid_axis_mask)
                    self.brunner.write_setting(self.pid_assist.pid_axis_mask, 0x70, "B", 1)
                    self.brunner.set_overrides(self.pid_assist.pid_axis_mask, True)
                except Exception:
                    pass
            self.phase, self.stage = 'RECORD', 'RECORDING BASELINE'
            self.details = 'Recording 5 seconds before the P3D throttle step'
            self.sim_advanced_at = now
            self.last_sim_time = self.latest.get('sim_time') if self.latest else None
        self.emit()

    def step(self, frame):
        now = self.clock()
        if self.phase in ('DONE', 'CLOSED'):
            return
        try:
            if self.phase == 'CONNECT':
                if now >= self.deadline:
                    raise RuntimeError(f'Timed out waiting for Sim Flight Model to connect ({self.startup_timeout:g}s timeout)')
                if not self.sim.is_connected():
                    self.stage = 'Connecting'
                    self.details = 'Connecting to Sim Flight Model...'
                    return
                if not frame:
                    self.stage = 'Connecting'
                    self.details = 'Waiting for Sim Flight Model telemetry...'
                    return
                if now - frame['receipt_time'] > 1 or frame['receipt_time'] > now:
                    self.stage = 'Connecting'
                    self.details = 'Waiting for fresh telemetry...'
                    return
                if not all(math.isfinite(frame.get(k, float('nan'))) for k in ('rpm', 'throttle', 'sim_time')):
                    self.stage = 'Connecting'
                    self.details = 'Waiting for valid engine measurements...'
                    return

                # SimConnect is available and valid telemetry is present.
                # Apply the selected test's initial conditions immediately.
                self.stage = 'Initializing aircraft'
                self.details = 'Applying position, configuration and initial throttle...'
                self.sim.pause_sim()
                if not self.sim.wait_for_paused(3.):
                    raise RuntimeError('Simulator did not confirm pause')
                # Always remove power before positioning or checking the brake.
                if not self.sim.set_initial_throttle(IDLE_THROTTLE):
                    raise RuntimeError('Initial idle throttle rejected')
                if not self.sim.queue_position_airborne(
                    alt_ft=self.cfg['alt_ft'], speed_kts=self.cfg['ias_kt'], lat=self.cfg['lat'], lon=self.cfg['lon'],
                    hdg_deg=self.cfg['heading_deg'], pitch_deg=self.cfg['pitch_deg'], bank_deg=-self.cfg['bank_deg'],
                    on_ground=self.cfg['on_ground']):
                    raise RuntimeError('Aircraft positioning rejected')
                if not self.sim.set_initial_flaps(self.cfg['flaps']):
                    raise RuntimeError('Initial flaps rejected')
                from interfaces.brunner_base.src.replay_support import reference_weather_metar
                metar = reference_weather_metar(dict(altitude_msl=self.cfg['alt_ft'], oat=self.cfg['temperature_c']),
                                                self.sim.get_global_weather_metar())
                if not metar or not self.sim.set_global_weather_metar(metar):
                    raise RuntimeError('Initial temperature could not be applied')
                self.phase, self.stage = 'PREPARE', 'Initializing aircraft'
                self.details = 'Verifying aircraft position and configuration...'
                self.deadline = now + 15
                self.sim_advanced_at = now
                self.last_sim_time = frame['sim_time']
                self.last_sequence = frame['sequence']
                self.latest = frame
                return
            if not self.sim.is_connected():
                raise RuntimeError('P3D disconnected')
            if not frame or now - frame['receipt_time'] > 1 or frame['receipt_time'] > now:
                if self.phase == 'PREPARE' and now < self.deadline:
                    self.details = 'Waiting for fresh engine measurements during initialization...'
                    return
                raise RuntimeError('Native engine telemetry stopped advancing')
            if frame['sequence'] == self.last_sequence:
                if self.sequence:
                    self.sequence.tick(now)
                    if self.sequence.phase in self.sequence.TERMINAL:
                        self.finish(self.sequence.phase, self.sequence.reason)
                return
            if self.last_sequence is not None and frame['sequence'] < self.last_sequence:
                raise RuntimeError('Engine measurement sequence moved backwards')
            self.last_sequence, self.latest = frame['sequence'], frame
            if not all(math.isfinite(frame[key]) for key in ('rpm', 'throttle', 'sim_time')):
                raise RuntimeError('Invalid engine measurements')
            if self.phase == 'PREPARE':
                checks = [('alt_ft', 25), ('ias_kt', 5), ('lat', .001), ('lon', .001), ('flaps', .1),
                          ('temperature_c', 2), ('heading_deg', 3), ('pitch_deg', 3), ('bank_deg', 3)]
                mismatches = [key for key, tolerance in checks
                              if not math.isfinite(frame[key]) or abs((frame[key] - self.cfg[key] + 180) % 360 - 180
                                     if key == 'heading_deg' else frame[key] - self.cfg[key]) > tolerance]
                if bool(frame['on_ground']) != self.cfg['on_ground']:
                    mismatches.append('on_ground')
                brake_verified = not self.cfg['on_ground']
                if self.cfg['on_ground']:
                    brake_verified = self.ground_brake is not None and self.ground_brake.verify(frame, now)
                    if not brake_verified:
                        mismatches.append('parking brake ON')
                # Positioning can reset throttle; correct it independently of weather,
                # which may only update after simulation time advances.
                idle_mismatch = abs(frame['throttle'] - IDLE_THROTTLE) > 2
                if idle_mismatch and not self.initialization_running and not self.initial_idle_corrected:
                    if not self.sim.set_initial_throttle(IDLE_THROTTLE):
                        raise RuntimeError('Idle throttle reapplication rejected')
                    self.initial_idle_corrected = True
                if not self.initialization_running and not idle_mismatch and brake_verified:
                    self.sim.unpause_sim()
                    self.initialization_running = True
                    self.details = 'Advancing simulator to apply initial position and weather...'
                    return
                if self.deceleration:
                    if not self.initialization_running:
                        mismatches.append('idle throttle and brake before simulator advance')
                    if abs(frame['throttle'] - IDLE_THROTTLE) > 2 and not self.idle_reapplied:
                        mismatches.append('idle throttle 5%')
                    if not mismatches and not self.idle_reapplied:
                        if not self.sim.set_initial_throttle(100.):
                            raise RuntimeError('Initial full throttle rejected')
                        self.idle_reapplied = True
                throttle_mismatch = abs(frame['throttle'] - self.cfg['throttle']) > 2
                if throttle_mismatch:
                    mismatches.append(f'throttle {self.cfg["throttle"]:g}% (measured {frame["throttle"]:.1f}%)')
                if mismatches:
                    self.details = 'Waiting for: ' + ', '.join(mismatches)
                    if now >= self.deadline:
                        raise RuntimeError(self.details)
                    return
                if frame['rpm'] < 300:
                    raise RuntimeError('A running engine is required; start the engine before Automatic Testing')
                # Initial conditions verified. Pause P3D and declare READY TO RECORD.
                self.sim.pause_sim()
                if not self.sim.wait_for_paused(3.0):
                    raise RuntimeError('Simulator did not confirm pause after initialization')
                apply_body_acceleration(self.sim, self.context)
                self.phase, self.stage = 'READY', 'READY TO RECORD'
                self.details = 'Click Start Recording: 5 seconds baseline, throttle step, then 3 seconds stable RPM'
                return
            if self.phase == 'READY':
                if self.cfg['on_ground']:
                    if frame.get('parking_brake') != 1:
                        raise RuntimeError('Parking brake engagement lost')
                if abs(frame['throttle'] - self.cfg['throttle']) > 2:
                    raise RuntimeError('Starting throttle changed; initialize again')
                return
            if self.phase == 'RECORD':
                if self.last_sim_time is not None and frame['sim_time'] > self.last_sim_time:
                    self.sim_advanced_at = now
                    self.last_sim_time = frame['sim_time']
                elif self.sim_advanced_at is not None and now - self.sim_advanced_at > 1.5:
                    raise RuntimeError('Simulator paused or simulation time stopped advancing')
                if self.cfg['on_ground']:
                    if frame.get('parking_brake') != 1:
                        raise RuntimeError('Parking brake engagement lost')

                sim_tel = self.sim.get_telemetry() if hasattr(self.sim, 'get_telemetry') else {}
                prop_rpm = frame.get('prop_rpm')
                if prop_rpm is None and sim_tel and 'prop_rpms' in sim_tel and len(sim_tel['prop_rpms']) > 0:
                    val = sim_tel['prop_rpms'][0]
                    if isinstance(val, (int, float)) and math.isfinite(val):
                        prop_rpm = float(val)

                map_psi = frame.get('manifold_pressure_psi')
                if map_psi is None and sim_tel and 'engine_manifold_pressure_psi_1' in sim_tel:
                    val = sim_tel['engine_manifold_pressure_psi_1']
                    if isinstance(val, (int, float)) and math.isfinite(val):
                        map_psi = float(val)

                vs = frame.get('vertical_speed')
                if vs is None and sim_tel:
                    vs = sim_tel.get('vertical_speed')
                wt = frame.get('total_weight')
                if wt is None and sim_tel:
                    wt = sim_tel.get('total_weight')
                cg = frame.get('cg_percent')
                if cg is None and sim_tel:
                    cg = sim_tel.get('cg_percent')
                h_agl = frame.get('height_agl_ft')
                if h_agl is None and sim_tel:
                    h_agl = sim_tel.get('radar_alt')

                if getattr(self, 'pid_assist', None) and self.pid_assist.has_active_pids:
                    tel = dict(sim_tel or {})
                    tel.update(frame or {})
                    self.pid_assist.update(now, tel, self.brunner)

                row = {
                    'Time (s)': now - self.sequence.events['recording_start'],
                    'Engine RPM': frame['rpm'],
                    'Throttle 1 Position (%)': frame['throttle'],
                    'Parking Brake': frame.get('parking_brake'),
                    'Ground Speed (kt)': frame['ground_speed'],
                    'Altitude MSL (ft)': frame['alt_ft'],
                    'Indicated Airspeed (kts)': frame['ias_kt'],
                    'Pitch Angle (deg)': frame.get('pitch_deg', 0.0),
                    'Bank Angle (deg)': frame.get('bank_deg', 0.0),
                    'Heading (deg)': frame.get('heading_deg', 0.0),
                    'Native Sequence': frame['sequence'],
                    'Receipt Monotonic (s)': frame['receipt_time'],
                    'Simulator Time (s)': frame['sim_time']
                }

                if h_agl is not None and math.isfinite(h_agl):
                    row['Height AGL (ft)'] = float(h_agl)
                else:
                    row['Height AGL (ft)'] = float('nan')
                    self.diagnostics.setdefault('height_agl_ft', 'PLANE ALT ABOVE GROUND channel unavailable from simulator')

                if prop_rpm is not None and math.isfinite(prop_rpm):
                    row['Propeller 1 RPM'] = float(prop_rpm)
                else:
                    row['Propeller 1 RPM'] = float('nan')
                    self.diagnostics.setdefault('prop_rpm', 'PROP RPM:1 telemetry channel unavailable from simulator')

                if map_psi is not None and math.isfinite(map_psi):
                    row['Engine Manifold Pressure (psi)'] = float(map_psi)
                    row['Manifold Pressure (inHg)'] = float(map_psi * 2.0360206576)
                else:
                    row['Engine Manifold Pressure (psi)'] = float('nan')
                    row['Manifold Pressure (inHg)'] = float('nan')
                    self.diagnostics.setdefault('manifold_pressure', 'RECIP ENG MANIFOLD PRESSURE:1 channel unavailable from simulator')

                if not self.deceleration:
                    fuel_flow = frame.get('fuel_flow_pph')
                    row['Fuel Flow (pph)'] = (
                        float(fuel_flow)
                        if fuel_flow is not None and math.isfinite(fuel_flow) and fuel_flow >= 0
                        else float('nan')
                    )

                row.update(body_acceleration_fields(frame))

                if vs is not None and math.isfinite(vs):
                    row['Rate of Climb/Descent (fpm)'] = float(vs)
                if wt is not None and math.isfinite(wt):
                    row['Gross Wt (kg)'] = float(wt)
                if cg is not None and math.isfinite(cg):
                    row['CG Pos (% MAC)'] = float(cg)

                self.rows.append(row)
                action = self.sequence.sample(now=now, sequence=frame['sequence'], receipt_time=frame['receipt_time'],
                                              rpm=frame['rpm'], throttle=frame['throttle'])
                if action and not self.sim.set_replay_throttle(action['percent']):
                    raise RuntimeError('P3D throttle command rejected')
                self.stage = self.sequence.phase
                self.details = f"Target {self.sequence.settings.target_rpm:g} RPM +/-50; recording ~15s total (stable for 3s)"
                if self.sequence.phase in self.sequence.TERMINAL:
                    self.finish(self.sequence.phase, self.sequence.reason)
        except Exception as exc:
            self.finish('FAILED', str(exc))
        finally:
            self.emit()

    def close(self):
        if self.brunner and getattr(self.brunner, 'sock', None):
            try:
                pid_mask = getattr(getattr(self, 'pid_assist', None), 'pid_axis_mask', 0)
                if pid_mask:
                    self.brunner.write_setting(pid_mask, 0x70, "B", 0)
                    self.brunner.set_overrides(pid_mask, False)
                    self.brunner.set_position_forwarding()
                self.brunner.sock.close()
            except Exception:
                pass
            self.brunner.sock = None

    def finish(self, result, reason):
        if self.phase == 'DONE':
            return
        self.phase, self.result = 'DONE', result
        self.stage, self.details = result, reason
        if self.sequence and self.sequence.phase not in self.sequence.TERMINAL:
            self.sequence.fail(self.clock(), reason)
        self.close()
        # Remove power on ground on all exits, before any disk/report work.
        try:
            if self.sim.is_connected():
                if self.deceleration and not self.sim.set_replay_throttle(IDLE_THROTTLE):
                    self.details += '; shutdown idle throttle rejected'
                self.sim.pause_sim()
        except Exception as exc:
            self.details += '; cleanup: '+str(exc)
        if self.rows:
            self.stage = 'Generating reports'
            self.emit()
            try:
                from reports.automatic_engine_report import save
                self.outputs = save(self.context, self.rows, self.sequence, result, self.details, diagnostics=self.diagnostics)
            except Exception as exc:
                # CSV is saved before PDF; expose it even when PDF generation fails.
                self.outputs = getattr(exc, 'outputs', self.outputs)
                self.stage, self.details = 'FAILED', self.details + '; report error: '+str(exc)
            else:
                self.stage = 'Completed' if result == 'COMPLETE' else result
        self.emit()


def run(context, publish, *, factory=None, reader_factory=None, sleep=time.sleep):
    """Own transports until shutdown. Stop wins over queued recording commands."""
    sim = reader = thread = controller = None
    try:
        if factory is None:
            root = Path(sys._MEIPASS) if getattr(sys, 'frozen', False) else Path(__file__).resolve().parents[1]
            candidates = [
                root / 'interfaces/brunner_base/src/p3d_simconnect_v15.py',
                root / 'brunner_base/src/p3d_simconnect_v15.py',
                Path(__file__).resolve().parents[1] / 'interfaces/brunner_base/src/p3d_simconnect_v15.py',
            ]
            source = next((p for p in candidates if p.is_file()), candidates[0])
            spec = importlib.util.spec_from_file_location('qtg_automatic_engine_p3d', source)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            factory = module.SimConnectClient
        sim = factory()
        controller = AutomaticEngine(context, sim, publish)
        controller.emit()
        thread = threading.Thread(target=sim.connect_and_run,
                                  kwargs=dict(place_at_rahwali=False, place_airborne=False), daemon=True)
        thread.start()
        if reader_factory is None:
            from interfaces.engine_measurements import CHANNELS, Measurements
            root = Path(sys._MEIPASS) if getattr(sys, 'frozen', False) else Path(__file__).resolve().parents[1]
            dll_candidates = [
                root / 'bridge/bin/qtg_simconnect.dll',
                root / 'qtg_simconnect.dll',
                Path(sys.executable).parent / 'bridge/bin/qtg_simconnect.dll',
                Path(sys.executable).parent / 'qtg_simconnect.dll',
                Path(__file__).resolve().parents[1] / 'bridge/bin/qtg_simconnect.dll',
            ]
            dll = next((p for p in dll_candidates if p.is_file()), dll_candidates[0])
            channels = CHANNELS + BODY_ACCELERATION_CHANNELS
            if not controller.deceleration:
                channels += (
                    ('fuel_flow_pph', 'RECIP ENG FUEL FLOW:1', 'pounds per hour'),
                )
            reader_factory = lambda: Measurements(dll, channels=channels)
        reader = reader_factory()
        from backend.parking_brake import GroundParkingBrake
        controller.ground_brake = GroundParkingBrake(reader.engage_parking_brake, require_stationary=False)
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
                previous_phase = controller.phase
                controller.step(reader.poll())
                if previous_phase == 'CONNECT' and controller.phase == 'PREPARE' and hasattr(reader, 'discard_pending'):
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
