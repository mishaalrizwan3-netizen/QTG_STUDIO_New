"""Automatic approach to stall: pitch up until stall warning, then return to neutral.

Only the pitch axis is controlled; the operator retains roll and rudder.
"""
import math
import time

from backend.automatic_control import AutomaticControl, _number, recorded_flight_fields
from backend.stall_events import StallWarningCycle

REF = '2.c.(8b)'
METHOD = 'stall'
PITCH_RATE_PER_SECOND = 0.20
FULL_BACK_POSITION = 1.0
NEUTRAL_TOLERANCE = 0.02


def selected(context):
    return context.get('mode') == 'auto' and context.get('test_reference') == REF and context.get('control_method') == METHOD


def _first(data, key):
    value = data.get(key)
    if isinstance(value, (list, tuple)) and value:
        try:
            value = float(value[0])
            return value if math.isfinite(value) else float('nan')
        except (ValueError, TypeError):
            pass
    return float('nan')


def _percent(sample, key):
    value = _number(sample, key)
    return value * 100.0 if math.isfinite(value) else float('nan')


def _acceleration(data, axis):
    # The existing bridge requests ACCELERATION BODY in feet per second squared.
    return _number(data, 'accel' + axis) / 32.1740485564


def _optional_axis(brunner, axis, force_scale):
    """Read an operator-owned axis without moving it or inventing missing data."""
    old = getattr(brunner, 'strict_io', False)
    try:
        brunner.strict_io = True
        values = brunner.read_axis(axis)
        position, force = float(values[0]), float(values[1])
        if not math.isfinite(position) or not math.isfinite(force):
            raise ValueError('Invalid axis reading')
        return position * 100., force * force_scale if abs(force) <= 5 else force
    except Exception:
        return float('nan'), float('nan')
    finally:
        brunner.strict_io = old


class AutomaticStall(AutomaticControl):
    def __init__(self, context, sim, brunner, publish, reader, *, clock=time.monotonic):
        if not selected(context):
            raise ValueError('Automatic Stall context required')
        super().__init__(dict(context, test_reference='2.a.(1)', control_method='static_control'),
                         sim, brunner, publish, clock=clock, live_hardware=True)
        self.context = dict(context)
        self.test_ref = REF
        self.reader = reader
        self.cycle = StallWarningCycle(max_age_s=1.0)
        self.axis_name = 'Pitch'
        self.sample_sequence = None
        self.last_frame_at = None
        self.ready_sample = None
        self.recovery = False
        self.warning_cleared_at = None
        self.last_tick = None
        self.owned = False
        self.mismatch_since = None
        self.idle_mismatch_since = None
        self.start_pos = 0.0
        self.position = 0.0

    def status(self):
        status = super().status()
        if self.phase == 'READY':
            sample = self.ready_sample
            fresh = False
            if sample is not None:
                try:
                    age = self.clock() - float(sample['receipt_time'])
                    fresh = 0 <= age <= 1.0 and float(sample['stall_warning']) == 0.0
                except (KeyError, TypeError, ValueError):
                    pass
            status['can_record'] = fresh
            if fresh:
                status['recording_text'] = 'READY TO RECORD'
                status['progress_text'] = 'Start Recording begins the stall approach. Operator controls roll and rudder.'
                status['details'] = 'Ready to record stall approach. Operator controls roll and rudder.'
            else:
                status['recording_text'] = 'Waiting for stall telemetry'
                status['progress_text'] = 'Waiting for fresh stall warning OFF readback.'
                status['details'] = status['progress_text']
        status['engine_text'] = 'Stall warning test'
        if hasattr(self, '_last_warning') and 'telemetry' in status:
            status['telemetry']['stall_warning'] = self._last_warning
        if hasattr(self, '_last_aoa') and 'telemetry' in status:
            status['telemetry']['aoa_deg'] = self._last_aoa
        return status

    def _request_paused_telemetry(self):
        # Initialization already requests fresh telemetry after positioning.
        # Avoid querying the asynchronous pause subscription before it is ready.
        return

    def command(self, command):
        action = command.get('action')
        if action in ('stop', 'shutdown', 'emergency_stop'):
            self.finish('ABORTED', 'Stopped by operator')
            if action == 'shutdown':
                self.closed = True
            return
        if action != 'record' or self.phase != 'READY':
            return
        try:
            if not self.sim.is_connected() or not self.brunner or not self.brunner.sock:
                raise RuntimeError('P3D and BRUNNER connections are required')
            sample = self.reader.poll()
            if sample is None:
                raise RuntimeError('Stall telemetry unavailable')
            if not 0 <= self.clock() - float(sample['receipt_time']) <= 1.0:
                raise RuntimeError('Stall telemetry is stale; wait for a fresh reading')
            if sample.get('stall_warning') == 1:
                raise ValueError('Stall warning is already active before the approach')
            self.cycle = StallWarningCycle(max_age_s=1.0)
            self.cycle.update(sample, self.clock())
            data = self.latest_sim or {}
            ias = _number(sample, 'ias_kt')
            if not math.isfinite(ias):
                ias = _number(data, 'airspeed_indicated', 'indicated_airspeed')
            pitch = _number(data, 'pitch_angle')
            bank = _number(data, 'bank_angle')
            alt = _number(sample, 'altitude_ft')
            if not math.isfinite(alt):
                alt = _number(data, 'indicated_altitude', 'pressure_alt')
            if not all(math.isfinite(v) for v in (ias, alt, pitch, bank)):
                raise RuntimeError('Fresh airspeed, altitude and attitude are required')

            # Only the pitch axis is automated; the pilot retains roll and rudder.
            self.brunner.strict_io = True
            axes = self.brunner.read_axis(self.brunner.AXIS_PITCH)
            start = float(axes[0])
            if not math.isfinite(start):
                raise RuntimeError('Pitch control position unavailable')
            self.owned = True
            self.brunner.configure_replay_mode(self.brunner.AXIS_PITCH)
            self.start_pos = start
            self.position = start
            self.initial_ias, self.initial_altitude = ias, alt
            # Capture the measured setup before idle throttle begins the maneuver.
            initial_row = {
                'Time (s)': 0.0,
                **recorded_flight_fields(data),
                'Indicated Airspeed (kts)': ias,
                'Pressure Alt (ft)': alt,
                'Aircraft Angle of Attack (deg)': _number(sample, 'aoa_deg'),
                'Elevator Surface (rad)': _number(data, 'surface_elevator_rad', 'elevator_deflection'),
                'Stall Warning (0/1)': sample['stall_warning'],
                'Fuel Flow (pph)': _number(sample, 'fuel_flow_pph') if math.isfinite(_number(sample, 'fuel_flow_pph')) else _number(data, 'fuel_flow_pph', 'fuelFlow'),
                'Height of Terrain (ft)': _number(sample, 'terrain_ft') if math.isfinite(_number(sample, 'terrain_ft')) else _number(data, 'ground_alt'),
                'CG Longitudinal (% reference chord)': _percent(sample, 'cg_long_fraction'),
                'CG Lateral (% reference chord)': _percent(sample, 'cg_lateral_fraction'),
                'Joystick Pitch Position (mm)': float(axes[0]) * 100.,
                'Joystick Pitch Force (N)': float(axes[1]) * 100. if abs(axes[1]) <= 5 else float(axes[1]),
            }
            initial_row['Pitch Applied Force (N)'] = initial_row['Joystick Pitch Force (N)']
            roll_pos, roll_force = _optional_axis(self.brunner, self.brunner.AXIS_ROLL, 60.)
            yaw_pos, yaw_force = _optional_axis(self.brunner, self.brunner.AXIS_RUDDER, 80.)
            initial_row.update({'Joystick Roll Position (mm)': roll_pos,
                                'Joystick Roll Force (N)': roll_force,
                                'Rudder Pedal Position (mm)': yaw_pos,
                                'Rudder Pedal Force (N)': yaw_force})
            self.recovery = False
            self.warning_cleared_at = None

            if hasattr(self.sim, 'unpause_sim'):
                self.sim.unpause_sim()
            if not self.sim.set_replay_throttle(0.):
                raise RuntimeError('P3D idle throttle command rejected')
            self.t_start = self.last_tick = self.clock()
            self.last_frame_at = self.t_start
            self.rows.append(initial_row)
            self.unpausing_until = self.t_start + 2.0
            self.sample_sequence = sample.get('sequence')
            self.phase, self.stage = 'RECORD', 'RECORDING'
            self.details = 'Idle throttle; pitching up towards stall. Operator controls roll and rudder.'
        except Exception as exc:
            self.finish('FAILED', str(exc))
        self.emit()

    def step(self):
        if self.phase != 'RECORD':
            if self.phase == 'READY':
                try:
                    sample = self.reader.poll()
                    if sample is not None:
                        self.last_frame_at = sample['receipt_time']
                        self.ready_sample = sample
                except Exception as exc:
                    self.finish('FAILED', str(exc))
                    return
                self._maybe_emit(self.clock())
                return
            return super().step()
        now = self.clock()
        try:
            if not self.sim.is_connected():
                raise RuntimeError('P3D disconnected during recording')
            if now > getattr(self, 'unpausing_until', 0.0) and hasattr(self.sim, 'simulation_running') and not self.sim.simulation_running():
                raise RuntimeError('P3D disconnected or paused during recording')
            sample = self.reader.poll()
            if sample is None or sample['sequence'] == self.sample_sequence:
                watchdog_limit = 2.0 if now <= getattr(self, 'unpausing_until', 0.0) else 0.5
                if now - self.last_frame_at > watchdog_limit:
                    raise RuntimeError('Stall telemetry stopped advancing')
                return
            self.sample_sequence = sample['sequence']
            self.last_frame_at = sample['receipt_time']
            self._last_warning = sample['stall_warning']
            self._last_aoa = _number(sample, 'aoa_deg')
            now = self.clock()
            event = self.cycle.update(sample, now)
            sequence = self.sim.get_telemetry_sequence()
            self.sim.request_telemetry_once()
            timeout = 1.0 if now <= getattr(self, 'unpausing_until', 0.0) else 0.3
            data = self.sim.wait_for_fresh_telemetry(sequence, timeout)
            if data is None:
                raise RuntimeError('Flight telemetry stopped advancing')
            self.latest_sim = data
            pitch = _number(data, 'pitch_angle')
            bank = _number(data, 'bank_angle')
            ias = _number(sample, 'ias_kt')
            altitude = _number(sample, 'altitude_ft')
            throttle = _number(data, 'engine_throttle_percent')
            if not all(math.isfinite(x) for x in (pitch, bank, ias, altitude, throttle)):
                raise RuntimeError('Flight feedback invalid')
            # Super Mushak idle throttle stop is 5.0% (offline unit tests mock 0.0%)
            idle_reached = throttle <= 7.0 or abs(throttle - 5.0) <= 2.0 or abs(throttle) <= 2.0
            if not idle_reached:
                self.idle_mismatch_since = now if self.idle_mismatch_since is None else self.idle_mismatch_since
                if now - self.idle_mismatch_since > 3.:
                    raise RuntimeError('Sim Flight Model throttle did not reach idle')
            else:
                self.idle_mismatch_since = None
            axes = self.brunner.read_axis(self.brunner.AXIS_PITCH)
            self.latest_brunner = {'pos': axes[0], 'force': axes[1], 'vf': axes[2]}
            now = self.clock()
            if not 0 <= now - float(sample['receipt_time']) <= 1.0:
                raise RuntimeError('Stall telemetry became stale during flight feedback')
            dt = now - self.last_tick
            self.last_tick = now
            if not 0 < dt <= .35:
                raise RuntimeError('Stall pitch control feedback stopped advancing')

            elapsed = now - self.t_start
            if event == 'WARNING' or sample.get('stall_warning') == 1:
                self.recovery = True

            if self.recovery and sample['stall_warning'] == 0:
                if self.warning_cleared_at is None:
                    self.warning_cleared_at = now
            else:
                self.warning_cleared_at = None

            if self.recovery:
                self.details = 'Stall warning observed; returning pitch to neutral with idle throttle.'
                target = 0.0
            else:
                target = FULL_BACK_POSITION
                self.details = f'Pulling pitch back toward stall warning; IAS {ias:.1f} kt. Operator controls roll and rudder.'
            travel = min(PITCH_RATE_PER_SECOND * dt, abs(target - self.position))
            if travel:
                self.position += math.copysign(travel, target - self.position)

            elevator_surf = _number(data, 'surface_elevator_rad', 'elevator_deflection')
            roll_pos, roll_force = _optional_axis(self.brunner, self.brunner.AXIS_ROLL, 60.)
            yaw_pos, yaw_force = _optional_axis(self.brunner, self.brunner.AXIS_RUDDER, 80.)
            if now - self.t_start > 90.:
                raise RuntimeError('Stall approach timed out before completing warning cycle')
            if abs(self.position - axes[0]) > 0.35:
                self.mismatch_since = now if self.mismatch_since is None else self.mismatch_since
                if now - self.mismatch_since > 3.0:
                    raise RuntimeError('Pitch loader did not track the command')
            else:
                self.mismatch_since = None

            self.rows.append({
                'Time (s)': now - self.t_start,
                'Indicated Airspeed (kts)': ias,
                'Indicated Altitude (ft)': _number(data, 'indicated_altitude'),
                'Pressure Alt (ft)': altitude,
                'Pitch Angle (deg)': pitch,
                'Bank Angle (deg)': bank,
                'Magnetic Heading (deg)': _number(data, 'magnetic_heading'),
                'Rate of Climb/Descent (fpm)': _number(data, 'vertical_speed'),
                'Aircraft Angle of Attack (deg)': _number(sample, 'aoa_deg'),
                'Elevator Surface (rad)': elevator_surf,
                'Stall Warning (0/1)': sample['stall_warning'],
                'Throttle 1 Position (%)': throttle,
                'Engine 1 RPM': _number(data, 'engine_rpm'),
                'Propeller 1 RPM': _first(data, 'prop_rpms'),
                'Fuel Flow (pph)': _number(sample, 'fuel_flow_pph'),
                'Height of Terrain (ft)': _number(sample, 'terrain_ft'),
                'Gross Wt (kg)': _number(data, 'total_weight'),
                'Total Fuel Quantity (kg)': _number(data, 'total_fuel_quantity'),
                'CG Longitudinal (% reference chord)': _percent(sample, 'cg_long_fraction'),
                'CG Lateral (% reference chord)': _percent(sample, 'cg_lateral_fraction'),
                'Baro Press (hPa)': _number(data, 'baro_pressure'),
                'OAT (deg C)': _number(data, 'ambient_temp'),
                'Wind Dir (mag,deg)': _number(data, 'ambient_wind_dir'),
                'Wind Spd (kt)': _number(data, 'ambient_wind_spd'),
                'EGT (deg C)': _number(data, 'egt_c', 'egt'),
                'Ground Speed (kt)': _number(data, 'ground_speed'),
                'Mach Number (-)': _number(data, 'mach_number'),
                'Radar Alt (ft)': _number(data, 'radar_alt'),
                'Sideslip Angle (deg)': _number(data, 'sideslip'),
                'Track Angle (deg)': _number(data, 'track_angle'),
                'Roll Rate (deg/s)': _number(data, 'roll_rate'),
                'Pitch Rate (deg/s)': _number(data, 'pitch_rate'),
                'Yaw Rate (deg/s)': _number(data, 'yaw_rate'),
                'X Body Acceleration (g)': _acceleration(data, 'X'),
                'Y Body Acceleration (g)': _acceleration(data, 'Y'),
                'Z Body Acceleration (g)': _acceleration(data, 'Z'),
                'Joystick Pitch Position (mm)': axes[0] * 100.,
                'Joystick Pitch Force (N)': axes[1] * 100. if abs(axes[1]) <= 5 else axes[1],
                'Pitch Applied Force (N)': axes[1] * 100. if abs(axes[1]) <= 5 else axes[1],
                'Joystick Roll Position (mm)': roll_pos,
                'Joystick Roll Force (N)': roll_force,
                'Rudder Pedal Position (mm)': yaw_pos,
                'Rudder Pedal Force (N)': yaw_force,
            })
            if self.clock() - sample['receipt_time'] > 1. or self.clock() - now > .35:
                raise RuntimeError('Control telemetry became stale before pitch command')
            self.brunner.move_actuators(self.position, None, None)
            if (self.warning_cleared_at is not None
                    and now - self.warning_cleared_at >= 2.0
                    and abs(self.position) <= NEUTRAL_TOLERANCE
                    and abs(float(axes[0])) <= NEUTRAL_TOLERANCE):
                self.finish('COMPLETE', 'Stall warning cleared; pitch returned to neutral')
                return
        except Exception as exc:
            self.finish('FAILED', str(exc))
        finally:
            self._maybe_emit(now)

    def _abort_movement(self, reason):
        self.finish('FAILED', reason)

    def _release_pitch(self):
        pitch_axis = self.brunner.AXIS_PITCH
        self.brunner.write_setting(pitch_axis, 0x70, 'B', 0)
        self.brunner.write_setting(pitch_axis, 0xA0, 'B', 0)
        if not self.brunner.set_overrides(pitch_axis, False):
            raise RuntimeError('BRUNNER pitch override release was not acknowledged')
        if not self.brunner.set_position_forwarding():
            raise RuntimeError('BRUNNER position forwarding was not acknowledged')

    def finish(self, result, reason):
        if self.phase == 'DONE':
            return
        self.phase, self.result, self.stage, self.details = 'DONE', result, result, reason
        if self.owned:
            try:
                self._release_pitch()
            except Exception as exc:
                result = 'FAILED'
                self.result = result
                self.stage, self.details = 'FAILED', reason + '; release failed: ' + str(exc)
            self.owned = False
        if self.brunner is not None:
            self.brunner.strict_io = False
        try:
            self.sim.pause_sim()
        except Exception:
            pass
        if self.rows:
            try:
                from reports.automatic_control_report import save
                self.outputs = save(self.context, self.rows, result, self.details)
                if result == 'COMPLETE' and self.stage != 'FAILED':
                    self.stage = 'Completed'
            except Exception as exc:
                self.stage, self.details = 'FAILED', reason + '; report error: ' + str(exc)
        self.emit()


def run(context, publish, *, sim_factory=None, brunner_factory=None, reader_factory=None, sleep=time.sleep):
    import json
    from pathlib import Path
    import sys
    import threading
    from interfaces.brunner_base.src.p3d_simconnect_v15 import SimConnectClient
    from interfaces.brunner_base.src.brunner_interface import BrunnerInterface
    from interfaces.engine_measurements import Measurements
    from backend.stall_measurements import CHANNELS
    sim = brunner = reader = thread = controller = None
    try:
        sim = (sim_factory or SimConnectClient)()
        thread = threading.Thread(target=sim.connect_and_run,
            kwargs={'place_at_rahwali': False, 'place_airborne': False}, daemon=True)
        thread.start()
        brunner = (brunner_factory or (lambda: BrunnerInterface(timeout=.02)))()
        if not getattr(brunner, 'sock', None) and not brunner.connect():
            raise RuntimeError('CLS2Sim connection unavailable')
        if reader_factory is None:
            root = Path(sys._MEIPASS) if getattr(sys, 'frozen', False) else Path(__file__).resolve().parents[1]
            dll_candidates = [
                root / 'bridge/bin/qtg_simconnect.dll',
                root / 'qtg_simconnect.dll',
                Path(sys.executable).parent / 'bridge/bin/qtg_simconnect.dll',
                Path(sys.executable).parent / 'qtg_simconnect.dll',
                Path(__file__).resolve().parents[1] / 'bridge/bin/qtg_simconnect.dll',
            ]
            dll = next((p for p in dll_candidates if p.is_file()), dll_candidates[0])
            reader_factory = lambda: Measurements(dll, channels=CHANNELS)
        reader = reader_factory()
        controller = AutomaticStall(context, sim, brunner, publish, reader)
        controller.emit()
        cmd_dir = context.get('command_directory')
        directory = Path(cmd_dir) if cmd_dir else None
        stop_file = context.get('stop_file')
        stop = Path(stop_file) if stop_file else None
        while not controller.closed:
            commands = []
            if directory and directory.exists():
                for path in sorted(directory.glob('*.json')):
                    try:
                        commands.append(json.loads(path.read_text(encoding='utf-8')))
                        path.unlink(missing_ok=True)
                    except Exception:
                        pass
            if stop and stop.exists():
                try:
                    stop.unlink(missing_ok=True)
                except Exception:
                    pass
                commands.insert(0, {'action': 'stop'})
            stopping = next((cmd for cmd in commands if cmd.get('action') in ('stop', 'shutdown', 'emergency_stop')), None)
            for cmd in ([stopping] if stopping else commands):
                controller.command(cmd)
            if not controller.closed:
                controller.step()
                sleep(.025)
        return 0
    except Exception as exc:
        if controller:
            controller.finish('FAILED', str(exc))
        else:
            publish(dict(context, stage='FAILED', details=str(exc), can_record=False, outputs=[]))
        return 1
    finally:
        if reader:
            reader.close()
        if controller and controller.owned:
            try:
                controller._release_pitch()
            except Exception:
                pass
        if brunner and getattr(brunner, 'sock', None):
            brunner.sock.close()
            brunner.sock = None
        if sim:
            sim.stop()
        if thread:
            thread.join(timeout=3)
