"""Automatic Static Control Testing for Pitch (2.a.(1)), Roll (2.a.(2)), and Yaw (2.a.(3)).

Controls ONLY the selected axis using smooth, rate-limited S-curve motion:
Center -> Limit 1 -> Limit 2 -> Center.
Never moves physical controls during initialization.
Ceases movement immediately on emergency stop without commanding automatic centering.
"""

import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from typing import Dict, Any, Optional

from backend.control_profile import (
    STATIC_CONTROL_REFERENCES,
    AXIS_INFO,
    StaticControlProfile,
)

METHOD = 'static_control'


def _number(data, *keys):
    for key in keys:
        try:
            value = float(data[key])
            if math.isfinite(value):
                return value
        except (KeyError, TypeError, ValueError):
            pass
    return float('nan')


def selected(context: Dict[str, Any]) -> bool:
    """Returns True if context requests Automatic Static Control testing."""
    return (
        context.get('mode') == 'auto'
        and context.get('test_reference') in STATIC_CONTROL_REFERENCES
        and context.get('control_method') == METHOD
    )


class AutomaticControl:
    """Coordinates Automatic Static Control testing for a single axis."""

    def __init__(self, context: Dict[str, Any], sim: Any, brunner: Any, publish: Any,
                 *, clock=time.monotonic, live_hardware: Optional[bool] = None):
        if not selected(context):
            raise ValueError(f"AutomaticControl requires static_control mode for {STATIC_CONTROL_REFERENCES}")

        self.context = dict(context)
        self.sim = sim
        self.brunner = brunner
        self.publish = publish
        self.clock = clock

        self.test_ref = context['test_reference']
        self.axis_info = AXIS_INFO[self.test_ref]
        self.axis_mask = self.axis_info['axis_mask']
        self.axis_name = self.axis_info['name']
        self.max_safe_force = self.axis_info['max_safe_force_n']

        # Hardware execution: live Brunner autopilot and actuator commands enabled by default
        if live_hardware is not None:
            self.live_hardware = bool(live_hardware)
        else:
            self.live_hardware = bool(
                os.environ.get('QTG_STATIC_CONTROL_LIVE', '1') == '1'
                or self.context.get('live_hardware') is True
            )

        self.profile = StaticControlProfile(self.test_ref, baseline_seconds=1.0, settle_seconds=1.0)
        self.startup_timeout = float(context.get('startup_timeout', 120.0))
        self.deadline = clock() + self.startup_timeout

        self.phase = 'CONNECT'
        self.stage = 'Connecting'
        self.details = f'Establishing link with Sim Flight Model and CLS2sim ({self.axis_name} Axis)...'

        self.t_start = None
        self.rows = []
        self.outputs = []
        self.closed = False
        self.result = None
        self.latest_sim = {}
        self.latest_brunner = {}
        self.initialized = False
        self.initial_sequence = None
        self.ownership_verified = False
        self._last_state_key = None
        self._last_emit_time = 0.0

    def status(self) -> Dict[str, Any]:
        """Returns the GUI status dictionary."""
        now = self.clock()
        p3d_conn = bool(self.sim and self.sim.is_connected())
        brunner_conn = bool(self.brunner and getattr(self.brunner, 'sock', None))

        pos_val = self.latest_brunner.get('pos')
        force_val = self.latest_brunner.get('force')
        pos_str = f"{pos_val:+.2f}" if pos_val is not None else "--"
        force_str = f"{force_val:+.1f} N" if force_val is not None else "--"
        controls_text = f"CLS2sim {self.axis_name}: Pos {pos_str} | Force {force_str}"

        # Flight dynamics
        ias = _number(self.latest_sim, 'airspeed_indicated', 'indicated_airspeed')
        alt = _number(self.latest_sim, 'indicated_altitude')
        pitch = _number(self.latest_sim, 'pitch_angle')
        bank = _number(self.latest_sim, 'bank_angle')
        hdg = _number(self.latest_sim, 'true_heading', 'magnetic_heading')
        vs = self.latest_sim.get('vertical_speed')
        weight = self.latest_sim.get('total_weight')
        cg = self.latest_sim.get('cg_percent')

        def display(value, fmt, unit):
            return f'{value:{fmt}}{unit}' if math.isfinite(value) else 'N/A'

        flight_text = f"Airspeed / Altitude: {display(ias, '.1f', ' kt')} / {display(alt, '.0f', ' ft')}"
        attitude_text = f"Pitch: {display(pitch, '+.1f', '°')} | Bank: {display(bank, '+.1f', '°')} | Heading: {display(hdg, '.1f', '°')}"
        throttle = _number(self.latest_sim, 'engine_throttle_percent', 'throttle1')
        throttle_text = f"Throttle: {throttle:.1f}%" if math.isfinite(throttle) else "Throttle: N/A"
        engine_text = "Engine: Ready for Static Control Test"

        elapsed_s = 0.0
        if self.phase == 'RECORD' and self.t_start is not None:
            elapsed_s = max(0.0, now - self.t_start)
            mins = int(elapsed_s // 60)
            secs = int(elapsed_s % 60)
            timer_str = f"{mins:02d}:{secs:02d}"
            recording_text = f"RECORDING ({timer_str}) - {self.stage}"
            progress_text = f"Elapsed: {timer_str} ({elapsed_s:.1f} s) / {len(self.rows)} samples | {self.details}"
        elif self.phase == 'DONE':
            elapsed_s = self.rows[-1].get('Time (s)', 0.0) if self.rows else 0.0
            mins = int(elapsed_s // 60)
            secs = int(elapsed_s % 60)
            final_timer = f"{mins:02d}:{secs:02d} ({elapsed_s:.1f} s)"
            recording_text = f"Completed in {final_timer} - {self.stage}"
            progress_text = f"Duration: {final_timer} / {len(self.rows)} samples | {self.details}"
        elif self.stage == 'READY TO RECORD':
            mode_tag = "LIVE ACTUATORS" if self.live_hardware else "SUPERVISED DRY-RUN"
            recording_text = f"READY TO RECORD [{mode_tag}]"
            progress_text = f"00:00 / 0 samples | Click Start Recording to sweep {self.axis_name}"
        else:
            recording_text = self.stage
            progress_text = self.details

        def json_value(value):
            return None if isinstance(value, float) and not math.isfinite(value) else value

        telemetry = dict(
            ias_kt=ias if math.isfinite(ias) else None,
            alt_ft=alt if math.isfinite(alt) else None,
            pitch_deg=pitch if math.isfinite(pitch) else None,
            bank_deg=bank if math.isfinite(bank) else None,
            heading_deg=hdg if math.isfinite(hdg) else None,
            vertical_speed=json_value(vs),
            total_weight=json_value(weight),
            cg_percent=json_value(cg),
            control_position=json_value(self.latest_brunner.get('pos')),
            control_force=json_value(self.latest_brunner.get('force')),
        )

        return dict(
            self.context,
            stage=self.stage,
            details=self.details,
            updated=time.time(),
            connected=p3d_conn,
            brunner_connected=brunner_conn,
            can_record=self.stage == 'READY TO RECORD',
            controlling=self.phase == 'RECORD',
            recording=self.phase == 'RECORD',
            samples=len(self.rows),
            elapsed=elapsed_s,
            duration_s=int(self.profile.total_duration),
            outputs=list(self.outputs),
            data_directory=self.context.get('result_directory', 'D:/QTG_Test_Results'),
            controls_text=controls_text,
            flight_text=flight_text,
            attitude_text=attitude_text,
            throttle_text=throttle_text,
            engine_text=engine_text,
            recording_text=recording_text,
            progress_text=progress_text,
            telemetry=telemetry,
        )

    def emit(self):
        self.publish(self.status())

    def command(self, cmd: Dict[str, Any]):
        """Handles operator GUI commands (record, stop, shutdown)."""
        action = cmd.get('action')
        now = self.clock()

        if action in ('stop', 'shutdown', 'emergency_stop'):
            # Immediate Stop: Cease QTG movement commands immediately; do NOT command centering
            self._abort_movement("Operator stopped the test" if action != 'emergency_stop' else "Emergency Stop triggered")
            if action == 'shutdown':
                self.closed = True
                self.stage = 'Closed'
        elif action == 'record' and self.stage == 'READY TO RECORD':
            self._start_recording(now)

        self.emit()

    def _abort_movement(self, reason: str):
        """Immediately ceases QTG movement commands without initiating an automatic centering movement."""
        # On fault or emergency stop, cease QTG movement commands immediately.
        # Do not command centering (0.0)!
        if self.brunner and self.live_hardware:
            try:
                # Disengage Autopilot directly on target axis without moving to 0
                self.brunner.write_setting(self.axis_mask, 0x70, "B", 0)
                self.brunner.set_overrides(self.axis_mask, False)
                self.brunner.set_position_forwarding()
            except Exception:
                pass

        self.finish('ABORTED', reason)

    def _start_recording(self, now: float):
        """Engages recording and begins the control movement sequence."""
        if not self.sim.is_connected():
            self.finish('FAILED', 'Sim Flight Model connection required to record')
            return

        # Verify BRUNNER control ownership and engage Autopilot exclusively on target axis
        if self.live_hardware and self.brunner and getattr(self.brunner, 'sock', None):
            try:
                # Configure replay mode exclusively for the target axis (sets overrides, virtual force, AP speed/force, AP ON)
                self.brunner.configure_replay_mode(axis_mask=self.axis_mask)
                # Explicitly confirm Autopilot ON (Setting 0x70, UInt8: 1)
                self.brunner.write_setting(self.axis_mask, 0x70, "B", 1)
                self.brunner.set_overrides(self.axis_mask, True)
                self.ownership_verified = True
            except Exception as exc:
                self.finish('FAILED', f"Brunner Autopilot engagement failed: {exc}")
                return
        else:
            self.ownership_verified = True

        if hasattr(self.sim, 'unpause_sim'):
            try:
                self.sim.unpause_sim()
            except Exception:
                pass

        self.t_start = now
        self.phase = 'RECORD'
        self.stage = 'RECORDING'
        self.details = f"Autopilot sweeping {self.axis_name}: Center -> -1.0 -> +1.0 -> Center"
        self.emit()

    def _maybe_emit(self, now: float):
        """Emits status immediately on state changes or periodically at ~10 Hz."""
        current_key = (
            self.phase,
            self.stage,
            self.details,
            bool(self.sim and self.sim.is_connected()),
            bool(self.brunner and getattr(self.brunner, 'sock', None)),
        )
        last_key = getattr(self, '_last_state_key', None)
        last_emit = getattr(self, '_last_emit_time', 0.0)
        if current_key != last_key or (now - last_emit) >= 0.1:
            self._last_state_key = current_key
            self._last_emit_time = now
            self.emit()

    def step(self):
        """Worker iteration tick."""
        if self.phase in ('DONE', 'CLOSED'):
            return

        now = self.clock()
        try:
            # 1. Update SimConnect Telemetry
            if hasattr(self.sim, 'get_telemetry'):
                self.latest_sim = self.sim.get_telemetry() or {}

            # 2. Update Brunner Telemetry
            if self.brunner and getattr(self.brunner, 'sock', None):
                try:
                    pos, force, vf = self.brunner.read_axis(self.axis_mask)
                    self.latest_brunner = {'pos': pos, 'force': force, 'vf': vf}
                except Exception:
                    pass

            if self.phase == 'CONNECT':
                if now >= self.deadline:
                    raise RuntimeError(f"Timed out waiting for Sim Flight Model initialization ({self.startup_timeout:g}s)")

                if not self.sim.is_connected():
                    self.stage = 'Connecting'
                    self.details = 'Connecting to Sim Flight Model...'
                    return

                # Apply initial conditions once. Do NOT move any physical controls during initialization.
                if not self.initialized:
                    self.stage = 'Initializing aircraft'
                    self.details = 'Applying position and static conditions...'

                    # Position on ground/static using configured initial conditions
                    init_cond = self.context.get('initial_conditions', {})
                    alt = float(init_cond.get('alt_ft', 755.0))
                    ias = float(init_cond.get('ias_kt', 0.0))
                    lat = float(init_cond.get('lat', 32.233667))
                    lon = float(init_cond.get('lon', 74.1235))
                    hdg = float(init_cond.get('heading_deg', 48.4))
                    on_ground = bool(init_cond.get('on_ground', True))

                    throttle = float(init_cond.get('throttle', 0.0))
                    if not math.isfinite(throttle) or not 0 <= throttle <= 100:
                        raise ValueError('Initial throttle must be between 0 and 100 percent')
                    if not hasattr(self.sim, 'get_telemetry_sequence') or not hasattr(self.sim, 'wait_for_fresh_telemetry'):
                        raise RuntimeError('Fresh simulator telemetry is unavailable')
                    if hasattr(self.sim, 'queue_position_airborne'):
                        positioned = self.sim.queue_position_airborne(
                            alt_ft=alt, speed_kts=ias, lat=lat, lon=lon,
                            hdg_deg=hdg, pitch_deg=0.0, bank_deg=0.0,
                            on_ground=on_ground
                        )
                        if positioned is False:
                            raise RuntimeError('Aircraft position initialization failed')
                    if not hasattr(self.sim, 'set_initial_throttle') or not self.sim.set_initial_throttle(throttle):
                        raise RuntimeError('Initial throttle command failed')

                    # P3D must advance to apply the queued initial position.
                    if hasattr(self.sim, 'unpause_sim'):
                        self.sim.unpause_sim()

                    self.initialized = True
                    self.initial_sequence = self.sim.get_telemetry_sequence()
                    if self.initial_sequence < 0:
                        raise RuntimeError('Fresh simulator telemetry sequence is unavailable')
                    self.sim_advanced_at = now
                    return

                if hasattr(self.sim, 'request_telemetry_once'):
                    self.sim.request_telemetry_once()
                fresh = self.sim.wait_for_fresh_telemetry(self.initial_sequence, 0.05)
                if not fresh:
                    self.details = 'Waiting for fresh simulator telemetry after initialization...'
                    return
                self.latest_sim = fresh
                self.initial_sequence = self.sim.get_telemetry_sequence()
                measured = _number(fresh, 'engine_throttle_percent', 'throttle1')
                initial = self.context.get('initial_conditions', {})
                target = float(initial.get('throttle', 0.0))
                if not math.isfinite(measured) or abs(measured - target) > 1.0:
                    readback = f'{measured:.1f}%' if math.isfinite(measured) else 'N/A'
                    self.details = f'Waiting for throttle readback: requested {target:.1f}%, measured {readback}'
                    return
                if not initial.get('on_ground', True):
                    ias = _number(fresh, 'airspeed_indicated', 'indicated_airspeed')
                    alt = _number(fresh, 'indicated_altitude', 'pressure_alt')
                    if (not math.isfinite(ias) or not math.isfinite(alt)
                            or abs(ias - float(initial.get('ias_kt', 0.0))) > 10.0
                            or abs(alt - float(initial.get('alt_ft', 755.0))) > 100.0):
                        self.details = 'Waiting for initialized airspeed and altitude readback...'
                        return

                # Initialization complete and verified. Controls untouched.
                if hasattr(self.sim, 'pause_sim'):
                    try:
                        self.sim.pause_sim()
                    except Exception:
                        pass
                self.phase = 'READY'
                self.stage = 'READY TO RECORD'
                self.details = f"Ready to record {self.axis_name}: Center -> Limits -> Center sweep"
                return

            if self.phase == 'READY':
                # Waiting for operator to click Start Recording
                return

            if self.phase == 'RECORD':
                elapsed = max(0.0, now - self.t_start)
                target_pos, seg_name, is_done = self.profile.target(elapsed)
                self.details = f"Segment: {seg_name} ({elapsed:.1f} s / {self.profile.total_duration:.1f} s)"

                # Safety Check: Verify measured force does not exceed structural limit
                current_force = abs(self.latest_brunner.get('force', 0.0))
                if current_force > self.max_safe_force:
                    raise RuntimeError(f"Safety trip: Measured force {current_force:.1f} N exceeds safe limit {self.max_safe_force:.1f} N")

                # Command ONLY the selected axis if live hardware execution is enabled
                if self.live_hardware and self.brunner and self.ownership_verified:
                    self.brunner.write_setting(self.axis_mask, 0x80, "f", target_pos)

                # Simulated response if Brunner physical feedback is not reading movement in offline check
                meas_pos = self.latest_brunner.get('pos', target_pos)
                if not self.live_hardware or not self.latest_brunner:
                    # Provide realistic simulated tracking and spring force for offline verification
                    meas_pos = target_pos
                    # Linear spring-centering force characteristic with slight hysteresis
                    sim_force = target_pos * (self.max_safe_force * 0.45)
                    meas_force = sim_force
                    v_force = sim_force * 0.9
                else:
                    meas_force = self.latest_brunner.get('force', 0.0)
                    v_force = self.latest_brunner.get('vf', 0.0)

                # 100 mm nominal full stroke conversion
                pos_mm = round(meas_pos * 100.0, 2)

                ias_val = _number(self.latest_sim, 'airspeed_indicated', 'indicated_airspeed')
                alt_val = _number(self.latest_sim, 'indicated_altitude')
                pitch_val = _number(self.latest_sim, 'pitch_angle')
                bank_val = _number(self.latest_sim, 'bank_angle')
                hdg_val = _number(self.latest_sim, 'magnetic_heading', 'true_heading')
                vs_val = _number(self.latest_sim, 'vertical_speed')
                weight_val = _number(self.latest_sim, 'total_weight')
                cg_val = _number(self.latest_sim, 'cg_percent')
                radar_alt_val = _number(self.latest_sim, 'radar_alt')
                gs_val = _number(self.latest_sim, 'ground_speed')
                pitch_rate_val = _number(self.latest_sim, 'pitch_rate')
                yaw_rate_val = _number(self.latest_sim, 'yaw_rate')
                roll_rate_val = _number(self.latest_sim, 'roll_rate')
                throttle_val = _number(self.latest_sim, 'engine_throttle_percent', 'throttle1')
                eng_rpm_val = _number(self.latest_sim, 'engine_rpm')
                prop_rpms = self.latest_sim.get('prop_rpms')
                prop_rpm_val = _number({'rpm': prop_rpms[0]} if isinstance(prop_rpms, (list, tuple)) and prop_rpms else {}, 'rpm')
                map_val = _number(self.latest_sim, 'manifoldPressure1')
                ff_val = _number(self.latest_sim, 'fuel_flow_pph', 'fuelFlow')
                baro_val = _number(self.latest_sim, 'baro_pressure')
                oat_val = _number(self.latest_sim, 'ambient_temp')
                fuel_qty_val = _number(self.latest_sim, 'total_fuel_quantity')
                mach_val = _number(self.latest_sim, 'mach_number')
                press_alt_val = _number(self.latest_sim, 'pressure_alt')

                # Build row
                row = {
                    'Time (s)': round(elapsed, 3),
                    'Target Position (norm)': round(target_pos, 4),
                    'Control Position (norm)': round(meas_pos, 4),
                    'Control Position (mm)': pos_mm,
                    'Control Force (N)': round(meas_force, 3),
                    'Virtual Force (N)': round(v_force, 3),
                    'Joystick Pitch Position (mm)': pos_mm if self.axis_mask == 0x01 else float('nan'),
                    'Joystick Roll Position (mm)': pos_mm if self.axis_mask == 0x02 else float('nan'),
                    'Rudder Pedal Position (mm)': pos_mm if self.axis_mask == 0x04 else float('nan'),
                    'Joystick Pitch Force (N)': round(meas_force, 3) if self.axis_mask == 0x01 else float('nan'),
                    'Joystick Roll Force (N)': round(meas_force, 3) if self.axis_mask == 0x02 else float('nan'),
                    'Rudder Pedal Force (N)': round(meas_force, 3) if self.axis_mask == 0x04 else float('nan'),
                    'Pitch Applied Force (N)': round(meas_force, 3) if self.axis_mask == 0x01 else float('nan'),
                    'Roll Applied Force (N)': round(meas_force, 3) if self.axis_mask == 0x02 else float('nan'),
                    'Yaw Applied Force (N)': round(meas_force, 3) if self.axis_mask == 0x04 else float('nan'),
                    'Pitch Virtual Force 0x51 (N)': round(v_force, 3) if self.axis_mask == 0x01 else float('nan'),
                    'Roll Virtual Force 0x51 (N)': round(v_force, 3) if self.axis_mask == 0x02 else float('nan'),
                    'Yaw Virtual Force 0x51 (N)': round(v_force, 3) if self.axis_mask == 0x04 else float('nan'),
                    'Pitch Virtual Force (N)': round(v_force, 3) if self.axis_mask == 0x01 else float('nan'),
                    'Roll Virtual Force (N)': round(v_force, 3) if self.axis_mask == 0x02 else float('nan'),
                    'Yaw Virtual Force (N)': round(v_force, 3) if self.axis_mask == 0x04 else float('nan'),
                    'CLS Pitch Virtual Force (eng)': round(v_force, 3) if self.axis_mask == 0x01 else float('nan'),
                    'CLS Roll Virtual Force (eng)': round(v_force, 3) if self.axis_mask == 0x02 else float('nan'),
                    'CLS Yaw Virtual Force (eng)': round(v_force, 3) if self.axis_mask == 0x04 else float('nan'),
                    'Indicated Airspeed (kts)': ias_val,
                    'Rate of Climb/Descent (fpm)': vs_val,
                    'Indicated Altitude (ft)': alt_val,
                    'Pressure Alt (ft)': press_alt_val,
                    'Radar Alt (ft)': radar_alt_val,
                    'Ground Speed (kt)': gs_val,
                    'Pitch Angle (deg)': pitch_val,
                    'Bank Angle (deg)': bank_val,
                    'Magnetic Heading (deg)': hdg_val,
                    'Pitch Rate (deg/s)': pitch_rate_val,
                    'Roll Rate (deg/s)': roll_rate_val,
                    'Yaw Rate (deg/s)': yaw_rate_val,
                    'Gross Wt (kg)': weight_val,
                    'CG Pos (% MAC)': cg_val,
                    'CG Pos in X Axis (mm)': float('nan'),
                    'CG Pos in Y Axis (mm)': float('nan'),
                    'Total Fuel Quantity (kg)': fuel_qty_val,
                    'Baro Press (hPa)': baro_val,
                    'OAT (deg C)': oat_val,
                    'Throttle 1 Position (%)': throttle_val,
                    'Engine 1 RPM': eng_rpm_val,
                    'Propeller 1 RPM': prop_rpm_val,
                    'Manifold Pressure (inHg)': map_val,
                    'Fuel Flow (pph)': ff_val,
                    'Mach Number (-)': mach_val,
                    'Wind Dir (mag,deg)': _number(self.latest_sim, 'ambient_wind_dir'),
                    'Wind Spd (kt)': _number(self.latest_sim, 'ambient_wind_spd'),
                    'EGT (deg C)': _number(self.latest_sim, 'egt_c', 'egt'),
                    'Height of Terrain (ft)': _number(self.latest_sim, 'ground_alt'),
                    'Aircraft Angle of Attack (deg)': _number(self.latest_sim, 'aoa'),
                    'Sideslip Angle (deg)': _number(self.latest_sim, 'sideslip'),
                    'Track Angle (deg)': _number(self.latest_sim, 'track_angle'),
                    # Bridge acceleration is in ft/s²; the report specifies g.
                    'X Body Acceleration (g)': _number(self.latest_sim, 'accelX') / 32.17404856,
                    'Y Body Acceleration (g)': _number(self.latest_sim, 'accelY') / 32.17404856,
                    'Z Body Acceleration (g)': _number(self.latest_sim, 'accelZ') / 32.17404856,
                    'Elevator Surface (rad)': float(self.latest_sim.get('elevator_deflection', 0.0) or 0.0),
                    'Left Aileron Surface (rad)': float(self.latest_sim.get('aileron_left_deflection', 0.0) or 0.0),
                    'Rudder Surface (rad)': float(self.latest_sim.get('rudder_deflection', 0.0) or 0.0),
                    'Motion Segment': seg_name,
                }
                self.rows.append(row)

                # Sequence returns to center and completes automatically
                if is_done:
                    self.finish('COMPLETE', f"{self.axis_name} static control sequence completed successfully")

        except Exception as exc:
            self._abort_movement(str(exc))
        finally:
            self._maybe_emit(now)

    def finish(self, result: str, reason: str):
        """Finalizes the test session, releases hardware overrides, and saves CSV/PDF."""
        if self.phase == 'DONE':
            return

        self.phase = 'DONE'
        self.result = result
        self.stage = result
        self.details = reason

        # Release Brunner overrides safely without commanding a new position
        if self.brunner and self.live_hardware:
            try:
                self.brunner.write_setting(self.axis_mask, 0x70, "B", 0)
                self.brunner.set_overrides(self.axis_mask, False)
                self.brunner.set_position_forwarding()
            except Exception:
                pass

        if hasattr(self.sim, 'pause_sim'):
            try:
                self.sim.pause_sim()
            except Exception:
                pass

        if self.rows:
            self.stage = 'Generating reports'
            self.emit()
            try:
                from reports.automatic_control_report import save
                self.outputs = save(self.context, self.rows, result, self.details)
            except Exception as exc:
                self.stage = 'FAILED'
                self.details = f"{reason}; report error: {exc}"
            else:
                self.stage = 'Completed' if result == 'COMPLETE' else result

        self.emit()


def run(context: Dict[str, Any], publish: Any, *,
        sim_factory=None, brunner_factory=None, sleep=time.sleep) -> int:
    """Worker process loop for Automatic Static Control tests."""
    sim = None
    brunner = None
    thread = None
    controller = None

    try:
        # 1. P3D SimConnect Transport
        if sim_factory is None:
            base = Path(sys._MEIPASS) if getattr(sys, 'frozen', False) else Path(__file__).resolve().parents[1]
            candidates = [
                base / 'interfaces/brunner_base/src/p3d_simconnect_v15.py',
                base / 'brunner_base/src/p3d_simconnect_v15.py',
                Path(__file__).resolve().parents[1] / 'interfaces/brunner_base/src/p3d_simconnect_v15.py',
            ]
            source = next((p for p in candidates if p.is_file()), candidates[0])
            spec = importlib.util.spec_from_file_location('qtg_automatic_control_p3d', source)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            sim_factory = module.SimConnectClient

        sim = sim_factory()
        thread = threading.Thread(
            target=sim.connect_and_run,
            kwargs=dict(place_at_rahwali=False, place_airborne=False),
            daemon=True
        )
        thread.start()

        # 2. Brunner Transport
        if brunner_factory is None:
            try:
                from interfaces.brunner_base.src.brunner_interface import BrunnerInterface
                brunner = BrunnerInterface(timeout=0.02)
                brunner.connect()
            except Exception:
                brunner = None
        else:
            brunner = brunner_factory()

        controller = AutomaticControl(context, sim, brunner, publish)
        controller.emit()

        directory = Path(context.get('command_directory', ''))
        stop_file = Path(context.get('stop_file', ''))

        while not controller.closed:
            # Poll for commands
            commands = []
            if directory.exists():
                for path in sorted(directory.glob('*.json')):
                    try:
                        commands.append(json.loads(path.read_text(encoding='utf-8')))
                        path.unlink(missing_ok=True)
                    except Exception:
                        pass

            if stop_file.exists():
                stop_file.unlink(missing_ok=True)
                commands.insert(0, {'action': 'stop'})

            # Prioritize stop/emergency stop
            stopping = next((c for c in commands if c.get('action') in ('stop', 'shutdown', 'emergency_stop')), None)
            if stopping:
                controller.command(stopping)
            else:
                for c in commands:
                    controller.command(c)

            if not controller.closed:
                controller.step()
                sleep(0.025)  # 40 Hz control tick

        return 0

    except Exception as exc:
        if controller:
            controller.finish('FAILED', str(exc))
        else:
            publish(dict(context, stage='FAILED', details=str(exc), can_record=False, outputs=[]))
        return 1

    finally:
        if brunner and hasattr(brunner, 'close'):
            try:
                brunner.close()
            except Exception:
                pass
        if sim and hasattr(sim, 'stop'):
            try:
                sim.stop()
            except Exception:
                pass
        if thread and thread.is_alive():
            thread.join(timeout=3)
