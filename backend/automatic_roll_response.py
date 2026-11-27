"""Dedicated Automatic Roll Response (rate) worker for 2.d.(2a) Approach and 2.d.(2b) Cruise.

Sequence:
1. Initialize airborne flight condition from GUI settings (bank angle 0 deg).
2. Engage Brunner Roll axis (0x02) and stabilize at bank 0 deg for 2.0 continuous seconds.
3. Step roll controller 30% left (-0.30) and hold until roll rate stabilizes (or safety bank limit).
4. Command roll controller back to 0.0, verify center return, and stop recording.
"""

from __future__ import annotations

import csv
from datetime import datetime
import importlib.util
import json
import math
from pathlib import Path
import sys
import threading
import time

from backend.automatic_control import measured_control_fields
from backend.automatic_power_change import AutomaticPowerDynamics, finite
from backend.bank_pid import BankPID, slew_position
from backend.initializer import apply_body_acceleration
from backend.verifier import validate_conditions
from interfaces.engine_measurements import BODY_ACCELERATION_CHANNELS, body_acceleration_fields
from interfaces.multi_pid_transport import MultiPidBrunnerTransport

APPROACH_REFERENCE = '2.d.(2a)'
CRUISE_REFERENCE = '2.d.(2b)'
REFERENCES = (APPROACH_REFERENCE, CRUISE_REFERENCE)
METHOD = 'roll_response'

BASELINE_STABLE_S = 2.0
BASELINE_BANK_TOL_DEG = 2.0
BASELINE_RATE_TOL_DEG_S = 3.0
BASELINE_TIMEOUT_S = 12.0

LEFT_ROLL_COMMAND = -0.30
MIN_ROLL_HOLD_S = 1.5
ROLL_RATE_SETTLE_WINDOW_S = 1.2
ROLL_RATE_SETTLE_BAND_DEG_S = 1.5
MIN_ACTIVE_ROLL_RATE_DEG_S = 2.0
MAX_ROLL_HOLD_S = 12.0
MAX_SAFE_BANK_DEG = 65.0

RETURN_ZERO_TOL_NORM = 0.05
RETURN_ZERO_TIMEOUT_S = 1.5
MAX_RECORDING_S = 35.0


def selected(context):
    return (
        context.get('mode') == 'auto'
        and context.get('control_method') == METHOD
        and context.get('test_reference') in REFERENCES
    )


def read_roll_virtual_force(brunner) -> float:
    """Best-effort read of Brunner 0x51 roll virtual force (N) on axis 0x02."""
    if not brunner or not getattr(brunner, 'sock', None) or not hasattr(brunner, 'read_axis'):
        return float('nan')
    try:
        _, _, v_force = brunner.read_axis(0x02)
        val = float(v_force)
        return val if math.isfinite(val) else float('nan')
    except Exception:
        return float('nan')


class AutomaticRollResponse(AutomaticPowerDynamics):
    """Automatic Roll Response runner owning only Brunner Roll axis (0x02)."""

    def __init__(
        self,
        context,
        sim,
        publish,
        *,
        clock=time.monotonic,
        transport_factory=None,
        bank_pid=None,
    ):
        if not selected(context):
            raise ValueError('Dedicated Automatic Roll Response session required')
        self.context = dict(context)
        self.test_reference = self.context['test_reference']
        self.condition_label = 'Approach' if self.test_reference == APPROACH_REFERENCE else 'Cruise'
        self.cfg = validate_conditions(context.get('initial_conditions'))
        if not self.cfg or self.cfg['on_ground'] or self.cfg['ias_kt'] < 40.0:
            raise ValueError('Roll Response requires an airborne condition (IAS >= 40 kt)')
        if abs(self.cfg['bank_deg']) > 0.5:
            raise ValueError('Roll Response must start with Bank[deg] = 0')
        if any((context.get('active_pids') or {}).values()):
            raise ValueError('Roll Response controls the roll axis directly; disable PID assistance')

        self.sim, self.publish, self.clock = sim, publish, clock
        self.phase, self.stage = 'CONNECT', 'Connecting'
        self.subphase = 'IDLE'
        self.details = f'Connecting to P3D for {self.condition_label} Roll Response (bank 0 -> -30% left -> 0)'
        self.deadline = clock() + 90.0
        self.latest = self.last_sequence = None
        self.last_sim_time = self.last_sim_advance = None
        self.initialization_running = False
        self.initial_throttle_corrected = False
        self.rows, self.outputs = [], []
        self.start_time = None
        self.last_sample_time = None
        self.last_control_time = None
        self._control_sample = {}
        self._next_control_sample = 0.0
        self.result, self.closed = None, False

        self.custom_transport_factory = transport_factory is not None
        self.transport_factory = transport_factory or (
            lambda b: MultiPidBrunnerTransport(b, threading.Event(), pitch_enabled=False, bank_enabled=True, yaw_enabled=False)
        )
        self.roll_transport = None
        self.bank_pid = bank_pid if bank_pid is not None else BankPID(kp=0.015, ki=0.0010, kd=0.020, invert=False)
        self.roll_cmd = 0.0
        self.roll_vel = 0.0
        self.measured_roll = 0.0

        self.baseline_stable_since = None
        self.roll_command_time = None
        self.roll_onset_time = None
        self.roll_rate_stabilized_time = None
        self.roll_return_command_time = None
        self.steady_roll_rate_deg_s = None
        self.stop_trigger = None
        self.roll_rate_samples = []

        self.brunner = self.context.pop('brunner', None)

    def status(self):
        frame = self.latest or {}
        elapsed = max(0.0, self.clock() - self.start_time) if self.start_time is not None else 0.0
        can_record = self.phase == 'READY'
        raw_bank = finite(frame, 'bank_deg')
        bank_display = -raw_bank if raw_bank is not None else None
        roll_rate = finite(frame, 'roll_rate_deg_s')
        return dict(
            self.context,
            stage=self.stage,
            details=self.details,
            updated=time.time(),
            connected=bool(self.sim.is_connected()),
            brunner_connected=bool(self.brunner and getattr(self.brunner, 'sock', None)),
            can_record=can_record,
            controlling=self.phase in ('PREPARE', 'RECORD'),
            recording=self.phase == 'RECORD',
            samples=len(self.rows),
            elapsed=elapsed,
            duration_s=12,
            outputs=list(self.outputs),
            data_directory=self.context['result_directory'],
            engine_text=(f"Engine RPM: {finite(frame, 'rpm') or 0:.0f}" if frame else 'Engine RPM: N/A'),
            throttle_text=(
                f"Roll Rate: {roll_rate:+.1f} deg/s | Cmd: {self.roll_cmd * 100:+.0f}%"
                if roll_rate is not None else 'Roll Rate: N/A'
            ),
            flight_text=(
                f"Airspeed / Altitude: {frame.get('ias_kt', 0):.1f} kt / {frame.get('alt_ft', 0):.0f} ft"
                if frame else 'Airspeed / Altitude: N/A'
            ),
            attitude_text=(
                f"Pitch: {frame.get('pitch_deg', 0):+.1f} deg | Bank: {bank_display:+.1f} deg"
                if bank_display is not None else 'Pitch / Bank: N/A'
            ),
            controls_text=f'Brunner Roll 0x02 ({self.condition_label}): 0 deg x 2 s -> -30% left -> 0',
            recording_text=(
                'READY TO RECORD (00:00)' if can_record else
                f'RECORDING ({int(elapsed // 60):02}:{int(elapsed % 60):02}) - {self.stage}'
                if self.phase == 'RECORD' else self.stage
            ),
            progress_text=f'{len(self.rows)} recorded frames | {self.details}',
            telemetry=dict(
                ias_kt=frame.get('ias_kt'),
                alt_ft=frame.get('alt_ft'),
                vertical_speed=frame.get('vertical_speed'),
                pitch_deg=frame.get('pitch_deg'),
                bank_deg=bank_display,
                roll_rate=roll_rate,
            ),
        )

    def command(self, command):
        action = command['action']
        if action in ('stop', 'shutdown', 'invalidate_initialization'):
            if self.phase != 'DONE':
                self.finish('ABORTED', 'Operator stopped Roll Response')
            if action == 'shutdown':
                self.closed = True
        elif action == 'record' and self.phase == 'READY':
            now = self.clock()
            if not self.latest or not self.sim.is_connected() or now - self.latest['receipt_time'] > 1.0:
                self.finish('FAILED', 'Fresh simulator measurements are required to start recording')
                self.emit()
                return
            try:
                sock = getattr(self.brunner, 'sock', None) if self.brunner is not None else None
                if not self.custom_transport_factory and (not self.brunner or not sock):
                    raise RuntimeError('Roll Response requires a connected Brunner CLS-P Roll loader')
                self.roll_transport = self.transport_factory(self.brunner)
                _, start_roll, _ = self.roll_transport.engage()
                start_roll = float(start_roll) if start_roll is not None and math.isfinite(start_roll) else 0.0
            except Exception as exc:
                self.finish('FAILED', f'Roll loader engagement failed: {exc}')
                self.emit()
                return

            self.measured_roll = start_roll
            self.roll_cmd = start_roll
            self.roll_vel = 0.0
            self.bank_pid.reset(initial_output=start_roll)

            self.baseline_stable_since = None
            self.roll_command_time = None
            self.roll_onset_time = None
            self.roll_rate_stabilized_time = None
            self.roll_return_command_time = None
            self.steady_roll_rate_deg_s = None
            self.stop_trigger = None
            self.roll_rate_samples = []

            self.sim.unpause_sim()
            self.start_time = now
            self.last_control_time = now
            self.last_sim_advance = now
            self.last_sim_time = self.latest.get('sim_time')
            self._last_unpause_retry = now
            self.phase = 'RECORD'
            self.subphase = 'BASELINE'
            self.stage = 'HOLDING LEVEL BANK (0 deg)'
            self.details = 'Holding wings level at 0 deg bank for 2.0 continuous seconds before left roll input'
        self.emit()

    def _record_frame(self, frame, now, *, bank_right_pos):
        if now >= self._next_control_sample:
            sample = measured_control_fields(self.brunner)
            vf = read_roll_virtual_force(self.brunner)
            if math.isfinite(vf):
                sample['Roll Virtual Force 0x51 (N)'] = vf
            self._control_sample = sample
            self._next_control_sample = now + 0.1
        controls = {
            key: self._control_sample.get(key, float('nan'))
            for key in (
                'Joystick Pitch Position (mm)', 'Joystick Pitch Force (N)',
                'Joystick Roll Position (mm)', 'Joystick Roll Force (N)',
                'Rudder Pedal Position (mm)', 'Rudder Pedal Force (N)',
            )
        }
        row = {
            'Time (s)': round(now - self.start_time, 4),
            'Simulator Time (s)': frame['sim_time'],
            'Native Sequence': frame['sequence'],
            'Indicated Airspeed (kts)': frame['ias_kt'],
            'Altitude MSL (ft)': frame['alt_ft'],
            'Height AGL (ft)': finite(frame, 'height_agl_ft'),
            'Ground Speed (kt)': finite(frame, 'ground_speed'),
            'Rate of Climb/Descent (fpm)': finite(frame, 'vertical_speed'),
            'Pitch Angle (deg)': frame['pitch_deg'],
            'Bank Angle (deg)': bank_right_pos,
            'Heading (deg)': finite(frame, 'heading_deg'),
            'Aircraft Angle of Attack (deg)': finite(frame, 'aoa_deg'),
            'Sideslip Angle (deg)': finite(frame, 'sideslip_deg'),
            'Roll Rate (deg/s)': finite(frame, 'roll_rate_deg_s'),
            'Pitch Rate (deg/s)': finite(frame, 'pitch_rate_deg_s'),
            'Yaw Rate (deg/s)': finite(frame, 'yaw_rate_deg_s'),
            'Flaps Handle Index': finite(frame, 'flaps'),
            'Flaps Handle Percent (%)': finite(frame, 'flaps_handle_percent'),
            'Ambient Temperature (C)': finite(frame, 'temperature_c'),
            'Throttle 1 Position (%)': frame['throttle'],
            'Engine 1 RPM': frame['rpm'],
            'Propeller 1 RPM': finite(frame, 'prop_rpm'),
            'Manifold Pressure (inHg)': (
                frame['manifold_pressure_psi'] * 2.0360206576
                if finite(frame, 'manifold_pressure_psi') is not None else None
            ),
            'Aileron Input (normalised)': finite(frame, 'aileron_input'),
            'Elevator Input (normalised)': finite(frame, 'elevator_input'),
            'Rudder Input (normalised)': finite(frame, 'rudder_input'),
            'Roll Command (norm)': round(self.roll_cmd, 5),
            'Roll Measured (norm)': round(self.measured_roll, 5),
            'Roll Virtual Force 0x51 (N)': self._control_sample.get('Roll Virtual Force 0x51 (N)', float('nan')),
            'Roll Step Command (0/1)': int(self.roll_command_time is not None and self.roll_return_command_time is None),
            'Autopilot Master (0/1)': finite(frame, 'autopilot_active'),
            **controls,
            **body_acceleration_fields(frame),
        }
        self.rows.append(row)

    def _roll_rate_stabilized(self, now):
        if self.roll_command_time is None or now - self.roll_command_time < MIN_ROLL_HOLD_S:
            return None
        cutoff = now - ROLL_RATE_SETTLE_WINDOW_S
        window = [(t, r) for t, r in self.roll_rate_samples if t >= cutoff]
        if len(window) < 4 or window[-1][0] - window[0][0] < ROLL_RATE_SETTLE_WINDOW_S * 0.85:
            return None
        rates = [r for _, r in window]
        spread = max(rates) - min(rates)
        mean_rate = sum(rates) / len(rates)
        if spread <= ROLL_RATE_SETTLE_BAND_DEG_S and abs(mean_rate) >= MIN_ACTIVE_ROLL_RATE_DEG_S:
            return mean_rate
        return None

    def step(self, frame):
        if self.phase != 'RECORD':
            previous = self.phase
            super().step(frame)
            if self.phase == 'READY' and previous != 'READY':
                self.details = (
                    f'{self.condition_label} initial conditions verified (bank 0 deg). '
                    'Ready for 2 s level baseline, 30% left roll until roll rate stabilizes, then return to 0.'
                )
                self.emit()
            return
        if self.phase == 'DONE':
            return
        now = self.clock()
        try:
            if frame is None or not (-0.2 <= now - frame.get('receipt_time', -1.0e6) <= 1.0):
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
                raise RuntimeError('Simulator Autopilot must be OFF for Roll Response')
            if frame.get('on_ground') == 1:
                raise RuntimeError('Aircraft is on the ground; airborne Roll Response cannot continue')
            agl = finite(frame, 'height_agl_ft')
            if agl is not None and agl < 300.0:
                raise RuntimeError('Aircraft too low to safely continue; recording stopped')
            if self.last_sim_time is not None and frame['sim_time'] > self.last_sim_time:
                self.last_sim_advance, self.last_sim_time = now, frame['sim_time']
            elif self.last_sim_advance is not None and now - self.last_sim_advance > 8.0:
                raise RuntimeError('P3D is paused or simulation time stopped advancing')

            if now - self.start_time > MAX_RECORDING_S:
                raise RuntimeError('Roll Response exceeded its maximum recording window')

            roll_rate = finite(frame, 'roll_rate_deg_s')
            if roll_rate is None:
                raise RuntimeError('Measured roll rate is unavailable')

            bank_right_pos = -float(frame['bank_deg'])
            dt = 0.04 if self.last_control_time is None else max(0.005, min(0.5, now - self.last_control_time))
            self.last_control_time = now

            if self.roll_transport is not None:
                self.roll_transport.refresh_watchdog()
                _, measured_roll, _ = self.roll_transport.read_positions()
                if measured_roll is not None and math.isfinite(measured_roll):
                    self.measured_roll = float(measured_roll)

            if self.subphase == 'BASELINE':
                desired_roll = self.bank_pid.update(0.0, bank_right_pos, dt)
                self.roll_cmd, self.roll_vel = slew_position(
                    self.roll_cmd, self.roll_vel, desired_roll, dt, max_rate=0.28, max_accel=0.90
                )
                self.roll_cmd = max(self.measured_roll - 0.28, min(self.measured_roll + 0.28, self.roll_cmd))
                if self.roll_transport is not None:
                    self.roll_transport.command(roll_pos=self.roll_cmd)

                if abs(bank_right_pos) <= BASELINE_BANK_TOL_DEG and abs(roll_rate) <= BASELINE_RATE_TOL_DEG_S:
                    if self.baseline_stable_since is None:
                        self.baseline_stable_since = now
                else:
                    self.baseline_stable_since = None

                self._record_frame(frame, now, bank_right_pos=bank_right_pos)

                if (self.baseline_stable_since is not None
                        and now - self.baseline_stable_since >= BASELINE_STABLE_S
                        and len(self.rows) >= 2):
                    self.roll_command_time = now
                    self.subphase = 'ROLL_HOLD'
                    self.stage = 'ROLLING LEFT (30%)'
                    self.details = 'Commanding 30% left roll controller (-0.30) until roll rate stabilizes'
                    self.roll_cmd = LEFT_ROLL_COMMAND
                    self.roll_vel = 0.0
                    if self.roll_transport is not None:
                        self.roll_transport.command(roll_pos=self.roll_cmd)
                elif now - self.start_time > BASELINE_TIMEOUT_S:
                    raise RuntimeError('Aircraft did not maintain stable 0 deg bank for 2 seconds')

            elif self.subphase == 'ROLL_HOLD':
                self.roll_cmd = LEFT_ROLL_COMMAND
                if self.roll_transport is not None:
                    self.roll_transport.command(roll_pos=self.roll_cmd)

                aileron_val = finite(frame, 'aileron_input')
                if self.roll_onset_time is None and (
                    abs(self.measured_roll) >= 0.05
                    or (aileron_val is not None and abs(aileron_val) >= 0.05)
                    or abs(roll_rate) >= MIN_ACTIVE_ROLL_RATE_DEG_S
                ):
                    self.roll_onset_time = now

                self.roll_rate_samples.append((now, roll_rate))
                self._record_frame(frame, now, bank_right_pos=bank_right_pos)

                mean_rate = self._roll_rate_stabilized(now)
                trigger = None
                if mean_rate is not None:
                    trigger = 'roll_rate_stabilized'
                    self.steady_roll_rate_deg_s = mean_rate
                elif abs(bank_right_pos) >= MAX_SAFE_BANK_DEG:
                    trigger = 'max_safe_bank_reached'
                    self.steady_roll_rate_deg_s = roll_rate
                elif now - self.roll_command_time >= MAX_ROLL_HOLD_S:
                    trigger = 'max_roll_hold_timeout'
                    self.steady_roll_rate_deg_s = roll_rate

                if trigger is not None:
                    self.stop_trigger = trigger
                    self.roll_rate_stabilized_time = now
                    self.roll_return_command_time = now
                    self.subphase = 'RETURN_ZERO'
                    self.stage = 'RETURNING ROLL CONTROLLER TO ZERO'
                    self.details = (
                        f'Roll rate stabilized at {self.steady_roll_rate_deg_s:+.2f} deg/s; returning roll controller to 0'
                        if trigger == 'roll_rate_stabilized'
                        else f'Safety return triggered ({trigger}); returning roll controller to 0'
                    )
                    self.roll_cmd = 0.0
                    if self.roll_transport is not None:
                        self.roll_transport.command(roll_pos=0.0)

            elif self.subphase == 'RETURN_ZERO':
                self.roll_cmd = 0.0
                if self.roll_transport is not None:
                    self.roll_transport.command(roll_pos=0.0)
                self._record_frame(frame, now, bank_right_pos=bank_right_pos)
                if abs(self.measured_roll) <= RETURN_ZERO_TOL_NORM or now - self.roll_return_command_time >= RETURN_ZERO_TIMEOUT_S:
                    self.finish(
                        'COMPLETE',
                        f'{self.condition_label} Roll Response captured (steady roll rate '
                        f'{(self.steady_roll_rate_deg_s or 0.0):+.2f} deg/s); roll controller returned to 0.',
                    )
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
        if self.roll_transport is not None:
            try:
                self.roll_transport.release()
            except Exception:
                pass
            finally:
                self.roll_transport = None
        if self.rows:
            self.stage = 'Generating reports'
            self.emit()
            try:
                self.outputs = save_report(
                    self.context,
                    self.rows,
                    result,
                    reason,
                    roll_command_time=self.roll_command_time,
                    roll_onset_time=self.roll_onset_time,
                    roll_rate_stabilized_time=self.roll_rate_stabilized_time,
                    roll_return_command_time=self.roll_return_command_time,
                    steady_roll_rate_deg_s=self.steady_roll_rate_deg_s,
                    stop_trigger=self.stop_trigger,
                    recording_start=self.start_time,
                )
                self.stage = 'Completed' if result == 'COMPLETE' else result
            except Exception as exc:
                self.outputs = list(getattr(exc, 'outputs', []))
                self.stage = 'FAILED'
                self.details += '; reporting error: ' + str(exc)
        self.emit()


def save_report(
    context,
    rows,
    result,
    reason,
    *,
    roll_command_time,
    roll_onset_time,
    roll_rate_stabilized_time,
    roll_return_command_time,
    steady_roll_rate_deg_s,
    stop_trigger,
    recording_start,
):
    from reports.power_dynamics import graph_availability
    from reports.roll_response import DEFAULT_GRAPHS, render_report

    folder = Path(context['result_directory'])
    folder.mkdir(parents=True, exist_ok=True)
    reference = context['test_reference']
    condition_slug = 'approach' if reference == APPROACH_REFERENCE else 'cruise'
    stem = folder / (
        f'2_d_2_roll_response_{condition_slug}_auto_'
        + datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        + '_' + str(time.time_ns())
    )
    csv_path = stem.with_suffix('.csv')
    meta_path = stem.with_suffix('.qtg_test.json')
    pdf_path = stem.with_name(stem.name + '_graphs.pdf')
    outputs = []
    try:
        with csv_path.open('x', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        outputs.append(str(csv_path))

        relative = lambda stamp: stamp - recording_start if stamp is not None else None
        selected_graphs = context.get('selected_graphs')
        requested = list(DEFAULT_GRAPHS) if selected_graphs is None else list(selected_graphs)
        graphed, unavailable = graph_availability(rows, requested)
        metadata = dict(
            context,
            mode='auto',
            control_method=METHOD,
            kind=f'Roll Response ({condition_slug.capitalize()})',
            condition=condition_slug,
            left_roll_command_norm=LEFT_ROLL_COMMAND,
            baseline_required_s=BASELINE_STABLE_S,
            result=result,
            reason=reason,
            status=result.lower(),
            event_time_s=relative(roll_command_time),
            roll_command_time_s=relative(roll_command_time),
            roll_measured_onset_s=relative(roll_onset_time),
            roll_rate_stabilized_time_s=relative(roll_rate_stabilized_time),
            roll_return_command_time_s=relative(roll_return_command_time),
            steady_roll_rate_deg_s=steady_roll_rate_deg_s,
            stop_trigger=stop_trigger,
            initial_recorded_at_start=True,
            graphed_channels=graphed,
            unavailable_selected_channels=unavailable,
            aircraft_conformity='NOT EVALUATED - approved aircraft reference data not supplied',
            measurement_source=(
                'Correlated P3D telemetry; Brunner Roll axis 0x02 automated step (-0.30 left -> 0.0 center)'
            ),
        )
        meta_path.write_text(json.dumps(metadata, indent=2, allow_nan=False), encoding='utf-8')
        outputs.append(str(meta_path))
        if roll_command_time is None:
            return outputs
        render_report(csv_path, pdf_path, metadata)
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
            candidates = (
                root / 'interfaces/brunner_base/src/p3d_simconnect_v15.py',
                root / 'brunner_base/src/p3d_simconnect_v15.py',
            )
            source = next((p for p in candidates if p.is_file()), candidates[0])
            spec = importlib.util.spec_from_file_location('qtg_auto_roll_p3d', source)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            factory = module.SimConnectClient
        sim = factory()
        controller = AutomaticRollResponse(context, sim, publish)
        from interfaces.brunner_base.src.brunner_interface import BrunnerInterface
        control_reader = BrunnerInterface(timeout=0.05)
        if control_reader.connect():
            controller.brunner = control_reader
        controller.emit()
        thread = threading.Thread(
            target=sim.connect_and_run,
            kwargs=dict(place_at_rahwali=False, place_airborne=False),
            daemon=True,
        )
        thread.start()
        if reader_factory is None:
            from interfaces.engine_measurements import Measurements, CHANNELS
            root = Path(sys._MEIPASS) if getattr(sys, 'frozen', False) else Path(__file__).resolve().parents[1]
            candidates = (
                root / 'bridge/bin/qtg_simconnect.dll',
                root / 'qtg_simconnect.dll',
                Path(sys.executable).parent / 'bridge/bin/qtg_simconnect.dll',
            )
            dll = next((p for p in candidates if p.is_file()), candidates[0])
            extra = (('autopilot_active', 'AUTOPILOT MASTER', 'bool'),)
            reader_factory = lambda: Measurements(dll, channels=CHANNELS + extra + BODY_ACCELERATION_CHANNELS)
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
                (c for c in commands if c['action'] in ('stop', 'invalidate_initialization')), None
            )
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
                sleep(0.04)
            elif not controller.closed:
                sleep(0.04)
        return 0
    except Exception as exc:
        if controller:
            controller.finish('FAILED', str(exc))
        else:
            publish(dict(context, stage='FAILED', details=str(exc), can_record=False, outputs=[]))
        return 1
    finally:
        if control_reader:
            control_reader.close()
        if reader:
            reader.close()
        if sim:
            sim.stop()
        if thread:
            thread.join(timeout=5)
