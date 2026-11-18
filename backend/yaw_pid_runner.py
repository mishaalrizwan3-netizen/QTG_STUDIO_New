"""Yaw/Heading PID worker lifecycle; existing initialization and CSV capture are reused.
Outer-loop: HeadingController steers to target magnetic heading via commanded bank.
Inner-loop Roll: BankPID drives BRUNNER Roll (Axis 2) to commanded bank.
Inner-loop Yaw: YawPID drives BRUNNER Rudder (Axis 4) for coordination & yaw damping.
Pitch (Axis 1) is NEVER commanded, armed or touched.
"""
import csv
import ctypes
import json
import math
import os
from pathlib import Path
import time

from backend.bank_pid import BankPID, slew_position
from backend.yaw_pid import HeadingController, YawPID, HeadingBankHandoff
from interfaces.yaw_pid_transport import YawBrunnerTransport


class YawStabilizationGate:
    """Monotonic-time state: heading and bank must remain within tolerance continuously for hold_s."""
    def __init__(self, hdg_tolerance_deg: float = 2.0, bank_tolerance_deg: float = 3.0, hold_s: float = 5.0):
        self.hdg_tolerance_deg = float(hdg_tolerance_deg)
        self.bank_tolerance_deg = float(bank_tolerance_deg)
        self.hold_s = float(hold_s)
        self.since = None

    def reset(self):
        self.since = None

    def update(self, now: float, target_hdg: float, actual_hdg: float, actual_bank: float) -> bool:
        if not (math.isfinite(actual_hdg) and math.isfinite(actual_bank)):
            raise RuntimeError('Missing/non-finite Yaw/Heading feedback')
        # Shortest-turn heading error wrapped to [-180 .. +180]
        norm_tgt = target_hdg % 360.0
        norm_act = actual_hdg % 360.0
        hdg_err = (norm_tgt - norm_act + 180.0) % 360.0 - 180.0

        if abs(hdg_err) <= self.hdg_tolerance_deg and abs(actual_bank) <= self.bank_tolerance_deg:
            if self.since is None:
                self.since = now
            return (now - self.since) >= self.hold_s
        else:
            self.since = None
            return False

    def elapsed_stable(self, now: float) -> float:
        if self.since is None:
            return 0.0
        return max(0.0, now - self.since)


def parent_alive(pid):
    if not pid:
        return False
    if os.name != 'nt':
        try:
            os.kill(pid, 0)
            return True
        except OSError:
            return False
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.OpenProcess.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_uint32]
    kernel.OpenProcess.restype = ctypes.c_void_p
    kernel.WaitForSingleObject.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    kernel.CloseHandle.argtypes = [ctypes.c_void_p]
    handle = kernel.OpenProcess(0x100000, False, pid)
    if not handle:
        return False
    try:
        return kernel.WaitForSingleObject(handle, 0) == 0x102
    finally:
        kernel.CloseHandle(handle)


def fresh(window, timeout_s=1.5, max_retries=3):
    if not window.p3d or not window.p3d.is_connected():
        raise RuntimeError('P3D readiness failure: Sim Flight Model disconnected')
    for attempt in range(max_retries + 1):
        seq = window.p3d.get_telemetry_sequence()
        if not window.p3d.request_telemetry_once():
            if attempt < max_retries and not window.stop_worker_event.is_set():
                window.stop_worker_event.wait(0.05)
                continue
            raise RuntimeError('Missing feedback: P3D telemetry request rejected')
        data = window.p3d.wait_for_fresh_telemetry(seq, timeout_s)
        if data is not None and window.p3d.get_telemetry_sequence() > seq:
            for key in ('magnetic_heading', 'bank_angle', 'yaw_rate', 'sideslip'):
                if not math.isfinite(float(data.get(key, float('nan')))):
                    raise RuntimeError('Missing feedback: non-finite telemetry ' + key)
            return data
        if attempt < max_retries and not window.stop_worker_event.is_set():
            window.stop_worker_event.wait(0.05)
            continue
    raise RuntimeError('Missing feedback: Sim Flight Model feedback stale')


def transition(window, state, reason=''):
    window.yaw_phase = state
    context = getattr(window, '_session_context', {})
    context['yaw_phase'] = state
    context.setdefault('yaw_transitions', []).append(dict(state=state, time=time.monotonic(), reason=reason))


def simulator_loaded(window):
    try:
        status = Path(window._session_context['status_file'])
        state = json.loads(status.with_suffix('.p3d_ready.json').read_text(encoding='utf-8'))
        age = time.time() - float(state['updated'])
        return (0 <= age < 2. and state['connected'] and state['sim_running']
                and state['telemetry_valid'] and 0 <= state['telemetry_age_ms'] < 1500)
    except (KeyError, OSError, ValueError, TypeError):
        return bool(window.p3d and window.p3d.is_connected() and window.p3d.simulation_running())


def wait_ready(window, transport, unpause=True, owner_alive=parent_alive):
    sim = window.p3d
    sent_unpause = not unpause
    consecutive = 0
    window.sig_update_stage.emit('Initializing aircraft',
        'Waiting for P3D to finish loading. Yaw PID OFF; recording OFF.')
    while not window.stop_worker_event.is_set():
        if not owner_alive(window._session_context.get('yaw_parent_pid', window._session_context.get('bank_parent_pid'))):
            raise RuntimeError('QTG GUI process exited during P3D loading')
        if not simulator_loaded(window):
            consecutive = 0
            window.stop_worker_event.wait(.1)
            continue
        if not sent_unpause:
            sim.unpause_sim()
            sent_unpause = True
            bridge = getattr(sim, '_bridge', None)
            if bridge is not None and hasattr(bridge, 'RequestSimulationRunningV1509'):
                bridge.RequestSimulationRunningV1509()
        try:
            fresh(window, timeout_s=1.5, max_retries=3)
            r_pos, y_pos = transport.read_positions()
            valid = sim.simulation_running() and math.isfinite(r_pos) and math.isfinite(y_pos) and simulator_loaded(window)
        except RuntimeError:
            valid = False
        consecutive = consecutive + 1 if valid else 0
        if consecutive >= 2:
            return
        window.stop_worker_event.wait(.04)
    raise RuntimeError('Yaw readiness cancelled while waiting for P3D loading')


def prepare(window, base):
    if window._session_context.get('initial_conditions') is None:
        raise RuntimeError('Yaw PID requires complete selected initial conditions')
    window.yaw_fault = None
    window.yaw_active = False
    window.yaw_live = {}
    window.active_manual_ref_path = None
    window.selected_ref_csv = None
    window.brunner._yaw_transport = YawBrunnerTransport(window.brunner, window.stop_worker_event)
    base.SuperMushakLoggerApp.start_manual_test(window)


def loop(window, base, owner_alive=parent_alive):
    device = window.brunner
    transport = getattr(device, '_yaw_transport', None)
    if transport is None:
        transport = YawBrunnerTransport(device, window.stop_worker_event)
        device._yaw_transport = transport

    rows = []
    failed = None
    recording_started = None
    context = getattr(window, '_session_context', {})
    
    # Target magnetic heading from GUI
    target_hdg = float(context.get('target_heading_mag', 270.0))
    hdg_tolerance_deg = float(context.get('yaw_hdg_tolerance_deg', 2.0))
    bank_tolerance_deg = float(context.get('yaw_bank_tolerance_deg', 3.0))
    hold_duration_s = float(context.get('yaw_hold_sec', 5.0))
    timeout_duration_s = float(context.get('yaw_acquisition_timeout_sec', 60.0))

    try:
        if window.stop_worker_event.is_set():
            raise RuntimeError('Yaw start cancelled')
        if not owner_alive(context.get('yaw_parent_pid', context.get('bank_parent_pid'))):
            raise RuntimeError('QTG GUI process exited')

        # 1. Unpause P3D after QTG initialization and wait for steady running state
        if not window.p3d or not window.p3d.is_connected():
            raise RuntimeError('P3D readiness failure: Sim Flight Model disconnected')

        window.p3d.unpause_sim()

        _unpause_timeout_s = float(context.get('yaw_unpause_timeout_sec', 20.0))
        _unpause_deadline = time.monotonic() + _unpause_timeout_s
        _running_since = None
        _pause_retries = 0
        while True:
            if window.stop_worker_event.wait(0.05):
                raise RuntimeError('Yaw start cancelled during P3D unpause wait')
            if not window.p3d.is_connected():
                raise RuntimeError('P3D readiness failure: Sim Flight Model disconnected during unpause')
            if window.p3d.simulation_running():
                if _running_since is None:
                    _running_since = time.monotonic()
                elif time.monotonic() - _running_since >= 0.25:
                    break
            else:
                _running_since = None
                if _pause_retries < 2 and time.monotonic() < _unpause_deadline - 5.0:
                    _pause_retries += 1
                    window.p3d.unpause_sim()
            if time.monotonic() > _unpause_deadline:
                raise RuntimeError(
                    f'P3D readiness failure: simulator did not confirm steady running state within {int(_unpause_timeout_s)}s'
                )

        # 2. Telemetry streaming confirmation
        data = fresh(window, timeout_s=2.0, max_retries=3)

        # 3. Verify BRUNNER roll and rudder readiness before engaging
        try:
            start_roll, start_rudder = transport.read_positions()
        except Exception as exc:
            raise RuntimeError(f'BRUNNER readiness failure: {exc}')
        if not (math.isfinite(start_roll) and math.isfinite(start_rudder)):
            raise RuntimeError('BRUNNER readiness failure: non-finite initial roll/rudder feedback')

        # 4. Engage Roll (Axis 2) and Rudder (Axis 4) simultaneously (Pitch untouched!)
        roll_pos, rudder_pos = transport.engage()

        # Initialize cascaded controllers
        actual_bank = float(data['bank_angle'])
        actual_hdg = float(data['magnetic_heading'])
        heading_ctrl = HeadingController(kp=0.80, ki=0.005, kd=0.70, max_bank=20.0, invert=False)
        heading_ctrl.reset(initial_bank=actual_bank)

        bank_pid = BankPID()
        bank_pid.reset(initial_output=roll_pos)

        yaw_pid = YawPID(kp=0.025, ki=0.0020, kd=0.035, invert=False)
        yaw_pid.reset(initial_output=rudder_pos)

        bank_handoff = HeadingBankHandoff()

        roll_vel = 0.0
        rudder_vel = 0.0
        mismatch_roll = None
        mismatch_rudder = None
        window.yaw_active = window._external_control_active = True

        gate = YawStabilizationGate(hdg_tolerance_deg=hdg_tolerance_deg,
                                    bank_tolerance_deg=bank_tolerance_deg,
                                    hold_s=hold_duration_s)
        phase = 'stabilizing'
        recording_started = None
        control_started = last = time.monotonic()
        recording_paused_since = None
        recording_pause_timeout_s = float(context.get('yaw_recording_pause_timeout_sec', 2.0))
        pending_readiness_reason = None

        if window.stop_worker_event.is_set():
            raise RuntimeError('Yaw start cancelled')

        # 5. Active control loop
        while not window.stop_worker_event.is_set():
            if not owner_alive(context.get('yaw_parent_pid', context.get('bank_parent_pid'))):
                raise RuntimeError('QTG GUI process exited')

            now = time.monotonic()

            # First priority: check if test recording duration has been completed
            if phase == 'recording' and recording_started is not None:
                rec_elapsed = now - recording_started
                if rec_elapsed >= window.target_duration_sec:
                    break

            is_paused = not window.p3d.simulation_running()

            if pending_readiness_reason or is_paused or (recording_started is None and not simulator_loaded(window)):
                gate.reset()
                if recording_started is not None:
                    # Debounce transient pauses during recording
                    if recording_paused_since is None:
                        recording_paused_since = now
                    elif now - recording_paused_since > recording_pause_timeout_s:
                        raise RuntimeError('Sim Flight Model paused during Yaw recording; test interrupted')
                    recording_started += (now - last)
                    last = time.monotonic()
                    window.stop_worker_event.wait(0.04)
                    continue
                recording_paused_since = None
                transport.release()
                window.yaw_active = False
                window.yaw_live = {}
                transition(window, 'WAITING_FOR_P3D', pending_readiness_reason or 'Simulator not ready')
                window.sig_update_stage.emit(getattr(base, 'STAGE_INITIALIZING', 'Initializing aircraft'),
                    'Waiting for fresh, running P3D feedback. Roll and Rudder released; recording OFF. Stability timer reset.')
                transport = YawBrunnerTransport(device, window.stop_worker_event)
                device._yaw_transport = transport
                wait_ready(window, transport, unpause=False, owner_alive=owner_alive)
                pending_readiness_reason = None
                roll_pos, rudder_pos = transport.engage()
                bank_pid.reset(initial_output=roll_pos)
                yaw_pid.reset(initial_output=rudder_pos)
                roll_vel = 0.0
                rudder_vel = 0.0
                mismatch_roll = None
                mismatch_rudder = None
                control_started = last = time.monotonic()
                window.yaw_active = True
                transition(window, 'YAW_PID_ACTIVE')
                transition(window, 'STABILIZING')
                continue

            recording_paused_since = None

            # Fetch fresh telemetry
            try:
                data = fresh(window, timeout_s=1.5, max_retries=3)
            except RuntimeError as exc:
                if recording_started is not None:
                    raise
                gate.reset()
                pending_readiness_reason = str(exc)
                continue

            now = time.monotonic()
            if not window.p3d.simulation_running():
                gate.reset()
                if recording_started is not None:
                    if recording_paused_since is None:
                        recording_paused_since = now
                    elif now - recording_paused_since > recording_pause_timeout_s:
                        raise RuntimeError('Sim Flight Model paused during Yaw recording; test interrupted')
                    recording_started += (now - last)
                    last = time.monotonic()
                    window.stop_worker_event.wait(0.04)
                    continue
                recording_paused_since = None
                pending_readiness_reason = 'P3D paused before recording'
                continue

            measured_roll = transport.read_axis_position(2)
            measured_rudder = transport.read_axis_position(4)

            raw_dt = now - last
            last = now
            if raw_dt > 1.50:
                raise RuntimeError('Yaw feedback/command scheduling watchdog expired')

            dt = min(0.1, max(0.01, raw_dt))
            actual_hdg = float(data['magnetic_heading'])
            actual_bank = float(data['bank_angle'])
            yaw_rate = float(data['yaw_rate'])
            sideslip = float(data['sideslip'])

            # -------------------------------------------------------------
            # Cascaded Yaw / Heading Control Math (verbatim PID Controller)
            # -------------------------------------------------------------
            norm_tgt = target_hdg % 360.0
            norm_act = actual_hdg % 360.0
            hdg_err = (norm_tgt - norm_act + 180.0) % 360.0 - 180.0

            # 1. Outer Loop: Heading to Bank
            demanded_bank = heading_ctrl.update(target_hdg, actual_hdg, yaw_rate, dt)
            captured = bank_handoff.update(True, True, target_hdg, actual_hdg, actual_bank,
                                           heading_ctrl.hdg_rate, dt, True)

            if captured:
                effective_target_bank = 0.0
                demanded_bank = 0.0
                heading_ctrl.integral = 0.0
                heading_ctrl.cmd_bank = 0.0
            else:
                effective_target_bank = demanded_bank

            # 2. Inner Loop Roll: Bank to Roll Command
            desired_roll = bank_pid.update(effective_target_bank, actual_bank, dt)
            roll_pos, roll_vel = slew_position(roll_pos, roll_vel, desired_roll, dt,
                                               max_rate=0.28, max_accel=0.90)

            # 3. Inner Loop Yaw: Sideslip & Rate to Rudder Command
            desired_rudder = yaw_pid.update(sideslip, yaw_rate, dt, heading_ctrl.hdg_rate)
            rudder_pos, rudder_vel = slew_position(rudder_pos, rudder_vel, desired_rudder, dt,
                                                   max_rate=0.28, max_accel=0.90)

            if window.stop_worker_event.is_set():
                break

            # Send Roll (2) and Rudder (4) commands (Pitch is completely untouched!)
            transport.command(roll_pos, rudder_pos)

            # BRUNNER tracking checks
            if abs(roll_pos - measured_roll) > 0.25:
                mismatch_roll = now if mismatch_roll is None else mismatch_roll
                if now - mismatch_roll > 2.5:
                    raise RuntimeError('BRUNNER command tracking failure: roll loader did not track within 2.5s')
            else:
                mismatch_roll = None

            if transport.rudder_available:
                if abs(rudder_pos - measured_rudder) > 0.25:
                    mismatch_rudder = now if mismatch_rudder is None else mismatch_rudder
                    if now - mismatch_rudder > 2.5:
                        raise RuntimeError('BRUNNER command tracking failure: rudder loader did not track within 2.5s')
                else:
                    mismatch_rudder = None

            # Phase management & 5-second stabilization
            if phase == 'stabilizing':
                stable = gate.update(now, target_hdg, actual_hdg, actual_bank)
                stable_elapsed = gate.elapsed_stable(now)

                if not stable and (now - control_started) > timeout_duration_s:
                    raise RuntimeError(
                        f'Acquisition timeout: heading did not settle within ±{hdg_tolerance_deg:.1f}° '
                        f'and bank within ±{bank_tolerance_deg:.1f}° for {hold_duration_s:.1f} continuous seconds '
                        f'within {int(timeout_duration_s)}s (current hdg: {actual_hdg:.1f}°, bank: {actual_bank:+.2f}°)'
                    )

                if stable:
                    phase = 'recording'
                    recording_started = now
                    window.is_recording = True
                    window.sig_update_stage.emit(
                        getattr(base, 'STAGE_RECORDING', 'Recording'),
                        f'Heading captured ({actual_hdg:.1f}° target {target_hdg:.1f}° held for {hold_duration_s:.1f}s); recording active.'
                    )
                    window.auto_t0_snapshot = dict(
                        telemetry=dict(data),
                        control_method='yaw_pid',
                        target_heading_deg=target_hdg,
                        roll_pos=measured_roll,
                        rudder_pos=measured_rudder
                    )
                else:
                    window.sig_update_stage.emit(
                        getattr(base, 'STAGE_INITIALIZING', 'Initializing'),
                        f'Yaw PID: Steering to heading {target_hdg:.1f}° (current {actual_hdg:.1f}°, bank {actual_bank:+.1f}°) '
                        f'Holding {stable_elapsed:.1f}/{hold_duration_s:.1f}s'
                    )

            if phase == 'recording':
                rec_elapsed = now - recording_started
                if rec_elapsed >= window.target_duration_sec:
                    break

                axes = []
                for ax_idx in (1, 2, 4):
                    try:
                        axes.append(device.read_axis(ax_idx))
                    except Exception:
                        axes.append((0.0, 0.0, 0.0))
                axes[1] = (measured_roll, axes[1][1], axes[1][2])
                axes[2] = (measured_rudder, axes[2][1], axes[2][2])
                window.rec_time.append(rec_elapsed)
                for values, positions, forces in zip(
                    axes,
                    (window.p_pos_buf, window.r_pos_buf, window.y_pos_buf),
                    (window.p_force_buf, window.r_force_buf, window.y_force_buf)
                ):
                    positions.append(values[0])
                    forces.append(values[1])

                window.p3d_tele_buf.append(data)
                window.applied_force_buf.append(tuple(a[1] for a in axes))
                window.virtual_force_buf.append(tuple(a[2] for a in axes))
                window.sig_record_progress.emit(int(rec_elapsed))

            # Diagnostic live logging
            rec_time_log = (now - recording_started) if recording_started is not None else 0.0
            row = dict(
                time_s=round(now - control_started, 4),
                recording_time_s=round(rec_time_log, 4),
                phase=phase,
                target_hdg_deg=target_hdg,
                actual_hdg_deg=round(actual_hdg, 2),
                hdg_error_deg=round(hdg_err, 2),
                demanded_bank_deg=round(demanded_bank, 2),
                actual_bank_deg=round(actual_bank, 2),
                yaw_rate_deg_s=round(yaw_rate, 3),
                sideslip_deg=round(sideslip, 3),
                roll_command=round(roll_pos, 4),
                roll_measured=round(measured_roll, 4),
                rudder_command=round(rudder_pos, 4),
                rudder_measured=round(measured_rudder, 4),
                p3d_paused=is_paused,
                controller_engaged=transport.owned,
                stability_timer_s=round(gate.elapsed_stable(now), 3),
                recording_started_s=round(recording_started - control_started, 4) if recording_started else 0.0,
                dt=round(dt, 4),
            )
            rows.append(row)
            window.yaw_live = row

            window.stop_worker_event.wait(max(0.0, 0.04 - (time.monotonic() - now)))

    except Exception as exc:
        failed = str(exc)
    finally:
        if window.stop_worker_event.is_set() and failed is None:
            failed = 'Stopped by user'

        window.stop_worker_event.set()

        if getattr(window, 'p3d', None):
            try:
                window.p3d.pause_sim()
            except Exception:
                pass

        try:
            transport.release()
        except Exception as exc:
            failed = failed or str(exc)

        window.yaw_fault = failed
        window.yaw_active = window._external_control_active = False
        window._prepared = window._preparing = window.is_recording = False
        context.update(yaw_fault=failed, yaw_complete=not bool(failed), target_heading_mag=target_hdg)

        # Write detailed diagnostic CSV
        if rows:
            try:
                for r in rows:
                    r['stop_fault_reason'] = failed or 'Completed'
                path = Path(window.data_dir) / ('yaw_pid_diagnostics_' + str(time.time_ns()) + '.csv')
                with path.open('w', newline='', encoding='utf-8') as stream:
                    writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                    writer.writeheader()
                    writer.writerows(rows)
                window._outputs.append(str(path))
                context['yaw_diagnostics_path'] = str(path)
            except Exception as exc:
                failed = failed or str(exc)
                window.yaw_fault = context['yaw_fault'] = failed

        context['yaw_complete'] = not bool(failed)
        if recording_started is not None and window.rec_time:
            window.sig_recording_finished.emit()
        if failed:
            window.sig_update_stage.emit(base.STAGE_CONNECTION_ERROR,
                ('Incomplete Yaw recording: ' if recording_started is not None else
                 'Yaw startup/stabilization failed before recording: ') + failed)
