"""Bank-only worker lifecycle; existing initialization and CSV capture are reused."""
import csv
import ctypes
import json
import math
import os
from pathlib import Path
import time

from backend.bank_pid import BankPID, slew_position
from interfaces.bank_pid_transport import BankRollTransport


class BankStabilizationGate:
    """Monotonic-time state: bank must remain within tolerance continuously for hold_s."""
    def __init__(self, tolerance_deg: float = 1.0, hold_s: float = 5.0):
        self.tolerance_deg = float(tolerance_deg)
        self.hold_s = float(hold_s)
        self.since = None

    def reset(self):
        self.since = None

    def update(self, now: float, bank_deg: float) -> bool:
        if not math.isfinite(bank_deg):
            raise RuntimeError('Missing/non-finite Bank feedback: invalid bank angle')
        if abs(bank_deg) <= self.tolerance_deg:
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
            for key in ('bank_angle', 'airspeed_indicated'):
                if not math.isfinite(float(data.get(key, float('nan')))):
                    raise RuntimeError('Missing feedback: non-finite telemetry ' + key)
            return data
        if attempt < max_retries and not window.stop_worker_event.is_set():
            window.stop_worker_event.wait(0.05)
            continue
    raise RuntimeError('Missing feedback: Sim Flight Model feedback stale')


def prepare(window, base):
    if window._session_context.get('initial_conditions') is None:
        raise RuntimeError('Bank PID requires complete selected initial conditions')
    window.bank_fault = None
    window.bank_active = False
    window.bank_live = {}
    window.active_manual_ref_path = None
    window.selected_ref_csv = None
    window.brunner._bank_transport = BankRollTransport(window.brunner, window.stop_worker_event)
    # Exact existing Manual preparation, without replay motors or a PID-specific teleport.
    base.SuperMushakLoggerApp.start_manual_test(window)


def loop(window, base, owner_alive=parent_alive):
    device = window.brunner
    transport = getattr(device, '_bank_transport', None)
    if transport is None:
        transport = BankRollTransport(device, window.stop_worker_event)
        device._bank_transport = transport

    rows = []
    failed = None
    recording_started = None
    context = getattr(window, '_session_context', {})
    tolerance_deg = float(context.get('bank_tolerance_deg', 1.0))
    hold_duration_s = float(context.get('bank_hold_sec', 5.0))
    timeout_duration_s = float(context.get('bank_acquisition_timeout_sec', 45.0))

    try:
        if window.stop_worker_event.is_set():
            raise RuntimeError('Bank start cancelled')
        if not owner_alive(context.get('bank_parent_pid')):
            raise RuntimeError('QTG GUI process exited')

        # 1. Command unpause after QTG initialization.
        # QTG initialization teleports aircraft and leaves P3D paused.
        # Unpause and wait up to 20 seconds for steady running state.
        if not window.p3d or not window.p3d.is_connected():
            raise RuntimeError('P3D readiness failure: Sim Flight Model disconnected')

        window.p3d.unpause_sim()

        _unpause_timeout_s = float(context.get('bank_unpause_timeout_sec', 20.0))
        _unpause_deadline = time.monotonic() + _unpause_timeout_s
        _running_since = None
        _pause_retries = 0
        while True:
            if window.stop_worker_event.wait(0.05):
                raise RuntimeError('Bank start cancelled during P3D unpause wait')
            if not window.p3d.is_connected():
                raise RuntimeError('P3D readiness failure: Sim Flight Model disconnected during unpause')
            if window.p3d.simulation_running():
                if _running_since is None:
                    _running_since = time.monotonic()
                elif time.monotonic() - _running_since >= 0.25:
                    break
            else:
                _running_since = None
                # Allow retries if simulator paused itself during initialization reload
                if _pause_retries < 2 and time.monotonic() < _unpause_deadline - 5.0:
                    _pause_retries += 1
                    window.p3d.unpause_sim()
            if time.monotonic() > _unpause_deadline:
                raise RuntimeError(
                    f'P3D readiness failure: simulator did not confirm steady running state within {int(_unpause_timeout_s)}s'
                )

        # 2. Verify fresh telemetry after unpause confirms flight model is streaming
        fresh(window, timeout_s=2.0, max_retries=3)

        # 3. Verify BRUNNER roll feedback and control readiness before engaging
        try:
            initial_stick = transport.read_position()
        except Exception as exc:
            raise RuntimeError(f'BRUNNER readiness failure: {exc}')
        if not math.isfinite(initial_stick):
            raise RuntimeError('BRUNNER readiness failure: non-finite initial roll feedback')

        # 4. Engage Bank PID immediately after readiness is confirmed
        position = transport.engage()
        pid = BankPID()
        pid.reset(initial_output=position)
        velocity = 0.0
        mismatch = None
        window.bank_active = window._external_control_active = True

        gate = BankStabilizationGate(tolerance_deg=tolerance_deg, hold_s=hold_duration_s)
        phase = 'stabilizing'
        recording_started = None
        control_started = last = time.monotonic()
        paused_since = None
        recording_paused_since = None
        recording_pause_timeout_s = float(context.get('bank_recording_pause_timeout_sec', 2.0))
        pending_readiness_reason = None

        if window.stop_worker_event.is_set():
            raise RuntimeError('Bank start cancelled')

        # 5. Active control loop
        while not window.stop_worker_event.is_set():
            if not owner_alive(context.get('bank_parent_pid')):
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
                    # Debounce transient pauses during recording using the same timeout
                    if recording_paused_since is None:
                        recording_paused_since = now
                    elif now - recording_paused_since > recording_pause_timeout_s:
                        raise RuntimeError('Sim Flight Model paused during Bank recording; test interrupted')
                    # Advance recording clock so pause gap doesn't count as recording time
                    recording_started += (now - last)
                    last = time.monotonic()
                    window.stop_worker_event.wait(0.04)
                    continue
                recording_paused_since = None
                transport.release()
                window.bank_active = False
                window.bank_live = {}
                transition(window, 'WAITING_FOR_P3D', pending_readiness_reason or 'Simulator not ready')
                window.sig_update_stage.emit(getattr(base, 'STAGE_INITIALIZING', 'Initializing aircraft'),
                    'Waiting for fresh, running P3D feedback. Roll released; recording OFF. Stability timer reset.')
                transport = BankRollTransport(device, window.stop_worker_event)
                device._bank_transport = transport
                wait_ready(window, transport, unpause=False, owner_alive=owner_alive)
                pending_readiness_reason = None
                position = transport.engage()
                pid.reset(initial_output=position)
                velocity = 0.
                mismatch = None
                control_started = last = time.monotonic()
                window.bank_active = True
                transition(window, 'BANK_PID_ACTIVE')
                transition(window, 'STABILIZING')
                continue

            recording_paused_since = None

            # Fetch fresh telemetry & read current roll position
            try:
                data = fresh(window, timeout_s=1.5, max_retries=3)
            except RuntimeError as exc:
                if recording_started is not None:
                    raise
                gate.reset()
                pending_readiness_reason = str(exc)
                continue  # Release and reacquire via the single readiness handoff.
            now = time.monotonic()
            if not window.p3d.simulation_running():
                gate.reset()
                if recording_started is not None:
                    # Debounce transient pauses: only abort after sustained pause
                    if recording_paused_since is None:
                        recording_paused_since = now
                    elif now - recording_paused_since > recording_pause_timeout_s:
                        raise RuntimeError('Sim Flight Model paused during Bank recording; test interrupted')
                    # Advance recording clock so pause gap doesn't count as recording time
                    recording_started += (now - last)
                    last = time.monotonic()
                    window.stop_worker_event.wait(0.04)
                    continue
                recording_paused_since = None
                pending_readiness_reason = 'P3D paused before recording'
                continue  # Next iteration releases roll and returns startup to waiting.
            else:
                recording_paused_since = None
            measured = transport.read_position()

            raw_dt = now - last
            last = now
            if raw_dt > 1.50:
                raise RuntimeError('Bank feedback/command scheduling watchdog expired')

            dt = min(0.1, max(0.01, raw_dt))
            actual_bank = float(data['bank_angle'])
            bank_error = 0.0 - actual_bank

            # Bank PID calculation (verbatim PID Controller math)
            desired = pid.update(0.0, actual_bank, dt)
            position, velocity = slew_position(position, velocity, desired, dt,
                                               max_rate=0.28, max_accel=0.90)

            if window.stop_worker_event.is_set():
                break

            # Send roll motor command ONLY while unpaused
            transport.command(position)

            # BRUNNER tracking check (Rule 4)
            if abs(position - measured) > 0.25:
                mismatch = now if mismatch is None else mismatch
                if now - mismatch > 2.5:
                    raise RuntimeError('BRUNNER command tracking failure: roll loader did not track Bank command within 2.5s')
            else:
                mismatch = None

            # PID term contributions for detailed diagnostic logging (Rule 5)
            p_term = pid.kp * bank_error
            i_term = pid.integral
            d_term = -pid.kd * pid.bank_rate
            stiction = ((bank_error / 0.50) * 0.005) if abs(bank_error) < 0.50 else 0.0
            raw_effort = p_term + i_term + d_term + stiction
            if pid.invert:
                raw_effort = -raw_effort

            # Phase management
            if phase == 'stabilizing':
                stable = gate.update(now, actual_bank)
                stable_elapsed = gate.elapsed_stable(now)

                # Acquisition timeout check (Rule 4)
                if not stable and (now - control_started) > timeout_duration_s:
                    raise RuntimeError(
                        f'Acquisition timeout: bank did not settle within ±{tolerance_deg:.1f}° '
                        f'for {hold_duration_s:.1f} continuous seconds within {int(timeout_duration_s)}s '
                        f'(last bank: {actual_bank:+.2f}°)'
                    )

                if stable:
                    phase = 'recording'
                    recording_started = now
                    window.is_recording = True
                    window.sig_update_stage.emit(
                        getattr(base, 'STAGE_RECORDING', 'Recording'),
                        f'Bank stabilized at 0° (held within ±{tolerance_deg:.1f}° for {hold_duration_s:.1f}s); recording active.'
                    )
                    window.auto_t0_snapshot = dict(
                        telemetry=dict(data),
                        control_method='bank_pid',
                        bank_target_deg=0.0,
                        roll_pos=measured
                    )
                else:
                    window.sig_update_stage.emit(
                        getattr(base, 'STAGE_INITIALIZING', 'Initializing'),
                        f'Bank PID: Stabilizing wings level {stable_elapsed:.1f}/{hold_duration_s:.1f} s (bank {actual_bank:+.2f}°)'
                    )

            if phase == 'recording':
                rec_elapsed = now - recording_started
                if rec_elapsed >= window.target_duration_sec:
                    break

                axes = [device.read_axis(axis) for axis in (1, 2, 4)]
                axes[1] = (measured, axes[1][1], axes[1][2])
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

            # Diagnostic logging (Rule 5)
            rec_time_log = (now - recording_started) if recording_started is not None else 0.0
            row = dict(
                time_s=round(now - control_started, 4),
                recording_time_s=round(rec_time_log, 4),
                phase=phase,
                target_bank_deg=0.0,
                bank_deg=actual_bank,
                bank_error_deg=round(bank_error, 4),
                pid_p_term=round(p_term, 6),
                pid_i_term=round(i_term, 6),
                pid_d_term=round(d_term, 6),
                raw_command=round(raw_effort, 6),
                roll_command=round(position, 6),
                roll_measured=round(measured, 6),
                tracking_error=round(position - measured, 6),
                p3d_paused=is_paused,
                controller_engaged=transport.owned,
                stability_timer_s=round(gate.elapsed_stable(now), 3),
                recording_started_s=round(recording_started - control_started, 4) if recording_started else 0.0,
                dt=round(dt, 4),
            )
            rows.append(row)
            window.bank_live = row

            window.stop_worker_event.wait(max(0.0, 0.04 - (time.monotonic() - now)))

    except Exception as exc:
        failed = str(exc)
    finally:
        if window.stop_worker_event.is_set() and failed is None:
            failed = 'Stopped by user'

        # Cancel worker event before releasing hardware
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

        window.bank_fault = failed
        window.bank_active = window._external_control_active = False
        window._prepared = window._preparing = window.is_recording = False
        context.update(bank_fault=failed, bank_complete=not bool(failed), bank_target_deg=0.0)

        # Write detailed diagnostic CSV
        if rows:
            try:
                for r in rows:
                    r['stop_fault_reason'] = failed or 'Completed'
                path = Path(window.data_dir) / ('bank_pid_diagnostics_' + str(time.time_ns()) + '.csv')
                with path.open('w', newline='', encoding='utf-8') as stream:
                    writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                    writer.writeheader()
                    writer.writerows(rows)
                window._outputs.append(str(path))
                context['bank_diagnostics_path'] = str(path)
            except Exception as exc:
                failed = failed or str(exc)
                window.bank_fault = context['bank_fault'] = failed

        context['bank_complete'] = not bool(failed)
        transition(window, 'FAILED' if failed else 'COMPLETED', failed or '')
        if recording_started is not None and window.rec_time:
            window.sig_recording_finished.emit()
        if failed:
            window.sig_update_stage.emit(base.STAGE_CONNECTION_ERROR,
                ('Incomplete Bank recording: ' if recording_started is not None else
                 'Bank startup/stabilization failed before recording: ') + failed)


def transition(window, state, reason=''):
    window.bank_phase = state
    context = window._session_context
    context['bank_phase'] = state
    context.setdefault('bank_transitions', []).append(dict(state=state, time=time.monotonic(), reason=reason))


def simulator_loaded(window):
    """Dashboard SimStart/SimStop signal, distinct from the worker Pause event.

    Missing/stale IPC fails closed. No new simulator/hardware connection is created.
    """
    try:
        status = Path(window._session_context['status_file'])
        state = json.loads(status.with_suffix('.p3d_ready.json').read_text(encoding='utf-8'))
        age = time.time() - float(state['updated'])
        return (0 <= age < 2. and state['connected'] and state['sim_running']
                and state['telemetry_valid'] and 0 <= state['telemetry_age_ms'] < 1500)
    except (KeyError, OSError, ValueError, TypeError):
        return False


def wait_ready(window, transport, unpause=True, owner_alive=parent_alive):
    """Wait cancellably for simulator lifecycle readiness; never time out loading.

    A single unpause is allowed AFTER SimStart confirms simulator operation.
    Pauses during subsequent loading are observed without repeatedly unpausing.
    """
    sim = window.p3d
    sent_unpause = not unpause
    consecutive = 0
    window.sig_update_stage.emit('Initializing aircraft',
        'Waiting for P3D to finish loading. Bank PID OFF; recording OFF.')
    while not window.stop_worker_event.is_set():
        if not owner_alive(window._session_context.get('bank_parent_pid')):
            raise RuntimeError('QTG GUI process exited during P3D loading')
        if not simulator_loaded(window):
            consecutive = 0
            window.stop_worker_event.wait(.1)
            continue
        if not sent_unpause:
            sim.unpause_sim()
            sent_unpause = True
            bridge = getattr(sim, '_bridge', None)
            if bridge is not None:
                bridge.RequestSimulationRunningV1509()
        try:
            fresh(window, timeout_s=1.5, max_retries=3)
            measured = transport.read_position()
            valid = sim.simulation_running() and math.isfinite(measured) and simulator_loaded(window)
        except RuntimeError:
            valid = False
        consecutive = consecutive + 1 if valid else 0
        if consecutive >= 2:
            return
        window.stop_worker_event.wait(.04)
    raise RuntimeError('Bank readiness cancelled while waiting for P3D loading')
