"""Bank-target sequence for Automatic Longitudinal Manoeuvring Stability."""
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
from backend.pid_assist import PidAssist
from backend.verifier import validate_conditions
from interfaces.engine_measurements import BODY_ACCELERATION_CHANNELS

REFERENCES = ('2.c.(6a)', '2.c.(6b)')
METHOD = 'manoeuvring_stability'
TARGETS = {'2.c.(6a)': (0.0, 15.0, 30.0, 45.0),
           '2.c.(6b)': (0.0, 15.0, 30.0)}
HOLD_SECONDS = 5.0
BANK_TOLERANCE_DEG = 2.0
MAX_BANK_DEG = 55.0
STAGE_TIMEOUT_S = 25.0


def selected(context):
    return (context.get('mode') == 'auto' and context.get('control_method') == METHOD
            and context.get('test_reference') in REFERENCES)


class AutomaticManoeuvringStability(AutomaticPowerDynamics):
    def __init__(self, context, sim, publish, *, clock=time.monotonic):
        if not selected(context):
            raise ValueError('Dedicated Automatic Manoeuvring Stability context required')
        cfg = validate_conditions(context['initial_conditions'])
        if not cfg or cfg['on_ground'] or abs(cfg['bank_deg']) > 0.5:
            raise ValueError('Manoeuvring Stability requires airborne Bank[deg] = 0 initial conditions')
        if any((context.get('active_pids') or {}).get(k) for k in ('airspeed', 'yaw')):
            raise ValueError('This sequence currently supports Bank PID only; VVI and sideslip inputs are pending')
        synthetic = dict(context, test_reference='2.c.(1)', control_method='power_dynamics',
                         initial_conditions=dict(cfg, throttle=min(cfg['throttle'], 90.0)), active_pids={})
        super().__init__(synthetic, sim, publish, clock=clock)
        self.context = dict(context)
        self.brunner = self.context.pop('brunner', None)
        self.cfg = cfg
        self.targets = TARGETS[context['test_reference']]
        self.stage_index = 0
        self.stage_started = None
        self.stable_since = None
        self.pid_assist = PidAssist(dict(self.context, active_pids={'bank': True}), owned_axis_mask=0x01)
        self.pid_assist._published_context = self.context
        self.pid_assist.heading_ctrl = None

    def status(self):
        state = super().status()
        state.update(self.context)
        state['telemetry'] = dict(state.get('telemetry') or {},
                                  heading_deg=finite(self.latest, 'heading_deg'))
        state.update(can_record=False, recording=self.phase == 'BANK_SEQUENCE',
                     controlling=self.phase in ('PREPARE', 'BANK_SEQUENCE'),
                     controls_text='Bank PID: staged 0, 15, 30' + (', 45 deg' if len(self.targets) == 4 else ' deg'),
                     recording_text=self.stage, progress_text=self.details)
        return state

    def command(self, command):
        if command.get('action') in ('stop', 'shutdown', 'invalidate_initialization'):
            self.finish('ABORTED', 'Operator stopped Manoeuvring Stability')
            if command.get('action') == 'shutdown':
                self.closed = True
            self.emit()

    def _begin(self, now):
        if not self.brunner or not getattr(self.brunner, 'sock', None):
            raise RuntimeError('Bank PID requires CLS2Sim connection')
        self.pid_assist.begin_stabilization(now, self.brunner)
        self.sim.unpause_sim()
        self.phase = 'BANK_SEQUENCE'
        self.start_time = self.stage_started = now
        self.last_sim_time = self.latest.get('sim_time')
        self.last_sim_advance = now
        self.stage = 'BANK TARGET 0 DEG'
        self.details = 'Acquiring 0 deg bank'

    def step(self, frame):
        if self.phase not in ('BANK_SEQUENCE', 'DONE'):
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
                try:
                    self._begin(self.clock())
                except Exception as exc:
                    self.finish('FAILED', str(exc))
                self.emit()
            return
        if self.phase == 'DONE':
            return
        now = self.clock()
        try:
            if not frame or not (-0.2 <= now - frame.get('receipt_time', -1.e6) <= 1.0):
                if now - (self.last_sample_time or self.start_time) > 1.5:
                    raise RuntimeError('Fresh P3D telemetry unavailable during bank sequence')
                return
            self._ensure_live(frame, now)
            if frame['sequence'] == self.last_sequence:
                return
            self.last_sequence, self.latest, self.last_sample_time = frame['sequence'], frame, now
            bank = finite(frame, 'bank_deg')
            if bank is None or abs(bank) > MAX_BANK_DEG:
                raise RuntimeError('Measured bank exceeded safe limit or became unavailable')
            target = self.targets[self.stage_index]
            self.pid_assist.target_bank = target
            # P3D channel is left-positive; BankPID expects right-positive.
            self.pid_assist.update(now, dict(frame, bank_deg=-bank), self.brunner)
            self._record_frame(frame, now)
            self.rows[-1]['Bank Target (deg)'] = target
            measured = -bank
            if abs(measured - target) <= BANK_TOLERANCE_DEG:
                if self.stable_since is None:
                    self.stable_since = now
                held = now - self.stable_since
                self.details = f'Bank {measured:+.1f}/{target:+.0f} deg; hold {held:.1f}/{HOLD_SECONDS:.0f} s'
                if held >= HOLD_SECONDS:
                    self.stage_index += 1
                    if self.stage_index == len(self.targets):
                        self.finish('COMPLETE', 'All bank targets held for five continuous seconds')
                        return
                    self.stage_started = now
                    self.stable_since = None
                    self.stage = f'BANK TARGET {self.targets[self.stage_index]:.0f} DEG'
            else:
                self.stable_since = None
                self.details = f'Acquiring bank {target:.0f} deg; measured {measured:+.1f} deg'
            if now - self.stage_started > STAGE_TIMEOUT_S:
                raise RuntimeError(f'Bank target {target:.0f} deg did not stabilize within {STAGE_TIMEOUT_S:.0f} s')
        except Exception as exc:
            self.finish('FAILED', str(exc))
        finally:
            self.emit()

    def finish(self, result, reason):
        if self.phase == 'DONE':
            return
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
                self.stage, self.details = 'FAILED', reason + '; reporting error: ' + str(exc)
        self.emit()


def save_report(context, rows, result, reason):
    folder = Path(context['result_directory'])
    folder.mkdir(parents=True, exist_ok=True)
    variant = 'cruise' if context['test_reference'].endswith('(6a)') else 'approach'
    stem = folder / (f'2_c_6_manoeuvring_stability_{variant}_auto_'
                     + datetime.now().strftime('%Y-%m-%d_%H-%M-%S') + '_' + str(time.time_ns()))
    csv_path, meta_path = stem.with_suffix('.csv'), stem.with_suffix('.qtg_test.json')
    pdf_path = stem.with_name(stem.name + '_graphs.pdf')
    with csv_path.open('x', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    metadata = dict(context, result=result, reason=reason, status=result.lower(),
                    kind='Longitudinal Manoeuvring Stability', bank_targets_deg=TARGETS[context['test_reference']])
    meta_path.write_text(json.dumps(metadata, indent=2, allow_nan=False), encoding='utf-8')
    from reports.plot_graphs import _render_report
    _render_report(sim_path=csv_path, ref_path=None, out_path=pdf_path,
                   titt_results=None, metadata=metadata, selected_graphs=context.get('selected_graphs'))
    return [str(csv_path), str(meta_path), str(pdf_path)]


def run(context, publish, *, factory=None, reader_factory=None, sleep=time.sleep):
    sim = reader = controller = brunner = None
    try:
        if factory is None:
            root = Path(sys._MEIPASS) if getattr(sys, 'frozen', False) else Path(__file__).resolve().parents[1]
            spec = importlib.util.spec_from_file_location('qtg_manoeuvre_p3d', root/'interfaces/brunner_base/src/p3d_simconnect_v15.py')
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            factory = module.SimConnectClient
        sim = factory()
        from interfaces.brunner_base.src.brunner_interface import BrunnerInterface
        brunner = BrunnerInterface(timeout=0.05)
        brunner.connect()
        controller = AutomaticManoeuvringStability(dict(context, brunner=brunner), sim, publish)
        controller.emit()
        threading.Thread(target=sim.connect_and_run,
                         kwargs=dict(place_at_rahwali=False, place_airborne=False), daemon=True).start()
        if reader_factory is None:
            from interfaces.engine_measurements import Measurements, CHANNELS
            root = Path(sys._MEIPASS) if getattr(sys, 'frozen', False) else Path(__file__).resolve().parents[1]
            reader_factory = lambda: Measurements(root/'bridge/bin/qtg_simconnect.dll',
                                                   channels=CHANNELS + BODY_ACCELERATION_CHANNELS)
        reader = reader_factory()
        directory = Path(context['command_directory'])
        while not controller.closed:
            for path in sorted(directory.glob('*.json')):
                command = json.loads(path.read_text(encoding='utf-8'))
                path.unlink()
                controller.command(command)
            stop = Path(context['stop_file'])
            if stop.exists():
                stop.unlink()
                controller.command({'action': 'stop'})
            if controller.phase != 'DONE':
                controller.step(reader.poll())
            sleep(0.04)
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
        if brunner:
            brunner.close()
        if sim:
            sim.stop()
