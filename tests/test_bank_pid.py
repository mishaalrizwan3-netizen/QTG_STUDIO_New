"""Offline Bank integration tests. All simulator and socket objects are fakes."""
import ast
import hashlib
import math
from pathlib import Path
import struct
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/'interfaces/brunner_base/src'))
from backend.bank_pid import BankPID, slew_position
from backend import bank_pid_runner as runner
from interfaces.bank_pid_transport import BankRollTransport
from brunner_interface import BrunnerInterface
import qtg_common_engine as engine


class Socket:
    def __init__(self):
        self.blocking=True
        self.packets=[]
        self.position=.12
        self.bad_feedback=False
        self.reject=False
        self.fail_release=False
    def setblocking(self,value): self.blocking=value
    def settimeout(self,value): self.blocking=True
    def send(self,packet): self.packets.append(packet)
    def recv(self,size):
        if not self.blocking: raise BlockingIOError()
        packet=self.packets[-1]
        cmd=struct.unpack_from('<I',packet)[0]
        if cmd==0xD0:
            if self.bad_feedback: return b'\x01\x00\x00'
            return struct.pack('<HBHf',7,0,1,self.position)
        if self.reject: return b'\x01\x00\x01'
        if self.fail_release and cmd==0xD1: return b'\x01\x00\x01'
        return b'\x01\x00\x00'


def device():
    d=BrunnerInterface();d.sock=Socket()
    return d


class Sim:
    def __init__(self, bank=0.0):
        self.sequence = 0
        self.connected = True
        self.paused = False
        self.bank = bank
    def is_connected(self): return self.connected
    def get_telemetry_sequence(self): return self.sequence
    def request_telemetry_once(self): self.sequence += 1; return True
    def wait_for_fresh_telemetry(self, sequence, timeout):
        return {'bank_angle': self.bank, 'airspeed_indicated': 80.}
    def unpause_sim(self): self.paused = False
    def pause_sim(self): self.paused = True
    def simulation_running(self): return not self.paused


def window(folder, bank=0.0):
    d = device()
    event = threading.Event()
    d.read_axis = Mock(return_value=(.12, 3., 4.))
    d._bank_transport = BankRollTransport(d, event)
    w = SimpleNamespace(
        brunner=d, p3d=Sim(bank=bank), stop_worker_event=event,
        _session_context={
            'bank_parent_pid': 123,
            'bank_hold_sec': 0.05,
            'bank_acquisition_timeout_sec': 1.0,
        },
        target_duration_sec=.075,
        data_dir=Path(folder), _outputs=[], sig_recording_finished=Mock(),
        sig_update_stage=Mock(), sig_record_progress=Mock(), is_recording=False
    )
    for name in ('rec_time', 'p_pos_buf', 'r_pos_buf', 'y_pos_buf', 'p_force_buf',
                 'r_force_buf', 'y_force_buf', 'p3d_tele_buf', 'applied_force_buf',
                 'virtual_force_buf', 'replay_command_buf'):
        setattr(w, name, [])
    return w

BASE=SimpleNamespace(STAGE_CONNECTION_ERROR='Connection error')


class AlgorithmTests(unittest.TestCase):
    def test_extracted_bank_matches_reference_ast_fingerprint(self):
        tree=ast.parse((ROOT/'backend/bank_pid.py').read_text(encoding='utf-8'))
        node=next(n for n in tree.body if isinstance(n,ast.ClassDef))
        self.assertEqual(hashlib.sha256(ast.dump(node).encode()).hexdigest()[:12],'3ed8076dc0bc')

    def test_zero_target_and_sign(self):
        self.assertLess(BankPID().update(0.,10.,.04),0.)
        self.assertGreater(BankPID().update(0.,-10.,.04),0.)
        self.assertEqual(BankPID().update(0.,0.,.04),0.)

    def test_authority_integral_filter_and_slew(self):
        pid=BankPID()
        self.assertEqual(pid.update(0.,-100.,.04),.65)
        for _ in range(10000): pid.update(0.,-3.,.04)
        self.assertLessEqual(abs(pid.integral),.25)
        pid.reset();pid.update(0.,0.,.04);pid.update(0.,1.,.04)
        self.assertAlmostEqual(pid.bank_rate,25./3.)
        p,v=slew_position(0.,0.,.65,.04,max_rate=.28,max_accel=.90)
        self.assertAlmostEqual(v,.036);self.assertAlmostEqual(p,.00144)

    def test_zero_deadband(self):
        self.assertNotEqual(BankPID().update(0.,.001,.04),0.)


class TransportTests(unittest.TestCase):
    def setUp(self):
        self.d=device();self.stop=threading.Event();self.t=BankRollTransport(self.d,self.stop)

    def test_shared_connection_engagement_and_roll_only_commands(self):
        self.assertIs(self.t.device.sock,self.d.sock)
        start=self.t.engage();self.assertAlmostEqual(start,.12)
        self.t.command(-.1)
        settings=[p for p in self.d.sock.packets if struct.unpack_from('<I',p)[0]==0xCE]
        self.assertTrue(settings)
        self.assertTrue(all(struct.unpack_from('<I',p,4)[0]==2 for p in settings))
        self.assertIn(struct.pack('<IIIH',0xCE,2,0x50,2400),settings)
        self.assertIn(struct.pack('<IIIH',0xCE,2,0x60,80),settings)
        first_pos=next(p for p in settings if struct.unpack_from('<I',p,8)[0]==0x80)
        self.assertAlmostEqual(struct.unpack_from('<f',first_pos,12)[0],start)

    def test_release_never_centers_and_cannot_rearm(self):
        self.t.engage();self.d.sock.packets.clear()
        self.t.release();self.t.release()
        packets=self.d.sock.packets
        self.assertFalse(any(struct.unpack_from('<I',p)[0]==0xCE and struct.unpack_from('<I',p,8)[0]==0x80 for p in packets))
        self.assertEqual(packets[-1],struct.pack('<IIH',0xD8,0,0))
        with self.assertRaises(RuntimeError):self.t.command(.2)
        with self.assertRaises(RuntimeError):self.t.engage()

    def test_stop_blocks_commands_and_engagement(self):
        self.stop.set()
        with self.assertRaises(RuntimeError):self.t.engage()
        self.assertFalse(self.d.sock.packets)
        self.stop.clear();self.t.engage();self.stop.set()
        n=len(self.d.sock.packets)
        with self.assertRaises(RuntimeError):self.t.command(.2)
        self.assertEqual(n,len(self.d.sock.packets))

    def test_stale_missing_and_wrong_node_feedback(self):
        self.t.engage();self.t.feedback_time=time.monotonic()-1
        with self.assertRaisesRegex(RuntimeError,'stale'):self.t.command(.1)
        self.d.sock.bad_feedback=True
        with self.assertRaises(RuntimeError):self.t.read_position()
        self.d.sock.bad_feedback=False;self.d.sock.position=float('nan')
        with self.assertRaises(RuntimeError):self.t.read_position()
        with patch.object(self.t,'exchange',return_value=struct.pack('<HBHf',7,0,5,.1)):
            with self.assertRaisesRegex(RuntimeError,'Wrong axis'):self.t.read_position()

    def test_measured_engagement_above_pid_authority_is_not_centered(self):
        self.d.sock.position=.8
        self.assertAlmostEqual(self.t.engage(),.8)
        self.t.command(1.)
        packet=self.d.sock.packets[-1]
        self.assertAlmostEqual(struct.unpack_from('<f',packet,12)[0],.95)

    def test_disconnect_and_failed_acknowledgement(self):
        self.d.sock=None
        with self.assertRaisesRegex(RuntimeError,'disconnected'):self.t.engage()
        self.d.sock=Socket();self.d.sock.reject=True
        with self.assertRaises(RuntimeError):self.t.engage()
        self.assertTrue(self.t.closed)

    def test_competing_airspeed_owner_rejected(self):
        self.d._pid_transport=object()
        with self.assertRaisesRegex(RuntimeError,'Airspeed'):self.t.engage()
        self.assertFalse(self.d.sock.packets)

    def test_generic_release_routes_to_bank_without_centering(self):
        self.d._bank_transport=self.t;self.t.engage();self.d.sock.packets.clear()
        self.d.release_controls()
        self.assertFalse(self.t.owned)
        self.assertFalse(any(struct.unpack_from('<I',p)[0]==0xCE and struct.unpack_from('<I',p,8)[0]==0x80 for p in self.d.sock.packets))

    def test_failed_release_retains_watchdog_and_can_retry(self):
        self.t.engage();self.d.sock.packets.clear();self.d.sock.fail_release=True
        with self.assertRaises(RuntimeError):self.t.release()
        self.assertNotIn(struct.pack('<IIH',0xD8,0,0),self.d.sock.packets)
        self.assertTrue(self.t.owned)
        self.d.sock.fail_release=False;self.t.release();self.assertFalse(self.t.owned)


class LifecycleTests(unittest.TestCase):
    def setUp(self):
        loaded=patch('backend.bank_pid_runner.simulator_loaded',return_value=True)
        alive=patch('backend.bank_pid_runner.parent_alive',side_effect=lambda pid:bool(pid))
        loaded.start();alive.start()
        self.addCleanup(loaded.stop);self.addCleanup(alive.stop)

    def test_records_bank_acquisition_and_releases_roll(self):
        with tempfile.TemporaryDirectory() as folder:
            w=window(folder, bank=0.5);runner.loop(w,BASE,owner_alive=lambda pid:True)
            self.assertTrue(w.rec_time);self.assertIsNone(w.bank_fault)
            self.assertEqual(w.auto_t0_snapshot['bank_target_deg'],0.)
            self.assertLess(w.bank_live['roll_command'],.12)
            self.assertTrue(w.p3d.paused);self.assertFalse(w.brunner._bank_transport.owned)
            w.sig_recording_finished.emit.assert_called_once()
            self.assertTrue(w._session_context['bank_complete'])
            self.assertTrue(w._outputs)

    def test_faults_stop_and_release(self):
        for failure in ('stop','disconnect','missing_bank','stale','pause','gui_exit','brunner'):
            with self.subTest(failure=failure),tempfile.TemporaryDirectory() as folder:
                w=window(folder, bank=0.5)
                alive=lambda pid:True
                if failure=='stop':w.stop_worker_event.set()
                if failure=='disconnect':w.p3d.connected=False
                if failure=='missing_bank':w.p3d.bank=float('nan')
                if failure=='stale':w.p3d.wait_for_fresh_telemetry=lambda *a:None
                if failure=='pause':w.p3d.simulation_running=lambda:False
                if failure=='gui_exit':alive=lambda pid:False
                if failure=='brunner':w.brunner.sock.bad_feedback=True
                runner.loop(w,BASE,owner_alive=alive)
                self.assertTrue(w.bank_fault)
                self.assertFalse(w._session_context['bank_complete'])
                self.assertFalse(w.brunner._bank_transport.owned)
                self.assertTrue(w.stop_worker_event.is_set())
                self.assertFalse(w.bank_active)

    def test_feedback_loss_after_active_command_marks_partial_recording_incomplete(self):
        with tempfile.TemporaryDirectory() as folder:
            w=window(folder, bank=0.5)
            w._session_context['bank_hold_sec'] = 0.01
            w.target_duration_sec = 2.0
            original=w.p3d.wait_for_fresh_telemetry
            count=[0]
            def feedback(*args):
                count[0]+=1
                return None if count[0]>=5 else original(*args)
            w.p3d.wait_for_fresh_telemetry=feedback
            runner.loop(w,BASE,owner_alive=lambda pid:True)
            self.assertTrue(w.rec_time)
            self.assertIn('stale',w.bank_fault)
            self.assertFalse(w._session_context['bank_complete'])
            self.assertFalse(w.brunner._bank_transport.owned)
            w.sig_recording_finished.emit.assert_called_once()

    def test_stop_after_engagement_prevents_position_command(self):
        with tempfile.TemporaryDirectory() as folder:
            w=window(folder)
            original=w.brunner._bank_transport.engage
            def engage():
                result=original();w.stop_worker_event.set();return result
            w.brunner._bank_transport.engage=engage
            runner.loop(w,BASE,owner_alive=lambda pid:True)
            self.assertTrue(w.bank_fault)
            self.assertFalse(w.brunner._bank_transport.owned)
            positions=[p for p in w.brunner.sock.packets if struct.unpack_from('<I',p)[0]==0xCE and struct.unpack_from('<I',p,8)[0]==0x80]
            self.assertEqual(len(positions),1)  # Measured engagement seed only.

    def test_selected_controller_dispatch(self):
        parent=type('Parent',(),{'_auto_replay_loop':lambda self:'replay'})
        cls=engine.worker_class(SimpleNamespace(SuperMushakLoggerApp=parent),{},lambda payload:None)
        host=object.__new__(cls)
        with patch('backend.bank_pid_runner.loop',return_value='bank') as bank,patch('qtg_climb_pid.loop',return_value='airspeed') as air:
            host._control_method='bank_pid';self.assertEqual(host._auto_replay_loop(),'bank')
            bank.assert_called_once();air.assert_not_called()
            host._control_method='pid';self.assertEqual(host._auto_replay_loop(),'airspeed')
            host._control_method='replay';self.assertEqual(host._auto_replay_loop(),'replay')

    def test_mode_switch_is_blocked_while_worker_alive(self):
        cls=engine.worker_class(SimpleNamespace(SuperMushakLoggerApp=object),{},lambda payload:None)
        w=object.__new__(cls);w._control_method='bank_pid';w.is_recording=False
        w.worker_thread=Mock(is_alive=lambda:True);w._publish=Mock()
        w.handle_command({'action':'initialize','mode':'auto','control_method':'replay'})
        self.assertEqual(w._control_method,'bank_pid');self.assertIn('wait',w._stage_details)

    def test_stop_and_shutdown_use_existing_worker_stop(self):
        def stop_test(w):
            w.stop_worker_event.set();w.brunner.release_controls()
        parent=type('Parent',(),{'stop_test':stop_test})
        for action in ('stop','shutdown'):
            cls=engine.worker_class(SimpleNamespace(SuperMushakLoggerApp=parent),{},lambda payload:None)
            w=object.__new__(cls);w._control_method='bank_pid';w._session_context={}
            w.stop_worker_event=threading.Event();w.brunner=device()
            t=BankRollTransport(w.brunner,w.stop_worker_event);w.brunner._bank_transport=t;t.engage()
            w.handle_command({'action':action})
            self.assertTrue(w.stop_worker_event.is_set());self.assertFalse(t.owned)
            if action=='shutdown':self.assertTrue(w._shutting_down)

    def test_initialization_dispatch_reuses_base_manual_only(self):
        base=SimpleNamespace(SuperMushakLoggerApp=SimpleNamespace(start_manual_test=Mock()))
        w=SimpleNamespace(_session_context={'initial_conditions':{'bank_deg':10}},
            bank_active=False,brunner=device(),stop_worker_event=threading.Event())
        runner.prepare(w,base)
        base.SuperMushakLoggerApp.start_manual_test.assert_called_once_with(w)
        self.assertFalse(w.brunner.sock.packets)  # No Bank motor engagement before readiness.

    def test_bank_allowed_every_test_but_airspeed_policy_unchanged(self):
        from backend.catalog import TESTS
        from backend.control import validate_method
        for spec in TESTS:self.assertEqual(validate_method(spec.reference,'bank_pid'),'bank_pid')
        with self.assertRaises(ValueError):validate_method('2.c.(1)','pid')
        with self.assertRaises(ValueError):validate_method('1.c.(1)','invalid_method')

    def test_dropdown_contains_bank_in_both_branches_and_no_yaw(self):
        tree=ast.parse((ROOT/'frontend/main.py').read_text(encoding='utf-8'))
        assignments=[n for n in ast.walk(tree) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='methods' for t in n.targets)]
        choice=assignments[0].value
        for node in (choice.body,choice.orelse):
            labels=ast.literal_eval(node)
            self.assertIn('Bank PID',labels);self.assertIn('Recorded Replay',labels)
            self.assertIn('Yaw PID',labels)

    def test_bank_pdf_routing_needs_no_reference_and_marks_fault(self):
        import importlib.util
        path=ROOT/'interfaces/brunner_base/src/plot_graphs.py'
        spec=importlib.util.spec_from_file_location('bank_plot_test',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with patch('reports.bank_pid_report.report',return_value='bank.pdf') as render:
            self.assertEqual(module.plot_aligned_comparison('bank.csv',control_method='bank_pid',test_reference='1.f.(2)',run_fault='Stopped'),'bank.pdf')
            self.assertEqual(render.call_args.kwargs['run_fault'],'Stopped')


    def test_gui_launches_bank_for_each_executable_test_without_reference(self):
        from frontend import main
        from backend.catalog import TESTS
        values={'Alt[ft]':'3000','IAS[kts]':'80','Latitude[deg]':'32',
                'Longitude[deg]':'74','Heading[deg]':'270','Pitch[deg]':'0',
                'Bank[deg]':'10','Flaps':'0','Throttle[%]':'100',
                'Temperature[C]':'15','Flt cond':'Clean'}
        for spec in TESTS:
            if spec.reference=='1.c.(2)':continue  # Existing settings-only test.
            with self.subTest(reference=spec.reference):
                fields={key:Mock(get=lambda v=v:v) for key,v in values.items()}
                host=SimpleNamespace(_common_duration_var=Mock(get=lambda:'10'),
                    _automatic_methods={spec.reference:Mock(get=lambda:'Bank PID')},
                    _test_settings={spec.reference:(fields,{})},
                    _test_defaults={spec.reference:dict(values, **{'Bank[deg]':'0'})},
                    _apply_test_settings=lambda spec:True,_common_process=None,
                    common_record_button=Mock(),common_status=Mock(),after=Mock(),
                    _poll_common_test=Mock())
                with patch.object(engine,'launch',return_value=(Mock(),{})) as launch:
                    main.App._launch_common_test(host,'auto',spec)
                launch.assert_called_once()
                self.assertEqual(launch.call_args.kwargs['control_method'],'bank_pid')
                self.assertIsNone(launch.call_args.args[3])
                self.assertEqual(float(launch.call_args.kwargs['initial_conditions']['bank_deg']),10.)
                self.assertIsNone(launch.call_args.kwargs['pid_conditions'])

    def test_initialize_mode_bank_never_enters_replay_or_airspeed(self):
        w=SimpleNamespace(_begin_initialization=Mock(),_control_method='bank_pid',
                          brunner=device(),p3d=Sim(),_base_module=BASE,_session_context={})
        with patch('backend.engine_initialization_adapter.validate_trial'), \
             patch.object(engine,'configure_test_extension') as extensions, \
             patch('backend.bank_pid_runner.prepare') as prepare, \
             patch.object(engine,'_initialize_mode') as old_init:
            engine.initialize_mode(w,'auto')
        prepare.assert_called_once_with(w,BASE)
        old_init.assert_not_called();extensions.assert_called_once_with(w,None)

    def test_automatic_waits_for_preparation_thread_before_arming(self):
        parent=type('Parent',(),{'arm_recording':lambda self:setattr(self,'armed',True),
                                '_on_ui_timer_tick':lambda self:None})
        cls=engine.worker_class(SimpleNamespace(SuperMushakLoggerApp=parent),{},lambda p:None)
        w=object.__new__(cls);w._control_method='bank_pid';w._prepared=True
        w.stop_worker_event=threading.Event();w.is_recording=False;w._session_context={}
        w.preparation_thread=Mock(is_alive=lambda:True)
        w.arm_recording();self.assertFalse(getattr(w,'armed',False))
        w.preparation_thread=Mock(is_alive=lambda:False)
        for name in ('rec_time','p_pos_buf','r_pos_buf','y_pos_buf','p_force_buf','r_force_buf','y_force_buf','p3d_tele_buf','applied_force_buf','virtual_force_buf','replay_command_buf'):
            setattr(w,name,[])
        with patch('qtg_common_engine.threading.Thread') as worker:
            w.arm_recording()
            worker.return_value.start.assert_called_once()
        self.assertFalse(w.is_recording)
        self.assertFalse(getattr(w,'armed',False))
        self.assertEqual(w.test_mode,'auto');self.assertFalse(w._prepared)

    def test_bank_pdf_honors_selected_channels_and_fault_label(self):
        from reports.bank_pid_report import report
        import pymupdf
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'bank.csv'
            p.write_text('Time (s),Bank Angle (deg),Joystick Roll Position (mm)\n0,10,0.1\n1,8,0.05\n')
            output=report(p,selected_graphs=['bank_deg'],run_fault='Emergency Stop')
            with pymupdf.open(output) as pdf:
                self.assertEqual(len(pdf),2)
                text=''.join(page.get_text() for page in pdf)
                self.assertIn('INTERRUPTED: Emergency Stop',text)
                self.assertTrue('Bank Angle (°)' in text or 'Bank Angle (deg)' in text)
                self.assertNotIn('Joystick Roll Position',text)
            output=report(p,selected_graphs=[])
            with pymupdf.open(output) as pdf:self.assertEqual(len(pdf),2)

    def test_parent_process_liveness(self):
        import os
        self.assertTrue(runner.parent_alive(os.getpid()))
        self.assertFalse(runner.parent_alive(None))

    def test_bank_stabilization_gate_tolerance_and_reset(self):
        gate = runner.BankStabilizationGate(tolerance_deg=1.0, hold_s=5.0)
        # Bank at 0.5° (within tolerance)
        self.assertFalse(gate.update(100.0, 0.5))
        self.assertAlmostEqual(gate.elapsed_stable(100.0), 0.0)
        self.assertFalse(gate.update(102.5, 0.8))
        self.assertAlmostEqual(gate.elapsed_stable(102.5), 2.5)
        # Exceeds tolerance (+1.5°) -> resets immediately
        self.assertFalse(gate.update(103.0, 1.5))
        self.assertAlmostEqual(gate.elapsed_stable(103.0), 0.0)
        self.assertIsNone(gate.since)
        # Re-enters tolerance at 104.0
        self.assertFalse(gate.update(104.0, -0.2))
        self.assertFalse(gate.update(108.9, 0.9))
        self.assertAlmostEqual(gate.elapsed_stable(108.9), 4.9)
        # Completes 5.0 continuous seconds at 109.0
        self.assertTrue(gate.update(109.0, 0.0))
        self.assertAlmostEqual(gate.elapsed_stable(109.0), 5.0)

    def test_bank_acquisition_timeout_when_bank_unstable(self):
        with tempfile.TemporaryDirectory() as folder:
            w = window(folder, bank=5.0)  # Outside ±1.0°
            w._session_context['bank_acquisition_timeout_sec'] = 0.05
            runner.loop(w, BASE, owner_alive=lambda pid: True)
            self.assertTrue(w.bank_fault)
            self.assertIn('Acquisition timeout', w.bank_fault)
            self.assertFalse(w._session_context['bank_complete'])
            # Recording must NEVER start if bank could not stabilize
            self.assertFalse(w.rec_time)
            self.assertFalse(w.is_recording)

    def test_bank_pause_during_recording_aborts_test(self):
        with tempfile.TemporaryDirectory() as folder:
            w = window(folder, bank=0.5)
            w._session_context['bank_hold_sec'] = 0.01
            w._session_context['bank_recording_pause_timeout_sec'] = 0.05
            sim = w.p3d
            orig_running = sim.simulation_running
            def running_check():
                if w.is_recording:
                    return False
                return orig_running()
            sim.simulation_running = running_check
            runner.loop(w, BASE, owner_alive=lambda pid: True)
            self.assertTrue(w.bank_fault)
            self.assertIn('paused during Bank recording', w.bank_fault)
            self.assertFalse(w._session_context['bank_complete'])

    def test_bank_transient_pause_during_recording_recovers(self):
        with tempfile.TemporaryDirectory() as folder:
            w = window(folder, bank=0.5)
            w._session_context['bank_hold_sec'] = 0.01
            w._session_context['bank_recording_pause_timeout_sec'] = 2.0
            w.target_duration_sec = 0.10
            sim = w.p3d
            orig_running = sim.simulation_running
            pause_count = [0]
            def transient_pause_check():
                if w.is_recording and pause_count[0] < 2:
                    pause_count[0] += 1
                    return False  # momentarily paused for 2 ticks
                return orig_running()
            sim.simulation_running = transient_pause_check
            runner.loop(w, BASE, owner_alive=lambda pid: True)
            self.assertIsNone(w.bank_fault)
            self.assertTrue(w._session_context['bank_complete'])

    def test_diagnostics_csv_columns_and_data(self):
        import csv
        with tempfile.TemporaryDirectory() as folder:
            w = window(folder, bank=0.5)
            runner.loop(w, BASE, owner_alive=lambda pid: True)
            self.assertTrue(w._outputs)
            diag_file = next(Path(p) for p in w._outputs if 'diagnostics' in Path(p).name)
            with diag_file.open('r', encoding='utf-8') as f:
                reader = csv.DictReader(f)
                rows = list(reader)
            self.assertTrue(len(rows) > 0)
            first = rows[0]
            required_cols = [
                'time_s', 'recording_time_s', 'phase', 'target_bank_deg',
                'bank_deg', 'bank_error_deg', 'pid_p_term', 'pid_i_term',
                'pid_d_term', 'raw_command', 'roll_command', 'roll_measured',
                'tracking_error', 'p3d_paused', 'controller_engaged',
                'stability_timer_s', 'recording_started_s', 'stop_fault_reason'
            ]
            for col in required_cols:
                self.assertIn(col, first, f"Missing required diagnostic column: {col}")


if __name__=='__main__':unittest.main()
