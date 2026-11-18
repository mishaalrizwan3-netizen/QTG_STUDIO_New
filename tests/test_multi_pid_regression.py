"""Offline API-contract and combined-loop regressions; no hardware connections."""
import ast
import inspect
from itertools import combinations
from pathlib import Path
import tempfile
from types import SimpleNamespace
import threading
import unittest
from unittest.mock import Mock, patch
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'tests'))
sys.path.insert(0, str(ROOT / 'interfaces/brunner_base/src'))

try:
    from test_multi_pid import FakeSim, make_fake_brunner
except ImportError:
    from tests.test_multi_pid import FakeSim, make_fake_brunner
from backend import multi_pid_runner as runner
from backend.control import method_label
from p3d_simconnect_v15 import SimConnectClient

COMBINATIONS = [dict(airspeed=a,bank=b,yaw=y) for a,b,y in
                ((True,True,False),(True,False,True),(False,True,True),(True,True,True))]

class CombinedRegressionTests(unittest.TestCase):
    def test_real_signature_and_fake_reject_wrong_keyword(self):
        inspect.signature(SimConnectClient.wait_for_fresh_telemetry).bind(None,1,timeout_sec=.25)
        with self.assertRaises(TypeError):
            inspect.signature(SimConnectClient.wait_for_fresh_telemetry).bind(None,1,timeout=.25)
        w=SimpleNamespace(p3d=FakeSim())
        self.assertEqual(runner.fresh_telemetry(w,timeout_s=.25)['airspeed_indicated'],80.)

    def test_no_telemetry_call_uses_unsupported_timeout_keyword(self):
        root=Path(__file__).resolve().parents[1]
        for folder in ('backend','frontend','interfaces'):
            for path in (root/folder).rglob('*.py'):
                tree=ast.parse(path.read_text(encoding='utf-8-sig'))
                for node in ast.walk(tree):
                    if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute) and node.func.attr=='wait_for_fresh_telemetry':
                        self.assertNotIn('timeout',[k.arg for k in node.keywords],str(path))

    def test_labels_individual_replay_and_all_combinations(self):
        for method,label in [('replay','Recorded Replay'),('pid','Airspeed PID'),('bank_pid','Bank PID'),('yaw_pid','Yaw PID')]:
            self.assertEqual(method_label(dict(control_method=method)),label)
        for flags in COMBINATIONS:
            label=method_label(dict(control_method='multi_pid',active_pids=flags))
            self.assertNotIn('Replay',label)
            for key,name in [('airspeed','Airspeed'),('bank','Bank'),('yaw','Yaw')]:
                self.assertEqual(name in label,flags[key])

    def test_all_combined_loops_five_second_gate_axis_ownership_and_release(self):
        for flags in COMBINATIONS:
            with self.subTest(flags=flags),tempfile.TemporaryDirectory() as folder:
                clock=[0.]
                def sleep(seconds):clock[0]+=max(.02,seconds)
                device=make_fake_brunner()
                shared=device.sock
                w=SimpleNamespace(brunner=device,p3d=FakeSim(),stop_worker_event=threading.Event(),
                    _control_method='multi_pid',_session_context=dict(active_pids=flags,pid_conditions={'target_ias':80.}),
                    target_duration_sec=.08,is_recording=False,data_dir=Path(folder),_outputs=[],
                    sig_update_stage=Mock(),sig_recording_finished=Mock())
                for name in ('rec_time','p_pos_buf','r_pos_buf','y_pos_buf','p_force_buf','r_force_buf','y_force_buf'):
                    setattr(w,name,[])
                w.p3d.paused=True
                commands=[]
                original=runner.MultiPidBrunnerTransport.command
                def command(transport,**values):
                    commands.append((clock[0],dict(values)))
                    return original(transport,**values)
                with patch.object(runner.time,'monotonic',side_effect=lambda:clock[0]),patch.object(runner.time,'sleep',side_effect=sleep),patch.object(runner.MultiPidBrunnerTransport,'command',command),patch('reports.multi_pid_report.report',return_value=str(Path(folder)/'report.pdf')):
                    runner.loop(w,SimpleNamespace(STAGE_RECORDING='Recording',STAGE_INITIALIZING='Initializing',STAGE_CONNECTION_ERROR='Error'))
                self.assertIsNone(w.multi_pid_fault)
                self.assertIs(device.sock,shared)
                self.assertTrue(w.rec_time)
                self.assertGreaterEqual(clock[0],5.)
                self.assertGreaterEqual(sum(stage.args[0]=='Recording' for stage in w.sig_update_stage.emit.call_args_list),1)
                self.assertTrue(any(t<5 for t,cmd in commands)) # Control during stabilization.
                self.assertTrue(any(t>=5 for t,cmd in commands)) # Control during recording.
                for _,cmd in commands:
                    self.assertEqual(cmd['pitch_pos'] is not None,flags['airspeed'])
                    self.assertIsNotNone(cmd['roll_pos']) # One combined roll target per cycle.
                    self.assertEqual(cmd['rudder_pos'] is not None,flags['yaw'])
                self.assertFalse(w.multi_pid_transport.owned)
                w.sig_recording_finished.emit.assert_called_once()


    def test_shared_stop_release_selects_combined_owner_without_centering(self):
        for flags in COMBINATIONS:
            with self.subTest(flags=flags):
                d=make_fake_brunner()
                from interfaces.multi_pid_transport import MultiPidBrunnerTransport
                transport=MultiPidBrunnerTransport(d,threading.Event(),pitch_enabled=flags['airspeed'],bank_enabled=flags['bank'],yaw_enabled=flags['yaw'])
                d._multi_pid_transport=transport
                d._bank_transport=Mock() # Preparation may leave an inactive Bank adapter.
                transport.engage()
                d.sock.packets.clear()
                d.release_controls()
                self.assertFalse(transport.owned)
                d._bank_transport.release.assert_not_called()
                import struct
                positions=[p for p in d.sock.packets if struct.unpack_from('<I',p)[0]==0xCE and struct.unpack_from('<I',p,8)[0]==0x80]
                self.assertFalse(positions)
                with self.assertRaises(RuntimeError):transport.command(roll_pos=.1)


    def test_acquisition_can_exceed_ninety_seconds_then_records_with_all_pids_active(self):
        with tempfile.TemporaryDirectory() as folder:
            clock=[0.]
            device=make_fake_brunner()
            sim=FakeSim(ias=84.9)
            original_fresh=sim.wait_for_fresh_telemetry
            def fresh(sequence,timeout_sec=1.):
                sim.ias=84.9 if clock[0]<95. else 80.
                return original_fresh(sequence,timeout_sec)
            sim.wait_for_fresh_telemetry=fresh
            w=SimpleNamespace(brunner=device,p3d=sim,stop_worker_event=threading.Event(),
                _control_method='multi_pid',_session_context=dict(active_pids=dict(airspeed=True,bank=True,yaw=True),pid_conditions={'target_ias':80.},multi_pid_timeout_sec=90.),
                target_duration_sec=.3,is_recording=False,data_dir=Path(folder),_outputs=[],
                sig_update_stage=Mock(),sig_recording_finished=Mock())
            for name in ('rec_time','p_pos_buf','r_pos_buf','y_pos_buf','p_force_buf','r_force_buf','y_force_buf'):
                setattr(w,name,[])
            recording_at=[]
            original_command=runner.MultiPidBrunnerTransport.command
            def command(transport,**values):
                original_command(transport,**values)
                for key,attr in [('pitch_pos','pitch_pos'),('roll_pos','roll_pos'),('rudder_pos','rudder_pos')]:
                    value=values[key]
                    self.assertIsNotNone(value)
                    setattr(device.sock,attr,value)
                if w.is_recording:recording_at.append(clock[0])
            def sleep(seconds):
                self.assertFalse(w.is_recording and clock[0]<100.)
                clock[0]+=max(.1,seconds)
            with patch.object(runner.time,'monotonic',side_effect=lambda:clock[0]),patch.object(runner.time,'sleep',side_effect=sleep),patch.object(runner.MultiPidBrunnerTransport,'command',command),patch('reports.multi_pid_report.report',return_value=str(Path(folder)/'report.pdf')):
                runner.loop(w,SimpleNamespace(STAGE_RECORDING='Recording',STAGE_INITIALIZING='Initializing',STAGE_CONNECTION_ERROR='Error'))
            self.assertIsNone(w.multi_pid_fault)
            self.assertTrue(w.rec_time)
            self.assertTrue(recording_at)
            self.assertGreaterEqual(min(recording_at),100.)
            self.assertFalse(w.multi_pid_transport.owned)
            w.sig_recording_finished.emit.assert_called_once()

if __name__=='__main__':unittest.main()
