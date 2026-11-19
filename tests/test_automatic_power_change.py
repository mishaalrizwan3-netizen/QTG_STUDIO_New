"""Pure offline state-machine tests: no P3D/BRUNNER/USB hardware."""
from pathlib import Path
from unittest.mock import Mock, patch
import json
import tempfile
import unittest

from backend.automatic_power_change import AutomaticPowerDynamics, selected, REFERENCE


class Clock:
    def __init__(self): self.value = 100.
    def __call__(self): return self.value
    def advance(self, seconds): self.value += seconds


class Sim:
    def __init__(self):
        self.events = []; self.connected = True; self.paused = False
        self.throttle = 50.
    def is_connected(self): return self.connected
    def pause_sim(self): self.paused=True; self.events.append(('pause',))
    def wait_for_paused(self, timeout): return self.paused
    def unpause_sim(self): self.paused=False; self.events.append(('unpause',))
    def queue_position_airborne(self, **kwargs): self.events.append(('position',kwargs)); return True
    def get_global_weather_metar(self): return 'METAR OPRW 171000Z 00000KT CAVOK 15/10 Q1013'
    def set_global_weather_metar(self,value): self.events.append(('weather',value));return True
    def set_initial_flaps(self, value): self.events.append(('flaps',value)); return True
    def set_initial_throttle(self, value): self.throttle=value;self.events.append(('initial',value));return True
    def set_replay_throttle(self, value): self.throttle=value;self.events.append(('step',value));return True


class Workflow(unittest.TestCase):
    def setUp(self):
        self.clock = Clock(); self.sim=Sim(); self.published=[]
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.ctx=dict(mode='auto',control_method='power_dynamics',test_reference=REFERENCE,
                      result_directory=self.tmp.name,
                      initial_conditions=dict(alt_ft=4000.,ias_kt=90.,lat=32.235,lon=74.135,
                                              heading_deg=48.4,pitch_deg=3.,bank_deg=0.,flaps=1.,
                                              throttle=50.,temperature_c=15.,on_ground=False),
                      selected_graphs=['ias_kt','altitude_msl_ft','pitch_deg','throttle_pct'])
        self.worker=AutomaticPowerDynamics(self.ctx,self.sim,self.published.append,clock=self.clock)
        self.count=0
    def feed(self,dt=0.5,**changes):
        self.clock.advance(dt); self.count+=1
        frame=dict(sequence=self.count,receipt_time=self.clock(),sim_time=self.clock(),
                   rpm=2000.,throttle=self.sim.throttle,ias_kt=90.,alt_ft=4000.,
                   lat=32.235,lon=74.135,
                   height_agl_ft=3000.,on_ground=0.,autopilot_active=0.,
                   flaps=1.,temperature_c=15.,pitch_deg=3.,bank_deg=0.,heading_deg=48.4,
                   prop_rpm=2000.,manifold_pressure_psi=12.,vertical_speed=0.,
                   elevator_input=0.,elevator_trim_rad=0.)
        frame.update(changes)
        self.worker.step(frame)
    def prepare(self):
        self.feed()
        for i in range(10):
            if self.worker.phase=='READY':break
            self.feed()
        self.assertEqual(self.worker.phase,'READY',self.worker.details)
    def test_only_p3d_throttle_command_and_five_second_baseline(self):
        self.prepare()
        self.assertFalse(any(e[0]=='step' for e in self.sim.events))
        self.worker.command(dict(action='record'))
        for i in range(12):
            self.feed()
            if self.worker.power_command_time is not None: break
        self.assertIsNotNone(self.worker.power_command_time)
        self.assertGreaterEqual(self.worker.power_command_time - self.worker.start_time,5.)
        self.assertEqual([e for e in self.sim.events if e[0]=='step'], [('step',100.)])
        self.assertTrue(all(e[0] in ('pause','unpause','position','flaps','initial','step','weather') for e in self.sim.events))
        self.feed()
        self.assertIsNotNone(self.worker.power_completed_time)
        self.assertIsNotNone(self.worker.power_onset_time)
        with patch('backend.automatic_power_change.save_report',return_value=['recording.csv']):
            for i in range(35):
                if self.worker.phase=='DONE':break
                self.feed()
        self.assertEqual(self.worker.result,'COMPLETE',self.worker.details)
        self.assertGreaterEqual(self.worker.clock()-self.worker.power_completed_time,15.)
        self.assertEqual(self.worker.outputs,['recording.csv'])
    def test_unstable_baseline_blocks_throttle_step(self):
        self.prepare(); self.worker.command(dict(action='record'))
        with patch('backend.automatic_power_change.save_report',return_value=[]):
            for i in range(12):
                self.feed(0.5, ias_kt=90.+i*.8)
        self.assertEqual(self.worker.result,'FAILED')
        self.assertEqual([e for e in self.sim.events if e[0]=='step'],[])
    def test_selected_pid_uses_brunner_during_existing_power_step(self):
        brunner = Mock()
        brunner.sock = True
        self.ctx['active_pids'] = {'airspeed': False, 'bank': True, 'yaw': False}
        self.ctx['brunner'] = brunner
        self.worker = AutomaticPowerDynamics(self.ctx,self.sim,self.published.append,clock=self.clock)
        self.prepare()
        self.worker.command(dict(action='record'))
        self.assertEqual(self.worker.phase, 'RECORD')
        self.feed(bank_deg=3.)
        self.assertTrue(any(call.args[0] == 0x02 and call.args[1] == 0x80
                            for call in brunner.write_setting.call_args_list))
        with patch('backend.automatic_power_change.save_report',return_value=[]):
            self.worker.command(dict(action='stop'))
        brunner.set_overrides.assert_any_call(0x02, False)
        brunner.close.assert_called_once()
    def test_autopilot_blocks_free_response(self):
        self.feed(); self.feed(autopilot_active=1.)
        self.assertEqual(self.worker.result,'FAILED')
        self.assertIn('Autopilot',self.worker.details)
    def test_stop_wins_before_step(self):
        self.prepare();self.worker.command(dict(action='record'))
        with patch('backend.automatic_power_change.save_report',return_value=[]):
            self.worker.command(dict(action='stop'))
        self.assertEqual(self.worker.result,'ABORTED')
        self.assertEqual([e for e in self.sim.events if e[0]=='step'],[])
    def test_test_selection_and_reference_isolation(self):
        self.assertTrue(selected(self.ctx))
        self.assertTrue(selected(dict(self.ctx, test_reference='2.b.(11)')))
        other=dict(self.ctx,test_reference='2.d.(1)')
        self.assertFalse(selected(other))
        from backend.control import validate_method
        self.assertEqual(validate_method(REFERENCE,'power_dynamics'),'power_dynamics')
        self.assertEqual(validate_method('2.b.(11)','power_dynamics'),'power_dynamics')
        with self.assertRaises(ValueError):validate_method('2.d.(1)','power_dynamics')
    def test_missing_aircraft_reference_is_not_claimed_as_pass(self):
        self.prepare();self.worker.command(dict(action='record'))
        for i in range(12):self.feed()
        self.feed()
        with patch('reports.plot_graphs._render_report',return_value=None):
            for i in range(35):
                if self.worker.phase=='DONE':break
                self.feed()
        self.assertEqual(self.worker.result,'COMPLETE',self.worker.details)
        self.assertEqual(len(self.worker.outputs),3)
        metadata=json.loads(Path(self.worker.outputs[1]).read_text(encoding='utf-8'))
        self.assertEqual(metadata['aircraft_conformity'],
                         'NOT EVALUATED - approved aircraft reference data not supplied')
        self.assertGreaterEqual(metadata['throttle_command_time_s'],5.)
        self.assertGreaterEqual(metadata['throttle_completed_time_s'],5.)
        self.assertGreaterEqual(metadata['throttle_measured_onset_s'],5.)

if __name__=='__main__': unittest.main()
