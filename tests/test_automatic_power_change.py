"""Pure offline state-machine tests: no P3D/BRUNNER/USB hardware."""
import csv
from pathlib import Path
from unittest.mock import Mock, patch
import json
import tempfile
import unittest

from backend.automatic_power_change import AutomaticPowerDynamics, selected, REFERENCE, FORCE_REFERENCE


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
    def test_initialization_requires_stable_flight_and_rpm_before_ready(self):
        self.feed(rpm=2700.)
        self.feed(rpm=2200.)
        self.feed(rpm=1600.)
        self.assertEqual(self.worker.phase, 'PREPARE')
        for _ in range(6):
            self.feed(rpm=2000., prop_rpm=2000., pitch_deg=3.)
        self.assertEqual(self.worker.phase,'READY',self.worker.details)
        self.assertEqual(self.worker.stage,'READY TO RECORD')
        self.assertTrue(self.sim.paused)
    def test_initialization_still_requires_running_engine(self):
        self.feed()
        self.feed()
        with patch('backend.automatic_power_change.save_report',return_value=[]):
            self.feed(rpm=200.)
        self.assertEqual(self.worker.result,'FAILED')
        self.assertIn('running engine',self.worker.details)
    def test_only_p3d_throttle_command_and_five_second_baseline(self):
        self.prepare()
        self.assertFalse(any(e[0]=='step' for e in self.sim.events))
        self.worker.command(dict(action='record'))
        for i in range(12):
            self.feed()
            if self.worker.power_command_time is not None: break
        self.assertIsNotNone(self.worker.power_command_time)
        self.assertGreaterEqual(self.worker.power_command_time - self.worker.start_time,5.)
        self.assertTrue(all(row['Throttle 1 Position (%)'] == 50. for row in self.worker.rows))
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
        self.assertTrue(all(row['Throttle 1 Position (%)'] == 100.
                            for row in self.worker.rows if row['Time (s)'] >
                            self.worker.power_completed_time-self.worker.start_time))
        self.assertEqual(self.worker.outputs,['recording.csv'])
    def test_uses_configured_start_and_100_percent_step(self):
        self.ctx['initial_conditions']['throttle'] = 60.
        self.worker=AutomaticPowerDynamics(self.ctx,self.sim,self.published.append,clock=self.clock)
        self.prepare()
        self.assertIn(('initial',60.),self.sim.events)
        self.worker.command(dict(action='record'))
        for i in range(12):
            self.feed()
            if self.worker.power_command_time is not None: break
        self.assertIsNotNone(self.worker.power_command_time)
        self.assertGreaterEqual(self.worker.power_command_time-self.worker.start_time,5.)
        self.assertTrue(all(row['Throttle 1 Position (%)'] == 60. for row in self.worker.rows))
        self.assertEqual([e for e in self.sim.events if e[0]=='step'],[('step',100.)])
        self.feed()
        with patch('backend.automatic_power_change.save_report',return_value=[]):
            for i in range(35):
                if self.worker.phase=='DONE': break
                self.feed()
        self.assertEqual(self.worker.result,'COMPLETE',self.worker.details)
        self.assertGreaterEqual(self.worker.clock()-self.worker.power_completed_time,15.)
        self.ctx['initial_conditions']['throttle'] = 96.
        with self.assertRaisesRegex(ValueError, 'at least 5% below'):
            AutomaticPowerDynamics(self.ctx,self.sim,self.published.append,clock=self.clock)
        self.ctx['initial_conditions']['throttle'] = 60.
        self.ctx['go_around_throttle'] = 90.
        with self.assertRaisesRegex(ValueError, 'go-around throttle must be 100%'):
            AutomaticPowerDynamics(self.ctx,self.sim,self.published.append,clock=self.clock)
    def test_baseline_throttle_change_aborts_before_step(self):
        self.prepare(); self.worker.command(dict(action='record'))
        self.feed()
        self.feed(0.5, throttle=60.)
        self.assertEqual(self.worker.result,'FAILED')
        self.assertEqual([e for e in self.sim.events if e[0]=='step'],[])
        self.assertNotIn('reporting error',self.worker.details)
        self.assertEqual(len(self.worker.outputs),2)
        self.assertTrue(self.worker.outputs[0].endswith('.csv'))
        self.assertTrue(self.worker.outputs[1].endswith('.qtg_test.json'))
        metadata=json.loads(Path(self.worker.outputs[1]).read_text(encoding='utf-8'))
        self.assertIsNone(metadata['event_time_s'])
    def test_full_power_must_hold_for_response_window(self):
        self.prepare(); self.worker.command(dict(action='record'))
        for i in range(12): self.feed()
        self.feed()
        self.assertIsNotNone(self.worker.power_completed_time)
        with patch('backend.automatic_power_change.save_report',return_value=[]):
            self.feed(throttle=90.)
        self.assertEqual(self.worker.result,'FAILED')
        self.assertIn('remain at 100%',self.worker.details)
    def test_aircraft_free_response_does_not_block_throttle_step(self):
        self.prepare(); self.worker.command(dict(action='record'))
        for i in range(12):
            self.feed(0.5, ias_kt=90.+i*.8, alt_ft=4000.-i*8.,
                      pitch_deg=3.+i*.5, bank_deg=i*.4, vertical_speed=-900.)
            if self.worker.power_command_time is not None: break
        self.assertIsNotNone(self.worker.power_command_time)
        self.assertEqual([e for e in self.sim.events if e[0]=='step'],[('step',100.)])
    def test_pid_assistance_cannot_move_other_controls(self):
        self.ctx['active_pids'] = {'airspeed': False, 'bank': True, 'yaw': False}
        with self.assertRaisesRegex(ValueError,'commands throttle only'):
            AutomaticPowerDynamics(self.ctx,self.sim,self.published.append,clock=self.clock)
    def test_read_only_controller_measurements_are_recorded(self):
        brunner=Mock()
        brunner.sock=True
        brunner.read_axis.side_effect=lambda axis: ({1:(.1,.2,0.),2:(.2,.3,0.),4:(.3,.4,0.)}[axis])
        self.worker.brunner=brunner
        self.prepare(); self.worker.command(dict(action='record'))
        self.feed()
        row=self.worker.rows[0]
        self.assertAlmostEqual(row['Joystick Pitch Position (mm)'],10.)
        self.assertAlmostEqual(row['Joystick Pitch Force (N)'],20.)
        self.assertAlmostEqual(row['Joystick Roll Position (mm)'],20.)
        self.assertAlmostEqual(row['Joystick Roll Force (N)'],18.)
        self.assertAlmostEqual(row['Rudder Pedal Position (mm)'],30.)
        self.assertAlmostEqual(row['Rudder Pedal Force (N)'],32.)
        brunner.configure_replay_mode.assert_not_called()
        brunner.move_actuators.assert_not_called()
    def test_physical_measurements_do_not_depend_on_virtual_force_channel(self):
        class PhysicalOnlyBrunner:
            sock = True
            strict_io = False
            def read_axis_measurement(self, axis):
                return {1: (.1, 20.), 2: (.2, 18.), 4: (.3, 32.)}[axis]
            def read_axis(self, axis):
                raise RuntimeError('optional virtual-force channel unavailable')
        self.worker.brunner = PhysicalOnlyBrunner()
        self.prepare(); self.worker.command(dict(action='record'))
        self.feed()
        row = self.worker.rows[0]
        self.assertAlmostEqual(row['Joystick Pitch Position (mm)'], 10.)
        self.assertAlmostEqual(row['Joystick Pitch Force (N)'], 20.)
        self.assertAlmostEqual(row['Joystick Roll Position (mm)'], 20.)
        self.assertAlmostEqual(row['Joystick Roll Force (N)'], 18.)
        self.assertAlmostEqual(row['Rudder Pedal Position (mm)'], 30.)
        self.assertAlmostEqual(row['Rudder Pedal Force (N)'], 32.)
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
        self.assertTrue(selected(dict(self.ctx, test_reference=FORCE_REFERENCE)))
        other=dict(self.ctx,test_reference='2.d.(1)')
        self.assertFalse(selected(other))
        from backend.control import validate_method
        self.assertEqual(validate_method(REFERENCE,'power_dynamics'),'power_dynamics')
        self.assertEqual(validate_method('2.b.(11)','power_dynamics'),'power_dynamics')
        self.assertEqual(validate_method(FORCE_REFERENCE,'power_dynamics'),'power_dynamics')
        with self.assertRaises(ValueError):validate_method('2.d.(1)','power_dynamics')
    def test_force_entry_uses_same_throttle_sequence_and_own_report(self):
        from backend.catalog import TESTS
        from frontend.main import default_channels_for_spec
        references = [spec.reference for spec in TESTS]
        self.assertEqual(references[references.index(REFERENCE) + 1], FORCE_REFERENCE)
        self.assertIn('pitch_force', default_channels_for_spec(FORCE_REFERENCE))
        self.assertIn('pitch_virtual_force', default_channels_for_spec(FORCE_REFERENCE))
        self.ctx['test_reference'] = FORCE_REFERENCE
        self.ctx['title'] = 'Power change force'
        self.ctx['selected_graphs'] = ['throttle_pct', 'pitch_force']
        self.worker = AutomaticPowerDynamics(self.ctx, self.sim, self.published.append, clock=self.clock)
        brunner = Mock()
        brunner.sock = True
        brunner.read_axis.side_effect = lambda axis: (.1, .2, 0.)
        self.worker.brunner = brunner
        self.prepare(); self.worker.command(dict(action='record'))
        for i in range(12): self.feed()
        self.assertEqual([event for event in self.sim.events if event[0] == 'step'], [('step', 100.)])
        self.feed()
        for i in range(35):
            if self.worker.phase == 'DONE': break
            self.feed()
        self.assertEqual(self.worker.result, 'COMPLETE', self.worker.details)
        self.assertGreaterEqual(self.worker.clock() - self.worker.power_completed_time, 15.)
        self.assertIn('power_change_force_auto_', Path(self.worker.outputs[0]).name)
        metadata = json.loads(Path(self.worker.outputs[1]).read_text(encoding='utf-8'))
        self.assertEqual(metadata['kind'], 'Power Change Force')
        self.assertIn('pitch_force', metadata['graphed_channels'])
        brunner.move_actuators.assert_not_called()
    def test_missing_aircraft_reference_is_not_claimed_as_pass(self):
        self.ctx['selected_graphs']=['ias_kt','vertical_speed_fpm','altitude_msl_ft',
                                     'pitch_deg','bank_deg','pitch_rate','prop_rpm',
                                     'engine_rpm','throttle_pct','pitch_position',
                                     'pitch_force','stall_aoa']
        self.worker=AutomaticPowerDynamics(self.ctx,self.sim,self.published.append,clock=self.clock)
        self.prepare();self.worker.command(dict(action='record'))
        for i in range(12):self.feed()
        self.feed()
        for i in range(35):
            if self.worker.phase=='DONE':break
            self.feed()
        self.assertEqual(self.worker.result,'COMPLETE',self.worker.details)
        self.assertEqual(self.worker.stage,'Completed',self.worker.details)
        self.assertEqual(len(self.worker.outputs),3)
        self.assertTrue(Path(self.worker.outputs[2]).is_file())
        import pymupdf
        with pymupdf.open(self.worker.outputs[2]) as report:
            self.assertIn('Initial Recorded Conditions',report[0].get_text())
            report_text = '\n'.join(page.get_text() for page in report)
            self.assertIn('1st Quadrant Combined',report_text)
            # A selected channel remains in the PDF even when that run has no
            # finite samples for it; selection controls display, not capture.
            self.assertIn('Pitch position', report_text)
        metadata=json.loads(Path(self.worker.outputs[1]).read_text(encoding='utf-8'))
        self.assertEqual(metadata['aircraft_conformity'],
                         'NOT EVALUATED - approved aircraft reference data not supplied')
        self.assertGreaterEqual(metadata['throttle_command_time_s'],5.)
        self.assertGreaterEqual(metadata['throttle_completed_time_s'],5.)
        self.assertGreaterEqual(metadata['throttle_measured_onset_s'],5.)
        self.assertIn('vertical_speed_fpm',metadata['graphed_channels'])
        self.assertIn('pitch_deg',metadata['graphed_channels'])
        self.assertIn('bank_deg',metadata['graphed_channels'])
        self.assertEqual(metadata['unavailable_selected_channels'],
                         ['pitch_rate','pitch_position','pitch_force','stall_aoa'])
    def test_pdf_initial_table_uses_first_recorded_frame(self):
        import pymupdf
        from reports.power_dynamics import render_report
        source=Path(self.tmp.name)/'power_recording.csv'
        destination=Path(self.tmp.name)/'power_graphs.pdf'
        rows=[
            {'Time (s)':0.0,'Indicated Airspeed (kts)':80.,'Altitude MSL (ft)':4000.,
             'Rate of Climb/Descent (fpm)':-900.,'Pitch Angle (deg)':0.,'Bank Angle (deg)':0.,
             'Throttle 1 Position (%)':50.},
            {'Time (s)':5.0,'Indicated Airspeed (kts)':90.,'Altitude MSL (ft)':3900.,
             'Rate of Climb/Descent (fpm)':-1100.,'Pitch Angle (deg)':5.,'Bank Angle (deg)':2.,
             'Throttle 1 Position (%)':50.},
            {'Time (s)':20.0,'Indicated Airspeed (kts)':110.,'Altitude MSL (ft)':3700.,
             'Rate of Climb/Descent (fpm)':800.,'Pitch Angle (deg)':8.,'Bank Angle (deg)':3.,
             'Throttle 1 Position (%)':100.},
        ]
        with source.open('w',newline='',encoding='utf-8') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0]))
            writer.writeheader(); writer.writerows(rows)
        metadata=dict(initial_conditions=dict(throttle=50.,ias_kt=80.,alt_ft=4000.,
                                              pitch_deg=0.,bank_deg=0.),
                      selected_graphs=['ias_kt','vertical_speed_fpm','pitch_deg','pitch_position'],
                      throttle_command_time_s=5.,throttle_measured_onset_s=5.1,result='COMPLETE')
        graphed,unavailable=render_report(source,destination,metadata)
        self.assertEqual(graphed,['ias_kt','vertical_speed_fpm','pitch_deg'])
        self.assertEqual(unavailable,['pitch_position'])
        with pymupdf.open(destination) as pdf:
            first=pdf[0].get_text()
            graphs=' '.join(page.get_text() for page in list(pdf)[1:])
        self.assertIn('80.00 kt',first)
        self.assertNotIn('90.00 kt',first)
        self.assertIn('Joystick pitch position (BRUNNER not connected)',first)
        self.assertIn('Rate of climb/descent',graphs)
        self.assertIn('Pitch angle',graphs)

    def test_selected_controller_graphs_use_recorded_measurements(self):
        from reports.power_dynamics import graph_availability
        rows = [
            {'Joystick Pitch Position (mm)': value, 'Joystick Pitch Force (N)': value * 2,
             'Joystick Roll Position (mm)': value * 3, 'Joystick Roll Force (N)': value * 4,
             'Rudder Pedal Position (mm)': value * 5, 'Rudder Pedal Force (N)': value * 6}
            for value in (1., 2.)
        ]
        selected = ['pitch_position', 'pitch_force', 'roll_position', 'roll_force',
                    'rudder_position', 'rudder_force']
        self.assertEqual(graph_availability(rows, selected), (selected, []))

if __name__=='__main__': unittest.main()
