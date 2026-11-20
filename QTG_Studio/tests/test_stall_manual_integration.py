"""Exercise the real Manual recording loop with simulated input only."""
import threading
import time
import csv
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import qtg_common_engine


class StallManualIntegrationTests(unittest.TestCase):
    def test_loop_records_simulated_warning_without_control_commands(self):
        base = qtg_common_engine.load_base()
        sim = Mock()
        sim.get_telemetry_sequence.return_value = 1
        sim.get_telemetry.return_value = {'engine_rpm': 2000}
        sim.wait_for_fresh_telemetry.return_value = {'engine_rpm': 2000}
        brunner = Mock(AXIS_PITCH=1, AXIS_ROLL=2, AXIS_RUDDER=4)
        brunner.read_axis.return_value = (.2, .1, .3)
        reader = Mock()
        reader.poll.side_effect = lambda: {'ias_kt': 77., 'aoa_deg': 11.,
            'pitch_deg': 5., 'bank_deg': 1., 'stall_warning': 1., 'altitude_ft': 4000.,
            'sim_time_s': 12., 'receipt_time': time.monotonic()}
        host = SimpleNamespace(_session_context={'test_reference': '2.c.(8b)'},
            _refresh_paused_telemetry=Mock(return_value={}), p3d=sim, brunner=brunner,
            stop_worker_event=threading.Event(), target_duration_sec=.03, is_recording=True,
            rec_time=[], p_pos_buf=[], p_force_buf=[], r_pos_buf=[], r_force_buf=[],
            y_pos_buf=[], y_force_buf=[], p3d_tele_buf=[], applied_force_buf=[], virtual_force_buf=[],
            sig_update_stage=Mock(), sig_record_progress=Mock(), sig_recording_finished=Mock())
        with patch('interfaces.engine_measurements.Measurements', return_value=reader), \
             patch('backend.airborne_preparation.verify_recording_sample'):
            base.SuperMushakLoggerApp._manual_recording_loop(host)
        self.assertGreater(len(host.rec_time), 0)
        self.assertEqual(host.p3d_tele_buf[0]['stall_warning'], 1.)
        self.assertEqual(host.p3d_tele_buf[0]['aoa'], 11.)
        reader.close.assert_called_once()
        sim.unpause_sim.assert_called_once()
        sim.pause_sim.assert_called_once()
        brunner.write_setting.assert_not_called()
        host.sig_recording_finished.emit.assert_called_once()

    def test_real_csv_and_pdf_use_recorded_measurements(self):
        base = qtg_common_engine.load_base()
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            tel = dict(airspeed_indicated=88, indicated_altitude=4000, pressure_alt=4000,
                vertical_speed=-100, pitch_angle=2, bank_angle=1, true_heading=45,
                magnetic_heading=45, engine_rpm=2500, eng_rpms=[2500], prop_rpms=[2500],
                engine_throttle_percent=80, aoa=4, stall_warning=1, stall_ground_alt=745,
                stall_fuel_flow_pph=51, stall_cg_long_pct=24, stall_cg_lateral_pct=1,
                accelX=0, accelY=-32.1740485564, accelZ=0)
            host = SimpleNamespace(data_dir=root, test_mode='manual', rec_time=[0.],
                p3d_tele_buf=[tel], p_pos_buf=[.2], p_force_buf=[.12], r_pos_buf=[.3], r_force_buf=[4.],
                y_pos_buf=[.1], y_force_buf=[3.], applied_force_buf=[(.12,4.,3.)],
                virtual_force_buf=[(float('nan'),)*3], _session_context={'test_reference':'2.c.(8b)'},
                manual_t0_snapshot={'telemetry':tel}, active_manual_ref_path=None,
                report_graphs=['stall_warning'], lbl_rec_status=Mock(), lbl_rec_progress=Mock(),
                rec_progress=Mock(), target_duration_sec=30, _refresh_manual_reference_selector=Mock())
            with patch.object(base, 'CURRENT_DIR', str(root)), patch.object(base, 'QProgressDialog'), \
                 patch.object(base, 'QApplication'), patch.object(base.os, 'startfile'), \
                 patch.object(base.QMessageBox, 'critical', side_effect=AssertionError('CSV save failed')):
                base.SuperMushakLoggerApp._save_csv_and_generate_reports(host)
            source = next(root.glob('*.csv'))
            with source.open(newline='', encoding='utf-8') as stream:
                row = next(csv.DictReader(stream))
            self.assertEqual(float(row['Height of Terrain (ft)']), 745)
            self.assertEqual(float(row['Fuel Flow (pph)']), 51)
            self.assertEqual(float(row['BRUNNER Pitch Applied Force']), .12)
            self.assertEqual(float(row['Joystick Pitch Position (mm)']), 20)
            self.assertEqual(float(row['Joystick Pitch Force (N)']), 12)
            self.assertEqual(float(row['Y Body Acceleration (g)']), -1)
            self.assertTrue(list(root.glob('*.pdf')))
