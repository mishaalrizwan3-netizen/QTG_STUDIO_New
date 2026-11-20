import threading
import time
import unittest
import tempfile
import csv
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import qtg_common_engine


class BUCManualIntegrationTests(unittest.TestCase):
    def test_manual_loop_records_cockpit_axes_with_sim_telemetry(self):
        base = qtg_common_engine.load_base()
        sim = Mock()
        sim.get_telemetry_sequence.return_value = 1
        telemetry = dict(engine_rpm=1800, engine_throttle_percent=40)
        sim.get_telemetry.return_value = telemetry
        sim.wait_for_fresh_telemetry.return_value = telemetry
        brunner = Mock(AXIS_PITCH=1, AXIS_ROLL=2, AXIS_RUDDER=4)
        brunner.read_axis.return_value = (0., 0., 0.)
        reader = Mock()
        reader.poll.side_effect = lambda: dict(throttle_pct=60., throttle_demand_pct=40.,
            propeller_pct=70., mixture_pct=80., throttle_raw=600, propeller_raw=700,
            mixture_raw=800, receipt_time=time.monotonic())
        host = SimpleNamespace(_session_context={'test_reference':'2.a.(8)'},
            _refresh_paused_telemetry=Mock(return_value={}), p3d=sim, brunner=brunner,
            stop_worker_event=threading.Event(), target_duration_sec=.03, is_recording=True,
            rec_time=[], p_pos_buf=[], p_force_buf=[], r_pos_buf=[], r_force_buf=[],
            y_pos_buf=[], y_force_buf=[], p3d_tele_buf=[], applied_force_buf=[], virtual_force_buf=[],
            sig_update_stage=Mock(), sig_record_progress=Mock(), sig_recording_finished=Mock())
        with patch('backend.bu0836x.Reader', return_value=reader), \
             patch('backend.airborne_preparation.verify_recording_sample'):
            base.SuperMushakLoggerApp._manual_recording_loop(host)
        self.assertTrue(host.rec_time)
        self.assertEqual(host.p3d_tele_buf[0]['engine_throttle_percent'], 40)
        self.assertEqual(host.p3d_tele_buf[0]['buc_sample']['throttle_pct'], 60)
        self.assertEqual(host.p3d_tele_buf[0]['buc_sample']['propeller_pct'], 70)
        reader.close.assert_called_once()
        host.sig_recording_finished.emit.assert_called_once()

    def test_controller_unavailable_blocks_recording_before_unpause(self):
        base = qtg_common_engine.load_base()
        sim = Mock()
        host = SimpleNamespace(_session_context={'test_reference':'2.a.(8)'},
            _refresh_paused_telemetry=Mock(return_value={}), p3d=sim, brunner=Mock(),
            is_recording=True, sig_update_stage=Mock())
        with patch('backend.bu0836x.Reader', side_effect=RuntimeError('missing device')), \
             patch('backend.airborne_preparation.verify_recording_sample'):
            base.SuperMushakLoggerApp._manual_recording_loop(host)
        self.assertFalse(host.is_recording)
        sim.unpause_sim.assert_not_called()
        self.assertIn('missing device', host.sig_update_stage.emit.call_args.args[1])

    def test_cockpit_and_simulator_values_are_written_together(self):
        base = qtg_common_engine.load_base()
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            sample = dict(throttle_pct=60., throttle_demand_pct=40., propeller_pct=70., mixture_pct=80.,
                          throttle_raw=600, propeller_raw=700, mixture_raw=800, receipt_time=1.)
            telemetry = dict(engine_rpm=1800, eng_rpms=[1800], prop_rpms=[1800],
                engine_throttle_percent=40., engine_propeller_percent=70., engine_mixture_percent=80.,
                engine_torque_ft_lb=150., buc_sample=sample, buc_sample_age_s=.1)
            host = SimpleNamespace(data_dir=root, test_mode='manual', rec_time=[0.], p3d_tele_buf=[telemetry],
                p_pos_buf=[0.], p_force_buf=[0.], r_pos_buf=[0.], r_force_buf=[0.], y_pos_buf=[0.], y_force_buf=[0.],
                applied_force_buf=[(0.,0.,0.)], virtual_force_buf=[(0.,0.,0.)],
                _session_context={'test_reference':'2.a.(8)'}, manual_t0_snapshot={'telemetry':telemetry},
                active_manual_ref_path=None, report_graphs=['buc_throttle','buc_propeller','buc_mixture'],
                lbl_rec_status=Mock(), lbl_rec_progress=Mock(), rec_progress=Mock(), target_duration_sec=30,
                _refresh_manual_reference_selector=Mock())
            with patch.object(base,'CURRENT_DIR',str(root)), patch.object(base,'QProgressDialog'), \
                 patch.object(base,'QApplication'), patch.object(base.os,'startfile'), \
                 patch.object(base.QMessageBox,'critical', side_effect=AssertionError('CSV save failed')):
                base.SuperMushakLoggerApp._save_csv_and_generate_reports(host)
            with next(root.glob('*.csv')).open(newline='', encoding='utf-8') as stream:
                row = next(csv.DictReader(stream))
            self.assertEqual(float(row['BUC Throttle Z Rotation (%)']),60.)
            self.assertEqual(float(row['Throttle 1 Position (%)']),40.)
            self.assertEqual(float(row['BUC Propeller Slider (%)']),70.)
            self.assertEqual(float(row['Engine Torque (ft lb)']),150.)
            self.assertTrue(list(root.glob('*.pdf')))
