"""Offline coverage for the additive Normal Climb PID modes."""

import os
import csv
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from tkinter import ttk
import tkinter as tk

from backend.pid_selection import PID_CHOICES, axes_for, validate
from frontend import main as gui
import qtg_common_engine as engine


class PidGuiIntegrationTests(unittest.TestCase):
    def test_each_test_sidebar_has_three_pid_checkboxes_and_no_combo_choices(self):
        with tempfile.TemporaryDirectory() as folder, \
                patch.dict(os.environ, {'QTG_OFFLINE_GUI_CHECK': '1'}), \
                patch.object(gui, 'RESULTS_ROOT', Path(folder)):
            app = gui.App()
            app.withdraw()
            try:
                app._settings_unlocked = {'1.c.(1)', '2.a.(7)', '2.c.(8b)'}
                for reference in ('1.c.(1)', '2.a.(7)', '2.c.(8b)'):
                    index = next(i for i, spec in enumerate(gui.TESTS) if spec.reference == reference)
                    app.select_test(index)
                    app._settings_unlocked.add(reference)
                    app._build_settings_panel(gui.TESTS[index])
                    def descendants(widget):
                        for child in widget.winfo_children():
                            yield child
                            yield from descendants(child)
                    all_methods = [widget for widget in descendants(app.settings_panel)
                                   if isinstance(widget, ttk.Combobox)]
                    labels = {value for widget in all_methods for value in widget.cget('values')}
                    self.assertFalse(set(PID_CHOICES) & labels)
                    check_labels = {widget.cget('text') for widget in descendants(app.settings_panel)
                                    if isinstance(widget, tk.Checkbutton)}
                    self.assertTrue({'Airspeed PID', 'Bank PID', 'Yaw PID'}.issubset(check_labels),
                                    (reference, check_labels))
                    self.assertFalse(any(var.get() for var in app._pid_axis_checks[reference].values()))
            finally:
                app.destroy()

    def test_selection_is_scoped_and_validated(self):
        for label in PID_CHOICES:
            self.assertEqual(validate('1.c.(1)', 'auto', axes_for(label)), axes_for(label))
        self.assertEqual(validate('2.a.(4)', 'auto', axes_for('Bank PID')),
                         axes_for('Bank PID'))
        with self.assertRaises(ValueError):
            validate('1.c.(1)', 'manual', axes_for('Bank PID'))

    def test_launch_keeps_heading_and_selected_axes_without_replay_reference(self):
        conditions = dict(alt_ft=3000, ias_kt=80, lat=32.235, lon=74.135,
                          heading_deg=48.4, pitch_deg=0, bank_deg=0, flaps=0,
                          throttle=100, temperature_c=15, on_ground=False)
        spec = SimpleNamespace(reference='1.c.(1)', title='Normal climb')
        with tempfile.TemporaryDirectory() as folder, patch.object(engine.subprocess, 'Popen'):
            _, context = engine.launch('auto', spec, Path(folder) / 'sessions',
                                       control_method='multi_pid', initial_conditions=conditions,
                                       active_pids=axes_for('Bank + Yaw PID'),
                                       selected_graphs=['bank_deg', 'heading_deg'])
        self.assertIsNone(context['reference_csv'])
        self.assertEqual(context['target_heading_mag'], 48.4)
        self.assertEqual(context['active_pids'], axes_for('Bank + Yaw PID'))
        self.assertEqual(context['selected_graphs'], ['bank_deg', 'heading_deg'])

    def test_selected_pid_graphs_use_existing_qtg_report_layout(self):
        import fitz
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'Super_Mushak_auto_takeoff_data.csv'
            with path.open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=['Time (s)', 'Indicated Airspeed (kts)',
                    'Bank Angle (deg)', 'Magnetic Heading (deg)'])
                writer.writeheader()
                writer.writerows([
                    {'Time (s)': 0, 'Indicated Airspeed (kts)': 80,
                     'Bank Angle (deg)': 0.5, 'Magnetic Heading (deg)': 48.4},
                    {'Time (s)': 1, 'Indicated Airspeed (kts)': 80,
                     'Bank Angle (deg)': 0.1, 'Magnetic Heading (deg)': 48.5},
                ])
            report = engine.load_base().plot_graphs.plot_aligned_comparison(
                path, test_reference='1.c.(1)', control_method='multi_pid',
                selected_graphs=['bank_deg', 'heading_deg'])
            with fitz.open(report) as document:
                self.assertEqual(len(document), 2)
                self.assertIn('Selected test graphs', document[1].get_text())
                self.assertIn('Bank Angle', document[1].get_text())
                self.assertIn('Magnetic Heading', document[1].get_text())

    def test_automatic_test_runs_automatic_and_pid_assist_concurrently(self):
        from backend.pid_assist import PidAssist
        from unittest.mock import Mock

        conditions = dict(alt_ft=3000, ias_kt=80, lat=32.235, lon=74.135,
                          heading_deg=48.4, pitch_deg=0, bank_deg=0, flaps=0,
                          throttle=80, temperature_c=15, on_ground=False)
        spec = SimpleNamespace(reference='2.a.(1)', title='Pitch controller position vs force and surface position calibration')
        active_pids = {'airspeed': False, 'bank': True, 'yaw': False}

        with tempfile.TemporaryDirectory() as folder, patch.object(engine.subprocess, 'Popen'):
            _, context = engine.launch('auto', spec, Path(folder) / 'sessions',
                                       control_method='static_control', initial_conditions=conditions,
                                       active_pids=active_pids,
                                       selected_graphs=['pitch_deg', 'bank_deg'])

        # Verify test keeps static_control method, while saving active_pids
        self.assertEqual(context['control_method'], 'static_control')
        self.assertEqual(context['active_pids'], active_pids)

        # Test PidAssist axis ownership: Pitch (0x01) is owned, Bank (0x02) runs on Roll
        assist = PidAssist(context, owned_axis_mask=0x01)
        self.assertFalse(assist.pitch_enabled)
        self.assertTrue(assist.bank_enabled)
        self.assertFalse(assist.yaw_enabled)
        self.assertEqual(assist.pid_axis_mask, 0x02)
        self.assertEqual(assist.total_axis_mask(), 0x03)

        mock_brunner = Mock()
        mock_brunner.sock = True
        telemetry = {'bank_deg': 5.0, 'heading_deg': 48.4, 'ias_kt': 80.0, 'pitch_deg': 2.0}
        cmds = assist.update(1.0, telemetry, mock_brunner)
        self.assertIn('roll', cmds)
        self.assertNotIn('pitch', cmds)
        mock_brunner.write_setting.assert_called_with(0x02, 0x80, 'f', cmds['roll'])



if __name__ == '__main__':
    unittest.main()
