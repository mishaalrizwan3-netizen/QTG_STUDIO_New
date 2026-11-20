"""Offline coverage for the additive Normal Climb PID modes."""

import os
import csv
import time
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from tkinter import ttk
import tkinter as tk

from backend.pid_selection import PID_CHOICES, axes_for, validate
from frontend import main as gui
import qtg_common_engine as engine


class PidGuiIntegrationTests(unittest.TestCase):
    def test_live_pid_status_shows_bank_target_and_measured_hold(self):
        state = dict(mode='auto', control_method='engine_step', test_reference='1.f.(1)',
                     active_pids={'airspeed':False,'bank':True,'yaw':False},
                     pid_conditions={'target_bank':0.0}, telemetry={'bank_deg':0.7},
                     recording=True, brunner_connected=True)
        status = gui.format_active_pid_status(state)
        self.assertIn('Bank PID: RUNNING, holding', status)
        self.assertIn('measured 0.7°', status)
        self.assertIn('target 0.0°', status)
        state['telemetry'] = {'bank_deg':8.0}
        self.assertIn('RUNNING, correcting', gui.format_active_pid_status(state))
        state['pid_runtime'] = {'command_axes': [], 'updated_at': time.time()}
        self.assertIn('waiting for controller command', gui.format_active_pid_status(state))
        state['pid_runtime']['command_axes'] = ['roll']
        self.assertIn('RUNNING, correcting', gui.format_active_pid_status(state))
        state['recording'] = False
        self.assertIn('waiting for recording', gui.format_active_pid_status(state))
        state['active_pids']['bank'] = False
        self.assertEqual(gui.format_active_pid_status(state), 'PID assistance: Off')

    def test_pid_status_does_not_claim_stall_pitch_is_running(self):
        state = dict(mode='auto', control_method='stall', test_reference='2.c.(8b)',
                     active_pids={'airspeed':True,'bank':False,'yaw':False},
                     telemetry={'ias_kt':80.0}, recording=True, brunner_connected=True)
        self.assertEqual(gui.format_active_pid_status(state),
                         'Airspeed PID: test controls this axis')
        state.update(test_reference='2.a.(2)',
                     active_pids={'airspeed':False,'bank':False,'yaw':True},
                     telemetry={'heading_deg':48.4},
                     pid_conditions={'target_heading':48.4})
        self.assertIn('rudder damping; test controls roll', gui.format_active_pid_status(state))
        self.assertNotIn('holding', gui.format_active_pid_status(state))

    def test_test_view_displays_live_pid_bar(self):
        with tempfile.TemporaryDirectory() as folder, \
                patch.dict(os.environ, {'QTG_OFFLINE_GUI_CHECK': '1'}), \
                patch.object(gui, 'RESULTS_ROOT', Path(folder)):
            app = gui.App()
            app.withdraw()
            try:
                reference = '1.f.(1)'
                index = next(i for i, spec in enumerate(gui.TESTS)
                             if spec.reference == reference)
                app.select_test(index)
                state = dict(test_reference=reference, mode='auto',
                             control_method='engine_step', stage='Recording',
                             active_pids={'bank': True},
                             pid_conditions={'target_bank': 0.0},
                             telemetry={'bank_deg': 0.7}, recording=True,
                             brunner_connected=True, duration_s=15)
                app._render_common_state(state)
                self.assertIn('Bank PID: RUNNING, holding',
                              app.common_pid_status.cget('text'))
            finally:
                app.destroy()

    def test_selected_pid_keeps_generic_test_on_recorded_replay(self):
        spec = next(item for item in gui.TESTS if item.reference == '2.c.(2)')
        values = {'Flt cond':'Clean','Alt[ft]':'3000','IAS[kts]':'80',
                  'Latitude[deg]':'32.235','Longitude[deg]':'74.135',
                  'Heading[deg]':'48.4','Pitch[deg]':'0','Bank[deg]':'0',
                  'Flaps':'0','Throttle[%]':'80','Temperature[C]':'15'}
        fields = {key: Mock(get=lambda value=value: value) for key, value in values.items()}
        host = SimpleNamespace(_common_duration_var=Mock(get=lambda:'10'),
            _test_settings={spec.reference:(fields,{})},
            _apply_test_settings=lambda spec: True,
            _common_reference_var=Mock(get=lambda:'manual.csv'),
            _pid_axis_checks={spec.reference:{'airspeed':Mock(get=lambda:False),
                                             'bank':Mock(get=lambda:True),
                                             'yaw':Mock(get=lambda:False)}},
            _pid_targets={spec.reference:Mock(get=lambda:'80')},
            _pid_targets_bank={spec.reference:Mock(get=lambda:'0')},
            _pid_targets_heading={spec.reference:Mock(get=lambda:'48.4')},
            _common_process=None, common_record_button=Mock(),
            common_status=Mock(), after=Mock(), _poll_common_test=Mock())
        with patch.object(engine, 'validate_reference', return_value='manual.csv'), \
                patch.object(engine, 'launch', return_value=(Mock(), {})) as launch:
            gui.App._launch_common_test(host, 'auto', spec)
        launch.assert_called_once()
        self.assertEqual(launch.call_args.kwargs['control_method'], 'replay')
        self.assertEqual(launch.call_args.args[3], 'manual.csv')
        self.assertEqual(launch.call_args.kwargs['active_pids'],
                         {'airspeed':False,'bank':True,'yaw':False})

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
                    pid_buttons = [widget for widget in descendants(app.settings_panel)
                                   if isinstance(widget, ttk.Button)
                                   and widget.cget('text') == 'Open full PID GUI']
                    self.assertEqual(len(pid_buttons), 1, reference)
                    if reference == '1.c.(1)':
                        with patch('qtg_pid_monitor.PidMonitor') as monitor_type:
                            pid_buttons[0].invoke()
                            monitor_type.assert_called_once()
                    self.assertFalse(any(var.get() for var in app._pid_axis_checks[reference].values()))
            finally:
                app.destroy()

    def test_pid_gui_button_opens_full_monitor_with_current_state(self):
        state = {'test_reference': '1.f.(1)', 'active_pids': {'bank': True}}
        host = SimpleNamespace(_common_last_state=state, _common_command=Mock())
        with patch('qtg_pid_monitor.PidMonitor') as monitor_type:
            gui.App._open_pid_monitor(host)
            monitor = monitor_type.return_value
            monitor.update.assert_called_once_with(state)
            gui.App._open_pid_monitor(host)
            monitor_type.assert_called_once()
            monitor.window.deiconify.assert_called_once()
            monitor.window.lift.assert_called_once()

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

    def test_custom_pid_targets_propagate_to_engine_and_assist(self):
        from backend.pid_assist import PidAssist
        from unittest.mock import Mock

        with tempfile.TemporaryDirectory() as folder, \
                patch.dict(os.environ, {'QTG_OFFLINE_GUI_CHECK': '1'}), \
                patch.object(gui, 'RESULTS_ROOT', Path(folder)), \
                patch.object(engine.subprocess, 'Popen'):
            app = gui.App()
            app.withdraw()
            try:
                ref = '2.a.(1)'
                index = next(i for i, spec in enumerate(gui.TESTS) if spec.reference == ref)
                app.select_test(index)
                app._settings_unlocked = {ref}
                app._build_settings_panel(gui.TESTS[index])

                # Verify default target entries exist and match defaults
                self.assertEqual(app._pid_targets[ref].get(), '80')
                self.assertEqual(app._pid_targets_bank[ref].get(), '0.0')

                # Populate initial conditions required for automated flight execution
                for key, val in [('Flt cond', 'Clean'), ('Alt[ft]', '2000'), ('IAS[kts]', '80'),
                                 ('Flaps', '0'), ('Latitude[deg]', '32.235'), ('Longitude[deg]', '74.135'),
                                 ('Heading[deg]', '0'), ('Pitch[deg]', '0'), ('Bank[deg]', '0'),
                                 ('Throttle[%]', '80'), ('Temperature[C]', '15')]:
                    app._test_settings[ref][0][key].set(val)

                # Set custom target values in GUI text boxes
                app._pid_targets[ref].set('95.0')
                app._pid_targets_bank[ref].set('12.0')
                app._pid_targets_heading[ref].set('270.0')
                app._pid_axis_checks[ref]['bank'].set(True)

                app._launch_common_test('auto', gui.TESTS[index])

                launched_context = getattr(app, '_common_context', {})
                self.assertEqual(launched_context['active_pids'], {'airspeed': False, 'bank': True, 'yaw': False})
                self.assertEqual(launched_context['pid_conditions']['target_ias'], 95.0)
                self.assertEqual(launched_context['pid_conditions']['target_bank'], 12.0)
                self.assertEqual(launched_context['pid_conditions']['target_heading'], 270.0)
                self.assertEqual(launched_context['target_bank'], 12.0)
                self.assertEqual(launched_context['target_heading_mag'], 270.0)

                # Verify PidAssist consumes custom targets
                assist = PidAssist(launched_context, owned_axis_mask=0x01)
                self.assertEqual(assist.target_ias, 95.0)
                self.assertEqual(assist.target_bank, 12.0)
                self.assertEqual(assist.target_hdg, 270.0)

                mock_brunner = Mock()
                mock_brunner.sock = True
                telemetry = {'bank_deg': 0.0, 'heading_deg': 270.0, 'ias_kt': 95.0, 'pitch_deg': 0.0}
                cmds = assist.update(1.0, telemetry, mock_brunner)
                self.assertIn('roll', cmds)
                self.assertGreater(cmds['roll'], 0.0)
            finally:
                app.destroy()

    def test_yaw_selection_holds_heading_and_explicit_zero_targets(self):
        from backend.pid_assist import PidAssist

        assist = PidAssist({
            'active_pids': {'airspeed': False, 'bank': False, 'yaw': True},
            'initial_conditions': {'heading_deg': 48.4, 'bank_deg': 12.0},
            'pid_conditions': {'target_heading': 0.0, 'target_bank': 0.0},
        })
        self.assertTrue(assist.bank_enabled)
        self.assertTrue(assist.yaw_enabled)
        self.assertEqual(assist.target_hdg, 0.0)
        self.assertEqual(assist.target_bank, 0.0)
        commands = assist.update(1.0, {'heading_deg': 48.4, 'bank_deg': 12.0,
                                       'sideslip_deg': 2.0, 'yaw_rate_deg_s': 0.0})
        self.assertIn('roll', commands)
        self.assertIn('rudder', commands)

    def test_pid_assist_publishes_live_command_axes(self):
        from backend.pid_assist import PidAssist

        context = {'active_pids': {'airspeed':False,'bank':True,'yaw':False},
                   'pid_conditions': {'target_bank':0.0}}
        assist = PidAssist(context)
        assist.update(1.0, {'bank_deg':4.0})
        self.assertEqual(context['pid_runtime']['command_axes'], ['roll'])



if __name__ == '__main__':
    unittest.main()
