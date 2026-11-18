"""Offline packaged-GUI smoke check; never connects to simulator or hardware."""

def require(condition, message):
    if not condition:
        raise RuntimeError(str(message))
import importlib
import ctypes
import json
import os
from pathlib import Path
import tempfile
import traceback

def run(gui):
    os.environ['QTG_OFFLINE_GUI_CHECK'] = '1'
    result = {'ok': False, 'checks': []}
    app = None
    try:
        for name in ('backend.deceleration_config', 'backend.pdf_graph_settings', 'backend.initial_condition_settings', 'qtg_common_engine', 'qtg_climb_pid', 'SMK_Engine_From_Climb', 'reports.plot_graphs', 'PySide6.QtWidgets', 'pandas', 'numpy', 'matplotlib', 'backend.bank_pid', 'backend.bank_pid_runner', 'interfaces.bank_pid_transport', 'reports.bank_pid_report', 'backend.yaw_pid', 'backend.yaw_pid_runner', 'interfaces.yaw_pid_transport', 'reports.yaw_pid_report', 'backend.multi_pid_runner', 'interfaces.multi_pid_transport', 'reports.multi_pid_report'):
            importlib.import_module(name)
        result['checks'].append('packaged imports')
        import qtg_common_engine as engine
        from backend.registry import suite_root
        require((engine.brunner_root() / 'native/bin/p3d_bridge_engine_init_v1517.dll').is_file(), 'Packaged deployment check failed')
        require((suite_root() / 'engine_deceleration/initialization.json').is_file(), 'Packaged deployment check failed')
        require((suite_root() / 'engine_acceleration/procedure.py').is_file(), 'Packaged deployment check failed')
        result['checks'].append('worker DLL and test resources')
        import sys
        bundled = Path(sys._MEIPASS) if getattr(sys, 'frozen', False) else Path(__file__).resolve().parents[1]
        for relative in ('bridge/bin/qtg_p3d_bridge.dll', 'bridge/bin/qtg_simconnect.dll', 'p3d_bridge.dll', 'brunner_base/native/bin/p3d_bridge_engine_init_v1517.dll'):
            library = bundled / relative
            if not getattr(sys, 'frozen', False) and relative.startswith('brunner_base/'):
                library = bundled / 'interfaces' / relative
            ctypes.CDLL(str(library))
        result['checks'].append('all four existing native bridges load without connecting')
        from backend.bank_pid import BankPID, slew_position
        require(BankPID().update(0.0, 10.0, 0.04) < 0.0, 'Packaged deployment check failed')
        require(BankPID().update(0.0, -10.0, 0.04) > 0.0, 'Packaged deployment check failed')
        require(BankPID().update(0.0, 0.0, 0.04) == 0.0, 'Packaged deployment check failed')
        require(abs(slew_position(0.0, 0.0, 0.65, 0.04, max_rate=0.28, max_accel=0.9)[0]) <= 0.28 * 0.04, 'Packaged deployment check failed')
        from qtg_climb_pid import controller_types, CLIMB_PROFILE
        controller_types()
        require(CLIMB_PROFILE['ias']['kp'] == 0.32, 'Packaged deployment check failed')
        result['checks'].append('frozen Bank sign/zero/slew and existing Airspeed source/profile')
        with tempfile.TemporaryDirectory() as tmp:
            gui.RESULTS_ROOT = Path(tmp)
            app = gui.App()
            app.withdraw()
            index = next((i for i, s in enumerate(gui.TESTS) if s.reference == '1.f.(2)'))
            app.select_test(index)
            spec = gui.TESTS[index]
            fields, graphs = app._test_settings[spec.reference]
            fields['Alt[ft]'].set('756')
            app._save_test_initial_conditions(spec, quiet=True)
            from backend.initial_condition_settings import load
            require(load(Path(tmp) / 'initial_condition_settings', spec.reference)['Alt[ft]'] == '756', 'Packaged deployment check failed')
            graphs[0][2].set(not graphs[0][2].get())
            from backend.pdf_graph_settings import load as load_graphs
            require(load_graphs(Path(tmp) / 'pdf_graph_settings', spec.reference) is not None, 'Packaged deployment check failed')
            require(load_graphs(Path(tmp) / 'pdf_graph_settings', '1.f.(1)') is None, 'Packaged deployment check failed')
            app.update_idletasks()
            result['checks'].append('Tk test panel, editable settings and per-test persistence')
            require('TCL_LIBRARY' in os.environ and (Path(os.environ['TCL_LIBRARY']) / 'init.tcl').is_file(), 'Tcl library init.tcl not found')
            result['checks'].append('Tcl/Tk library environment and init.tcl verified')
            for index, spec in enumerate(gui.TESTS):
                app.select_test(index)
                app._settings_unlocked = {spec.reference}
                app._build_settings_panel(spec)
                pending = [app.settings_panel]
                check_texts = []
                while pending:
                    widget = pending.pop()
                    pending.extend(widget.winfo_children())
                    if widget.winfo_class() == 'Checkbutton':
                        check_texts.append(widget.cget('text'))
                require(any('Bank PID' in t for t in check_texts), spec.reference)
                require(any('Yaw PID' in t for t in check_texts), spec.reference)
                require(any('Airspeed PID' in t for t in check_texts), spec.reference)
            result['checks'].append('Airspeed, Bank and Yaw PID checkbuttons present in every packaged test panel')
            from reports.bank_pid_report import report as bank_report
            sample = Path(tmp) / 'bank.csv'
            sample.write_text('Time (s),Bank Angle (deg)\n0,10\n1,8\n', encoding='utf-8')
            output = Path(bank_report(sample, selected_graphs=['bank_deg']))
            require(output.is_file() and output.read_bytes().startswith(b'%PDF'), 'Packaged deployment check failed')
            result['checks'].append('Bank PDF generation from bundled runtime')
            from reports.yaw_pid_report import report as yaw_report
            sample_yaw = Path(tmp) / 'yaw.csv'
            sample_yaw.write_text('Time (s),Heading (deg),Bank Angle (deg),Rudder Position (-),Aileron Position (-)\n0,270,0,0,0\n1,270,0,0,0\n', encoding='utf-8')
            output_yaw = Path(yaw_report(sample_yaw, selected_graphs=['heading_deg']))
            require(output_yaw.is_file() and output_yaw.read_bytes().startswith(b'%PDF'), 'Packaged deployment check failed')
            result['checks'].append('Yaw PDF generation from bundled runtime')
            from reports.multi_pid_report import report as multi_report
            output_multi = Path(multi_report(sample_yaw))
            require(output_multi.is_file() and output_multi.read_bytes().startswith(b'%PDF'), 'Packaged deployment check failed')
            result['checks'].append('Multi-PID PDF generation from bundled runtime')
            source = os.environ.get('QTG_OFFLINE_RECORDED_CSV', '')
            if source:
                from reports.plot_graphs import plot_aligned_comparison
                output = Path(tmp) / 'recorded_report.pdf'
                plot_aligned_comparison(source, output_pdf_path=output, test_reference='1.f.(2)', selected_graphs=['ground_speed_kt'])
                require(output.is_file() and output.read_bytes().startswith(b'%PDF'), 'Packaged deployment check failed')
                result['checks'].append('PDF from existing recorded CSV')
        result['ok'] = True
    except BaseException:
        result['error'] = traceback.format_exc()
    finally:
        if app is not None:
            app.destroy()
        output = os.environ.get('QTG_DEPLOYMENT_CHECK_OUTPUT')
        if output:
            Path(output).write_text(json.dumps(result, indent=2), encoding='utf-8')
    return 0 if result['ok'] else 1
