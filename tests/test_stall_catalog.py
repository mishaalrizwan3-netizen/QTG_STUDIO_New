"""Offline checks for the stall catalog entry and its isolated settings."""
import tempfile
from pathlib import Path
from types import SimpleNamespace
from unittest import TestCase
from unittest.mock import Mock

from backend.catalog import TESTS
from backend.initial_condition_settings import FIELDS, catalog_defaults, load, save
from backend import pdf_graph_settings


class StallCatalogTests(TestCase):
    def test_replaces_rudder_trim_slot_with_separate_reference(self):
        stall = next(test for test in TESTS if test.reference == '2.c.(8b)')
        self.assertEqual(stall.reference, '2.c.(8b)')
        self.assertIn('Stall warning', stall.title)
        self.assertEqual(stall.group, 'Longitudinal Handling')
        values = catalog_defaults(stall)
        for key, expected in {'Flt cond': 'Approach', 'Alt[ft]': '4000', 'IAS[kts]': '90',
                              'Flaps': '0', 'Temperature[C]': '15', 'Throttle[%]': '80'}.items():
            self.assertEqual(values[key], expected)

    def test_saved_settings_do_not_reuse_rudder_trim_settings(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            rudder = dict.fromkeys(FIELDS, '')
            rudder['IAS[kts]'] = '50'
            stall = dict(rudder, **{'IAS[kts]': '90'})
            save(root, '2.a.(5)', rudder)
            save(root, '2.c.(8b)', stall)
            pdf_graph_settings.save(root / 'graphs', '2.c.(8b)', ['ias_kt'])
            self.assertEqual(load(root, '2.a.(5)'), rudder)
            self.assertEqual(load(root, '2.c.(8b)'), stall)
            self.assertEqual(pdf_graph_settings.load(root / 'graphs', '2.c.(8b)')['selected_graphs'], ['ias_kt'])

    def test_manual_launch_passes_conditions_and_stall_graph(self):
        from frontend.main import App
        import qtg_common_engine as engine
        from unittest.mock import patch
        stall = next(test for test in TESTS if test.reference == '2.c.(8b)')
        values = catalog_defaults(stall)
        values.update({'Latitude[deg]': '32', 'Longitude[deg]': '74', 'Heading[deg]': '48',
                       'Pitch[deg]': '0', 'Bank[deg]': '0'})
        fields = {key: Mock(get=Mock(return_value=value)) for key, value in values.items()}
        gui = SimpleNamespace(common_status=Mock(), common_record_button=Mock(),
            _common_duration_var=Mock(get=Mock(return_value='30')),
            _common_duration_vars={stall.reference: Mock(get=Mock(return_value='30'))},
            _test_settings={stall.reference: (fields, [])}, _apply_test_settings=Mock(return_value=True),
            _common_process=None, _manual_gui_bindings={}, after=Mock(), _poll_common_test=Mock())
        with patch('backend.engine_initialization_adapter.requested', return_value=False), \
             patch.object(engine, 'launch', return_value=(Mock(), {})) as launch:
            App._launch_common_test(gui, 'manual', stall)
        launch.assert_called_once()
        self.assertEqual(launch.call_args.kwargs['initial_conditions']['ias_kt'], 90.)
        self.assertEqual(launch.call_args.kwargs['initial_conditions']['throttle'], 80.)
        self.assertIn('stall_warning', launch.call_args.kwargs['selected_graphs'])

    def test_automatic_launch_uses_stall_worker_without_replay(self):
        from frontend.main import App
        import qtg_common_engine as engine
        from unittest.mock import patch
        stall = next(test for test in TESTS if test.reference == '2.c.(8b)')
        values = catalog_defaults(stall)
        values.update({'Latitude[deg]':'32', 'Longitude[deg]':'74', 'Heading[deg]':'48',
                       'Pitch[deg]':'0', 'Bank[deg]':'0'})
        fields = {key: Mock(get=Mock(return_value=value)) for key, value in values.items()}
        gui = SimpleNamespace(common_status=Mock(), common_record_button=Mock(),
            _common_duration_var=Mock(get=Mock(return_value='30')),
            _test_settings={stall.reference:(fields, [])}, _apply_test_settings=Mock(return_value=True),
            _common_process=None, _manual_gui_bindings={}, after=Mock(), _poll_common_test=Mock())
        with patch('backend.engine_initialization_adapter.requested', return_value=False), \
             patch.object(engine, 'launch', return_value=(Mock(), {})) as launch:
            App._launch_common_test(gui, 'auto', stall)
        launch.assert_called_once()
        self.assertEqual(launch.call_args.kwargs['control_method'], 'stall')
        self.assertIsNone(launch.call_args.args[3])
