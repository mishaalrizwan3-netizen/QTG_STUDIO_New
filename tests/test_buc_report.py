import csv
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock
from types import SimpleNamespace

import fitz
from frontend.main import App, TESTS, read_graph_csv
from interfaces.brunner_base.src.plot_graphs import _manual_report


class BUCreportTests(unittest.TestCase):
    def test_catalog_is_ground_manual_and_previous_entries_remain(self):
        test = next(x for x in TESTS if x.reference == '2.a.(8)')
        self.assertIn('Ground', test.initial_conditions)
        status = Mock()
        App._launch_common_test(SimpleNamespace(common_status=status), 'auto', test)
        self.assertIn('Manual', status.configure.call_args.kwargs['text'])
        self.assertTrue(any(x.reference == '2.a.(4)' for x in TESTS))

    def test_selected_cockpit_axes_render_in_pdf(self):
        with tempfile.TemporaryDirectory() as folder:
            source, output = Path(folder)/'buc.csv', Path(folder)/'buc.pdf'
            rows = [dict(zip(['Time (s)', 'BUC Throttle Z Rotation (%)', 'BUC Propeller Slider (%)',
                              'BUC Mixture Dial (%)', 'Engine Torque (ft lb)'], values))
                    for values in [(0, 100, 0, 0, 20), (1, 0, 100, 100, 150)]]
            with source.open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
            self.assertEqual(read_graph_csv(source)['buc_throttle'], [100., 0.])
            _manual_report(source, output, ['buc_throttle','buc_propeller','buc_mixture'])
            with fitz.open(output) as document:
                text = ''.join(page.get_text() for page in document)
            self.assertIn('BUC Throttle Z Rotation (%)', text)
            self.assertIn('BUC Propeller Slider (%)', text)
            self.assertIn('BUC Mixture Dial (%)', text)
            with fitz.open(output) as document:
                self.assertIn('Time (s)', document[1].get_text())

