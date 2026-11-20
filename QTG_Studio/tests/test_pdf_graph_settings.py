import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from backend.pdf_graph_settings import load, save
from frontend.mousewheel import install


class GraphSettingsTests(unittest.TestCase):
    def test_three_graphs_share_page_after_details_and_use_actual_time(self):
        import pymupdf
        from interfaces.brunner_base.src.plot_graphs import plot_aligned_comparison
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / 'example_manual.csv'
            source.write_text('Time (s),Indicated Airspeed (kts),Engine 1 RPM,Elevator Trim (%)\n0,80,900,0\n10,81,950,10\n')
            output = Path(tmp) / 'report.pdf'
            plot_aligned_comparison(source, output_pdf_path=output, test_reference='2.a.(4)', selected_graphs=['ias_kt','engine_rpm','trim_value_pct'])
            with pymupdf.open(output) as pdf:
                self.assertEqual(len(pdf), 2)
                self.assertIn('Initial Test Conditions', pdf[0].get_text())
                page = pdf[1].get_text()
                self.assertIn('Elevator Trim (%)', page)
                self.assertNotIn('\n75\n', page)

    def test_other_test_pdf_selection_and_empty_selection(self):
        import pymupdf
        from interfaces.brunner_base.src.plot_graphs import plot_aligned_comparison
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / 'example_manual.csv'
            source.write_text('Time (s),Indicated Airspeed (kts),Engine 1 RPM\n0,80,900\n1,81,950\n')
            output = Path(tmp) / 'report.pdf'
            plot_aligned_comparison(source, output_pdf_path=output, test_reference='2.a.(1)', selected_graphs=['ias_kt'])
            with pymupdf.open(output) as pdf:
                self.assertEqual(len(pdf), 2)
                self.assertIn('Indicated Airspeed', pdf[1].get_text())
                self.assertNotIn('Engine 1 RPM', pdf[1].get_text())
            plot_aligned_comparison(source, output_pdf_path=output, test_reference='2.a.(1)', selected_graphs=[])
            with pymupdf.open(output) as pdf:
                self.assertEqual(len(pdf), 1)

    def test_engine_pdf_contains_only_requested_graphs(self):
        import fitz
        from reports.plot_graphs import plot_aligned_comparison
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / 'engine_deceleration_manual.csv'
            source.write_text('Time (s),Ground Speed (kt),Engine 1 RPM,Throttle 1 Position (%)\n0,0,900,10\n1,0,950,20\n')
            output = Path(tmp) / 'report.pdf'
            plot_aligned_comparison(source, output_pdf_path=output, test_reference='1.f.(2)',
                                    selected_graphs=['ground_speed_kt'])
            with fitz.open(output) as pdf:
                self.assertEqual(len(pdf), 2)
                self.assertIn('Ground Speed', pdf[1].get_text())
                self.assertIn('Time from recording start (s)', pdf[1].get_text())
            plot_aligned_comparison(source, output_pdf_path=output, test_reference='1.f.(2)',
                                    selected_graphs=['throttle_pct'])
            with fitz.open(output) as pdf:
                page = pdf[1].get_text()
                self.assertIn('10% throttle guide', page)
                self.assertIn('90% throttle guide', page)
            plot_aligned_comparison(source, output_pdf_path=output, test_reference='1.f.(2)', selected_graphs=[])
            with fitz.open(output) as pdf:
                self.assertEqual(len(pdf), 1)

    def test_engine_graph_x_axis_begins_at_recording_zero(self):
        from matplotlib.axes import Axes
        from unittest.mock import patch
        from reports.plot_graphs import plot_aligned_comparison

        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / 'engine_acceleration_auto.csv'
            source.write_text('Time (s),Engine RPM,Throttle 1 Position (%)\n'
                              '0,1500,5\n1,1500,5\n2,2700,100\n')
            original = Axes.set_xlim
            limits = []
            def capture(axis, left=None, right=None, *args, **kwargs):
                limits.append((left, right))
                return original(axis, left, right, *args, **kwargs)
            with patch.object(Axes, 'set_xlim', capture):
                plot_aligned_comparison(source, output_pdf_path=Path(tmp) / 'graph.pdf',
                                        test_reference='1.f.(1)', selected_graphs=['engine_rpm'])
            self.assertIn((0.0, 2.0), limits)

    def test_choices_and_empty_selection_are_independent(self):
        with tempfile.TemporaryDirectory() as tmp:
            save(tmp, '1.f.(1)', ['ias_kt'], 'recorded.csv')
            save(tmp, '1.f.(2)', [], '')
            self.assertEqual(load(tmp, '1.f.(1)')['selected_graphs'], ['ias_kt'])
            self.assertEqual(load(tmp, '1.f.(1)')['graph_csv_path'], 'recorded.csv')
            self.assertEqual(load(tmp, '1.f.(2)')['selected_graphs'], [])

    def test_removed_initial_condition_stays_removed_on_save(self):
        from backend.initial_condition_settings import FIELDS, load_visible_fields, load as load_conditions, save as save_conditions
        with tempfile.TemporaryDirectory() as tmp:
            values = dict.fromkeys(FIELDS, '')
            visible = [key for key in FIELDS if key != 'IAS[kts]']
            save_conditions(tmp, '1.f.(2)', values, visible_fields=visible)
            save_conditions(tmp, '1.f.(2)', values)
            self.assertNotIn('IAS[kts]', load_visible_fields(tmp, '1.f.(2)'))
            self.assertEqual(load_conditions(tmp, '1.f.(2)')['IAS[kts]'], '')
            self.assertEqual(load_visible_fields(tmp, '1.f.(1)'), list(FIELDS))

    def test_mousewheel_routes_to_nearest_area_and_preserves_entry(self):
        class Widget:
            def __init__(self, kind, master=None):
                self.kind, self.master, self.scrolls = kind, master, []
            def winfo_class(self): return self.kind
            def winfo_ismapped(self): return True
            def yview_scroll(self, *args): self.scrolls.append(args)
            def bind(self, *args, **kwargs): pass
        class Root:
            def __init__(self): self.callbacks = {}; self.under = None
            def bind_all(self, name, callback, **kwargs): self.callbacks[name] = callback
            def winfo_containing(self, x, y): return self.under
        root = Root(); register = install(root)
        outer = Widget('Canvas'); inner = Widget('Canvas', outer)
        register(outer); register(inner)
        root.under = Widget('Checkbutton', inner)
        event = SimpleNamespace(x_root=1, y_root=1, delta=-120, num=None)
        self.assertEqual(root.callbacks['<MouseWheel>'](event), 'break')
        self.assertEqual(inner.scrolls, [(1, 'units')]); self.assertFalse(outer.scrolls)
        root.under = Widget('TEntry', inner)
        self.assertIsNone(root.callbacks['<MouseWheel>'](event))
        self.assertEqual(len(inner.scrolls), 1)
