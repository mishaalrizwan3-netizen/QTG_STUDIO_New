"""Offline GUI coverage: no simulator or BRUNNER connections."""
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from tkinter import ttk

from frontend import main
from backend.initial_condition_settings import load


def descendants(widget):
    for child in widget.winfo_children():
        yield child
        yield from descendants(child)


class AllTestSettingsGUI(unittest.TestCase):
    def test_compact_display_and_add_remove_update_immediately(self):
        with tempfile.TemporaryDirectory() as folder, \
                patch.dict(os.environ, {'QTG_OFFLINE_GUI_CHECK': '1'}), \
                patch.object(main, 'RESULTS_ROOT', Path(folder)), \
                patch.object(main.simpledialog, 'askstring', return_value=main.TEST_ADMIN_PASSWORD), \
                patch.object(main.messagebox, 'showinfo'), \
                patch.object(main.messagebox, 'showerror', side_effect=AssertionError('Unexpected GUI error')):
            app = main.App()
            app.withdraw()
            try:
                index = next(i for i, spec in enumerate(main.TESTS) if spec.reference == '2.a.(1)')
                app.select_test(index)
                spec = main.TESTS[index]
                fields = app._test_settings[spec.reference][0]
                self.assertEqual(fields['IAS[kts]'].get(), '')
                self.assertFalse(any(isinstance(w, ttk.Entry) and str(w.cget('textvariable')) == str(fields['IAS[kts]'])
                                     for w in descendants(app.workspace)))
                app._open_test_password_dialog(index)
                def condition_combo():
                    return next(w for w in descendants(app.settings_panel) if isinstance(w, ttk.Combobox)
                                and 'IAS[kts]' in w.cget('values'))
                def button(text):
                    return next(w for w in descendants(app.settings_panel)
                                if isinstance(w, ttk.Button) and w.cget('text') == text)
                condition_combo().set('IAS[kts]')
                button('Remove condition').invoke()
                self.assertNotIn('IAS[kts]', app._visible_condition_fields[spec.reference])
                self.assertFalse(any(getattr(w, 'cget', lambda *_: None)('text') == 'IAS[kts]'
                                     for w in descendants(app.workspace) if w.winfo_class() == 'Label'))
                condition_combo().set('IAS[kts]')
                button('Add condition').invoke()
                self.assertIn('IAS[kts]', app._visible_condition_fields[spec.reference])
                self.assertTrue(any(getattr(w, 'cget', lambda *_: None)('text') == 'IAS[kts]'
                                    for w in descendants(app.workspace) if w.winfo_class() == 'Label'))
            finally:
                app.destroy()

    def test_each_catalog_test_can_open_settings_and_save_its_own_values(self):
        with tempfile.TemporaryDirectory() as folder, \
                patch.dict(os.environ, {'QTG_OFFLINE_GUI_CHECK': '1'}), \
                patch.object(main, 'RESULTS_ROOT', Path(folder)), \
                patch.object(main.simpledialog, 'askstring', return_value=main.TEST_ADMIN_PASSWORD), \
                patch.object(main.messagebox, 'showinfo'), \
                patch.object(main.messagebox, 'showerror', side_effect=AssertionError('Unexpected GUI error')):
            app = main.App()
            app.withdraw()
            try:
                self.assertEqual(len(main.TESTS), 30)
                for index, spec in enumerate(main.TESTS):
                    with self.subTest(reference=spec.reference):
                        app.select_test(index)
                        buttons = [w for w in descendants(app.workspace)
                                   if isinstance(w, ttk.Button) and w.cget('text') == 'Test settings']
                        self.assertEqual(len(buttons), 1)
                        buttons[0].invoke()
                        self.assertEqual(app.settings_panel.winfo_manager(), 'pack')
                        self.assertIn(spec.reference, app._settings_unlocked)
                        app._test_settings[spec.reference][0]['IAS[kts]'].set(str(index))
                        save_buttons = [w for w in descendants(app.settings_panel)
                                        if isinstance(w, ttk.Button) and w.cget('text') == 'Save initial conditions']
                        self.assertEqual(len(save_buttons), 1)
                        save_buttons[0].invoke()
                        self.assertEqual(load(Path(folder)/'initial_condition_settings', spec.reference)['IAS[kts]'], str(index))
                app.destroy()
                app = main.App()
                app.withdraw()
                for index, spec in enumerate(main.TESTS):
                    app._ensure_test_settings(spec)
                    self.assertEqual(app._test_settings[spec.reference][0]['IAS[kts]'].get(), str(index))
            finally:
                app.destroy()


if __name__ == '__main__':
    unittest.main()
