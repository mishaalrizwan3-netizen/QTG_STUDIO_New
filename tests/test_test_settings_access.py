"""Password setup for Test Settings; no simulator connection is needed."""

import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from backend import test_settings_access


class TestSettingsAccessTests(unittest.TestCase):
    def test_local_password_is_created_once_and_verified(self):
        with tempfile.TemporaryDirectory() as folder, patch.dict(os.environ, {'LOCALAPPDATA': folder}):
            path = Path(folder) / 'QTG Studio' / 'test_settings_admin.json'
            self.assertFalse(test_settings_access.is_configured())
            test_settings_access.set_password('chosen-password')
            self.assertTrue(test_settings_access.is_configured())
            self.assertNotIn('chosen-password', path.read_text(encoding='utf-8'))
            self.assertTrue(test_settings_access.verify_password('chosen-password'))
            self.assertFalse(test_settings_access.verify_password('wrong-password'))
            with self.assertRaises(FileExistsError):
                test_settings_access.set_password('replacement')

    def test_blank_password_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder, patch.dict(os.environ, {'LOCALAPPDATA': folder}):
            with self.assertRaises(ValueError):
                test_settings_access.set_password('  ')
            self.assertFalse(test_settings_access.is_configured())


if __name__ == '__main__':
    unittest.main()
