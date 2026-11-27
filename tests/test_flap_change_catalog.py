import unittest

from backend.catalog import TESTS
from backend.initial_condition_settings import catalog_defaults
from frontend.main import FLAP_CHANGE_POSITIONS, default_channels_for_spec


class FlapChangeCatalogTests(unittest.TestCase):
    def test_retraction_and_extension_positions(self):
        specs = {spec.reference: spec for spec in TESTS}
        self.assertEqual(FLAP_CHANGE_POSITIONS['2.c.(2a)'], (1, 0))
        self.assertEqual(FLAP_CHANGE_POSITIONS['2.c.(2b)'], (0, 1))
        self.assertEqual(FLAP_CHANGE_POSITIONS['2.c.(2a)-force'], (1, 0))
        self.assertEqual(FLAP_CHANGE_POSITIONS['2.c.(2b)-force'], (1, 3))
        self.assertEqual(catalog_defaults(specs['2.c.(2a)'])['Flaps'], '1')
        self.assertEqual(catalog_defaults(specs['2.c.(2b)'])['Flaps'], '0')
        self.assertEqual(catalog_defaults(specs['2.c.(2a)-force'])['Flaps'], '1')
        self.assertEqual(catalog_defaults(specs['2.c.(2b)-force'])['Flaps'], '1')
        for reference in ('2.c.(2a)-force', '2.c.(2b)-force'):
            self.assertIn('pitch_position', default_channels_for_spec(reference))
            self.assertIn('pitch_force', default_channels_for_spec(reference))

    def test_entries_replace_old_flap_test_in_order(self):
        references = [spec.reference for spec in TESTS]
        start = references.index('2.c.(2a)')
        self.assertEqual(references[start:start + 5],
                         ['2.c.(2a)', '2.c.(2a)-force', '2.c.(2b)', '2.c.(2b)-force', '2.c.(5a)'])
        self.assertNotIn('2.c.(2)', references)
        self.assertNotIn('2.c.(3)', references)
        self.assertNotIn('2.c.(4)', references)


if __name__ == '__main__':
    unittest.main()
