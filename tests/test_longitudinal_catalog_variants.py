import unittest

from backend.catalog import TESTS
from backend.initial_condition_settings import catalog_defaults


class LongitudinalCatalogVariantTests(unittest.TestCase):
    def test_six_independent_longitudinal_variants_are_in_order(self):
        references = [spec.reference for spec in TESTS]
        expected = ['2.c.(5a)', '2.c.(5b)', '2.c.(6a)', '2.c.(6b)', '2.c.(7a)', '2.c.(7b)']
        start = references.index(expected[0])
        self.assertEqual(references[start:start + len(expected)], expected)
        self.assertTrue(set(expected).isdisjoint({'2.c.(5)', '2.c.(6)', '2.c.(7)'}))

    def test_each_variant_has_independent_editable_defaults(self):
        specs = {spec.reference: spec for spec in TESTS}
        for reference in ('2.c.(5a)', '2.c.(5b)', '2.c.(6a)', '2.c.(6b)', '2.c.(7a)', '2.c.(7b)'):
            self.assertEqual(specs[reference].initial_conditions, '')
            defaults = catalog_defaults(specs[reference])
            self.assertIn('Alt[ft]', defaults)
            self.assertIn('IAS[kts]', defaults)


if __name__ == '__main__':
    unittest.main()
