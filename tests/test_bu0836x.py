import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from backend.bu0836x import percentages, Reader, USAGES


class BU0836XTests(unittest.TestCase):
    def test_axis_directions_and_raw_values(self):
        values = percentages({'throttle':0, 'propeller':65535, 'mixture':65535},
                             {name:(0,65535) for name in ('throttle','propeller','mixture')})
        self.assertEqual(values['throttle_pct'], 0)
        self.assertEqual(values['throttle_demand_pct'], 100)
        self.assertEqual(values['propeller_pct'], 100)
        self.assertEqual(values['mixture_pct'], 100)
        self.assertEqual(values['throttle_raw'], 0)

    def test_unavailable_or_invalid_ranges_are_not_invented(self):
        with self.assertRaises(ValueError):
            percentages({'throttle':0, 'propeller':1, 'mixture':0},
                        {'throttle':(0,0), 'propeller':(0,1), 'mixture':(0,1)})

    def test_reader_uses_interface_2_hid_usages_and_closes(self):
        from pywinusb.hid.core import HidP_Input
        class Report:
            report_id = 1
            def keys(self): return [(1 << 16) | value for value in USAGES.values()]
            def set_raw_data(self, data): self.data = data
            def __getitem__(self, key): return SimpleNamespace(value={0x35:0,0x36:1023,0x37:1023}[key & 0xffff])
        caps = [SimpleNamespace(report_id=1, is_value=True, usage_page=1, is_range=False,
                               usage=usage, logical_min=0, logical_max=1023) for usage in USAGES.values()]
        device = Mock(product_name='BU0836X Interface 2', usages_storage={HidP_Input:caps})
        device.find_input_reports.return_value = [Report()]
        device.is_plugged.return_value = True
        reader = Reader(devices=[device], clock=lambda:10.)
        device.set_raw_data_handler.call_args.args[0]([1, 0])
        self.assertEqual(reader.poll()['throttle_demand_pct'], 100.)
        self.assertEqual(reader.poll()['mixture_pct'], 100.)
        reader.close()
        device.close.assert_called_once()

    def test_missing_interface_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, 'Interface 2'):
            Reader(devices=[])
