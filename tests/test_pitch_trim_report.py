"""Offline checks for the Pitch Trim report's measured channels."""
import csv
import ctypes as C
from pathlib import Path
import tempfile
import unittest

import fitz
from frontend.main import default_channels_for_spec
from interfaces.brunner_base.src.plot_graphs import _manual_report
from interfaces.engine_measurements import Measurements


class PitchTrimReportTests(unittest.TestCase):
    def test_indicator_is_received_as_a_separate_simconnect_measurement(self):
        reader = object.__new__(Measurements)
        reader.channels = (('indicator', 'ELEVATOR TRIM INDICATOR', 'position'),)
        reader.pending, reader.sequence, reader.object_id = 100, 0, None
        reader.clock, reader.requested_at, reader.error = lambda: 10.0, 9.9, ''
        packet = C.create_string_buffer(48)
        header = (C.c_uint32 * 10).from_buffer(packet)
        header[2], header[3], header[4], header[5], header[9] = 8, 100, 2, 1, 1
        C.c_double.from_buffer(packet, 40).value = -.42
        reader._receive(C.addressof(packet), len(packet), None)
        self.assertEqual(reader.sequence, 1)
        self.assertAlmostEqual(reader.latest['indicator'], -.42)

    def test_manual_report_graphs_trim_and_elevator_samples(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source, output = root / 'trim.csv', root / 'trim.pdf'
            fields = ['Time (s)', 'Elevator Trim Angle (rad)', 'Elevator Trim Indicator (position)', 'Elevator Trim (%)', 'Elevator Surface (rad)']
            with source.open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerows([dict(zip(fields, row)) for row in [(0, -.1, -.8, -20, -.02), (1, .1, -.2, 20, .03)]])
            selected = {'trim_position_rad', 'trim_indicator', 'trim_value_pct', 'trim_elevator_surface_rad'}
            self.assertTrue(selected.issubset(default_channels_for_spec('2.a.(7)')))
            self.assertTrue(selected.issubset(default_channels_for_spec('2.a.(4)')))
            _manual_report(source, output, selected_graphs=selected)
            with fitz.open(output) as document:
                text = ''.join(page.get_text() for page in document)
            for label in fields[1:]:
                self.assertIn(label, text)

