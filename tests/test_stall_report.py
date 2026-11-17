"""Offline exercise of stall CSV channels and PDF graph selection."""
import csv
from pathlib import Path
import tempfile
import unittest

import fitz
from frontend.main import default_channels_for_spec, read_graph_csv
from interfaces.brunner_base.src.plot_graphs import _manual_report


class StallReportTests(unittest.TestCase):
    def test_warning_samples_are_available_to_graph_settings_and_pdf(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / 'stall.csv'
            output = Path(folder) / 'stall.pdf'
            fields = ['Time (s)', 'Stall Warning (0/1)', 'Aircraft Angle of Attack (deg)',
                      'Indicated Airspeed (kts)']
            with source.open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerows([dict(zip(fields, row)) for row in [(0, 0, 4, 90), (10, 1, 12, 60)]])
            self.assertIn('stall_warning', default_channels_for_spec('2.c.(8b)'))
            self.assertEqual(read_graph_csv(source)['stall_warning'], [0.0, 1.0])
            _manual_report(source, output, selected_graphs=['stall_warning', 'stall_aoa', 'ias_kt'])
            with fitz.open(output) as document:
                self.assertEqual(len(document), 2)
                text = ''.join(page.get_text() for page in document)
            self.assertIn('Stall Warning (0/1)', text)
            self.assertIn('Aircraft Angle of Attack (deg)', text)

    def test_initial_conditions_use_measured_stall_fields_and_force_units(self):
        with tempfile.TemporaryDirectory() as folder:
            source, output = Path(folder) / 'stall.csv', Path(folder) / 'stall.pdf'
            sample = {'Time (s)': 0, 'Stall Warning (0/1)': 0,
                      'BRUNNER Pitch Position (raw)': .25, 'BRUNNER Roll Position (raw)': -.5,
                      'BRUNNER Rudder Position (raw)': .1,
                      'Joystick Pitch Position (mm)': 25, 'Joystick Roll Position (mm)': -50,
                      'Rudder Pedal Position (mm)': 10, 'Joystick Pitch Force (N)': 12.3,
                      'Joystick Roll Force (N)': 4.5, 'Rudder Pedal Force (N)': 3.2,
                      'BRUNNER Pitch Applied Force': 12.3, 'BRUNNER Pitch Applied Force Unit': 'N',
                      'BRUNNER Roll Applied Force': 4.5, 'BRUNNER Roll Applied Force Unit': 'Nm',
                      'BRUNNER Rudder Applied Force': 3.2, 'BRUNNER Rudder Applied Force Unit': 'N',
                      'CG Longitudinal (% reference chord)': 24.5, 'CG Lateral (% reference chord)': .5,
                      'Height of Terrain (ft)': 745, 'Fuel Flow (pph)': 51.5}
            with source.open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=sample)
                writer.writeheader()
                writer.writerow(sample)
            _manual_report(source, output, selected_graphs=['stall_warning'])
            with fitz.open(output) as document:
                text = document[0].get_text()
            for expected in ('CG Longitudinal (% reference chord)', '24.5', 'Pitch Control Position (mm)',
                             'Roll Applied Force (N)', '4.50', '745', '51.5'):
                self.assertIn(expected, text)

    def test_existing_control_selection_uses_calibrated_mm_and_newtons(self):
        with tempfile.TemporaryDirectory() as folder:
            source, output = Path(folder) / 'stall.csv', Path(folder) / 'stall.pdf'
            sample = {'Time (s)': 0, 'Stall Warning (0/1)': 0,
                      'Joystick Pitch Position (mm)': 25,
                      'Joystick Pitch Force (N)': -12}
            with source.open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=sample)
                writer.writeheader()
                writer.writerow(sample)
            _manual_report(source, output, selected_graphs=['pitch_position', 'pitch_force'])
            with fitz.open(output) as document:
                text = ''.join(page.get_text() for page in document)
            self.assertIn('Pitch Control Position (mm)', text)
            self.assertIn('Pitch Applied Force (N)', text)
            self.assertNotIn('Measurement not available', text)

    def test_automatic_stall_report_populates_validation_and_sim_columns(self):
        from reports.automatic_control_report import save
        with tempfile.TemporaryDirectory() as folder:
            context = {
                'mode': 'auto',
                'test_reference': '2.c.(8b)',
                'title': 'Stall warning (approach-to-stall characteristics)',
                'result_directory': folder,
                'initial_conditions': {
                    'alt_ft': 4000.0,
                    'ias_kt': 90.0,
                    'heading_deg': 48.4,
                    'throttle': 80.0,
                    'pitch_deg': 0.0,
                    'bank_deg': 0.0,
                    'temperature_c': 15.0,
                    'gross_weight': 848.6,
                    'on_ground': False,
                },
                'selected_graphs': ['ias_kt', 'stall_warning', 'pitch_deg'],
            }
            rows = [
                {
                    'Time (s)': 0.0,
                    'Stall Warning (0/1)': 0,
                    'Indicated Airspeed (kts)': 88.5,
                    'Pressure Alt (ft)': 4013.0,
                    'Pitch Angle (deg)': -1.5,
                    'Bank Angle (deg)': 0.2,
                    'Magnetic Heading (deg)': 48.4,
                    'Aircraft Angle of Attack (deg)': 0.53,
                    'Throttle 1 Position (%)': 5.0,
                    'Gross Wt (kg)': 848.6,
                    'Engine 1 RPM': 2097.0,
                    'Fuel Flow (pph)': 0.0,
                    'Height of Terrain (ft)': 745.0,
                    'Joystick Pitch Position (mm)': -2.07,
                    'Joystick Pitch Force (N)': -7.26,
                    'Pitch Applied Force (N)': -7.26,
                },
                {
                    'Time (s)': 1.0,
                    'Stall Warning (0/1)': 1,
                    'Indicated Airspeed (kts)': 65.0,
                    'Pressure Alt (ft)': 3950.0,
                    'Pitch Angle (deg)': 12.0,
                    'Bank Angle (deg)': 0.5,
                    'Magnetic Heading (deg)': 48.4,
                    'Aircraft Angle of Attack (deg)': 14.5,
                    'Throttle 1 Position (%)': 5.0,
                    'Gross Wt (kg)': 848.6,
                    'Engine 1 RPM': 2000.0,
                    'Fuel Flow (pph)': 0.0,
                    'Height of Terrain (ft)': 745.0,
                    'Joystick Pitch Position (mm)': 25.0,
                    'Joystick Pitch Force (N)': 20.0,
                    'Pitch Applied Force (N)': 20.0,
                }
            ]
            outputs = save(context, rows, 'COMPLETE', 'Stall warning verified')
            pdf_path = next(p for p in outputs if p.endswith('.pdf'))
            with fitz.open(pdf_path) as doc:
                page1 = doc[0].get_text()
                self.assertNotIn('Validation', page1)
                self.assertIn('Sim', page1)
                self.assertIn('N/A', page1)
                self.assertNotIn('90.0', page1)
                self.assertNotIn('4000', page1)
                self.assertNotIn('80.0', page1)
                self.assertIn('848.6', page1)
                self.assertIn('88.5', page1)
                self.assertIn('4013', page1)
                self.assertIn('-2.07', page1)

