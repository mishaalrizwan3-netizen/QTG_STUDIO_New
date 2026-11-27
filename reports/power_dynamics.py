"""Measured-only PDF report for the throttle-only Power Change Dynamics run."""

import csv
from datetime import datetime
import math
from pathlib import Path
import textwrap

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


# These names are the actual columns written by the dedicated P3D worker.
CHANNELS = {
    'ias_kt': ('Indicated airspeed', 'kt', 'Indicated Airspeed (kts)'),
    'altitude_msl_ft': ('Altitude MSL', 'ft', 'Altitude MSL (ft)'),
    'height_agl_ft': ('Height AGL', 'ft', 'Height AGL (ft)'),
    'vertical_speed_fpm': ('Rate of climb/descent', 'ft/min', 'Rate of Climb/Descent (fpm)'),
    'pitch_deg': ('Pitch angle', 'deg', 'Pitch Angle (deg)'),
    'bank_deg': ('Bank angle', 'deg', 'Bank Angle (deg)'),
    'heading_deg': ('Heading', 'deg', 'Heading (deg)'),
    'ground_speed_kt': ('Ground speed', 'kt', 'Ground Speed (kt)'),
    'roll_rate': ('Roll rate', 'deg/s', 'Roll Rate (deg/s)'),
    'pitch_rate': ('Pitch rate', 'deg/s', 'Pitch Rate (deg/s)'),
    'yaw_rate': ('Yaw rate', 'deg/s', 'Yaw Rate (deg/s)'),
    'stall_aoa': ('Angle of attack', 'deg', 'Aircraft Angle of Attack (deg)'),
    'stall_warning': ('Stall warning', '0/1', 'Stall Warning (0/1)'),
    'throttle_pct': ('Throttle position', '%', 'Throttle 1 Position (%)'),
    'engine_rpm': ('Engine speed', 'rpm', 'Engine 1 RPM'),
    'prop_rpm': ('Propeller speed', 'rpm', 'Propeller 1 RPM'),
    'manifold_inhg': ('Manifold pressure', 'inHg', 'Manifold Pressure (inHg)'),
    'flaps_handle_index': ('Flap lever position', 'number', 'Flaps Handle Index'),
    'flaps_handle_percent': ('Flap lever percentage', '%', 'Flaps Handle Percent (%)'),
    'flaps_left_angle': ('Trailing edge flaps left angle', 'rad', 'TRAILING EDGE FLAPS LEFT ANGLE'),
    'flaps_right_angle': ('Trailing edge flaps right angle', 'rad', 'TRAILING EDGE FLAPS RIGHT ANGLE'),
    'elevator_input_norm': ('Elevator input', 'normalised', 'Elevator Input (normalised)'),
    'aileron_input_norm': ('Aileron input', 'normalised', 'Aileron Input (normalised)'),
    'rudder_input_norm': ('Rudder input', 'normalised', 'Rudder Input (normalised)'),
    'trim_position_rad': ('Elevator trim', 'rad', 'Elevator Trim Angle (rad)'),
    'trim_indicator': ('Elevator trim indicator', 'position', 'Elevator Trim Indicator (position)'),
    'trim_value_pct': ('Elevator trim value', '%', 'Elevator Trim (%)'),
    'trim_elevator_surface_rad': ('Elevator surface', 'rad', 'Elevator Surface (rad)'),
    'fuel_flow_pph': ('Fuel flow', 'lb/h', 'Fuel Flow (pph)'),
    'pitch_position': ('Joystick pitch position', 'mm', 'Joystick Pitch Position (mm)'),
    'pitch_force': ('Joystick pitch force', 'N', 'Joystick Pitch Force (N)'),
    'pitch_virtual_force': ('Pitch virtual force 0x51', 'N', 'Pitch Virtual Force 0x51 (N)'),
    'roll_position': ('Joystick roll position', 'mm', 'Joystick Roll Position (mm)'),
    'roll_force': ('Joystick roll force', 'N', 'Joystick Roll Force (N)'),
    'rudder_position': ('Rudder pedal position', 'mm', 'Rudder Pedal Position (mm)'),
    'rudder_force': ('Rudder pedal force', 'N', 'Rudder Pedal Force (N)'),
    'body_accel_x': ('Body X acceleration', 'ft/s²', 'ACCELERATION BODY X'),
    'body_accel_y': ('Body Y acceleration', 'ft/s²', 'ACCELERATION BODY Y'),
    'body_accel_z': ('Body Z acceleration', 'ft/s²', 'ACCELERATION BODY Z'),
}

DEFAULT_GRAPHS = (
    'ias_kt', 'altitude_msl_ft', 'height_agl_ft', 'vertical_speed_fpm',
    'pitch_deg', 'bank_deg', 'heading_deg', 'throttle_pct',
    'engine_rpm', 'prop_rpm', 'manifold_inhg',
)

UNAVAILABLE_LABELS = {
    'pitch_rate': 'Pitch rate (not sampled)',
    'pitch_position': 'Joystick pitch position (BRUNNER not connected)',
    'pitch_force': 'Joystick pitch force (BRUNNER not connected)',
    'pitch_virtual_force': 'Pitch virtual force 0x51 (BRUNNER not connected)',
    'stall_aoa': 'Angle of attack (not sampled)',
}

INITIAL_ROWS = (
    ('Throttle', 'throttle', 'Throttle 1 Position (%)', '%'),
    ('Airspeed', 'ias_kt', 'Indicated Airspeed (kts)', 'kt'),
    ('Altitude MSL', 'alt_ft', 'Altitude MSL (ft)', 'ft'),
    ('Pitch', 'pitch_deg', 'Pitch Angle (deg)', 'deg'),
    ('Bank', 'bank_deg', 'Bank Angle (deg)', 'deg'),
    ('Heading', 'heading_deg', 'Heading (deg)', 'deg'),
    ('Flaps', 'flaps', 'Flaps Handle Index', 'index'),
    ('Temperature', 'temperature_c', 'Ambient Temperature (C)', '°C'),
)


def _number(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _display(value, unit):
    number = _number(value)
    if number is None:
        return 'N/A'
    digits = 0 if unit in ('ft', 'rpm', 'index') else 2
    return f'{number:.{digits}f} {unit}'


def graph_availability(rows, requested=None):
    """Return selected channels with real samples and selections lacking data."""
    keys = list(dict.fromkeys(requested if requested is not None else DEFAULT_GRAPHS))
    available, unavailable = [], []
    for key in keys:
        entry = CHANNELS.get(key)
        if entry and sum(_number(row.get(entry[2])) is not None for row in rows) >= 2:
            available.append(key)
        else:
            unavailable.append(key)
    return available, unavailable


def render_report(csv_path, pdf_path, metadata):
    """Plot recorded data only; never synthesize a missing flight or engine channel."""
    csv_path, pdf_path = Path(csv_path), Path(pdf_path)
    with csv_path.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError('Power Change Dynamics report has no recorded samples')
    chosen, unavailable = graph_availability(rows, metadata.get('selected_graphs'))
    first = rows[0]
    cfg = metadata.get('initial_conditions') or {}
    start_stamp = datetime.fromtimestamp(csv_path.stat().st_mtime).strftime('%d %b %Y %H:%M:%S')
    command = _number(metadata.get('throttle_command_time_s'))
    onset = _number(metadata.get('throttle_measured_onset_s'))
    end_time = _number(rows[-1].get('Time (s)')) or 0.0

    with PdfPages(pdf_path) as pdf:
        fig = plt.figure(figsize=(8.27, 11.69))
        report_title = (
            'Power Change Force  |  2.c.(1)-force'
            if metadata.get('kind') == 'Power Change Force'
            else 'Power Change Dynamics  |  2.c.(1)'
        )
        fig.text(.08, .95, report_title, fontsize=16, weight='bold')
        fig.text(.08, .915, f'Automatic test  |  {start_stamp}  |  {metadata.get("result", "UNKNOWN")}', fontsize=9)
        fig.text(.08, .889, 'Configured Initial Conditions vs first recorded frame (graph time zero)', fontsize=10, weight='bold')
        fig.text(.08, .862, f'{"Parameter":<23}{"Configured":<23}Recorded at t=0', fontsize=9, family='monospace', weight='bold')
        for index, (label, setting, column, unit) in enumerate(INITIAL_ROWS):
            fig.text(.08, .837-index*.027,
                     f'{label:<23}{_display(cfg.get(setting), unit):<23}{_display(first.get(column), unit)}',
                     fontsize=9, family='monospace')
        fig.text(.08, .605,
                 'Engine at t=0: RPM ' + _display(first.get('Engine 1 RPM'), 'rpm') +
                 '  |  Propeller ' + _display(first.get('Propeller 1 RPM'), 'rpm') +
                 '  |  Manifold ' + _display(first.get('Manifold Pressure (inHg)'), 'inHg'),
                 fontsize=8)
        fig.text(.08, .56, 'Recorded sequence', fontsize=10, weight='bold')
        fig.text(.08, .53, f'Baseline: 5.0 s at configured throttle  |  Command: {_display(command, "s")}', fontsize=9)
        fig.text(.08, .505, f'Measured throttle onset: {_display(onset, "s")}  |  Recording end: {end_time:.2f} s', fontsize=9)
        fig.text(.08, .48, 'Aircraft response is uncontrolled; speed, altitude and attitude may change.', fontsize=8)
        fig.text(.08, .445, 'Selected graph channels without recorded data', fontsize=10, weight='bold')
        missing_names = [UNAVAILABLE_LABELS.get(key, CHANNELS[key][0] if key in CHANNELS else key.replace('_', ' '))
                         for key in unavailable]
        missing_text = ', '.join(missing_names) if missing_names else 'None'
        fig.text(.08, .418, textwrap.fill(missing_text, 86), fontsize=8, va='top')
        fig.text(.08, .33, 'Fuel flow was not recorded; no estimated value is shown.', fontsize=8)
        fig.text(.08, .305, 'Only measured channels appear in the following graphs.', fontsize=9)
        fig.text(.08, .28, 'Aircraft reference comparison: NOT EVALUATED', fontsize=9, weight='bold')
        fig.text(.08, .12, 'Source: ' + csv_path.name, fontsize=7, color='#475569')
        fig.text(.08, .10, 'Doc Number: 161310G00000-00  |  Rev: 008', fontsize=7, color='#475569')
        pdf.savefig(fig)
        plt.close(fig)

        times = [_number(row.get('Time (s)')) for row in rows]
        for start in range(0, len(chosen), 3):
            keys = chosen[start:start+3]
            fig, axes = plt.subplots(len(keys), 1, figsize=(8.27, 11.69), squeeze=False)
            fig.subplots_adjust(left=.13, right=.95, top=.90, bottom=.075, hspace=.47)
            fig.suptitle('Power Change Dynamics — measured response', fontsize=13, weight='bold')
            for axis, key in zip(axes[:, 0], keys):
                title, unit, column = CHANNELS[key]
                pairs = [(t, y) for t, row in zip(times, rows)
                         if t is not None and (y := _number(row.get(column))) is not None]
                axis.plot([t for t, _ in pairs], [y for _, y in pairs], color='#2563eb', linewidth=1.2)
                if command is not None:
                    axis.axvline(command, color='#d97706', linestyle='--', linewidth=1, label='Throttle command')
                if onset is not None:
                    axis.axvline(onset, color='#475569', linestyle=':', linewidth=1, label='Measured throttle onset')
                axis.set(title=title, ylabel=unit, xlabel='Time from recording start (s)')
                axis.set_xlim(0, max(end_time, 1.0))
                axis.grid(True, alpha=.35)
                axis.legend(loc='best', fontsize=7)
            fig.text(.08, .025, f'Source: {csv_path.name}  |  Page {start//3+2}', fontsize=6.5, color='#475569')
            pdf.savefig(fig)
            plt.close(fig)

    return chosen, unavailable
