"""Measured Static Control Automatic CSV, metadata and PDF generation.

Matches the official Climb test report style:
- Page 1: Standard Super Mushshak QTG cover page (_initial_page)
- Page 2+: Selected parameters subplots from Test Settings, plotted in BLUE (#0066cc).
Default parameters:
- 2.a.(1) Pitch: Airspeed, Joystick Pitch Position, Pitch Virtual Force
- 2.a.(2) Roll: Airspeed, Joystick Roll Position, Roll Virtual Force
- 2.a.(3) Yaw: Airspeed, Rudder Pedal Position, Rudder Virtual Force
"""

import csv
from datetime import datetime
import json
import math
from pathlib import Path
import time
from typing import List, Dict, Any, Optional

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
import pandas as pd

from backend.control_profile import AXIS_INFO, STATIC_CONTROL_REFERENCES

# Import official climb report cover page and style helpers
from interfaces.brunner_base.src.plot_graphs import _initial_page, _style_axis, _series


DEFAULT_SELECTED_BY_REF = {
    '2.a.(1)': ['ias_kt', 'pitch_position', 'pitch_force'],
    '2.a.(2)': ['ias_kt', 'roll_position', 'roll_force'],
    '2.a.(3)': ['ias_kt', 'rudder_position', 'rudder_force'],
}

CHANNEL_CONFIG = {
    'ias_kt': {
        'col': 'Indicated Airspeed (kts)',
        'alt_cols': ['Indicated Airspeed (kts)', 'airspeed_indicated'],
        'label': 'Indicated Airspeed (kts)',
        'group': 'Group 1 (Airspeed, Climb, Altitude)',
    },
    'vertical_speed_fpm': {
        'col': 'Rate of Climb/Descent (fpm)',
        'alt_cols': ['Rate of Climb/Descent (fpm)', 'vertical_speed'],
        'label': 'Rate of Climb/Descent (fpm)',
        'group': 'Group 1 (Airspeed, Climb, Altitude)',
    },
    'altitude_msl_ft': {
        'col': 'Indicated Altitude (ft)',
        'alt_cols': ['Indicated Altitude (ft)', 'indicated_altitude'],
        'label': 'Indicated Altitude (ft)',
        'group': 'Group 1 (Airspeed, Climb, Altitude)',
    },
    'height_agl_ft': {
        'col': 'Radar Alt (ft)',
        'alt_cols': ['Radar Alt (ft)', 'radar_alt'],
        'label': 'Radar Alt (ft)',
        'group': 'Group 1 (Airspeed, Climb, Altitude)',
    },
    'ground_speed_kt': {
        'col': 'Ground Speed (kt)',
        'alt_cols': ['Ground Speed (kt)', 'ground_speed'],
        'label': 'Ground Speed (kt)',
        'group': 'Group 1 (Airspeed, Climb, Altitude)',
    },
    'pitch_deg': {
        'col': 'Pitch Angle (deg)',
        'alt_cols': ['Pitch Angle (deg)', 'pitch_angle'],
        'label': 'Pitch Angle (deg)',
        'group': 'Group 2 (Attitude and Heading)',
    },
    'bank_deg': {
        'col': 'Bank Angle (deg)',
        'alt_cols': ['Bank Angle (deg)', 'bank_angle'],
        'label': 'Bank Angle (deg)',
        'group': 'Group 2 (Attitude and Heading)',
    },
    'heading_deg': {
        'col': 'Magnetic Heading (deg)',
        'alt_cols': ['Magnetic Heading (deg)', 'true_heading'],
        'label': 'Magnetic Heading (deg)',
        'group': 'Group 2 (Attitude and Heading)',
    },
    'pitch_rate': {
        'col': 'Pitch Rate (deg/s)',
        'alt_cols': ['Pitch Rate (deg/s)', 'pitch_rate'],
        'label': 'Pitch Rate (deg/s)',
        'group': 'Group 2 (Attitude and Heading)',
    },
    'yaw_rate': {
        'col': 'Yaw Rate (deg/s)',
        'alt_cols': ['Yaw Rate (deg/s)', 'yaw_rate'],
        'label': 'Yaw Rate (deg/s)',
        'group': 'Group 2 (Attitude and Heading)',
    },
    'prop_rpm': {
        'col': 'Propeller 1 RPM',
        'alt_cols': ['Propeller 1 RPM'],
        'label': 'Propeller 1 RPM',
        'group': 'Group 3 (Engine)',
    },
    'engine_rpm': {
        'col': 'Engine 1 RPM',
        'alt_cols': ['Engine 1 RPM', 'engine_rpm'],
        'label': 'Engine 1 RPM',
        'group': 'Group 3 (Engine)',
    },
    'throttle_pct': {
        'col': 'Throttle 1 Position (%)',
        'alt_cols': ['Throttle 1 Position (%)'],
        'label': 'Throttle 1 Position (%)',
        'group': 'Group 3 (Engine)',
    },
    'manifold_inhg': {
        'col': 'Manifold Pressure (inHg)',
        'alt_cols': ['Manifold Pressure (inHg)'],
        'label': 'Manifold Pressure (inHg)',
        'group': 'Group 3 (Engine)',
    },
    'fuel_flow_pph': {
        'col': 'Fuel Flow (pph)',
        'alt_cols': ['Fuel Flow (pph)', 'fuelFlow'],
        'label': 'Fuel Flow (pph)',
        'group': 'Group 3 (Engine)',
    },
    'pitch_position': {
        'col': 'Joystick Pitch Position (mm)',
        'alt_cols': ['Joystick Pitch Position (mm)', 'Control Position (mm)'],
        'label': 'Joystick Pitch Position (mm)',
        'group': 'Group 4 (Controls)',
    },
    'pitch_force': {
        'col': 'Pitch Virtual Force (N)',
        'alt_cols': [
            'Pitch Virtual Force 0x51 (N)',
            'Pitch Virtual Force (N)',
            'CLS Pitch Virtual Force (eng)',
            'Joystick Pitch Force (N)',
            'Pitch Applied Force (N)',
            'Control Force (N)',
        ],
        'label': 'Pitch Virtual Force (N)',
        'group': 'Group 4 (Controls)',
    },
    'roll_position': {
        'col': 'Joystick Roll Position (mm)',
        'alt_cols': ['Joystick Roll Position (mm)', 'Control Position (mm)'],
        'label': 'Joystick Roll Position (mm)',
        'group': 'Group 4 (Controls)',
    },
    'roll_force': {
        'col': 'Roll Virtual Force (N)',
        'alt_cols': [
            'Roll Virtual Force 0x51 (N)',
            'Roll Virtual Force (N)',
            'CLS Roll Virtual Force (eng)',
            'Joystick Roll Force (N)',
            'Roll Applied Force (N)',
            'Control Force (N)',
        ],
        'label': 'Roll Virtual Force (N)',
        'group': 'Group 4 (Controls)',
    },
    'rudder_position': {
        'col': 'Rudder Pedal Position (mm)',
        'alt_cols': ['Rudder Pedal Position (mm)', 'Control Position (mm)'],
        'label': 'Rudder Pedal Position (mm)',
        'group': 'Group 4 (Controls)',
    },
    'rudder_force': {
        'col': 'Yaw Virtual Force (N)',
        'alt_cols': [
            'Yaw Virtual Force 0x51 (N)',
            'Yaw Virtual Force (N)',
            'CLS Yaw Virtual Force (eng)',
            'Rudder Pedal Force (N)',
            'Yaw Applied Force (N)',
            'Control Force (N)',
        ],
        'label': 'Yaw Virtual Force (N)',
        'group': 'Group 4 (Controls)',
    },
}


def save(context: Dict[str, Any], rows: List[Dict[str, Any]], result: str, reason: str) -> List[str]:
    """Saves CSV, metadata JSON and renders the climb-style PDF report."""
    test_ref = context.get('test_reference', '')
    title = context.get('title', 'Static Control Test')
    axis_data = AXIS_INFO.get(test_ref, {})
    axis_name = axis_data.get('name', 'Control')

    folder = Path(context.get('result_directory', 'D:/QTG_Test_Results'))
    folder.mkdir(parents=True, exist_ok=True)

    clean_ref = test_ref.replace('.', '_').replace('(', '').replace(')', '').replace(' ', '')
    clean_title = title.replace(' ', '_').replace('-', '_')
    prefix = f"{clean_ref}_{clean_title}"
    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    stem = folder / f"{prefix}_auto_{timestamp}_{time.time_ns()}"

    csv_path = stem.with_suffix('.csv')
    metadata_path = stem.with_suffix('.qtg_test.json')
    pdf_path = stem.with_name(stem.name + '_graphs.pdf')

    outputs = []

    # 1. Write CSV
    if rows:
        fieldnames = list(rows[0].keys())
        with csv_path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        outputs.append(str(csv_path))

    # 2. Write Metadata
    metadata = dict(
        context,
        result=result,
        reason=reason,
        status='complete' if result == 'COMPLETE' else result.lower(),
        mode='auto',
        control_method='static_control',
        axis_name=axis_name,
        samples=len(rows),
        measurement_source='BRUNNER CLS2Sim UDP load-cell force & position; P3D SimConnect',
        validation='Static position vs force characteristic verification',
    )
    metadata_path.write_text(json.dumps(metadata, indent=2, allow_nan=False), encoding='utf-8')
    outputs.append(str(metadata_path))

    # 3. Generate Climb-style PDF
    _render_climb_style_report(csv_path, pdf_path, context, rows)
    outputs.append(str(pdf_path))

    return outputs


def _render_climb_style_report(csv_path: Path, out_path: Path, context: Dict[str, Any], rows: List[Dict[str, Any]]):
    """Renders official QTG report matching the climb test format with selected graphs in BLUE."""
    test_ref = context.get('test_reference', '2.a.(1)')
    selected = context.get('selected_graphs')
    if not selected:
        selected = DEFAULT_SELECTED_BY_REF.get(test_ref, ['ias_kt', 'pitch_position', 'pitch_force'])

    df = pd.DataFrame(rows) if rows else pd.read_csv(csv_path)
    tm = _series(df, 'Time (s)')
    t_max = max(15.0, float(np.nanmax(tm))) if len(tm) > 0 and np.isfinite(np.nanmax(tm)) else 15.0

    # Determine which items are requested
    chosen_items = []
    for key in selected:
        cfg = CHANNEL_CONFIG.get(key)
        if cfg:
            # Look up series
            series = None
            for col_name in cfg['alt_cols']:
                if col_name in df.columns:
                    val = _series(df, col_name)
                    if np.isfinite(val).any():
                        series = val
                        break
            if series is None:
                series = _series(df, cfg['col'])
            chosen_items.append((cfg['label'], series, cfg['group']))

    with plt.rc_context({
        'font.family': 'DejaVu Sans',
        'axes.facecolor': 'white',
        'figure.facecolor': 'white',
        'text.color': 'black',
        'axes.labelcolor': 'black',
        'xtick.color': 'black',
        'ytick.color': 'black'
    }), PdfPages(str(out_path)) as pdf:

        # Page 1: Standard Super Mushshak QTG Cover Page
        _initial_page(pdf, 'auto', str(csv_path), df, None, virtual_forces=True, missing_label='N/A')

        # Page 2+: Selected Graphs
        if not chosen_items:
            # Fallback if nothing was matched
            fig, ax = plt.subplots(1, 1, figsize=(8.27, 11.69), constrained_layout=True)
            fig.suptitle('1st Quadrant Combined Flight & Control Parameters', fontsize=15, fontweight='bold')
            ax.text(0.5, 0.5, 'No parameters selected for display', transform=ax.transAxes, ha='center', color='gray')
            pdf.savefig(fig)
            plt.close(fig)
            return

        # If 4 or fewer items, present them together on Page 2 for direct visual correlation
        if len(chosen_items) <= 4:
            fig, axes = plt.subplots(len(chosen_items), 1, figsize=(8.27, 11.69), constrained_layout=True, squeeze=False)
            fig.suptitle('1st Quadrant Combined Flight & Control Parameters', fontsize=15, fontweight='bold')
            for ax, (label, values, _) in zip(axes[:, 0], chosen_items):
                ax.plot(tm, values, color='#0066cc', linewidth=2.0, label='Automatic Test (Sim)')
                if not np.isfinite(np.asarray(values, dtype=float)).any():
                    ax.text(0.5, 0.5, 'Measurement not available', transform=ax.transAxes, ha='center', color='gray')
                _style_axis(ax, label)
                ax.set_xlim(0, t_max)
                ax.set_xticks(np.arange(0, t_max + 1, 5 if t_max <= 60 else 10))
                ax.legend(loc='best', fontsize=8, framealpha=0.95)
            pdf.savefig(fig)
            plt.close(fig)
        else:
            # Group into pages by group name
            groups = {}
            for label, values, grp in chosen_items:
                groups.setdefault(grp, []).append((label, values))

            for grp_title, grp_items in groups.items():
                fig, axes = plt.subplots(len(grp_items), 1, figsize=(8.27, 11.69), constrained_layout=True, squeeze=False)
                fig.suptitle(f'1st Quadrant Combined {grp_title}', fontsize=15, fontweight='bold')
                for ax, (label, values) in zip(axes[:, 0], grp_items):
                    ax.plot(tm, values, color='#0066cc', linewidth=2.0, label='Automatic Test (Sim)')
                    if not np.isfinite(np.asarray(values, dtype=float)).any():
                        ax.text(0.5, 0.5, 'Measurement not available', transform=ax.transAxes, ha='center', color='gray')
                    _style_axis(ax, label)
                    ax.set_xlim(0, t_max)
                    ax.set_xticks(np.arange(0, t_max + 1, 5 if t_max <= 60 else 10))
                    ax.legend(loc='best', fontsize=8, framealpha=0.95)
                pdf.savefig(fig)
                plt.close(fig)

