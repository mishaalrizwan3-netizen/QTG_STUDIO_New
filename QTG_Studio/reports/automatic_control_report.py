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
from interfaces.brunner_base.src.plot_graphs import _initial_page, _style_axis, _series, _recorded_duration, _set_recorded_time


DEFAULT_SELECTED_BY_REF = {
    '2.a.(1)': ['ias_kt', 'pitch_position', 'pitch_force'],
    '2.a.(2)': ['ias_kt', 'roll_position', 'roll_force'],
    '2.a.(3)': ['ias_kt', 'rudder_position', 'rudder_force'],
    '2.a.(4)': ['trim_indicator', 'trim_position_rad', 'trim_value_pct', 'trim_elevator_surface_rad'],
    '2.a.(7)': ['trim_indicator', 'trim_position_rad', 'trim_value_pct', 'trim_elevator_surface_rad'],
    '2.c.(8b)': ['ias_kt', 'stall_warning', 'stall_aoa', 'pitch_deg', 'bank_deg', 'pitch_position'],
}

CHANNEL_CONFIG = {
    'stall_warning': {'col':'Stall Warning (0/1)', 'alt_cols':['Stall Warning (0/1)'], 'label':'Stall Warning (0/1)', 'group':'Stall Warning'},
    'stall_aoa': {'col':'Aircraft Angle of Attack (deg)', 'alt_cols':['Aircraft Angle of Attack (deg)'], 'label':'Aircraft Angle of Attack (deg)', 'group':'Stall Warning'},
    'stall_elevator': {'col':'Elevator Surface (rad)', 'alt_cols':['Elevator Surface (rad)'], 'label':'Elevator Surface (rad)', 'group':'Stall Warning'},
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


from interfaces.engine_measurements import BODY_ACCELERATION_CHANNELS

CHANNEL_CONFIG.update({
    key: {'col': simvar, 'alt_cols': [simvar, key],
          'label': f'Body {simvar[-1]} acceleration (ft/s²)', 'group': 'Body-axis Acceleration'}
    for key, simvar, _ in BODY_ACCELERATION_CHANNELS
})

CHANNEL_CONFIG.update({
    key: {'col': column, 'alt_cols': [column], 'label': column, 'group': 'Pitch trim'}
    for key, column in (
        ('trim_position_rad', 'Elevator Trim Angle (rad)'),
        ('trim_indicator', 'Elevator Trim Indicator (position)'),
        ('trim_value_pct', 'Elevator Trim (%)'),
        ('trim_elevator_surface_rad', 'Elevator Surface (rad)'),
    )
})


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
    is_trim = test_ref in ('2.a.(7)', '2.a.(4)')
    is_stall = test_ref == '2.c.(8b)'
    trim_metrics = context.get('pitch_trim_metrics', {})
    travel_time = trim_metrics.get('total_travel_time_s')
    avg_rate_pct = trim_metrics.get('average_trim_rate_pct_s')
    avg_rate_pos = trim_metrics.get('average_trim_rate_pos_s')
    if is_trim and travel_time is None and rows:
        try:
            t0 = float(rows[0].get('Time (s)', 0.0))
            t1 = float(rows[-1].get('Time (s)', 0.0))
            travel_time = max(0.1, t1 - t0)
            p0 = float(rows[0].get('Elevator Trim Indicator (position)', 0.0))
            p1 = float(rows[-1].get('Elevator Trim Indicator (position)', 0.0))
            d_pos = abs(p0 - p1)
            avg_rate_pos = d_pos / travel_time
            avg_rate_pct = avg_rate_pos * 100.0
        except Exception:
            travel_time = avg_rate_pct = avg_rate_pos = 0.0

    metadata = dict(
        context,
        result=result,
        reason=reason,
        status='complete' if result == 'COMPLETE' else result.lower(),
        mode='auto',
        control_method='stall' if is_stall else 'pitch_trim' if is_trim else 'static_control',
        axis_name=axis_name,
        samples=len(rows),
        measurement_source=('CLS2Sim trim button actions; P3D SimConnect' if is_trim else 'BRUNNER CLS2Sim UDP load-cell force & position; P3D SimConnect'),
        validation=('Stall warning cycle; no automatic grading' if is_stall else 'Pitch trim sweep; no automatic grading' if is_trim else 'Static position vs force characteristic verification'),
    )
    if is_trim:
        metadata.update({
            'total_travel_time_s': travel_time,
            'average_trim_rate_pct_s': avg_rate_pct,
            'average_trim_rate_pos_s': avg_rate_pos,
            'trim_start_position': trim_metrics.get('start_trim_pos'),
            'trim_end_position': trim_metrics.get('final_trim_pos'),
        })
    metadata_path.write_text(json.dumps(metadata, indent=2, allow_nan=False), encoding='utf-8')
    outputs.append(str(metadata_path))

    # 3. Generate Climb-style PDF
    _render_climb_style_report(csv_path, pdf_path, context, rows)
    outputs.append(str(pdf_path))

    return outputs


def _build_reference_df(context: Dict[str, Any], sim_df: pd.DataFrame) -> Optional[pd.DataFrame]:
    """Load a real comparison file when one was selected by the operator."""
    for key in ('reference_csv', 'validation_csv'):
        ref_path = context.get(key)
        if ref_path and Path(ref_path).is_file():
            try:
                loaded = pd.read_csv(ref_path)
                if not loaded.empty:
                    return loaded
            except Exception:
                pass

    return None


def _render_climb_style_report(csv_path: Path, out_path: Path, context: Dict[str, Any], rows: List[Dict[str, Any]]):
    """Renders official QTG report matching the climb test format with selected graphs in BLUE."""
    test_ref = context.get('test_reference', '2.a.(1)')
    selected = context.get('selected_graphs')
    if not selected:
        selected = DEFAULT_SELECTED_BY_REF.get(test_ref, ['ias_kt', 'pitch_position', 'pitch_force'])

    df = pd.DataFrame(rows) if rows else pd.read_csv(csv_path)
    ref_df = _build_reference_df(context, df)
    tm = _series(df, 'Time (s)')
    t_max = _recorded_duration(tm)

    trim_subtitle = None
    if test_ref in ('2.a.(7)', '2.a.(4)'):
        metrics = context.get('pitch_trim_metrics', {})
        t_time = metrics.get('total_travel_time_s')
        r_pct = metrics.get('average_trim_rate_pct_s')
        r_pos = metrics.get('average_trim_rate_pos_s')
        if t_time is None and rows:
            try:
                t0 = float(rows[0].get('Time (s)', 0.0))
                t1 = float(rows[-1].get('Time (s)', 0.0))
                t_time = max(0.1, t1 - t0)
                p0 = float(rows[0].get('Elevator Trim Indicator (position)', 0.0))
                p1 = float(rows[-1].get('Elevator Trim Indicator (position)', 0.0))
                r_pos = abs(p0 - p1) / t_time
                r_pct = r_pos * 100.0
            except Exception:
                t_time = r_pct = r_pos = 0.0
        if t_time is not None:
            trim_subtitle = f"Total Travel Time: {t_time:.1f} s  |  Average Trim Rate: {r_pct:.2f} %/s ({r_pos:.3f} pos/s)"

    # Determine which items are requested
    from reports.plot_graphs import OPTIONAL_GRAPH_COLUMNS
    chosen_items = []
    for key in selected:
        cfg = CHANNEL_CONFIG.get(key)
        if cfg is None and key in OPTIONAL_GRAPH_COLUMNS:
            column, _ = OPTIONAL_GRAPH_COLUMNS[key]
            cfg = dict(col=column, alt_cols=[column, key], label=column, group='Additional measurements')
        if cfg:
            # Look up series
            series = None
            for col_name in cfg['alt_cols']:
                if col_name in df.columns:
                    val = _series(df, col_name)
                    if np.isfinite(val).any():
                        series = -val if test_ref in STATIC_CONTROL_REFERENCES and 'Virtual Force' in col_name else val
                        break
            if series is None:
                series = _series(df, cfg['col'])
            label = 'Pitch Applied Force (N)' if test_ref == '2.c.(8b)' and key == 'pitch_force' else cfg['label']
            chosen_items.append((label, series, cfg['group']))

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
        if 0 < len(chosen_items) <= 3:
            _initial_page(pdf, 'auto', str(csv_path), df, ref_df, virtual_forces=ref_df is not None and test_ref != '2.c.(8b)', missing_label='N/A')
            count = len(chosen_items)
            fig, axes = plt.subplots(count, 1, figsize=(8.27, 11.69), constrained_layout=True, squeeze=False)
            fig.suptitle(f'{test_ref} Pitch Trim System Rate and Authority' if test_ref in ('2.a.(7)', '2.a.(4)') else 'Selected test graphs', fontsize=14, fontweight='bold')
            if trim_subtitle:
                fig.text(0.5, 0.965, trim_subtitle, ha='center', va='top', fontsize=9, fontweight='bold', color='#1D4ED8')
            for ax, (label, values, _) in zip(axes[:, 0], chosen_items):
                ax.plot(tm, values, color='#0066cc', linewidth=1.6, label='Automatic Test (Sim)')
                _style_axis(ax, label)
                _set_recorded_time(ax, t_max)
                ax.tick_params(labelsize=7)
                ax.xaxis.label.set_size(7); ax.yaxis.label.set_size(7)
                if not np.isfinite(np.asarray(values, dtype=float)).any():
                    ax.text(.5, .5, 'Measurement not available', transform=ax.transAxes, ha='center', va='center', color='gray')
                ax.legend(loc='best', fontsize=6)
            pdf.savefig(fig)
            plt.close(fig)
            return
        _initial_page(pdf, 'auto', str(csv_path), df, ref_df, virtual_forces=ref_df is not None and test_ref != '2.c.(8b)', missing_label='N/A')

        # Page 2+: Selected Graphs
        if not chosen_items:
            # Fallback if nothing was matched
            fig, ax = plt.subplots(1, 1, figsize=(8.27, 11.69), constrained_layout=True)
            fig.suptitle(f'{test_ref} Pitch Trim System Rate and Authority' if test_ref in ('2.a.(7)', '2.a.(4)') else '1st Quadrant Combined Flight & Control Parameters', fontsize=14, fontweight='bold')
            if trim_subtitle:
                fig.text(0.5, 0.965, trim_subtitle, ha='center', va='top', fontsize=9, fontweight='bold', color='#1D4ED8')
            ax.text(0.5, 0.5, 'No parameters selected for display', transform=ax.transAxes, ha='center', color='gray')
            pdf.savefig(fig)
            plt.close(fig)
            return

        # If 4 or fewer items, present them together on Page 2 for direct visual correlation
        if len(chosen_items) <= 4:
            fig, axes = plt.subplots(len(chosen_items), 1, figsize=(8.27, 11.69), constrained_layout=True, squeeze=False)
            fig.suptitle(f'{test_ref} Pitch Trim System Rate and Authority' if test_ref in ('2.a.(7)', '2.a.(4)') else '1st Quadrant Combined Flight & Control Parameters', fontsize=14, fontweight='bold')
            if trim_subtitle:
                fig.text(0.5, 0.965, trim_subtitle, ha='center', va='top', fontsize=9, fontweight='bold', color='#1D4ED8')
            for ax, (label, values, _) in zip(axes[:, 0], chosen_items):
                ax.plot(tm, values, color='#0066cc', linewidth=2.0, label='Automatic Test (Sim)')
                if not np.isfinite(np.asarray(values, dtype=float)).any():
                    ax.text(0.5, 0.5, 'Measurement not available', transform=ax.transAxes, ha='center', color='gray')
                _style_axis(ax, label)
                _set_recorded_time(ax, t_max)
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
                    _set_recorded_time(ax, t_max)
                    ax.legend(loc='best', fontsize=8, framealpha=0.95)
                pdf.savefig(fig)
                plt.close(fig)

