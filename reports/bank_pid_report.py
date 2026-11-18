"""Bank PID measured-data report — professional QTG Studio layout.

Structure:
  Page 1  — Cover / Summary (header, metadata table, initial conditions, status)
  Page 2  — Bank Control Performance (target vs actual bank, roll command vs
             measured, tracking error)
  Page 3+ — Flight parameters (3-per-page, matching existing QTG layout)

No Replay comparison, no EASA tolerance assessment, no Ti/Tt evaluation.
Interrupted runs are clearly identified on every page.
"""
import csv
import math
from datetime import datetime
from pathlib import Path

# ---------------------------------------------------------------------------
# Channel map: logical key → CSV column header written by bank_pid_runner.py
# and the standard QTG telemetry recorder.
# ---------------------------------------------------------------------------
CHANNELS = {
    'ias_kt':           'Indicated Airspeed (kts)',
    'vertical_speed_fpm': 'Rate of Climb/Descent (fpm)',
    'altitude_msl_ft':  'Altitude MSL (ft)',
    'height_agl_ft':    'Radar Alt (ft)',
    'ground_speed_kt':  'Ground Speed (kt)',
    'pitch_deg':        'Pitch Angle (deg)',
    'bank_deg':         'Bank Angle (deg)',
    'heading_deg':      'Magnetic Heading (deg)',
    'pitch_rate':       'Pitch Rate (deg/s)',
    'yaw_rate':         'Yaw Rate (deg/s)',
    'prop_rpm':         'Propeller 1 RPM',
    'engine_rpm':       'Engine 1 RPM',
    'throttle_pct':     'Throttle 1 Position (%)',
    'manifold_inhg':    'Manifold Pressure (inHg)',
    'fuel_flow_pph':    'Fuel Flow (pph)',
    'pitch_position':   'Joystick Pitch Position (mm)',
    'pitch_force':      'Pitch Virtual Force 0x51 (N)',
    'roll_position':    'Joystick Roll Position (mm)',
    'roll_force':       'Roll Virtual Force 0x51 (N)',
    'rudder_position':  'Rudder Pedal Position (mm)',
    'rudder_force':     'Yaw Virtual Force 0x51 (N)',
    'elevator_input_norm': 'Measured Elevator Input (normalised)',
    'aileron_input_norm':  'Measured Aileron Input (normalised)',
    'rudder_input_norm':   'Measured Rudder Input (normalised)',
    'titt':             'Ti/Tt response analysis',
}

# Flight-parameter channels shown after the Bank Control Performance page.
# Ordered to match the existing QTG climb report layout.
FLIGHT_PARAM_ORDER = [
    'ias_kt', 'vertical_speed_fpm', 'altitude_msl_ft',
    'pitch_deg', 'bank_deg', 'heading_deg',
    'pitch_rate', 'yaw_rate',
    'prop_rpm', 'engine_rpm', 'throttle_pct',
    'manifold_inhg', 'fuel_flow_pph',
    'pitch_position', 'pitch_force',
    'roll_position', 'roll_force',
    'rudder_position', 'rudder_force',
    'elevator_input_norm', 'aileron_input_norm', 'rudder_input_norm',
]

# QTG Studio colour palette (matches frontend/main.py COL dict)
_BLUE   = '#2563EB'
_NAVY   = '#1E3A8A'
_SLATE  = '#475569'
_GREEN  = '#047857'
_RED    = '#991B1B'
_AMBER  = '#B45309'
_GRAY   = '#94A3B8'
_ORANGE = '#EA580C'


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _col(rows, *candidates):
    """Return float list for the first column name found in *rows*, else NaN list."""
    if not rows:
        return []
    for name in candidates:
        if name in rows[0]:
            out = []
            for r in rows:
                try:
                    out.append(float(r[name]))
                except (TypeError, ValueError):
                    out.append(float('nan'))
            return out
    return [float('nan')] * len(rows)


def _has_data(vals):
    return any(math.isfinite(v) for v in vals)


def _status_color(run_fault):
    return _RED if run_fault else _GREEN


def _fmt_ic(ic_dict):
    """Format an initial-conditions dict as readable key-value pairs."""
    if not ic_dict:
        return 'Not available'
    labels = {
        'alt_ft': 'Altitude (ft)', 'ias_kt': 'IAS (kt)',
        'heading_deg': 'Heading (°)', 'pitch_deg': 'Pitch (°)',
        'bank_deg': 'Bank (°)', 'flaps': 'Flaps',
        'throttle': 'Throttle (%)', 'temperature_c': 'OAT (°C)',
        'on_ground': 'On ground',
    }
    parts = []
    for k, label in labels.items():
        if k in ic_dict:
            v = ic_dict[k]
            if isinstance(v, float):
                parts.append(f'{label}: {v:.1f}')
            else:
                parts.append(f'{label}: {v}')
    return '    |    '.join(parts) if parts else str(ic_dict)


# ---------------------------------------------------------------------------
# Page builders
# ---------------------------------------------------------------------------

def _page_cover(pdf, rows, run_fault, kwargs):
    """Page 1 — Cover with QTG-standard header, metadata table, IC, status."""
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    test_ref  = str(kwargs.get('test_reference', '1.c.(1)'))
    test_title = str(kwargs.get('test_title', 'Normal Take-Off and Climb'))
    exec_date  = datetime.now().strftime('%Y-%m-%d  %H:%M:%S')
    n_samples  = len(rows)
    duration   = float(rows[-1].get('time_s', rows[-1].get('Time (s)', 0))) if rows else 0.0
    ic_dict    = kwargs.get('initial_conditions') or {}

    status_text  = f'INTERRUPTED: {run_fault}' if run_fault else 'COMPLETED — Recorded data, NOT EVALUATED'
    status_color = _status_color(run_fault)

    fig = plt.figure(figsize=(8.27, 11.69))

    # ── Header band ──────────────────────────────────────────────────────────
    fig.text(0.08, 0.955, 'Aeromix Multi-Crew FNTP II',
             fontsize=17, fontweight='bold', color='#0F172A')
    fig.text(0.08, 0.930, 'EASA CS-FSTD(A) Issue 2 — Objective Validation Test Report',
             fontsize=10, fontweight='bold', color=_BLUE)
    fig.text(0.08, 0.912, 'Qualification Standard: FNPT II (Multi-Crew)  |  AMC1 FSTD(A).300(b)(3)',
             fontsize=8.5, color=_SLATE)

    # Thin rule under header
    ax_rule = fig.add_axes([0.08, 0.907, 0.84, 0.002])
    ax_rule.set_facecolor(_BLUE)
    ax_rule.axis('off')

    # ── Metadata table ───────────────────────────────────────────────────────
    ax_meta = fig.add_axes([0.08, 0.780, 0.84, 0.120])
    ax_meta.axis('off')
    meta_data = [
        ['Test Reference:',   test_ref,                   'Execution Date:',      exec_date],
        ['Test Title:',       test_title,                  'Control Method:',      'Bank PID (Automatic)'],
        ['Bank Target:',      '0.00 °  (wings level)',     'FSTD Level:',          'FNPT II'],
        ['Compliance Status:', 'RECORDED — NOT ASSESSED',  'Recording Duration:',  f'{duration:.1f} s  /  {n_samples} samples'],
    ]
    tbl = ax_meta.table(cellText=meta_data, cellLoc='left', loc='center',
                        colWidths=[0.20, 0.35, 0.20, 0.25])
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(8.5)
    tbl.scale(1, 1.45)
    for (r, c), cell in tbl.get_celld().items():
        cell.set_edgecolor('#CBD5E1')
        if c in (0, 2):
            cell.set_facecolor('#F1F5F9')
            cell.set_text_props(weight='bold', color='#1E293B')
        else:
            cell.set_facecolor('#FFFFFF')
        if r == 3 and c == 1:
            cell.set_text_props(color=_GREEN, weight='bold')

    # ── Initial conditions box ───────────────────────────────────────────────
    ax_ic = fig.add_axes([0.08, 0.640, 0.84, 0.130])
    ax_ic.axis('off')
    rect_ic = plt.Rectangle((0, 0), 1, 1, facecolor='#F8FAFC', edgecolor='#94A3B8',
                              linewidth=1, transform=ax_ic.transAxes)
    ax_ic.add_patch(rect_ic)
    ax_ic.text(0.015, 0.90, 'INITIAL CONDITIONS:', fontsize=9, fontweight='bold',
               color='#1E293B', transform=ax_ic.transAxes, va='top')
    ax_ic.text(0.015, 0.68, _fmt_ic(ic_dict), fontsize=8.5, fontweight='bold',
               color='#0F172A', transform=ax_ic.transAxes, va='top', wrap=True)
    ax_ic.text(0.015, 0.30, 'OBJECTIVE:', fontsize=9, fontweight='bold',
               color='#1E293B', transform=ax_ic.transAxes, va='top')
    ax_ic.text(0.015, 0.10,
               'Verify that the Bank PID holds wings-level (bank = 0°) using the BRUNNER roll '
               'loader only. No P3D flight-model writes. Acquisition is excluded from the '
               'recording. No EASA tolerance assessment is performed.',
               fontsize=8, color='#334155', transform=ax_ic.transAxes, va='top', wrap=True)

    # ── Status banner ────────────────────────────────────────────────────────
    ax_st = fig.add_axes([0.08, 0.565, 0.84, 0.062])
    ax_st.axis('off')
    bg_col = '#FEF2F2' if run_fault else '#ECFDF5'
    bd_col = '#EF4444' if run_fault else '#10B981'
    rect_st = plt.Rectangle((0, 0), 1, 1, facecolor=bg_col, edgecolor=bd_col,
                              linewidth=1.5, transform=ax_st.transAxes)
    ax_st.add_patch(rect_st)
    ax_st.text(0.015, 0.60, 'RUN STATUS:', fontsize=9, fontweight='bold',
               color=status_color, transform=ax_st.transAxes, va='center')
    ax_st.text(0.18, 0.60, status_text, fontsize=9, fontweight='bold',
               color=status_color, transform=ax_st.transAxes, va='center', wrap=True)

    # ── Report contents list ─────────────────────────────────────────────────
    ax_cont = fig.add_axes([0.08, 0.300, 0.84, 0.255])
    ax_cont.axis('off')
    ax_cont.text(0, 0.97, 'REPORT CONTENTS:', fontsize=9, fontweight='bold',
                 color='#1E293B', transform=ax_cont.transAxes, va='top')
    contents = [
        'Page 2  —  Bank Control Performance',
        '            • Bank Angle (°): recorded vs 0° target',
        '            • BRUNNER Roll Command vs Measured Roll Position',
        '            • Roll Tracking Error (command − measured)',
        'Pages 3+  —  Flight Parameters (Airspeed, Altitude, ROC, Pitch, Heading, Engine, Controls)',
        '',
        'Note: Ti/Tt procedure evaluation is not performed for Bank PID runs.',
        'Note: No Recorded Replay comparison is included.',
    ]
    ax_cont.text(0, 0.84, '\n'.join(contents), fontsize=8.5, color='#334155',
                 transform=ax_cont.transAxes, va='top', family='monospace')

    # ── Footer ───────────────────────────────────────────────────────────────
    fig.text(0.08, 0.055,
             f'Test Reference: {test_ref}    |    Duration: {duration:.1f} s    |    '
             f'Samples: {n_samples}    |    Compliance: NOT ASSESSED',
             fontsize=8.5, fontweight='bold', color='#0F172A')
    fig.text(0.08, 0.035,
             'Generated automatically by Aeromix Multi-Crew FNTP II Studio — Bank PID Module.',
             fontsize=7.5, style='italic', color=_SLATE)

    pdf.savefig(fig)
    plt.close(fig)


def _page_bank_control(pdf, rows, run_fault, kwargs):
    """Page 2 — Bank Control Performance: the three primary PID diagnostic plots."""
    import matplotlib.pyplot as plt

    test_ref = str(kwargs.get('test_reference', ''))
    status   = f'INTERRUPTED: {run_fault}' if run_fault else 'Recorded data — NOT EVALUATED'

    t          = _col(rows, 'time_s', 'Time (s)')
    bank_actual = _col(rows, 'bank_deg', 'Bank Angle (deg)')
    roll_cmd   = _col(rows, 'roll_command', 'Roll Command')
    roll_meas  = _col(rows, 'roll_measured', 'Measured Roll Position', 'roll_measured')
    track_err  = _col(rows, 'tracking_error', 'Tracking Error')

    # Derive tracking error if not in CSV
    if not _has_data(track_err) and _has_data(roll_cmd) and _has_data(roll_meas):
        track_err = [
            (c - m if math.isfinite(c) and math.isfinite(m) else float('nan'))
            for c, m in zip(roll_cmd, roll_meas)
        ]

    fig, axes = plt.subplots(3, 1, figsize=(8.27, 11.69))
    fig.suptitle(f'Test {test_ref} — Bank PID Control Performance',
                 fontsize=12, fontweight='bold', color=_NAVY, y=0.975)

    # Plot 1: Bank angle vs target
    ax = axes[0]
    if _has_data(bank_actual):
        ax.plot(t, bank_actual, color=_BLUE, linewidth=1.4, label='Bank Angle (recorded)')
    else:
        ax.text(0.5, 0.5, 'Bank angle telemetry not available',
                transform=ax.transAxes, ha='center', color=_SLATE)
    ax.axhline(0.0, color=_GRAY, linestyle='--', linewidth=1.2, label='Target: 0°')
    ax.set_ylabel('Bank Angle (°)', fontsize=9)
    ax.set_xlabel('Time (s)', fontsize=9)
    ax.set_title('Bank Angle vs Target (0°)', fontsize=9, fontweight='bold', color=_NAVY)
    ax.legend(fontsize=8, loc='upper right')
    ax.grid(True, alpha=0.35)
    ax.tick_params(labelsize=8)

    # Plot 2: Roll command vs measured roll position
    ax = axes[1]
    if _has_data(roll_cmd):
        ax.plot(t, roll_cmd, color=_BLUE, linewidth=1.4, label='BRUNNER Roll Command')
    if _has_data(roll_meas):
        ax.plot(t, roll_meas, color=_ORANGE, linestyle='--', linewidth=1.2,
                label='Measured Roll Position')
    if not _has_data(roll_cmd) and not _has_data(roll_meas):
        ax.text(0.5, 0.5, 'Roll command / position data not available',
                transform=ax.transAxes, ha='center', color=_SLATE)
    else:
        ax.legend(fontsize=8, loc='upper right')
    ax.set_ylabel('Position (normalised)', fontsize=9)
    ax.set_xlabel('Time (s)', fontsize=9)
    ax.set_title('BRUNNER Roll Command vs Measured Roll Position', fontsize=9,
                 fontweight='bold', color=_NAVY)
    ax.grid(True, alpha=0.35)
    ax.tick_params(labelsize=8)

    # Plot 3: Tracking error
    ax = axes[2]
    if _has_data(track_err):
        ax.plot(t, track_err, color=_AMBER, linewidth=1.2, label='Tracking Error (cmd − meas)')
        ax.axhline(0.0, color=_GRAY, linestyle='--', linewidth=0.8)
        ax.legend(fontsize=8, loc='upper right')
    else:
        ax.text(0.5, 0.5, 'Tracking error data not available',
                transform=ax.transAxes, ha='center', color=_SLATE)
    ax.set_ylabel('Error (normalised)', fontsize=9)
    ax.grid(True, alpha=0.35)
    ax.tick_params(labelsize=8)

    fig.text(0.08, 0.012, status, fontsize=7.5, color=_RED if run_fault else _SLATE)
    fig.tight_layout(rect=(0, 0.025, 1, 0.965))
    pdf.savefig(fig)
    plt.close(fig)


def _page_flight_params(pdf, rows, run_fault, chosen, kwargs):
    """Pages 3+ — Flight parameters, 3-per-page, matching existing QTG layout."""
    import matplotlib.pyplot as plt

    test_ref = str(kwargs.get('test_reference', ''))
    status   = f'INTERRUPTED: {run_fault}' if run_fault else 'Recorded data — NOT EVALUATED'

    t = _col(rows, 'time_s', 'Time (s)')

    def _get(key):
        col_name = CHANNELS.get(key, key)
        if col_name == 'Manifold Pressure (inHg)':
            vals = _col(rows, col_name)
            if not _has_data(vals):
                vals = [v * 2.0360206576 if math.isfinite(v) else float('nan')
                        for v in _col(rows, 'Engine Manifold Pressure (psi)')]
            return vals
        return _col(rows, col_name, key)

    # Skip bank_deg — already on the Bank Control page
    keys = [k for k in chosen if k != 'bank_deg' and k != 'titt']

    for offset in range(0, len(keys), 3):
        batch = keys[offset:offset + 3]
        fig, axes = plt.subplots(len(batch), 1, figsize=(8.27, 11.69), squeeze=False)
        fig.suptitle(f'Test {test_ref} — Bank PID  |  Flight Parameters',
                     fontsize=11, fontweight='bold', color=_NAVY, y=0.978)

        for ax, key in zip(axes[:, 0], batch):
            col_label = CHANNELS.get(key, key)
            y = _get(key)
            if _has_data(y):
                ax.plot(t, y, color=_BLUE, linewidth=1.3, label='Recorded measurement')
            else:
                ax.text(0.5, 0.5, 'Measurement not available in this recording',
                        transform=ax.transAxes, ha='center', color=_SLATE, fontsize=8)
            ax.set_ylabel(col_label, fontsize=8.5)
            ax.set_xlabel('Time (s)', fontsize=8.5)
            ax.set_title(col_label, fontsize=8.5, fontweight='bold', color=_NAVY)
            ax.grid(True, alpha=0.35)
            ax.tick_params(labelsize=8)

        fig.text(0.08, 0.012, status, fontsize=7.5, color=_RED if run_fault else _SLATE)
        fig.tight_layout(rect=(0, 0.025, 1, 0.965))
        pdf.savefig(fig)
        plt.close(fig)


# ---------------------------------------------------------------------------
# Public entry point — same signature as before; fully backward-compatible.
# ---------------------------------------------------------------------------

def report(csv_path, output_pdf_path=None, run_fault=None, selected_graphs=None, **kwargs):
    """Generate the Bank PID PDF report.

    Parameters
    ----------
    csv_path        : path to the bank_pid_diagnostics_*.csv written by bank_pid_runner
    output_pdf_path : optional override for the output PDF path
    run_fault       : fault string if the run was interrupted (None = completed)
    selected_graphs : list of channel keys to include in flight-param pages
    **kwargs        : test_reference, test_title, initial_conditions, etc.
    """
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages

    path   = Path(csv_path)
    output = Path(output_pdf_path) if output_pdf_path else path.with_name(path.stem + '_report.pdf')

    with path.open(newline='', encoding='utf-8-sig') as stream:
        rows = list(csv.DictReader(stream))

    diagnostics = kwargs.get('bank_diagnostics_path')
    if diagnostics:
        with Path(diagnostics).open(newline='', encoding='utf-8-sig') as stream:
            recorded = [row for row in csv.DictReader(stream) if row.get('phase') == 'recording']
        if len(recorded) != len(rows):
            raise RuntimeError('Bank diagnostic/recording sample counts differ; report cannot align controls')
        for row, control in zip(rows, recorded):
            row.update({key: control[key] for key in ('roll_command','roll_measured','tracking_error','target_bank_deg')})
    for row in rows:
        row['time_s'] = row.get('Time (s)', row.get('recording_time_s', row.get('time_s', 0)))

    # Flight-parameter channel list: use the requested selection or the default order.
    # Always exclude 'titt' (not applicable to Bank PID runs).
    if selected_graphs is not None:
        chosen = [k for k in selected_graphs if k != 'titt']
    else:
        chosen = [k for k in FLIGHT_PARAM_ORDER if k in CHANNELS]

    with PdfPages(str(output)) as pdf:
        _page_cover(pdf, rows, run_fault, kwargs)
        _page_bank_control(pdf, rows, run_fault, kwargs)
        _page_flight_params(pdf, rows, run_fault, chosen, kwargs)

    return str(output)
