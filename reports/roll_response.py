"""Measured-only PDF report for Roll Response (rate) tests 2.d.(2a) and 2.d.(2b)."""

import csv
from datetime import datetime
from pathlib import Path
import textwrap

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

from reports.power_dynamics import CHANNELS, INITIAL_ROWS, _display, _number, graph_availability

DEFAULT_GRAPHS = (
    'roll_rate',
    'bank_deg',
    'roll_position',
    'roll_force',
    'aileron_input_norm',
    'ias_kt',
    'altitude_msl_ft',
    'pitch_deg',
    'heading_deg',
)


def render_report(csv_path, pdf_path, metadata):
    """Plot measured roll-response time histories with roll-input and stabilization markers."""
    csv_path, pdf_path = Path(csv_path), Path(pdf_path)
    with csv_path.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError('Roll Response report has no recorded samples')

    chosen, unavailable = graph_availability(
        rows,
        metadata.get('selected_graphs') if metadata.get('selected_graphs') is not None else DEFAULT_GRAPHS,
    )
    first = rows[0]
    cfg = metadata.get('initial_conditions') or {}
    start_stamp = datetime.fromtimestamp(csv_path.stat().st_mtime).strftime('%d %b %Y %H:%M:%S')
    roll_cmd_t = _number(metadata.get('roll_command_time_s'))
    roll_stab_t = _number(metadata.get('roll_rate_stabilized_time_s'))
    roll_ret_t = _number(metadata.get('roll_return_command_time_s'))
    steady_rate = _number(metadata.get('steady_roll_rate_deg_s'))
    end_time = _number(rows[-1].get('Time (s)')) or 0.0
    ref = metadata.get('test_reference', '2.d.(2a)')
    kind = metadata.get('kind', 'Roll Response')

    with PdfPages(pdf_path) as pdf:
        fig = plt.figure(figsize=(8.27, 11.69))
        fig.text(.08, .95, f'{kind}  |  {ref}', fontsize=16, weight='bold')
        fig.text(.08, .915, f'Automatic test  |  {start_stamp}  |  {metadata.get("result", "UNKNOWN")}', fontsize=9)
        fig.text(.08, .889, 'Configured Initial Conditions vs first recorded frame (graph time zero)', fontsize=10, weight='bold')
        fig.text(.08, .862, f'{"Parameter":<23}{"Configured":<23}Recorded at t=0', fontsize=9, family='monospace', weight='bold')
        for index, (label, setting, column, unit) in enumerate(INITIAL_ROWS):
            fig.text(
                .08,
                .837 - index * .027,
                f'{label:<23}{_display(cfg.get(setting), unit):<23}{_display(first.get(column), unit)}',
                fontsize=9,
                family='monospace',
            )
        fig.text(.08, .585, 'Recorded Roll Response Sequence', fontsize=10, weight='bold')
        fig.text(
            .08,
            .555,
            f'Stable level bank (0 deg): 2.0 s  |  Left roll command (-30%): {_display(roll_cmd_t, "s")}',
            fontsize=9,
        )
        fig.text(
            .08,
            .530,
            f'Roll rate stabilized: {_display(roll_stab_t, "s")}  |  Steady roll rate: {_display(steady_rate, "deg/s")}',
            fontsize=9,
        )
        fig.text(
            .08,
            .505,
            f'Return roll to 0: {_display(roll_ret_t, "s")}  |  Recording stopped: {end_time:.2f} s',
            fontsize=9,
        )
        fig.text(.08, .455, 'Selected graph channels without recorded data', fontsize=10, weight='bold')
        missing_names = [
            CHANNELS[key][0] if key in CHANNELS else key.replace('_', ' ')
            for key in unavailable
        ]
        missing_text = ', '.join(missing_names) if missing_names else 'None'
        fig.text(.08, .428, textwrap.fill(missing_text, 86), fontsize=8, va='top')
        fig.text(.08, .31, 'Only measured channels appear in the following graphs.', fontsize=9)
        fig.text(.08, .285, 'Aircraft reference comparison: NOT EVALUATED', fontsize=9, weight='bold')
        fig.text(.08, .12, 'Source: ' + csv_path.name, fontsize=7, color='#475569')
        fig.text(.08, .10, 'Doc Number: 161310G00000-00  |  Rev: 008', fontsize=7, color='#475569')
        pdf.savefig(fig)
        plt.close(fig)

        times = [_number(row.get('Time (s)')) for row in rows]
        for start in range(0, len(chosen), 3):
            keys = chosen[start:start + 3]
            fig, axes = plt.subplots(len(keys), 1, figsize=(8.27, 11.69), squeeze=False)
            fig.subplots_adjust(left=.13, right=.95, top=.90, bottom=.075, hspace=.47)
            fig.suptitle(f'{kind} ({ref}) — measured response', fontsize=13, weight='bold')
            for axis, key in zip(axes[:, 0], keys):
                title, unit, column = CHANNELS[key]
                pairs = [
                    (t, y) for t, row in zip(times, rows)
                    if t is not None and (y := _number(row.get(column))) is not None
                ]
                axis.plot([t for t, _ in pairs], [y for _, y in pairs], color='#2563eb', linewidth=1.2)
                if roll_cmd_t is not None:
                    axis.axvline(roll_cmd_t, color='#d97706', linestyle='--', linewidth=1, label='Roll -30% command')
                if roll_stab_t is not None:
                    axis.axvline(roll_stab_t, color='#16a34a', linestyle=':', linewidth=1.1, label='Roll rate stabilized / Return 0')
                axis.set(title=title, ylabel=unit, xlabel='Time from recording start (s)')
                axis.set_xlim(0, max(end_time, 1.0))
                axis.grid(True, alpha=.35)
                axis.legend(loc='best', fontsize=7)
            fig.text(.08, .025, f'Source: {csv_path.name}  |  Page {start // 3 + 2}', fontsize=6.5, color='#475569')
            pdf.savefig(fig)
            plt.close(fig)

    return chosen, unavailable
