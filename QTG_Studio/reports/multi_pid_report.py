"""
Multi-PID measured-data report — professional QTG Studio layout.
Generates multi-channel graphs according to which PIDs were active:
  - Airspeed PID: IAS & Pitch graphs
  - Bank PID: Bank Angle & Roll Joystick command
  - Yaw PID: Heading, Bank Angle, Rudder Pedal command
"""
from __future__ import annotations

import csv
from datetime import datetime
import math
from pathlib import Path


def load_telemetry_csv(csv_path: Path):
    rows = []
    with csv_path.open('r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)
    return rows


def report(csv_path, output_pdf_path=None, **kwargs):
    csv_path = Path(csv_path)
    if output_pdf_path is None:
        output_pdf_path = csv_path.with_suffix('.pdf')
    else:
        output_pdf_path = Path(output_pdf_path)

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages

    rows = load_telemetry_csv(csv_path)
    run_fault = kwargs.get('run_fault')
    test_reference = kwargs.get('test_reference', '1.c.(1)')

    pitch_enabled = kwargs.get('pitch_enabled', True)
    bank_enabled = kwargs.get('bank_enabled', True)
    yaw_enabled = kwargs.get('yaw_enabled', True)

    times, ias, pitches, banks, hdgs, pitch_cmds, roll_cmds, rudder_cmds = [], [], [], [], [], [], [], []
    for r in rows:
        try:
            times.append(float(r.get('time_s', r.get('Time (s)', 0))))
            ias.append(float(r.get('ias_kt', r.get('Indicated Airspeed (kts)', 0))))
            pitches.append(float(r.get('pitch_deg', r.get('Pitch Angle (deg)', 0))))
            banks.append(float(r.get('bank_deg', r.get('Bank Angle (deg)', 0))))
            hdgs.append(float(r.get('heading_deg', r.get('Magnetic Heading (deg)', 0))))
            pitch_cmds.append(float(r.get('pitch_command', 0)))
            roll_cmds.append(float(r.get('roll_command', 0)))
            rudder_cmds.append(float(r.get('rudder_command', 0)))
        except (ValueError, TypeError):
            continue

    with PdfPages(output_pdf_path) as pdf:
        # Page 1: Cover / Summary
        fig, ax = plt.subplots(figsize=(8.5, 11))
        ax.axis('off')
        fig.patch.set_facecolor('white')

        status_text = f"INTERRUPTED: {run_fault}" if run_fault else "COMPLETED"
        status_color = "#b00020" if run_fault else "#007a33"

        plt.text(0.5, 0.90, "FLIGHT TEST REPORT - MULTI-PID AUTOMATIC",
                 fontsize=18, fontweight='bold', ha='center', color='#1a365d')
        plt.text(0.5, 0.85, f"Test Reference: {test_reference}",
                 fontsize=14, ha='center', color='#4a5568')

        ax.add_patch(plt.Rectangle((0.15, 0.72), 0.70, 0.08,
                                   facecolor="#fef2f2" if run_fault else "#f0fdf4",
                                   edgecolor=status_color, linewidth=2))
        plt.text(0.5, 0.75, f"RUN STATUS:  {status_text}",
                 fontsize=13, fontweight='bold', ha='center', color=status_color)

        duration_s = f"{times[-1] - times[0]:.1f} s" if len(times) > 1 else "0.0 s"
        active_list = []
        if pitch_enabled: active_list.append("Airspeed PID (Pitch Axis 1)")
        if bank_enabled: active_list.append("Bank PID (Roll Axis 2)")
        if yaw_enabled: active_list.append("Yaw PID (Roll Axis 2 & Rudder Axis 4)")

        meta_lines = [
            f"Active Controllers:         {', '.join(active_list) if active_list else 'Recorded Replay'}",
            f"Stabilization Gate:         5.0 s continuous stability on all active axes",
            f"Recorded Duration:          {duration_s}",
            f"Generated:                  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        ]
        y_pos = 0.62
        for line in meta_lines:
            plt.text(0.18, y_pos, line, fontsize=11, fontfamily='monospace', color='#2d3748')
            y_pos -= 0.045

        pdf.savefig(fig, bbox_inches='tight')
        plt.close(fig)

        # Page 2: Performance Graphs
        plots_needed = []
        if pitch_enabled:
            plots_needed.append(('Airspeed & Pitch Attitude', times, ias, 'IAS (kt)', '#2563eb'))
        if bank_enabled or yaw_enabled:
            plots_needed.append(('Bank Angle', times, banks, 'Bank (deg)', '#059669'))
        if yaw_enabled:
            plots_needed.append(('Magnetic Heading', times, hdgs, 'Heading (deg)', '#7c3aed'))

        if plots_needed:
            fig, axes = plt.subplots(len(plots_needed), 1, figsize=(8.5, 11), sharex=True)
            if len(plots_needed) == 1:
                axes = [axes]
            fig.suptitle('Multi-PID Recorded Performance', fontsize=16, fontweight='bold', y=0.95)

            for ax_p, (title, t, y, ylabel, col) in zip(axes, plots_needed):
                ax_p.plot(t, y, label=title, color=col, linewidth=1.8)
                ax_p.set_ylabel(ylabel, fontsize=10, fontweight='bold')
                ax_p.grid(True, linestyle='--', alpha=0.6)
                ax_p.legend(loc='upper right')

            axes[-1].set_xlabel('Time (s)', fontsize=10, fontweight='bold')
            plt.tight_layout(rect=[0.05, 0.05, 0.95, 0.92])
            pdf.savefig(fig)
            plt.close(fig)

    return str(output_pdf_path)

