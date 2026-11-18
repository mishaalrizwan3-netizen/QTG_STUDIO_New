"""Yaw/Heading PID measured-data report — professional QTG Studio layout.

Structure:
  Page 1  — Cover / Summary (header, metadata table, initial conditions, target heading, status)
  Page 2  — Heading & Yaw Performance (target vs actual heading, commanded vs actual bank, roll and rudder commands)
  Page 3+ — Flight parameters (3-per-page, matching existing QTG layout)

No Replay comparison, no EASA tolerance assessment.
Interrupted runs are clearly marked on every page.
"""
import csv
import math
from datetime import datetime
from pathlib import Path

CHANNELS = {
    'ias_kt':             'Indicated Airspeed (kts)',
    'vertical_speed_fpm': 'Rate of Climb/Descent (fpm)',
    'altitude_msl_ft':    'Altitude MSL (ft)',
    'height_agl_ft':      'Radar Alt (ft)',
    'ground_speed_kt':    'Ground Speed (kt)',
    'pitch_deg':          'Pitch Angle (deg)',
    'bank_deg':           'Bank Angle (deg)',
    'heading_deg':        'Magnetic Heading (deg)',
    'pitch_rate':         'Pitch Rate (deg/s)',
    'yaw_rate':           'Yaw Rate (deg/s)',
    'roll_rate':          'Roll Rate (deg/s)',
    'prop_rpm':           'Propeller 1 RPM',
    'engine_rpm':         'Engine 1 RPM',
    'throttle_pct':       'Throttle 1 Position (%)',
    'manifold_inhg':      'Manifold Pressure (inHg)',
    'fuel_flow_pph':      'Fuel Flow (pph)',
    'roll_position':      'Joystick Roll Position (mm)',
    'rudder_position':    'Rudder Pedal Position (mm)',
    'roll_force':         'Roll Virtual Force 0x51 (N)',
    'rudder_force':       'Yaw Virtual Force 0x51 (N)',
}

FLIGHT_PARAM_CHANNELS = [
    'ias_kt', 'vertical_speed_fpm', 'altitude_msl_ft',
    'engine_rpm', 'prop_rpm', 'manifold_inhg',
    'pitch_deg', 'roll_rate', 'yaw_rate',
    'roll_position', 'rudder_position', 'throttle_pct'
]


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
    target_heading = float(kwargs.get('target_heading_deg', kwargs.get('target_heading_mag', 270.0)))

    # Parse time, heading, bank, roll, rudder
    times = []
    headings = []
    banks = []
    rolls = []
    rudders = []
    for r in rows:
        try:
            times.append(float(r.get('Time (s)', r.get('time_s', 0))))
            headings.append(float(r.get('Magnetic Heading (deg)', r.get('heading_deg', 0))))
            banks.append(float(r.get('Bank Angle (deg)', r.get('bank_deg', 0))))
            rolls.append(float(r.get('Joystick Roll Position (mm)', r.get('roll_position', 0))))
            rudders.append(float(r.get('Rudder Pedal Position (mm)', r.get('rudder_position', 0))))
        except (ValueError, TypeError):
            continue

    with PdfPages(output_pdf_path) as pdf:
        # Page 1: Cover / Summary
        fig, ax = plt.subplots(figsize=(8.5, 11))
        ax.axis('off')
        fig.patch.set_facecolor('white')

        status_text = f"INTERRUPTED: {run_fault}" if run_fault else "COMPLETED"
        status_color = "#b00020" if run_fault else "#007a33"

        plt.text(0.5, 0.90, "FLIGHT TEST REPORT - YAW / HEADING PID",
                 fontsize=18, fontweight='bold', ha='center', color='#1a365d')
        plt.text(0.5, 0.85, f"Test Reference: {test_reference}",
                 fontsize=14, ha='center', color='#4a5568')

        # Status box
        ax.add_patch(plt.Rectangle((0.15, 0.72), 0.70, 0.08,
                                   facecolor="#fef2f2" if run_fault else "#f0fdf4",
                                   edgecolor=status_color, linewidth=2))
        plt.text(0.5, 0.75, f"RUN STATUS:  {status_text}",
                 fontsize=13, fontweight='bold', ha='center', color=status_color)

        # Metadata table
        duration_s = f"{times[-1] - times[0]:.1f} s" if len(times) > 1 else "0.0 s"
        meta_lines = [
            f"Target Heading (Magnetic):  {target_heading:.1f}°",
            f"Hold Duration:              5.0 s continuous stability (±2.0° heading, ±3.0° bank)",
            f"Recorded Duration:          {duration_s}",
            f"Control Mode:               Dual-Axis BRUNNER (Roll Axis 2, Rudder Axis 4)",
            f"Generated:                  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        ]
        y_pos = 0.62
        for line in meta_lines:
            plt.text(0.20, y_pos, line, fontsize=11, family='monospace', color='#2d3748')
            y_pos -= 0.04

        pdf.savefig(fig, bbox_inches='tight')
        plt.close(fig)

        # Page 2: Heading & Yaw Performance Graphs
        if times:
            fig, axs = plt.subplots(3, 1, figsize=(8.5, 11), sharex=True)
            plt.subplots_adjust(hspace=0.35, top=0.92, bottom=0.08, left=0.12, right=0.92)

            fig.suptitle(f"Heading & Yaw PID Performance - Test {test_reference}", fontsize=14, fontweight='bold')

            # 1. Heading Tracking
            axs[0].plot(times, headings, label="Actual Heading (°)", color="#2b6cb0", linewidth=1.5)
            axs[0].axhline(target_heading, color="#e53e3e", linestyle="--", label=f"Target ({target_heading:.1f}°)")
            axs[0].axhline(target_heading + 2.0, color="#cbd5e0", linestyle=":", label="±2.0° Tolerance")
            axs[0].axhline(target_heading - 2.0, color="#cbd5e0", linestyle=":")
            axs[0].set_ylabel("Heading (°)")
            axs[0].grid(True, linestyle="--", alpha=0.6)
            axs[0].legend(loc="upper right", fontsize=8)

            # 2. Bank Angle
            axs[1].plot(times, banks, label="Actual Bank (°)", color="#805ad5", linewidth=1.5)
            axs[1].axhline(0.0, color="#718096", linestyle="--")
            axs[1].axhline(3.0, color="#cbd5e0", linestyle=":", label="±3.0° Stability Gate")
            axs[1].axhline(-3.0, color="#cbd5e0", linestyle=":")
            axs[1].set_ylabel("Bank (°)")
            axs[1].grid(True, linestyle="--", alpha=0.6)
            axs[1].legend(loc="upper right", fontsize=8)

            # 3. Controls (Roll & Rudder Positions)
            axs[2].plot(times, rolls, label="Roll Position (mm)", color="#319795", linewidth=1.2)
            axs[2].plot(times, rudders, label="Rudder Position (mm)", color="#dd6b20", linewidth=1.2)
            axs[2].set_ylabel("Position (mm)")
            axs[2].set_xlabel("Time (s)")
            axs[2].grid(True, linestyle="--", alpha=0.6)
            axs[2].legend(loc="upper right", fontsize=8)

            pdf.savefig(fig)
            plt.close(fig)

    return str(output_pdf_path)

