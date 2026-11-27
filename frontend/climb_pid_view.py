"""Three-axis Normal Climb monitor styled after PID Controller all 3 (V4.x)."""
from collections import deque
import math
import tkinter as tk
from tkinter import messagebox, ttk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from backend.multi_pid_runner import GAIN_KEYS, validate_climb_pid_settings


def _num(value):
    try:
        out = float(value)
        return out if math.isfinite(out) else None
    except (TypeError, ValueError):
        return None


def _fmt(value, pattern, suffix=''):
    val = _num(value)
    return 'N/A' if val is None else f'{val:{pattern}}{suffix}'


def _wrap_heading_error(actual, target):
    if actual is None or target is None:
        return None
    return ((actual - target + 180.0) % 360.0) - 180.0


class ClimbPidView:
    BG_DARK = '#0d1117'
    PANEL_BG = '#161b22'
    CARD_BG = '#21262d'
    BORDER = '#30363d'
    TEXT = '#e6edf3'
    MUTED = '#8b949e'
    BLUE = '#58a6ff'
    CYAN = '#38bdf8'
    GREEN = '#39d353'
    YELLOW = '#e3b341'
    RED = '#f85149'
    ORANGE = '#ffa657'
    PURPLE = '#a371f7'

    CHARTS = (
        (
            ('Pitch Attitude Tracking & Error (deg)', 'Pitch (°)', GREEN, (
                ('pitch', 'Pitch Actual', GREEN, '-'),
                ('target_pitch', 'Pitch Target / Cmd', YELLOW, '--'),
                ('pitch_err', 'Pitch Error', RED, '-'),
            )),
            ('Airspeed Tracking & Error (knots)', 'IAS (kt)', CYAN, (
                ('ias', 'Actual IAS', CYAN, '-'),
                ('target_ias', 'Target IAS', PURPLE, '--'),
                ('ias_err', 'Airspeed Error', ORANGE, '-'),
            )),
            ('Brunner CLS-P Pitch Stick Motor Drive', 'Stick [-1..1]', ORANGE, (
                ('pitch_pos', 'Pitch Meas', ORANGE, '-'),
                ('pitch_cmd', 'Pitch Cmd', CYAN, '--'),
            )),
        ),
        (
            ('Bank Angle Tracking & Error (deg)', 'Bank (°)', CYAN, (
                ('bank', 'Actual Bank', CYAN, '-'),
                ('target_bank', 'Target Bank', YELLOW, '--'),
                ('bank_err', 'Bank Error', RED, '-'),
            )),
            ('Brunner CLS-P Roll Motor Drive Channel', 'Roll [-1..1]', ORANGE, (
                ('roll_pos', 'Roll Meas', ORANGE, '-'),
                ('roll_cmd', 'Roll Cmd', CYAN, '--'),
            )),
        ),
        (
            ('Heading Angle Tracking & Error (deg)', 'Heading (°)', CYAN, (
                ('heading', 'Actual Heading', CYAN, '-'),
                ('target_heading', 'Target Heading', YELLOW, '--'),
                ('heading_err', 'Heading Error', RED, '-'),
            )),
            ('Brunner CLS-P Hardware Drive (Rudder & Roll)', 'Motor [-1..1]', ORANGE, (
                ('rudder_pos', 'Rudder Meas', ORANGE, '-'),
                ('rudder_cmd', 'Rudder Cmd', CYAN, '--'),
                ('roll_cmd_tab3', 'Roll Cmd', PURPLE, ':'),
            )),
        ),
    )

    def __init__(self, parent, checks, targets, send):
        self.window = tk.Toplevel(parent)
        self.window.title('Prepar3D Flight Controller — Brunner CLS-P Pitch, Roll & Rudder Control')
        self.window.geometry('1540x880')
        self.window.minsize(1120, 680)
        self.window.configure(bg=self.BG_DARK)
        self.send, self.targets = send, targets
        self.last_settings = None
        self.samples, self.session = deque(maxlen=300), None
        self.gains, self.gain_inputs, self.inputs = {}, [], []
        self.preset_buttons = []
        self.lines, self.axes, self.canvases = {}, [], []
        self._last_log_key = None
        self._diag_event_count = 0

        style = ttk.Style(self.window)
        style.configure('Climb.TNotebook', background=self.BG_DARK, borderwidth=0)
        style.configure('Climb.TNotebook.Tab', background='#21262d', foreground='#c9d1d9',
                        font=('Segoe UI', 10, 'bold'), padding=(20, 7))
        style.map('Climb.TNotebook.Tab',
                  background=[('selected', '#1f6feb'), ('active', '#30363d')],
                  foreground=[('selected', '#ffffff'), ('active', '#ffffff')])
        style.configure('Climb.TCheckbutton', background=self.PANEL_BG, foreground=self.TEXT,
                        font=('Segoe UI', 9, 'bold'))
        style.configure('Climb.TEntry', fieldbackground=self.BG_DARK, foreground=self.CYAN)

        self._build_header()

        notebook = ttk.Notebook(self.window, style='Climb.TNotebook')
        notebook.pack(fill='both', expand=True, padx=10, pady=(2, 8))
        dashboard = tk.Frame(notebook, bg=self.BG_DARK)
        diagnostics = tk.Frame(notebook, bg=self.BG_DARK)
        notebook.add(dashboard, text='  Flight Controls  ')
        notebook.add(diagnostics, text='  BRUNNER Diagnostics  ')

        self._build_flight_controls(dashboard, checks, targets)
        self._build_diagnostics_tab(diagnostics)

    def _build_header(self):
        header = tk.Frame(self.window, bg=self.PANEL_BG, padx=12, pady=8)
        header.pack(fill='x', padx=10, pady=(10, 4))
        for col in range(6):
            header.columnconfigure(col, weight=1, uniform='status')

        tk.Label(
            header,
            text='P3D BRUNNER CLS-P FLIGHT CONTROLLER',
            bg=self.PANEL_BG,
            fg=self.BLUE,
            font=('Segoe UI', 11, 'bold'),
        ).grid(row=0, column=0, columnspan=3, sticky='w', pady=(0, 6))

        self.status = tk.Label(
            header,
            text='Select the PIDs and targets in test settings.',
            bg=self.PANEL_BG,
            fg='#c9d1d9',
            font=('Segoe UI', 9),
            anchor='e',
        )
        self.status.grid(row=0, column=3, sticky='e', padx=6, pady=(0, 6))

        self.apply_button = ttk.Button(
            header,
            text='Apply live tuning',
            command=self.apply,
            state='disabled',
        )
        self.apply_button.grid(row=0, column=4, sticky='e', padx=4, pady=(0, 6))

        self.disarm_btn = tk.Button(
            header,
            text='DISARM / STOP',
            font=('Segoe UI', 9, 'bold'),
            bg='#da3633',
            fg='#ffffff',
            activebackground='#b62324',
            activeforeground='#ffffff',
            relief=tk.FLAT,
            padx=12,
            pady=3,
            cursor='hand2',
            command=lambda: self.send('stop'),
        )
        self.disarm_btn.grid(row=0, column=5, sticky='e', padx=4, pady=(0, 6))

        self.conn_badge = self._make_badge(header, 'SimConnect: Disconnected', 1, 0)
        self.brunner_badge = self._make_badge(header, 'Brunner CLS-P: Disconnected', 1, 1)
        self.mode_badge = self._make_badge(header, 'CONTROLLERS: IDLE', 1, 2)
        self.rate_badge = self._make_badge(header, 'PID Rate: 25.0 Hz', 1, 3, bg='#1f6feb')
        self.test_badge = self._make_badge(header, 'Test: --', 1, 4)
        self.elapsed_badge = self._make_badge(header, 'Elapsed: 0.0 s', 1, 5)

    def _make_badge(self, parent, text, row, col, *, bg='#484f58', fg='#ffffff'):
        lbl = tk.Label(
            parent,
            text=text,
            font=('Segoe UI', 8, 'bold'),
            bg=bg,
            fg=fg,
            padx=6,
            pady=3,
        )
        lbl.grid(row=row, column=col, sticky='ew', padx=2)
        return lbl

    def _build_flight_controls(self, dashboard, checks, targets):
        dashboard.rowconfigure(0, weight=1)
        specs = (
            ('airspeed', 'IAS / PITCH', 'kt', (('airspeed', 'IAS PID', self.CYAN), ('pitch', 'Pitch PID', self.GREEN)), (
                ('Target Airspeed (kt)', 'target_ias', (('65', 65), ('80', 80), ('90', 90), ('100', 100))),
            )),
            ('bank', 'BANK', 'deg', (('bank', 'Bank PID', self.CYAN),), (
                ('Target Bank (deg)', 'target_bank', (('0.0° Level', 0.0),)),
            )),
            ('yaw', 'HEADING / YAW', 'deg', (('heading', 'Heading PID', self.CYAN), ('yaw', 'Yaw PID', self.PURPLE)), (
                ('Target Heading (deg)', 'target_heading', (('N', 360.0), ('E', 90.0), ('S', 180.0), ('W', 270.0))),
            )),
        )
        self.readouts = {}
        control_cards = []
        for col, (key, banner, unit, groups, target_cfg) in enumerate(specs):
            dashboard.columnconfigure(col, weight=1, uniform='controllers')
            column_frame = tk.Frame(dashboard, bg=self.BG_DARK)
            column_frame.grid(row=0, column=col, sticky='nsew', padx=3)

            tk.Label(
                column_frame,
                text=banner,
                bg='#1f6feb',
                fg='#ffffff',
                font=('Segoe UI', 10, 'bold'),
                pady=5,
            ).pack(side='top', fill='x')

            ctrl_box = tk.Frame(column_frame, bg=self.PANEL_BG, padx=6, pady=5)
            ctrl_box.pack(side='top', fill='x', pady=(2, 3))
            control_cards.append(ctrl_box)

            target_frame = tk.LabelFrame(
                ctrl_box,
                text=f'  {target_cfg[0][0]}  ',
                font=('Segoe UI', 8, 'bold'),
                bg=self.PANEL_BG,
                fg=self.CYAN,
                padx=6,
                pady=3,
            )
            target_frame.pack(fill='x', pady=(0, 3))
            check = ttk.Checkbutton(
                target_frame,
                text='Enable for Automatic',
                variable=checks[key],
                style='Climb.TCheckbutton',
            )
            check.pack(side='left', padx=(0, 6))
            self.inputs.append(check)

            ttk.Label(target_frame, text=f'Target ({unit}):', background=self.PANEL_BG, foreground=self.TEXT,
                      font=('Segoe UI', 8, 'bold')).pack(side='left', padx=(2, 3))
            entry = tk.Entry(
                target_frame,
                textvariable=targets[col],
                width=6,
                font=('Segoe UI', 9, 'bold'),
                bg=self.BG_DARK,
                fg=self.CYAN,
                insertbackground='#ffffff',
                justify='center',
                relief=tk.FLAT,
            )
            entry.pack(side='left', padx=3)
            for label_txt, preset_val in target_cfg[0][2]:
                btn = tk.Button(
                    target_frame,
                    text=label_txt,
                    font=('Segoe UI', 7, 'bold'),
                    bg='#30363d',
                    fg=self.TEXT,
                    activebackground='#484f58',
                    activeforeground='#ffffff',
                    relief=tk.FLAT,
                    padx=5,
                    pady=1,
                    command=lambda var=targets[col], val=preset_val: var.set(f'{val:g}'),
                )
                btn.pack(side='left', padx=1)
                self.preset_buttons.append(btn)

            gains_row = tk.Frame(ctrl_box, bg=self.PANEL_BG)
            gains_row.pack(fill='x', pady=(0, 3))
            for idx, (group, group_title, group_color) in enumerate(groups):
                gains_row.columnconfigure(idx, weight=1)
                g_box = tk.LabelFrame(
                    gains_row,
                    text=f'  {group_title} Gains  ',
                    font=('Segoe UI', 8, 'bold'),
                    bg=self.PANEL_BG,
                    fg=group_color,
                    padx=5,
                    pady=3,
                )
                g_box.grid(row=0, column=idx, sticky='nsew', padx=(0 if idx == 0 else 3, 0))
                self.gains[group] = {}
                for g_col, gain in enumerate(GAIN_KEYS):
                    variable = self.gains[group][gain] = tk.StringVar(self.window)
                    tk.Label(g_box, text=f'{gain.capitalize()}:', font=('Segoe UI', 8, 'bold'),
                             bg=self.PANEL_BG, fg=group_color).grid(row=0, column=g_col * 2, padx=2, sticky='w')
                    field = ttk.Entry(g_box, width=6, textvariable=variable, state='disabled',
                                      justify='center', style='Climb.TEntry')
                    field.grid(row=0, column=g_col * 2 + 1, padx=2)
                    self.gain_inputs.append(field)

            data_box = tk.LabelFrame(
                ctrl_box,
                text='  Live Telemetry & Actuator Readouts  ',
                font=('Segoe UI', 8, 'bold'),
                bg=self.PANEL_BG,
                fg='#79c0ff',
                padx=6,
                pady=3,
            )
            data_box.pack(fill='x')
            self._populate_readouts(data_box, key)

            graph_frame = tk.Frame(column_frame, bg=self.PANEL_BG)
            graph_frame.pack(side='top', fill='both', expand=True)
            figure = Figure(figsize=(4.6, 5.4), facecolor=self.PANEL_BG)
            axes = figure.subplots(len(self.CHARTS[col]), 1, sharex=True)
            if len(self.CHARTS[col]) == 1:
                axes = (axes,)
            for ax_idx, (axis, (chart_title, y_label, accent, series_specs)) in enumerate(zip(axes, self.CHARTS[col])):
                axis.set_facecolor(self.BG_DARK)
                axis.set_title(chart_title, fontsize=8, fontweight='bold', color=accent, pad=3)
                axis.set_ylabel(y_label, fontsize=7, color=accent)
                if ax_idx == len(self.CHARTS[col]) - 1:
                    axis.set_xlabel('Time (s)', fontsize=7, color='#c9d1d9')
                axis.tick_params(colors=accent, labelsize=7)
                axis.grid(True, linestyle='--', alpha=0.35, color=self.BORDER)
                for sig_key, leg_label, color, ls in series_specs:
                    self.lines[sig_key] = axis.plot([], [], linestyle=ls, color=color, linewidth=1.5, label=leg_label)[0]
                axis.legend(loc='upper left', facecolor=self.PANEL_BG, edgecolor=self.BORDER, fontsize=6.5,
                            labelcolor='#e6edf3')
                self.axes.append(axis)
            figure.subplots_adjust(left=0.14, right=0.97, bottom=0.08, top=0.93, hspace=0.42)
            canvas = FigureCanvasTkAgg(figure, master=graph_frame)
            canvas.draw()
            canvas.get_tk_widget().pack(fill='both', expand=True)
            self.canvases.append(canvas)

        self.window.update_idletasks()
        max_h = max(c.winfo_reqheight() for c in control_cards)
        for card in control_cards:
            card.configure(height=max_h)
            card.pack_propagate(False)

    def _populate_readouts(self, box, key):
        for c in range(3):
            box.columnconfigure(c, weight=1)
        if key == 'airspeed':
            items = (
                ('pitch_act', 'Pitch: --', self.GREEN, 0, 0),
                ('pitch_err', 'Pitch Err: --', self.RED, 0, 1),
                ('throttle', 'Throttle: --', self.YELLOW, 0, 2),
                ('ias_act', 'IAS: --', self.CYAN, 1, 0),
                ('ias_err', 'IAS Err: --', self.ORANGE, 1, 1),
                ('cmd_pitch', 'Cmd Pitch: --', '#d2a8ff', 1, 2),
                ('brunner_pitch', 'Brunner Pitch (Meas/Cmd): --', self.BLUE, 2, 0),
            )
            for name, text, color, r, c in items:
                lbl = tk.Label(box, text=text, font=('Segoe UI', 8, 'bold'), fg=color, bg=self.PANEL_BG, anchor='w')
                lbl.grid(row=r, column=c, columnspan=(3 if name == 'brunner_pitch' else 1), padx=4, sticky='w')
                self.readouts[name] = lbl
        elif key == 'bank':
            items = (
                ('bank_act', 'Bank: --', self.CYAN, 0, 0),
                ('bank_err', 'Bank Err: --', self.RED, 0, 1),
                ('bank_tgt', 'Target: 0.00°', self.YELLOW, 0, 2),
                ('brunner_roll', 'Brunner Roll (Meas/Cmd): --', self.BLUE, 1, 0),
            )
            for name, text, color, r, c in items:
                lbl = tk.Label(box, text=text, font=('Segoe UI', 8, 'bold'), fg=color, bg=self.PANEL_BG, anchor='w')
                lbl.grid(row=r, column=c, columnspan=(3 if name == 'brunner_roll' else 1), padx=4, sticky='w')
                self.readouts[name] = lbl
            tk.Label(box, text='Hardware Drive: Pure Brunner CLS-P (P3D Writes: NONE)',
                     font=('Segoe UI', 8, 'italic'), fg='#7ee787', bg=self.PANEL_BG, anchor='w').grid(
                row=2, column=0, columnspan=3, padx=4, sticky='w')
        else:
            items = (
                ('hdg_act', 'Heading: --', self.CYAN, 0, 0),
                ('hdg_err', 'Heading Err: --', self.RED, 0, 1),
                ('hdg_tgt', 'Target: --', self.YELLOW, 0, 2),
                ('brunner_rudder', 'Brunner Rudder (Meas/Cmd): --', self.PURPLE, 1, 0),
            )
            for name, text, color, r, c in items:
                lbl = tk.Label(box, text=text, font=('Segoe UI', 8, 'bold'), fg=color, bg=self.PANEL_BG, anchor='w')
                lbl.grid(row=r, column=c, columnspan=(3 if name == 'brunner_rudder' else 1), padx=4, sticky='w')
                self.readouts[name] = lbl
            tk.Label(box, text='Hardware Drive: Pure Brunner CLS-P (P3D Writes: NONE)',
                     font=('Segoe UI', 8, 'italic'), fg='#7ee787', bg=self.PANEL_BG, anchor='w').grid(
                row=2, column=0, columnspan=3, padx=4, sticky='w')

    def _build_diagnostics_tab(self, parent):
        main_frame = tk.Frame(parent, bg=self.BG_DARK, padx=10, pady=8)
        main_frame.pack(fill='both', expand=True)

        top = tk.Frame(main_frame, bg=self.PANEL_BG, padx=12, pady=8)
        top.pack(fill='x', pady=(0, 6))
        tk.Label(top, text='BRUNNER CLS-P HARDWARE & PROTOCOL DIAGNOSTICS',
                 font=('Segoe UI', 11, 'bold'), bg=self.PANEL_BG, fg=self.BLUE).pack(side='left')
        tk.Label(top, text=' (READ-ONLY MONITOR) ', font=('Segoe UI', 9, 'bold'),
                 bg=self.PANEL_BG, fg='#7ee787').pack(side='left', padx=6)

        banner = tk.LabelFrame(
            main_frame,
            text='  CLS2Sim Remote Interface & Hardware Override Ownership  ',
            font=('Segoe UI', 9, 'bold'),
            bg=self.PANEL_BG,
            fg='#79c0ff',
            padx=12,
            pady=6,
        )
        banner.pack(fill='x', pady=(0, 6))
        caps = tk.Frame(banner, bg=self.PANEL_BG)
        caps.pack(fill='x', pady=(0, 4))
        self.lbl_diag_ovr_pitch = self._diag_pill(caps, 'Pitch Override: MANUAL')
        self.lbl_diag_ovr_roll = self._diag_pill(caps, 'Roll Override: MANUAL')
        self.lbl_diag_ovr_yaw = self._diag_pill(caps, 'Rudder Override: MANUAL')
        self.lbl_diag_overall = tk.Label(caps, text='LINK: IDLE', font=('Segoe UI', 9, 'bold'),
                                         bg='#30363d', fg='#ffffff', padx=10, pady=3)
        self.lbl_diag_overall.pack(side='right', padx=4)

        advisory = tk.Frame(banner, bg=self.BG_DARK, padx=8, pady=4)
        advisory.pack(fill='x')
        tk.Label(
            advisory,
            text='ADVISORY: UDP telemetry confirms CLS2Sim software state. Physical actuation also requires CAN bus active, drive power ON, and safety interlocks closed.',
            font=('Segoe UI', 8, 'italic'),
            bg=self.BG_DARK,
            fg=self.YELLOW,
            anchor='w',
        ).pack(anchor='w')

        cards_frame = tk.Frame(main_frame, bg=self.BG_DARK)
        cards_frame.pack(fill='x', pady=(0, 6))
        self.diag_cards = {}
        axes_cfg = (
            ('pitch', 'Pitch Axis (Elevator | CAN Node 2)', self.CYAN, 0),
            ('roll', 'Roll Axis (Aileron | CAN Node 1)', self.YELLOW, 1),
            ('rudder', 'Rudder Axis (Yaw | CAN Node 5)', self.PURPLE, 2),
        )
        for axis_key, title, color, col in axes_cfg:
            cards_frame.columnconfigure(col, weight=1, uniform='diag_col')
            box = tk.LabelFrame(cards_frame, text=f'  {title}  ', font=('Segoe UI', 9, 'bold'),
                                bg=self.PANEL_BG, fg=color, padx=10, pady=6)
            box.grid(row=0, column=col, sticky='nsew', padx=4)

            r1 = tk.Frame(box, bg=self.PANEL_BG)
            r1.pack(fill='x', pady=(0, 4))
            lbl_link = tk.Label(r1, text='OFFLINE', font=('Segoe UI', 8, 'bold'),
                                bg='#484f58', fg='#ffffff', padx=6, pady=2)
            lbl_link.pack(side='left', padx=(0, 4))
            lbl_valid = tk.Label(r1, text='NO TELEMETRY', font=('Segoe UI', 8, 'bold'),
                                 bg=self.CARD_BG, fg=self.MUTED, padx=6, pady=2)
            lbl_valid.pack(side='left')

            r2 = tk.Frame(box, bg=self.PANEL_BG)
            r2.pack(fill='x', pady=(2, 2))
            tk.Label(r2, text='Measured Stick:', font=('Segoe UI', 9), bg=self.PANEL_BG, fg=self.TEXT).pack(side='left')
            lbl_pos = tk.Label(r2, text='+0.0000', font=('Segoe UI', 12, 'bold'), bg=self.PANEL_BG, fg=color)
            lbl_pos.pack(side='right')

            cvs_bar = tk.Canvas(box, height=22, bg=self.BG_DARK, highlightthickness=1, highlightbackground=self.BORDER)
            cvs_bar.pack(fill='x', pady=(2, 4))

            r3 = tk.Frame(box, bg=self.PANEL_BG)
            r3.pack(fill='x', pady=(2, 2))
            lbl_cmd = tk.Label(r3, text='Last Cmd: +0.0000', font=('Segoe UI', 8, 'bold'),
                               bg=self.PANEL_BG, fg=self.TEXT)
            lbl_cmd.pack(side='left')
            lbl_ack = tk.Label(r3, text='IDLE', font=('Segoe UI', 8, 'bold'),
                               bg=self.CARD_BG, fg=self.MUTED, padx=5, pady=1)
            lbl_ack.pack(side='right')

            self.diag_cards[axis_key] = dict(
                lbl_link=lbl_link, lbl_valid=lbl_valid, lbl_pos=lbl_pos,
                cvs_bar=cvs_bar, lbl_cmd=lbl_cmd, lbl_ack=lbl_ack, color=color,
            )

        log_frame = tk.LabelFrame(main_frame, text='  Live Diagnostic Event Log & Axis Summary  ',
                                  font=('Segoe UI', 9, 'bold'), bg=self.PANEL_BG, fg='#79c0ff', padx=8, pady=6)
        log_frame.pack(fill='both', expand=True)
        self.diagnostics = tk.Label(
            log_frame,
            text='BRUNNER measurements appear when the test is running.',
            bg=self.PANEL_BG,
            fg=self.TEXT,
            justify='left',
            anchor='w',
            font=('Consolas', 9),
            pady=4,
        )
        self.diagnostics.pack(fill='x')

        txt_wrap = tk.Frame(log_frame, bg=self.BG_DARK)
        txt_wrap.pack(fill='both', expand=True, pady=(4, 0))
        self.txt_diag_log = tk.Text(
            txt_wrap,
            bg=self.BG_DARK,
            fg='#c9d1d9',
            insertbackground='#ffffff',
            font=('Consolas', 9),
            wrap='none',
            height=8,
            relief=tk.FLAT,
        )
        scroll_y = tk.Scrollbar(txt_wrap, orient='vertical', command=self.txt_diag_log.yview)
        self.txt_diag_log.configure(yscrollcommand=scroll_y.set)
        scroll_y.pack(side='right', fill='y')
        self.txt_diag_log.pack(side='left', fill='both', expand=True)
        self.txt_diag_log.tag_configure('INFO', foreground='#79c0ff')
        self.txt_diag_log.tag_configure('ACK', foreground='#7ee787')
        self.txt_diag_log.tag_configure('ERROR', foreground=self.RED)

    def _diag_pill(self, parent, text):
        lbl = tk.Label(parent, text=text, font=('Segoe UI', 9, 'bold'),
                       bg=self.CARD_BG, fg=self.MUTED, padx=8, pady=3)
        lbl.pack(side='left', padx=(0, 6))
        return lbl

    def _draw_axis_bar(self, card, pos, cmd):
        cvs = card['cvs_bar']
        cvs.delete('all')
        cw = max(cvs.winfo_width(), 240)
        ch = 22
        cx = cw / 2.0
        scale = (cw - 30.0) / 2.0
        cvs.create_line(15, ch / 2, cw - 15, ch / 2, fill=self.BORDER, width=3)
        cvs.create_line(cx, 3, cx, ch - 3, fill=self.MUTED, width=1)
        if cmd is not None:
            x_cmd = max(15.0, min(cw - 15.0, cx + cmd * scale))
            cvs.create_polygon(x_cmd, 2, x_cmd - 4, 7, x_cmd + 4, 7, fill=self.ORANGE, outline='')
        if pos is not None:
            x_meas = max(15.0, min(cw - 15.0, cx + pos * scale))
            if abs(x_meas - cx) > 1:
                cvs.create_rectangle(min(cx, x_meas), ch / 2 - 3, max(cx, x_meas), ch / 2 + 3,
                                     fill=card['color'], outline='')
            cvs.create_line(x_meas, 3, x_meas, ch - 3, fill='#ffffff', width=2)

    def _append_log(self, tag, message):
        key = (tag, message)
        if key == self._last_log_key:
            return
        self._last_log_key = key
        self._diag_event_count += 1
        self.txt_diag_log.insert('end', f'[{self._diag_event_count:04d}] {message}\n', tag)
        self.txt_diag_log.see('end')

    def apply(self):
        settings = dict(zip(('target_ias', 'target_bank', 'target_heading'),
                            (target.get() for target in self.targets)))
        settings.update({group: {key: variable.get() for key, variable in gains.items()}
                         for group, gains in self.gains.items()})
        try:
            settings = validate_climb_pid_settings(settings)
        except (TypeError, ValueError) as exc:
            messagebox.showerror('Climb PID tuning', str(exc), parent=self.window)
            return
        self.send('climb_pid_tune', settings=settings)

    def update(self, state):
        current = state.get('test_reference') == getattr(self, 'reference', '1.c.(1)')
        active = current and (state.get('controlling') or state.get('recording'))
        for widget in self.inputs:
            widget.state(['disabled'] if active else ['!disabled'])
        for btn in self.preset_buttons:
            btn.configure(state='disabled' if active else 'normal')

        settings = state.get('climb_pid_settings') if current and state.get('control_method') == 'multi_pid' else None
        tunable = bool(active and settings)
        for widget in (*self.gain_inputs, self.apply_button):
            widget.state(['!disabled'] if tunable else ['disabled'])
        if tunable and settings != self.last_settings:
            for variable, key in zip(self.targets, ('target_ias', 'target_bank', 'target_heading')):
                variable.set(f'{settings[key]:g}')
            for group, gains in self.gains.items():
                for key, variable in gains.items():
                    variable.set(f'{settings[group][key]:g}')
        self.last_settings = settings if tunable else None

        # Top header status pills
        sim_ok = bool(state.get('connected'))
        brunner_ok = bool(state.get('brunner_connected'))
        self.conn_badge.configure(
            text=f"SimConnect: {'Connected' if sim_ok else 'Disconnected'}",
            bg='#238636' if sim_ok else '#484f58',
        )
        self.brunner_badge.configure(
            text=f"Brunner CLS-P: {'Connected' if brunner_ok else 'Disconnected'}",
            bg='#238636' if brunner_ok else '#484f58',
        )
        stage_txt = (state.get('stage') or 'IDLE').upper()
        self.mode_badge.configure(
            text='CONTROLLERS: RECORDING' if state.get('recording') else f'CONTROLLERS: {stage_txt}' if active else 'CONTROLLERS: IDLE',
            bg='#238636' if state.get('recording') else '#1f6feb' if active else '#484f58',
        )
        self.test_badge.configure(text=f"Test: {state.get('test_reference') or '--'}")
        self.elapsed_badge.configure(text=f"Elapsed: {_num(state.get('elapsed')) or 0.0:.1f} s")

        status_text = (state.get('details') or state.get('stage') or '') if current else 'Select the PIDs and targets in test settings.'
        self.status.configure(text=status_text)

        runtime = state.get('pid_runtime') or {}
        cmd_axes = set(runtime.get('command_axes') or ())
        for pill, axis_name, title in (
            (self.lbl_diag_ovr_pitch, 'pitch', 'Pitch Override'),
            (self.lbl_diag_ovr_roll, 'roll', 'Roll Override'),
            (self.lbl_diag_ovr_yaw, 'rudder', 'Rudder Override'),
        ):
            armed = active and axis_name in cmd_axes
            pill.configure(
                text=f'{title}: {"ARMED" if armed else "MANUAL"}',
                bg='#238636' if armed else self.CARD_BG,
                fg='#ffffff' if armed else self.MUTED,
            )
        self.lbl_diag_overall.configure(
            text='CLS2SIM LINK: ACTIVE' if brunner_ok and active else 'CLS2SIM LINK: READY' if brunner_ok else 'CLS2SIM LINK: OFFLINE',
            bg='#238636' if brunner_ok and active else '#1f6feb' if brunner_ok else '#484f58',
        )

        if not current:
            return
        session = state.get('status_file')
        reset = session != self.session
        if reset:
            self.session = session
            self.samples.clear()
            self._append_log('INFO', f"Session attached: {state.get('test_reference')} ({state.get('control_method')})")

        sample = state.get('climb_pid_sample')
        if sample is None and state.get('control_method') == 'pid' and state.get('pid_live'):
            live = state['pid_live']
            sample = dict(live, heading=live.get('heading_deg'), bank=live.get('bank_deg'),
                          pitch_pos=live.get('actual_position'), pitch_cmd=live.get('command'))
        if sample:
            enriched = dict(sample)
            p_act, p_tgt = _num(enriched.get('pitch')), _num(enriched.get('target_pitch'))
            ias_act, ias_tgt = _num(enriched.get('ias')), _num(enriched.get('target_ias'))
            b_act, b_tgt = _num(enriched.get('bank')), _num(enriched.get('target_bank'))
            h_act, h_tgt = _num(enriched.get('heading')), _num(enriched.get('target_heading'))
            enriched['pitch_err'] = None if (p_act is None or p_tgt is None) else (p_tgt - p_act)
            enriched['ias_err'] = None if (ias_act is None or ias_tgt is None) else (ias_act - ias_tgt)
            enriched['bank_err'] = None if (b_act is None or b_tgt is None) else (b_tgt - b_act)
            enriched['heading_err'] = _wrap_heading_error(h_act, h_tgt)
            enriched['roll_cmd_tab3'] = enriched.get('roll_cmd')
            sample = enriched

        if sample and (not self.samples or sample['time_s'] > self.samples[-1]['time_s']):
            self.samples.append(sample)
        elif not reset:
            return

        telem = state.get('telemetry') or {}
        if sample:
            self.readouts['pitch_act'].configure(text=f"Pitch: {_fmt(sample.get('pitch'), '+.2f', '°')}")
            self.readouts['pitch_err'].configure(text=f"Pitch Err: {_fmt(sample.get('pitch_err'), '+.2f', '°')}")
            self.readouts['throttle'].configure(text=f"Throttle: {_fmt(telem.get('engine_throttle_percent'), '.1f', '%')}")
            self.readouts['ias_act'].configure(text=f"IAS: {_fmt(sample.get('ias'), '.1f', ' kt')}")
            self.readouts['ias_err'].configure(text=f"IAS Err: {_fmt(sample.get('ias_err'), '+.1f', ' kt')}")
            self.readouts['cmd_pitch'].configure(text=f"Cmd Pitch: {_fmt(sample.get('target_pitch'), '+.2f', '°')}")
            self.readouts['brunner_pitch'].configure(
                text=f"Brunner Pitch (Meas/Cmd): {_fmt(sample.get('pitch_pos'), '+.3f')} / {_fmt(sample.get('pitch_cmd'), '+.3f')}"
            )
            self.readouts['bank_act'].configure(text=f"Bank: {_fmt(sample.get('bank'), '+.2f', '°')}")
            self.readouts['bank_err'].configure(text=f"Bank Err: {_fmt(sample.get('bank_err'), '+.2f', '°')}")
            self.readouts['bank_tgt'].configure(text=f"Target: {_fmt(sample.get('target_bank'), '+.2f', '°')}")
            self.readouts['brunner_roll'].configure(
                text=f"Brunner Roll (Meas/Cmd): {_fmt(sample.get('roll_pos'), '+.3f')} / {_fmt(sample.get('roll_cmd'), '+.3f')}"
            )
            self.readouts['hdg_act'].configure(text=f"Heading: {_fmt(sample.get('heading'), '.1f', '°')}")
            self.readouts['hdg_err'].configure(text=f"Heading Err: {_fmt(sample.get('heading_err'), '+.1f', '°')}")
            self.readouts['hdg_tgt'].configure(text=f"Target: {_fmt(sample.get('target_heading'), '.1f', '°')}")
            self.readouts['brunner_rudder'].configure(
                text=f"Brunner Rudder (Meas/Cmd): {_fmt(sample.get('rudder_pos'), '+.3f')} / {_fmt(sample.get('rudder_cmd'), '+.3f')}"
            )

        for axis_key, pos_k, cmd_k, runtime_name in (
            ('pitch', 'pitch_pos', 'pitch_cmd', 'pitch'),
            ('roll', 'roll_pos', 'roll_cmd', 'roll'),
            ('rudder', 'rudder_pos', 'rudder_cmd', 'rudder'),
        ):
            card = self.diag_cards[axis_key]
            pos_v = _num(sample.get(pos_k)) if sample else None
            cmd_v = _num(sample.get(cmd_k)) if sample else None
            online = brunner_ok and (pos_v is not None)
            card['lbl_link'].configure(
                text='ONLINE' if online else ('READY' if brunner_ok else 'OFFLINE'),
                bg='#238636' if online else ('#1f6feb' if brunner_ok else '#484f58'),
            )
            card['lbl_valid'].configure(
                text='VALID TELEMETRY' if pos_v is not None else 'NO TELEMETRY',
                fg='#3fb950' if pos_v is not None else self.MUTED,
            )
            card['lbl_pos'].configure(text=f'{pos_v:+.4f}' if pos_v is not None else '+0.0000')
            card['lbl_cmd'].configure(text=f'Last Cmd: {cmd_v:+.4f}' if cmd_v is not None else 'Last Cmd: +0.0000')
            armed = active and runtime_name in cmd_axes and cmd_v is not None
            card['lbl_ack'].configure(
                text='DRIVING' if armed else 'IDLE',
                bg='#1f6feb' if armed else self.CARD_BG,
                fg='#ffffff' if armed else self.MUTED,
            )
            self._draw_axis_bar(card, pos_v, cmd_v)

        self.diagnostics.configure(
            text='   |   '.join(
                f"{label}: {_fmt(sample.get(key) if sample else None, '+.4f')}"
                for label, key in (('Pitch motor', 'pitch_pos'), ('Roll motor', 'roll_pos'), ('Rudder motor', 'rudder_pos'))
            )
        )
        if status_text:
            tag = 'ERROR' if 'error' in status_text.lower() or 'fail' in status_text.lower() else 'ACK' if state.get('recording') else 'INFO'
            self._append_log(tag, f"[{state.get('stage') or 'STATUS'}] {status_text}")

        times = [row['time_s'] for row in self.samples]
        for key, line in self.lines.items():
            line.set_data(times, [float('nan') if row.get(key) is None else row[key]
                                  for row in self.samples])
        for axis in self.axes:
            axis.relim()
            axis.autoscale_view()
        for canvas in self.canvases:
            canvas.draw_idle()
