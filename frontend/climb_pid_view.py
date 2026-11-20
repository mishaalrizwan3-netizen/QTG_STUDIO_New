"""Three-axis monitor for the existing Normal Climb worker."""
from collections import deque
import tkinter as tk
from tkinter import ttk, messagebox

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from backend.multi_pid_runner import GAIN_KEYS, validate_climb_pid_settings


class ClimbPidView:
    def __init__(self, parent, checks, targets, send):
        self.window = tk.Toplevel(parent)
        self.window.title('Normal Climb · PID Monitor')
        self.window.geometry('1180x680')
        self.window.minsize(780, 500)
        self.status = ttk.Label(self.window, padding=(12, 8))
        self.status.pack(fill='x')
        self.send = send
        self.targets = targets
        self.last_settings = None
        self.gains = {}
        self.gain_inputs = []
        controls = ttk.Frame(self.window, padding=(10, 0))
        controls.pack(fill='x')
        self.inputs = []
        for column, (key, title, unit, groups) in enumerate((
            ('airspeed', 'IAS / Pitch', 'kt', ('airspeed', 'pitch')),
            ('bank', 'Bank', 'deg', ('bank',)),
            ('yaw', 'Heading / Yaw', 'deg', ('heading', 'yaw')),
        )):
            controls.columnconfigure(column, weight=1, uniform='pid')
            panel = ttk.LabelFrame(controls, text=title, padding=8)
            panel.grid(row=0, column=column, sticky='nsew', padx=4)
            target_row = ttk.Frame(panel)
            target_row.pack(fill='x')
            check = ttk.Checkbutton(target_row, text='Enable for Automatic', variable=checks[key])
            check.pack(side='left')
            entry = ttk.Entry(target_row, textvariable=targets[column], width=8)
            ttk.Label(target_row, text=f'Target ({unit})').pack(side='left', padx=(8, 4))
            entry.pack(side='left')
            self.inputs.append(check)
            for group in groups:
                row = ttk.Frame(panel)
                row.pack(fill='x', pady=(4, 0))
                ttk.Label(row, text=group.capitalize(), width=8).pack(side='left')
                self.gains[group] = {}
                for gain in GAIN_KEYS:
                    variable = self.gains[group][gain] = tk.StringVar(self.window)
                    ttk.Label(row, text=gain.upper()).pack(side='left', padx=(4, 2))
                    field = ttk.Entry(row, width=6, textvariable=variable, state='disabled')
                    field.pack(side='left')
                    self.gain_inputs.append(field)
        self.apply_button = ttk.Button(self.window, text='Apply live tuning', command=self.apply, state='disabled')
        self.apply_button.pack(anchor='e', padx=14, pady=5)

        figure = Figure(figsize=(11, 5.5), layout='constrained')
        self.axes = figure.subplots(3, 3)
        self.lines = {}
        for row, column, title, unit, actual, target in (
            (0, 0, 'Pitch', 'deg', 'pitch', 'target_pitch'),
            (1, 0, 'Airspeed', 'kt', 'ias', 'target_ias'),
            (2, 0, 'Pitch motor', 'Normalized', 'pitch_pos', 'pitch_cmd'),
            (0, 1, 'Bank', 'deg', 'bank', 'target_bank'),
            (1, 1, 'Roll motor', 'Normalized', 'roll_pos', 'roll_cmd'),
            (0, 2, 'Heading', 'deg', 'heading', 'target_heading'),
            (1, 2, 'Rudder motor', 'Normalized', 'rudder_pos', 'rudder_cmd'),
        ):
            axis = self.axes[row, column]
            axis.set(title=title, ylabel=unit, xlabel='Time (s)')
            axis.grid(alpha=.25)
            self.lines[actual] = axis.plot([], [], label='Measured')[0]
            self.lines[target] = axis.plot([], [], '--', label='Target / command')[0]
            axis.legend(fontsize='small', loc='upper right')
        for column in (1, 2):
            self.axes[2, column].set_visible(False)
        self.canvas = FigureCanvasTkAgg(figure, master=self.window)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)
        self.samples = deque(maxlen=300)
        self.session = None
        self.canvas.draw()

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
        climb = state.get('test_reference') == '1.c.(1)'
        active = climb and (state.get('controlling') or state.get('recording'))
        for widget in self.inputs:
            widget.state(['disabled'] if active else ['!disabled'])
        settings = state.get('climb_pid_settings') if climb and state.get('control_method') == 'multi_pid' else None
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
        self.status.configure(text=(state.get('details') or state.get('stage') or '') if climb
                              else 'Select the PIDs and targets here, then start Automatic in Normal Climb.')
        if not climb:
            return
        session = state.get('status_file')
        reset = session != self.session
        if reset:
            self.session = session
            self.samples.clear()
        sample = state.get('climb_pid_sample')
        if sample is None and state.get('control_method') == 'pid' and state.get('pid_live'):
            live = state['pid_live']
            sample = dict(live, heading=live.get('heading_deg'), bank=live.get('bank_deg'),
                          pitch_pos=live.get('actual_position'), pitch_cmd=live.get('command'))
        if sample and (not self.samples or sample['time_s'] > self.samples[-1]['time_s']):
            self.samples.append(sample)
        elif not reset:
            return
        times = [row['time_s'] for row in self.samples]
        for key, line in self.lines.items():
            line.set_data(times, [float('nan') if row.get(key) is None else row[key] for row in self.samples])
        for axis in self.axes.flat:
            if axis.get_visible():
                axis.relim()
                axis.autoscale_view()
        self.canvas.draw_idle()
