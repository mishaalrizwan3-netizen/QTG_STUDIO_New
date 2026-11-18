"""Separate PID console. Existing QTG test files and dispatch remain untouched.

Prepare the aircraft in P3D before using this console. Run only one application
that owns the BRUNNER axes at a time.
"""

from dataclasses import dataclass
import math
import os
from pathlib import Path
import queue
import sys
import threading
import time
from types import SimpleNamespace


@dataclass(frozen=True)
class PidOptions:
    airspeed: bool
    bank: bool
    yaw: bool
    target_ias: float
    target_heading: float
    duration_s: float
    output_dir: Path

    def validate(self):
        if not (self.airspeed or self.bank or self.yaw):
            raise ValueError('Select at least one PID axis.')
        if not all(math.isfinite(value) for value in
                   (self.target_ias, self.target_heading, self.duration_s)):
            raise ValueError('Targets and duration must be finite numbers.')
        if self.airspeed and self.target_ias <= 0:
            raise ValueError('Target airspeed must be positive.')
        if not 0 <= self.target_heading < 360:
            raise ValueError('Target heading must be from 0 to less than 360 degrees.')
        if self.duration_s <= 0:
            raise ValueError('Recording duration must be positive.')
        return self


def default_output_dir():
    root = os.environ.get('QTG_RESULTS_ROOT')
    return Path(root).expanduser() / 'PID_Addon' if root else Path('D:/QTG_Test_Results/PID_Addon')


def load_hardware():
    source = Path(__file__).resolve().parent / 'interfaces' / 'brunner_base' / 'src'
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))
    from p3d_simconnect_v15 import SimConnectClient
    from brunner_interface import BrunnerInterface

    class PidBrunnerInterface(BrunnerInterface):
        def release_controls(self):
            # The existing BRUNNER close() centers all axes. This separate PID
            # process must release only the axes it actually took ownership of.
            transport = getattr(self, '_multi_pid_transport', None)
            if transport is not None:
                transport.release()

        def close(self):
            try:
                self.release_controls()
            finally:
                with self._io_lock:
                    if self.sock is not None:
                        self.sock.close()
                        self.sock = None
                    self._connected = False

    return SimConnectClient('QTG Separate PID Console'), PidBrunnerInterface(timeout=0.05)


class Signal:
    def __init__(self, callback):
        self.callback = callback

    def emit(self, *args):
        self.callback(*args)


class PidSession:
    def __init__(self, options, notify, *, hardware_factory=load_hardware, runner=None):
        self.options = options.validate()
        self.notify = notify
        self.hardware_factory = hardware_factory
        self.runner = runner
        self.stop_event = threading.Event()
        self.outputs = []
        self.error = None

    def stop(self):
        self.stop_event.set()

    def run(self):
        sim = device = sim_thread = None
        control_started = False
        try:
            self.options.output_dir.mkdir(parents=True, exist_ok=True)
            sim, device = self.hardware_factory()
            connected = threading.Event()
            connection_error = []
            sim_thread = threading.Thread(
                target=sim.connect_and_run,
                kwargs=dict(on_connected=connected.set, on_error=connection_error.append),
                daemon=True,
            )
            sim_thread.start()
            self.notify('Connecting to P3D...')
            deadline = time.monotonic() + 35
            while not connected.wait(0.1):
                if self.stop_event.is_set():
                    self.notify('Stopped before PID engagement.')
                    return
                if connection_error or not sim_thread.is_alive() or time.monotonic() >= deadline:
                    raise RuntimeError(connection_error[0] if connection_error else 'P3D connection timed out.')
            if self.stop_event.is_set():
                self.notify('Stopped before PID engagement.')
                return
            if not device.connect():
                raise RuntimeError('CLS2Sim connection failed.')

            from backend import multi_pid_runner

            window = SimpleNamespace(
                p3d=sim, brunner=device, stop_worker_event=self.stop_event,
                _session_context={
                    'control_method': 'multi_pid',
                    'active_pids': {'airspeed': self.options.airspeed,
                                    'bank': self.options.bank, 'yaw': self.options.yaw},
                    'pid_conditions': {'target_ias': self.options.target_ias},
                    'target_heading_mag': self.options.target_heading,
                    'multi_pid_hold_sec': 5.0,
                },
                _control_method='multi_pid', target_duration_sec=self.options.duration_s,
                data_dir=self.options.output_dir, _outputs=self.outputs,
                sig_update_stage=Signal(lambda stage, detail: self.notify(f'{stage}: {detail}')),
                sig_record_progress=Signal(lambda elapsed: self.notify(f'Recording: {elapsed} s')),
                sig_recording_finished=Signal(lambda: None),
                is_recording=False, _prepared=False, _preparing=False,
            )
            for name in ('rec_time', 'p_pos_buf', 'r_pos_buf', 'y_pos_buf',
                         'p_force_buf', 'r_force_buf', 'y_force_buf'):
                setattr(window, name, [])
            base = SimpleNamespace(STAGE_INITIALIZING='Stabilizing',
                                   STAGE_RECORDING='Recording',
                                   STAGE_CONNECTION_ERROR='PID failed')
            self.notify('P3D and CLS2Sim connected. Starting selected PID axes...')
            control_started = True
            (self.runner or multi_pid_runner.loop)(window, base)
            if window.multi_pid_fault:
                raise RuntimeError(window.multi_pid_fault)
            if not self.outputs:
                self.notify('PID stopped before recording.')
            else:
                self.notify('PID run complete. Results: ' + ', '.join(self.outputs))
        except Exception as exc:
            self.error = str(exc)
            self.notify('PID failed: ' + self.error)
        finally:
            self.stop_event.set()
            if control_started and sim is not None:
                try:
                    sim.pause_sim()
                except Exception:
                    pass
            if device is not None:
                try:
                    device.close()
                except Exception as exc:
                    self.error = self.error or f'BRUNNER release failed: {exc}'
                    self.notify(self.error)
            if sim is not None:
                sim.stop()
            if sim_thread is not None:
                sim_thread.join(timeout=2)


def main():
    import tkinter as tk
    from tkinter import messagebox, ttk

    root = tk.Tk()
    root.title('QTG Separate PID Console')
    root.geometry('530x350')
    panel = ttk.Frame(root, padding=16)
    panel.pack(fill='both', expand=True)
    ttk.Label(panel, text='Prepare the aircraft in P3D first. Close the main QTG app before running this PID console.',
              wraplength=490).pack(anchor='w', pady=(0, 12))

    airspeed = tk.BooleanVar(value=True)
    bank = tk.BooleanVar(value=False)
    yaw = tk.BooleanVar(value=False)
    axes = ttk.Frame(panel)
    axes.pack(fill='x')
    for title, variable in (('Airspeed (pitch)', airspeed), ('Bank (roll)', bank), ('Yaw (heading/rudder)', yaw)):
        ttk.Checkbutton(axes, text=title, variable=variable).pack(side='left', padx=(0, 10))

    target_ias = tk.StringVar(value='80')
    target_heading = tk.StringVar(value='48.4')
    duration = tk.StringVar(value='15')
    for title, variable in (('Target IAS (kt)', target_ias), ('Target heading (deg)', target_heading),
                            ('Recording duration (s)', duration)):
        row = ttk.Frame(panel)
        row.pack(fill='x', pady=4)
        ttk.Label(row, text=title, width=25).pack(side='left')
        ttk.Entry(row, textvariable=variable, width=12).pack(side='left')

    status = tk.StringVar(value='Ready. PID control is off.')
    ttk.Label(panel, textvariable=status, wraplength=490).pack(anchor='w', fill='x', pady=(14, 6))
    messages = queue.Queue()
    active = {'session': None, 'thread': None, 'closing': False}

    def start():
        try:
            options = PidOptions(airspeed.get(), bank.get(), yaw.get(), float(target_ias.get()),
                                 float(target_heading.get()), float(duration.get()), default_output_dir()).validate()
        except ValueError as exc:
            messagebox.showerror('PID settings', str(exc), parent=root)
            return
        session = PidSession(options, messages.put)
        worker = threading.Thread(target=session.run, daemon=True)
        active.update(session=session, thread=worker)
        start_button.config(state='disabled')
        stop_button.config(state='normal')
        worker.start()

    def stop():
        if active['session'] is not None:
            active['session'].stop()
            status.set('Stopping PID and releasing controls...')

    buttons = ttk.Frame(panel)
    buttons.pack(anchor='w', pady=10)
    start_button = ttk.Button(buttons, text='Start PID', command=start)
    start_button.pack(side='left', padx=(0, 8))
    stop_button = ttk.Button(buttons, text='Stop PID', command=stop, state='disabled')
    stop_button.pack(side='left')

    def poll():
        try:
            while True:
                status.set(messages.get_nowait())
        except queue.Empty:
            pass
        worker = active['thread']
        if worker is not None and not worker.is_alive():
            active['thread'] = None
            active['session'] = None
            stop_button.config(state='disabled')
            start_button.config(state='normal')
        if active['closing'] and active['thread'] is None:
            root.destroy()
        else:
            root.after(100, poll)

    def close():
        active['closing'] = True
        stop()
        if active['thread'] is None:
            root.destroy()

    root.protocol('WM_DELETE_WINDOW', close)
    root.after(100, poll)
    root.mainloop()


if __name__ == '__main__':
    main()
