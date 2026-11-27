# QTG Studio

QTG Studio is a Windows application for preparing, recording, and reporting objective qualification tests for the Aeromix Multi-Crew **FNPT II** simulator. It uses Prepar3D for flight-model telemetry and CLS2Sim for BRUNNER control loading where the selected procedure requires it.

The application is under [QTG_Studio/](QTG_Studio/). The catalog includes 27 test entries. Available execution modes depend on the test and its implemented procedure.

## Run from source

Use 64-bit Windows, Python 3.10 or later with Tkinter, and a compatible Prepar3D installation. This setup uses a virtual environment without requiring PowerShell activation.

```powershell
Set-Location QTG_Studio
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt PySide6 numpy pandas matplotlib Pillow psutil
.\.venv\Scripts\python.exe main.py --check
.\.venv\Scripts\python.exe main.py
```

The startup check lists the configured tests and reports whether the expected SDK and bridge files are present. For live operation, start Prepar3D with the intended aircraft. For tests that use BRUNNER, start CLS2Sim, select the cockpit profile, and verify the relevant loaders are ready before starting the test.

[RUN_QTG.bat](QTG_Studio/RUN_QTG.bat) is also available. It can open an existing FNPT II package when one is present. Use the Python command above to run the current source.

## Test workflow

1. Select a test and review **Test Settings**. Unlock settings with the local administrator password when changes are needed. The password is configured on first use and stored as a salted hash in the current Windows user's Local AppData.
2. Set the required initial conditions and choose which variables to display under **Graphs in PDF**. The graph list is shared across tests. Each test keeps its own selections. A graph needs data from the recording or the selected CSV.
3. Select **Manual** or **Automatic**, where available. Supply a Manual reference CSV when the chosen Automatic procedure uses Recorded Replay.
4. Follow the test status through initialization. When **Start Recording** is available, use it to begin that workflow. Some automatic procedures stabilize before the official recording starts.
5. Open the saved CSV and PDF from the test results. Use **Stop** to end a run that is still active.

| Workflow | What the app does |
| --- | --- |
| Normal Climb | Supports Manual, Recorded Replay, and PID operation. With multiple PIDs selected, **Show all three PIDs** opens the live IAS/Pitch, Bank, and Heading/Yaw monitor. **Apply live tuning** sends target and gain changes to the running climb worker. |
| Engine Acceleration | Automatic mode records a baseline and commands the Prepar3D throttle from 5% to 100%. The CSV includes engine RPM, propeller RPM, manifold pressure, and piston-engine fuel flow when available. |
| Engine Deceleration | Automatic mode uses an on-ground, zero-airspeed setup and verifies the parking brake before applying full power. It records the response to the throttle reduction from 100% to 5%. |
| Pitch, Roll, and Rudder static checks | Automatic sweeps have per-test settings for commanded sweep speed and CLS2Sim motor speed. Reports display inverted automatic virtual-force traces and preserve the recorded values in CSV. |
| Approach Stall and Climb Stall | Approach Stall is the existing stall-warning test. Climb Stall is a separate entry with Manual recording available and its Automatic procedure pending. |

Engine acceleration and deceleration graphs start at **0 s** when initial RPM settling can be identified. The dotted line marks throttle movement. The graph time shift leaves the raw recording timestamps and Ti/Tt calculations intact. Estimated fuel flow is labeled in the applicable acceleration reports.

## Results and climb diagnostics

Source runs default to `D:\QTG_Test_Results`. Set a different results directory before launch if needed.

```powershell
$env:QTG_RESULTS_ROOT = 'C:\QTG_Test_Results'
.\.venv\Scripts\python.exe main.py
```

A packaged app also accepts a `qtg_settings.json` file beside the executable containing a `results_directory` value. The environment variable takes precedence. With neither setting, the packaged app uses the current user's `Documents\QTG_Test_Results`.

The results directory contains test-specific folders for recordings, metadata, and PDF reports. The `sessions` folder contains worker context, status JSON, and logs. Automatic Normal Climb with Multi-PID also writes `*_climb_pid_trace.csv` when the run ends or stops during stabilization. The trace records pre-engagement simulator state, PID demands, loader commands and feedback, and simulator surface positions. For a climb issue, keep that CSV together with the matching status JSON and log.

## Build a SimPC release

Run these commands from `QTG_Studio/` in the virtual environment created above.

```powershell
.\.venv\Scripts\python.exe -m pip install pyinstaller
.\.venv\Scripts\python.exe -m PyInstaller --noconfirm tools/packaging/QTG_Studio_SimPC.spec
```

The build creates `dist\QTG_Studio\QTG_Studio.exe`. Copy the entire `dist\QTG_Studio` folder, including `_internal`, to the simulator PC. The runtime depends on the native DLLs referenced by the [SimPC spec](QTG_Studio/tools/packaging/QTG_Studio_SimPC.spec). Confirm the Prepar3D and CLS2Sim versions are compatible with the bridges used on that PC.

## Offline checks

From `QTG_Studio/`, run a focused check that uses mocked simulator interfaces.

```powershell
.\.venv\Scripts\python.exe -B -m unittest discover -s tests -p test_climb_pid.py -v
```

Some repository tests require Windows GUI support, local reference data, or native components. Use the individual test setup to determine whether a test can run on your machine. Offline checks verify software behavior. Qualification conclusions require the applicable validation data and supervised simulator evidence.

## Project layout

| Folder | Purpose |
| --- | --- |
| [Frontend](QTG_Studio/frontend/) | Dashboard, test settings, and live PID monitor |
| [Backend](QTG_Studio/backend/) | Test selection, initialization, control, telemetry recording, and worker status |
| [Interfaces](QTG_Studio/interfaces/) | Prepar3D and CLS2Sim integration |
| [Reports](QTG_Studio/reports/) | CSV interpretation, graphing, and evaluation reports |
| [Test suites](QTG_Studio/test_suites/) | Test definitions and specialized procedures |
| [Tests](QTG_Studio/tests/) | Offline regression checks and legacy test scripts |
| [Tools](QTG_Studio/tools/) | Packaging and diagnostic utilities |

See the [application architecture](QTG_Studio/docs/APP_ARCHITECTURE.md), [source layout](QTG_Studio/README.md), and [test change boundaries](AGENTS.md) for maintenance details.
