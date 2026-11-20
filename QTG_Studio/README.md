# QTG Studio source layout

Run `python main.py` from this directory, or `python QTG_STUDIO/main.py` from the workspace root. `RUN_QTG.bat` may prefer an existing packaged EXE; use the Python command to run current source changes. No EXE was rebuilt in this reorganization.

| Path | Role |
| --- | --- |
| `main.py` | Source and packaged entry point; exposes the existing `main` import. |
| `frontend/main.py` | Tkinter GUI and test settings. The two older standalone Qt GUIs are also here. |
| `frontend/pid_monitor.py` | Existing PID monitor UI code. |
| `qtg_common_engine.py` | Existing worker entry and hardware state machine. |
| `backend/` | Test catalog, input contracts, registry, initialization, validation, control, recording, safety, engine helpers, and artifact naming. |
| `interfaces/p3d.py` | Dashboard P3D bridge; legacy SimConnect and climb-controller transport modules are also here. |
| `interfaces/brunner.py` | Adapter to the existing BRUNNER base at `interfaces/brunner_base/`. |
| `reports/` | Report routing, graph generation, engine report generation, timing rules, and Ti/Tt analysis. |
| `tools/` | Portable build implementation and standalone diagnostics. Root batch files remain launch/build entry points. |
| `test_suites/normal_climb/` | Climb definition, live PID procedure, validation, and original PID source. |
| `test_suites/engine_acceleration/` and `test_suites/engine_deceleration/` | Engine definitions, procedures, and validation adapters. |
| `bridge/`, `p3d_sdk/`, `easa_references/` | Native runtime/build dependencies and reference assets. |
| `tests/` | Offline regression tests. |
| `build/`, `dist/`, `release_backups/` | Generated build files and preserved releases. |

The root compatibility modules keep existing imports and commands working; their implementations now live under the role folders above. Active test code uses one BRUNNER base and the existing common worker. Runtime results remain in `D:\QTG_Test_Results`, grouped by test name; source-local historical recordings and built releases were not moved into runtime results.

See [application architecture](docs/APP_ARCHITECTURE.md) and [test suites](test_suites/README.md).

Engine Acceleration and Deceleration Automatic Testing use a P3D throttle step.
Both Manual and Automatic initialization use the selected test settings for
aircraft position, attitude, flaps and temperature. Engine Acceleration fixes
initial throttle at 5%; Engine Deceleration fixes initial throttle at 100% and
initializes on the ground at zero airspeed. These required test values are shown
in Test Settings. Manual Deceleration verifies a fresh parking-brake ON,
on-ground and stationary readback before applying full throttle.
From PowerShell, run the modified source with:

```powershell
Set-Location -LiteralPath 'C:\Users\Qasim 007\Desktop\QTG_STUDIO_New'
py -3 main.py
```

Select Engine acceleration or Engine deceleration, review Test Settings, and
click Automatic. A running engine is required. Deceleration requires Ground,
zero airspeed, and the parking brake engaged. Initialization verifies fresh
position, flaps, temperature and throttle measurements; Deceleration verifies
the parking brake and stationary aircraft before applying initial full throttle.
Acceleration starts at the engine test's 5% idle throttle; Deceleration starts at 100%, regardless of
the Manual throttle setting. Automatic does not require a Manual reference.

Wait for **READY TO RECORD**, then click **Start Recording**. The worker records
five seconds before commanding the P3D throttle to 100% for Acceleration or 5%
for Deceleration. It stops after three continuous seconds within 50 RPM of 2700
or 970 RPM respectively, with measured final throttle confirmed. The response
deadline is 30 seconds after the command. A pause, stale telemetry, disconnect,
or loss of the Deceleration parking brake fails the attempt. Stop cancels it.
The worker pauses P3D on exit and removes Deceleration power before saving.
CSV, metadata and a measured-response PDF are saved in the selected test folder
under `D:\QTG_Test_Results` (or `QTG_RESULTS_ROOT`). Interrupted recordings are
labelled incomplete. These reports do not claim conformity to aircraft reference
data. BRUNNER flight controls are never connected, commanded or centered by this
Automatic workflow. Manual and other QTG tests retain their existing workers.

Offline integration checks: `py -3 -m unittest discover -s tests -p test_automatic_engine_integration.py -v`.
These use mocked simulator transports and generate actual CSV/PDF files in a
temporary folder. They do not connect to a simulator or build an EXE.
