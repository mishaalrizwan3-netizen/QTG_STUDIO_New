Stage 2: Bank PID only
======================
Bank PID is available in every Test Settings dropdown. Existing settings locks and the
settings-only multi-engine test remain unchanged. Airspeed retains its existing
Normal Climb availability and implementation; only its display name changes.
Automatic snapshots bank_pid, runs the existing base Manual preparation with selected
conditions, then waits for confirmed running state and fresh native frames before Bank engagement.
Recording begins only after five continuous seconds within +/-1 degree; acquisition is diagnostic data only.
No replay control sequence, engine throttle sequence, heading guidance or rudder PID
runs. The Bank target is always 0 degrees. Bank telemetry is already right-positive.

Only roll position commands are issued. A per-run transport shares the existing
serialized socket. It retains reference gains, filters, integral rules, slew limits,
AP force/speed and watchdog settings. Engagement starts at measured roll position.
Stop cancels first, pauses P3D, releases roll AP/overrides without position centering,
and disables the watchdog. Closed transports reject all subsequent commands.
Normal cleanup is also used on missing/stale feedback, disconnect and GUI exit.
A process crash relies on CLS2Sim's 1000 ms hardware watchdog; this MUST be verified
under supervised hardware conditions. Python cannot guarantee cleanup after a crash.

Initialization adapters, Airspeed source/runner and native binaries are unchanged.
A Bank-only dispatch bypasses replay-specific automatic policy and engine extensions,
reusing the existing selected-condition base preparation. The initialization-only
trial interlock still applies. Bank reports need a separate early routing branch:
there is no replay reference or IAS evaluation. Standard measured CSV buffers and
report save lifecycle are retained, with additional roll-command diagnostics.
Bank PDF is a measured-data report, NOT EVALUATED, and identifies interrupted runs.
Existing reports and graph-selection behavior for other modes are unchanged.
Bank PDF honors per-test graph selections and labels missing measurements. Ti/Tt
procedure evaluation is not performed by a Bank-only run. One existing base report
call passes bank_fault for Bank runs so interrupted reports cannot appear complete.

Offline tests use fake devices only. No automatic simulator/hardware tests.
Supervised checks: actual roll feedback node, command polarity, tracking, loader
force/speed, manual forwarding, Stop/E-stop, unplug/disconnect, mode changes and
watchdog release on process termination. No live validation is claimed.

Startup correction: Bank bypasses the inherited arm_recording launcher, which marked recording active before readiness. Bank finalization emits recording-finished only for runs with actual recording samples. The initializer intentionally pauses P3D (backend/initializer.py apply_manual_conditions and base initialization commit). Bank issues one unpause and refreshes its running-state subscription; it does not repeatedly unpause or tolerate active-run pauses. Failures release roll and transition to FAILED. Native telemetry receipt sequences are checked with running-state feedback; this bridge exposes no independent physics timestamp. A live trace is still required to identify any external or simulator-triggered re-pause. Previous EXE/ZIP packages predate this source correction and must be rebuilt before simulator use.

Loading correction: the worker Pause subscription does NOT indicate that scenery/aircraft loading is complete. Bank readiness now also requires the EXISTING dashboard SimStart/SimStop state, published atomically by the GUI with freshness metadata. No native binaries are changed. Missing, stopped, stale or invalid dashboard state prevents engagement. Loading waits are cancellable and have no arbitrary 20-second failure timeout. A pre-recording pause releases roll, resets stabilization and returns to WAITING_FOR_P3D; recording pauses still interrupt the run. Only one startup unpause is issued, after the lifecycle signal is ready.

Zero-capture correction: recording requires abs(bank) < 0.05 degree continuously for five seconds; PID remains active at a 0-degree target throughout recording. Acquisition has no default timeout and remains cancellable. Dashboard loading IPC is used to authorize startup only, never to declare a pause during recording. Native pause state and fresh worker telemetry remain recording safety inputs. Bank reports merge only recording-phase diagnostic rows and use the recorded time column for duration. Exact physical zero cannot be guaranteed; supervised tracking validation remains required.

Pre-recording telemetry gaps now reset the bank stability timer and use the same release/wait/reseed handoff as startup pauses. Recovery requires fresh running feedback and a new continuous capture interval. Persistent missing feedback remains cancellable and does not create a recording report. Missing telemetry after recording starts still interrupts safely and saves incomplete data. No stale samples are integrated or counted toward capture.
