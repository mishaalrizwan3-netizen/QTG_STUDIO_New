# 2.b.(11) — Power change dynamics (longitudinal c.(1))

## Objective
Power change from thrust for approach or level flight to maximum continuous or go-around power. Time history of uncontrolled free response for a time increment equal to at least 5 s before initiation of the power change to completion of the power change + 15 s.

## Automatic sequence
Initialize airborne at the throttle selected in Initial Conditions (default 50%). After position, configuration, and live engine readback are verified, pause and show READY TO RECORD. There is no separate pre-recording RPM settling wait. After the operator starts recording, hold the selected throttle for at least 5 seconds of measured baseline, command 100%, and continue recording for at least 15 seconds after the simulator confirms 100%. The run fails if either throttle hold is lost. The initial setting must be at least 5% below 100%.

During recording, the test commands throttle only. Airspeed, altitude, and attitude may change naturally during the uncontrolled flight response; they do not block the throttle step. PID assistance must be off. If a run stops before measured throttle movement, its CSV and metadata are saved without a misleading event graph.

Controller position and applied force for pitch, roll, and rudder are sampled read-only from CLS2Sim during the run. Selecting their graph channels controls PDF display; it does not enable controller movement. If CLS2Sim measurements are unavailable, those columns remain missing measurements rather than invented traces.

The Automatic PDF keeps the shared climb and acceleration report layout. Its initial recorded values come from the first recorded frame at graph time zero. Available selected channels are plotted; selections without recorded data are listed in the metadata.

## Tolerances
- Airspeed: ± 3 kt
- Altitude: ± 30 m (100 ft)
- Pitch angle: ± 1.5° or ± 20%
