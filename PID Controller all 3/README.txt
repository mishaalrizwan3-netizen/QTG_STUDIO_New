SMK BRUNNER CLS-P FAST PITCH & BANK PID CONTROLLER V4.3 — SYSTEM DOCUMENTATION

1. Architecture Overview:
   - Pure Brunner CLS-P Hardware Control: NO flight control surfaces are written to Prepar3D via SimConnect.
   - P3D SimConnect is strictly READ-ONLY for flight telemetry (Pitch, Bank, IAS, Altitude, Weight, Temp).
   - All actuation is driven directly via UDP (127.0.0.1:15090) to Brunner CLS2Sim:
       * Pitch Axis (AX_PITCH = 0x0001): Elevator Control Loader Motor
       * Roll Axis  (AX_ROLL  = 0x0002): Aileron Control Loader Motor
   - CLS2Sim physically moves the Brunner yoke/stick and passes control inputs into Prepar3D.

2. Dual Guidance Tabs:
   - Tab 1: Airspeed & Pitch Hold
       * FLC Pitch-for-Speed Mode (dynamic pitch commands to maintain target IAS, e.g. 80 kt).
       * Pitch PID inner loop commands Brunner Pitch Motor [-0.55 .. +0.65].
   - Tab 2: Bank Angle Hold (Wings Level = 0.00°)
       * Target Bank: 0.00° (with fine-tuning slider and 0.00° Level quick button).
       * Dedicated Realtime Bank PID Gains: Bank Kp, Ki, Kd with live sliders and numeric entries.
       * Zero deadband integral control: continuously integrates residual bank error down to exactly 0.00°.
       * Dual Live Graphs: Bank Angle Tracking & Error (deg), Brunner CLS-P Roll Motor Channel.
       * Clean Interface: All clutter (altitude options, checkboxes, bank rate readouts) removed.

3. Automated Operation:
   - Clicking "START FLIGHT CONTROLLER" arms and runs BOTH Pitch/Airspeed hold and Wings-Level Bank hold simultaneously.
   - When stopped, all motor overrides are gracefully released back to manual pilot control.

4. Data Logging:
   - Telemetry, actuator commands, PID states, and gain settings recorded to CSV in PID_LOGS/.
