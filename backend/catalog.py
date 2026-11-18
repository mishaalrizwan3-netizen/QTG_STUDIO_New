"""Original QTG test catalog shared by the GUI and backend (CS-FSTD(A) aligned)."""

from dataclasses import dataclass


@dataclass
class TestSpec:
    reference: str
    title: str
    group: str
    page: int
    objective: str
    initial_conditions: str
    tolerance: str


TESTS = [
    # =========================================================================
    # 1. Performance (3 tests)
    # =========================================================================
    TestSpec(
        "1.c.(1)",
        "Normal climb -- all engines operating",
        "Performance",
        58,
        "To demonstrate that the simulation of engine thrust, aerodynamic drag and atmosphere in a steady state normal climb condition conforms to the aeroplane.",
        "Flt cond: Clean or specified climb configuration | Alt[ft]: 3000 | IAS[kts]: 80 | Flaps: 0",
        "± 3 kts airspeed | ± 5% or ± 0.5 m/s (100 ft/min) R/C (Rate of Climb)",
    ),
    TestSpec(
        "1.f.(1)",
        "Engine acceleration",
        "Performance",
        61,
        "To demonstrate that the simulated engine acceleration conforms to the validation data.",
        "Flt cond: Approach or Landing | Alt[ft]: 2000 | IAS[kts]: 80 | Flaps: 0 | Throttle[%]: 5",
        "± 10% Ti or ± 0.25 s | ± 10% Tt (Ti: 10% response, Tt: 90% go-around power)",
    ),
    TestSpec(
        "1.f.(2)",
        "Engine deceleration",
        "Performance",
        61,
        "To demonstrate that the simulated engine deceleration conforms to the aeroplane.",
        "Flt cond: Ground | Alt[ft]: 755 | IAS[kts]: 0 | Flaps: 0 | Throttle[%]: 100",
        "± 10% Tt (Total time from initial throttle movement until 90% flight idle throttle movement)",
    ),

    # =========================================================================
    # 2. Static Control Checks — 2.a (6 tests)
    # =========================================================================
    TestSpec(
        "2.a.(1)",
        "Pitch controller position vs force and surface position calibration",
        "Static Control Checks",
        62,
        "To demonstrate that the simulator pitch controller position vs. pitch controller force conforms to the validation data.",
        "",
        "± 2.2 daN (5 lbs) or ± 10% force | ± 0.5 cm (0.2 in) or ± 10% column position | ± 2º elevator (where applicable)",
    ),
    TestSpec(
        "2.a.(2)",
        "Roll controller position vs force and surface position calibration",
        "Static Control Checks",
        63,
        "To demonstrate that the simulator roll controller position vs. roll controller force conform to the aeroplane.",
        "",
        "± 1.3 daN (3 lbs) or ± 10% force | ± 0.5 cm (0.2 in) or ± 10% wheel position | ± 2º aileron/spoiler",
    ),
    TestSpec(
        "2.a.(3)",
        "Rudder pedal position vs force and surface position calibration",
        "Static Control Checks",
        63,
        "To demonstrate that the simulator rudder pedal position vs. rudder pedal force conform to the aeroplane.",
        "",
        "± 2.2 daN (5 lbs) or ± 10% force | ± 0.5 cm (0.2 in) or ± 10% pedal position | ± 2º rudder",
    ),
    TestSpec(
        "2.a.(7)",
        "Pitch trim rate and indicator calibration",
        "Static Control Checks",
        64,
        "To demonstrate that the pitch trim rate and indicator calibration conform to the aeroplane.",
        "",
        "± 10% travel rate | ± 0.5 s or ± 10% time for full travel | ± 0.5 cm (0.2 in) or ± 10% cockpit trim control",
    ),
    TestSpec(
        "2.a.(8)",
        "Alignment of cockpit throttle lever vs selected engine parameter",
        "Static Control Checks",
        64,
        "Record cockpit throttle, propeller and mixture lever positions with simulator engine parameters.",
        "Flt cond: Ground | IAS[kts]: 0",
        "± 5° TLA or ± 3% N1 or ± 0.03 EPR or ± 3% torque; ± 2 cm for non-angular propeller levers",
    ),

    # =========================================================================
    # 3. Longitudinal Handling — 2.c (10 tests)
    # =========================================================================
    TestSpec(
        "2.c.(1)",
        "Power change dynamics",
        "Longitudinal Handling",
        69,
        "Power change from thrust for approach or level flight to maximum continuous or go-around power. Time history of uncontrolled free response for at least 5 s before initiation to completion + 15 s.",
        "Flt cond: Approach | Alt[ft]: 4000 | IAS[kts]: 90 | Flaps: 1 | "
        "Latitude[deg]: 32.235 | Longitude[deg]: 74.135 | Heading[deg]: 48.4 | "
        "Pitch[deg]: 3 | Bank[deg]: 0 | Throttle[%]: 50 | Temperature[C]: 15",
        "± 3 kt airspeed | ± 30 m (100 ft) altitude | ± 1.5° or ± 20 % pitch angle",
    ),
    TestSpec(
        "2.c.(2)",
        "Flap change dynamics",
        "Longitudinal Handling",
        70,
        "Time history of uncontrolled free response for at least 5 s before initiation of reconfiguration change to completion + 15 s.",
        "",
        "± 3 kts airspeed | ± 30 m (100 ft) altitude | ± 1.5º or ± 20% pitch angle",
    ),
    TestSpec(
        "2.c.(3)",
        "Spoiler / speedbrake change dynamics",
        "Longitudinal Handling",
        70,
        "Time history of uncontrolled free response for at least 5 s before initiation of reconfiguration change to completion + 15 s. Extension and retraction.",
        "",
        "± 3 kts airspeed | ± 30 m (100 ft) altitude | ± 1.5º or ± 20% pitch angle",
    ),
    TestSpec(
        "2.c.(4)",
        "Gear change dynamics",
        "Longitudinal Handling",
        70,
        "Time history of uncontrolled free response for at least 5 s before initiation of configuration change to completion + 15 s.",
        "",
        "± 3 kts airspeed | ± 30 m (100 ft) altitude | ± 1.5º or ± 20% pitch angle",
    ),
    TestSpec(
        "2.c.(5)",
        "Longitudinal trim",
        "Longitudinal Handling",
        70,
        "Steady-state wings level trim with thrust for level flight.",
        "",
        "± 1º elevator | ± 0.5º stabilizer | ± 1º pitch angle | ± 5% net thrust",
    ),
    TestSpec(
        "2.c.(6)",
        "Longitudinal manoeuvring stability (stick force/g)",
        "Longitudinal Handling",
        71,
        "To demonstrate that the simulation of manoeuvring stability (stick force per g) conforms to the validation data.",
        "",
        "± 2.2 daN (5 lbs) or ± 10% pitch controller force | Alternative: ± 1º or ± 10% change of elevator",
    ),
    TestSpec(
        "2.c.(7)",
        "Longitudinal static stability",
        "Longitudinal Handling",
        71,
        "Data for at least two speeds above and two speeds below trim speed.",
        "",
        "± 2.2 daN (5 lbs) or ± 10% pitch controller force | Alternative: ± 1° or ± 10% change of elevator",
    ),
    TestSpec(
        "2.c.(8b)",
        "Approach-to-stall characteristics (Stall warning)",
        "Longitudinal Handling",
        73,
        "To record the aircraft's approach-to-stall characteristics.",
        "Flt cond: Approach | Alt[ft]: 4000 | IAS[kts]: 90 | Flaps: 0 | Temperature[C]: 15 | Throttle[%]: 80",
        "± 3 kt stall warning speed | ± 2° angle of attack at initial buffet | ± 2° pitch, angle of attack and bank | ± 10% or ± 5 lb stick force where applicable",
    ),
    TestSpec(
        "2.c.(9)",
        "Phugoid dynamics",
        "Longitudinal Handling",
        74,
        "Test should include at least three full cycles or that necessary to determine time to 1/2 or double amplitude.",
        "",
        "± 10% period | ± 10% time to 1/2 or double amplitude or ± 0.02 damping ratio",
    ),
    TestSpec(
        "2.c.(10)",
        "Short-period dynamics",
        "Longitudinal Handling",
        74,
        "To demonstrate that the short-period dynamic characteristics conform to the aeroplane.",
        "",
        "± 1.5° pitch angle or ± 2°/s pitch rate | ± 0.1 g normal acceleration",
    ),

    # =========================================================================
    # 4. Lateral-Directional Handling — 2.d (8 tests)
    # =========================================================================
    TestSpec(
        "2.d.(1)",
        "Minimum control speed, air (Vmca or Vmcl)",
        "Lateral-Directional Handling",
        75,
        "Minimum control speed air per applicable airworthiness standard, or low speed engine inoperative handling in the air.",
        "",
        "± 3 kt airspeed",
    ),
    TestSpec(
        "2.d.(2)",
        "Roll response (rate)",
        "Lateral-Directional Handling",
        75,
        "Test with normal roll control displacement (about 30% of maximum control wheel).",
        "",
        "± 10% or ± 2°/s roll rate | FS: ± 10% or ± 1.3 daN (3 lb) roll controller force",
    ),
    TestSpec(
        "2.d.(3)",
        "Step input of cockpit roll controller (roll overshoot)",
        "Lateral-Directional Handling",
        75,
        "Wings level, apply step roll input (~1/3 wheel travel). At 20° to 30° bank, abruptly return to neutral and allow at least 10 s free response.",
        "",
        "± 10% or ± 2° bank angle",
    ),
    TestSpec(
        "2.d.(4)",
        "Spiral stability",
        "Lateral-Directional Handling",
        76,
        "Aeroplane data averaged from multiple tests may be used. Test for both directions.",
        "",
        "Correct trend and ± 2° or ± 10% bank angle in 20 s (or correct trend and ± 2° aileron)",
    ),
    TestSpec(
        "2.d.(5)",
        "Engine inoperative trim",
        "Lateral-Directional Handling",
        76,
        "Trim an engine failure condition. 2nd segment climb at take-off thrust; approach or landing at level flight thrust.",
        "",
        "± 1º rudder angle or ± 1º tab angle or equivalent pedal | ± 2º sideslip angle",
    ),
    TestSpec(
        "2.d.(6)",
        "Rudder response",
        "Lateral-Directional Handling",
        76,
        "Test with step input at approximately 25% of full rudder pedal throw with stability augmentation OFF.",
        "",
        "± 2 deg/s or ± 10% yaw rate | ± 10% heading change",
    ),
    TestSpec(
        "2.d.(7)",
        "Dutch roll (yaw damper OFF)",
        "Lateral-Directional Handling",
        76,
        "Test for at least six cycles with stability augmentation OFF.",
        "",
        "± 0.5 s or ± 10% period | ± 10% time to 1/2 or double amplitude or ± 0.02 damping ratio",
    ),
    TestSpec(
        "2.d.(8)",
        "Steady state sideslip",
        "Lateral-Directional Handling",
        77,
        "Snapshot tests using at least two rudder positions in each direction, one near maximum allowable rudder.",
        "",
        "± 2° bank angle | ± 1° sideslip angle | ± 10% or ± 2° aileron | ± 10% or ± 1.3 daN wheel force | ± 10% or ± 2.2 daN pedal force",
    ),
]
