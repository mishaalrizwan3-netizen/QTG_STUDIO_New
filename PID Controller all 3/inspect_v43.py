import csv

with open(r"PID_LOGS\SMK_Brunner_DualAxis_V4_3_20261105_125336.csv", mode="r") as f:
    reader = csv.DictReader(f)
    rows = list(reader)

print(f"V4.3 total rows: {len(rows)}")
for idx in [0, len(rows)//4, len(rows)//2, 3*len(rows)//4, len(rows)-1]:
    r = rows[idx]
    print(f"t={r['elapsed_s']}s | Elev={r.get('elevator_position', '?')[:8]} | YokeY={r.get('yoke_y_position', '?')[:8]} | PitchCmd={r.get('brunner_pitch_cmd', '?')[:7]} | PitchMeas={r.get('brunner_pitch_meas', '?')[:7]} | PitchAct={r.get('actual_pitch_deg', '?')[:7]}")

elevs = [float(r['elevator_position']) for r in rows if 'elevator_position' in r]
pitch_cmds = [float(r['brunner_pitch_cmd']) for r in rows if 'brunner_pitch_cmd' in r]
pitch_meas = [float(r['brunner_pitch_meas']) for r in rows if 'brunner_pitch_meas' in r]
pitch_acts = [float(r['actual_pitch_deg']) for r in rows if 'actual_pitch_deg' in r]

print("\n--- V4.3 Summary ---")
print(f"Elev min/max: {min(elevs):.4f} / {max(elevs):.4f}")
print(f"Pitch Cmd min/max: {min(pitch_cmds):.4f} / {max(pitch_cmds):.4f}")
print(f"Pitch Meas min/max: {min(pitch_meas):.4f} / {max(pitch_meas):.4f}")
print(f"Pitch Act min/max: {min(pitch_acts):.4f} / {max(pitch_acts):.4f}")

