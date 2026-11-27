import csv

with open(r"PID_LOGS\SMK_Brunner_TriAxis_V4_4_20261106_103236.csv", mode="r") as f:
    reader = csv.DictReader(f)
    rows = list(reader)

print(f"Total rows: {len(rows)}")
for idx in [0, len(rows)//4, len(rows)//2, 3*len(rows)//4, len(rows)-1]:
    r = rows[idx]
    print(f"t={r['elapsed_s']}s | HdgTgt={r['target_heading_deg']} | HdgAct={r['actual_heading_deg'][:6]} | BankAct={r['actual_bank_deg'][:6]} | DemBank={r['demanded_bank_deg'][:6]} | RollCmd={r['brunner_roll_cmd'][:7]} | RollMeas={r['brunner_roll_meas'][:7]} | YawCmd={r['brunner_yaw_cmd'][:7]} | YawMeas={r['brunner_yaw_meas'][:7]} | YokeY={r['yoke_y_position'][:8]} | Rudder={r['rudder_position'][:8]} | BankKp={r['bank_kp']} | YawKp={r['yaw_kp']}")

# Check min/max across whole run
roll_cmds = [float(r['brunner_roll_cmd']) for r in rows]
roll_meas = [float(r['brunner_roll_meas']) for r in rows]
yaw_cmds = [float(r['brunner_yaw_cmd']) for r in rows]
yaw_meas = [float(r['brunner_yaw_meas']) for r in rows]
yoke_y = [float(r['yoke_y_position']) for r in rows]
rudder = [float(r['rudder_position']) for r in rows]
bank_kps = set(r['bank_kp'] for r in rows)
yaw_kps = set(r['yaw_kp'] for r in rows)

print("\n--- Summary ---")
print(f"Roll Cmd min/max: {min(roll_cmds):.4f} / {max(roll_cmds):.4f}")
print(f"Roll Meas min/max: {min(roll_meas):.4f} / {max(roll_meas):.4f}")
print(f"Yaw Cmd min/max: {min(yaw_cmds):.4f} / {max(yaw_cmds):.4f}")
print(f"Yaw Meas min/max: {min(yaw_meas):.4f} / {max(yaw_meas):.4f}")
print(f"P3D Yoke Y min/max: {min(yoke_y):.4f} / {max(yoke_y):.4f}")
print(f"P3D Rudder min/max: {min(rudder):.4f} / {max(rudder):.4f}")
print(f"Bank Kp values recorded: {bank_kps}")
print(f"Yaw Kp values recorded: {yaw_kps}")

