import csv

with open(r'PID_LOGS\SMK_Brunner_TriAxis_V4_4_20261106_103236.csv', mode='r') as f:
    reader = csv.DictReader(f)
    rows = list(reader)

prev_th = None
prev_ykp = None
prev_bkp = None
for r in rows:
    th = r['target_heading_deg']
    ykp = r['yaw_kp']
    bkp = r['bank_kp']
    if th != prev_th or ykp != prev_ykp or bkp != prev_bkp:
        t = float(r['elapsed_s'])
        act_h = float(r['actual_heading_deg'])
        rcmd = float(r['brunner_roll_cmd'])
        rmeas = float(r['brunner_roll_meas'])
        bact = float(r['actual_bank_deg'])
        ycmd = float(r['brunner_yaw_cmd'])
        ymeas = float(r['brunner_yaw_meas'])
        print(f"t={t:5.1f}s | TgtHdg={th:>5} ActHdg={act_h:5.1f} | YawKp={ykp:>5} BankKp={bkp} | RollCmd={rcmd:+5.2f} RollMeas={rmeas:+5.2f} | YawCmd={ycmd:+6.4f} YawMeas={ymeas:+5.2f} | BankAct={bact:+5.2f}")
        prev_th = th
        prev_ykp = ykp
        prev_bkp = bkp

