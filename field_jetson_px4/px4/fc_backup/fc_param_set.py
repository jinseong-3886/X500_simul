"""Back up ALL FC params, then (only if backup is complete) set ONE param and read it back.
Usage (Jetson): python3 -u fc_param_set.py --backup out.params [--set NAME=INT]
Backup format = QGC .params (Vehicle-Id Component-Id Name Value Type), loadable in QGroundControl.
"""
import argparse, struct, time
from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument("--dev", default="/dev/ttyACM0")
ap.add_argument("--baud", type=int, default=115200)
ap.add_argument("--backup", required=True)
ap.add_argument("--set", default=None)
a = ap.parse_args()

INT_TYPES = (1, 2, 3, 4, 5, 6)
m = mavutil.mavlink_connection(a.dev, baud=a.baud, source_system=245)
if not m.wait_heartbeat(timeout=10):
    raise SystemExit("no heartbeat")
ts, tc = m.target_system, 1  # MAV_COMP_ID_AUTOPILOT1 (heartbeat may come from comp 0)

def pv(timeout):
    """PARAM_VALUE from the autopilot only (ignore other components on the link)."""
    t_end = time.time() + timeout
    while time.time() < t_end:
        r = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=max(0.01, t_end - time.time()))
        if r is not None and r.get_srcSystem() == ts and r.get_srcComponent() == tc:
            return r
    return None

def decode(r):
    if r.param_type in INT_TYPES:
        return struct.unpack("<i", struct.pack("<f", r.param_value))[0]
    return r.param_value

# ---- 1. full backup
params, total = {}, None
m.mav.param_request_list_send(ts, tc)
last = time.time()
while time.time() - last < 3:
    r = pv(0.5)
    if r and r.param_index != 65535:
        total = r.param_count
        params[r.param_index] = (r.param_id.rstrip("\x00"), decode(r), r.param_type)
        last = time.time()
        if len(params) >= total:
            break
for rnd in range(5):  # fetch missing indices one by one
    missing = [i for i in range(total or 0) if i not in params]
    if not missing:
        break
    for i in missing:
        m.mav.param_request_read_send(ts, tc, b"", i)
        r = pv(1)
        if r and r.param_index != 65535:
            params[r.param_index] = (r.param_id.rstrip("\x00"), decode(r), r.param_type)
got = len([i for i in params if i < (total or 0)])
print(f"backup: {got}/{total} params")
if not total or got < total:
    raise SystemExit("backup INCOMPLETE -> not changing anything")
with open(a.backup, "w") as f:
    f.write(f"# FC param backup {time.strftime('%Y-%m-%d %H:%M:%S')}  ({total} params)\n")
    f.write("# Vehicle-Id\tComponent-Id\tName\tValue\tType\n")
    for i in sorted(params):
        n, v, t = params[i]
        vs = str(v) if t in INT_TYPES else f"{v:.18g}"
        f.write(f"{ts}\t{tc}\t{n}\t{vs}\t{t}\n")
print("saved", a.backup)

if not a.set:
    raise SystemExit(0)

# ---- 2. set one integer param, 3. read back
name, val = a.set.split("=")
val = int(val)
cur = [p for p in params.values() if p[0] == name]
if not cur:
    raise SystemExit(f"{name} not found on FC")
_, old, ptype = cur[0]
if ptype not in INT_TYPES:
    raise SystemExit(f"{name} is not an integer param (type {ptype})")
print(f"{name}: {old} -> {val}")
fval = struct.unpack("<f", struct.pack("<i", val))[0]
for _ in range(3):
    m.mav.param_set_send(ts, tc, name.encode(), fval, ptype)
    t0 = time.time(); ok = None
    while time.time() - t0 < 2:
        r = pv(0.5)
        if r and r.param_id.rstrip("\x00") == name:
            ok = decode(r); break
    if ok is not None:
        break
# independent read-back
m.mav.param_request_read_send(ts, tc, name.encode(), -1)
rb = None; t0 = time.time()
while time.time() - t0 < 3:
    r = pv(0.5)
    if r and r.param_id.rstrip("\x00") == name:
        rb = decode(r); break
print(f"read-back {name} = {rb}  ->  {'OK' if rb == val else 'FAILED'}")
