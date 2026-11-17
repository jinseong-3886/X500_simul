#!/bin/bash
cd ~/PX4-Autopilot/build/px4_sitl_default || exit 1
echo "== SITL log (since mission):"; grep -v "No connection to the GCS" /tmp/px4_sitl.log | tail -14
echo "== current local position / arming:"
./bin/px4-listener vehicle_local_position -n 1 2>/dev/null | grep -E "^\s+(z|xy_valid|z_valid|heading):"
./bin/px4-listener vehicle_status -n 1 2>/dev/null | grep -E "arming_state|nav_state:"
echo "== latest ulog:"; ls -t rootfs/log/*/*.ulg | head -1
python3 - <<'EOF'
import glob, os
try:
    from pyulog import ULog
except ImportError:
    print("pyulog not installed -> skip trajectory summary"); raise SystemExit
f = max(glob.glob(os.path.expanduser("~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/*/*.ulg")), key=os.path.getmtime)
u = ULog(f, ["vehicle_local_position", "vehicle_status"])
lp = u.get_dataset("vehicle_local_position").data
import numpy as np
z = -lp["z"]; t = (lp["timestamp"] - lp["timestamp"][0]) / 1e6
print(f"max altitude {z.max():.2f} m | heading range {np.degrees(lp['heading'].min()):.0f}..{np.degrees(lp['heading'].max()):.0f} deg | final alt {z[-1]:.2f} m")
vs = u.get_dataset("vehicle_status").data
names = {14: "OFFBOARD", 18: "AUTO_LAND", 4: "AUTO_LOITER", 2: "POSCTL"}
seq = []
for n in vs["nav_state"]:
    s = names.get(int(n), str(int(n)))
    if not seq or seq[-1] != s: seq.append(s)
print("nav_state sequence:", " -> ".join(seq))
print("arming_state sequence:", sorted(set(int(a) for a in vs["arming_state"])), "(1=disarmed, 2=armed)")
EOF
