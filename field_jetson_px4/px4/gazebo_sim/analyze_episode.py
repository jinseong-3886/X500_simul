"""Ground truth from the PX4 ulog of the latest Flytrap episode: true camera-to-board distance over time."""
import glob, os, sys, numpy as np
from pyulog import ULog

KIND = sys.argv[1] if len(sys.argv) > 1 else "clean"
BOARD_NED = (0.0, 2.5)     # gz world x=2.5 (east)  ->  PX4 local NED y=2.5
CAM_FWD = 0.12             # camera ahead of body centre [m]
f = max(glob.glob(os.path.expanduser("~/flytrap_sim/rootfs/log/*/*.ulg")), key=os.path.getmtime)
u = ULog(f, ["vehicle_local_position", "vehicle_status"])
lp = u.get_dataset("vehicle_local_position").data
vs = u.get_dataset("vehicle_status").data
armed_t = vs["timestamp"][vs["arming_state"] == 2]
if armed_t.size == 0:
    sys.exit("no armed flight in log")
m = (lp["timestamp"] >= armed_t.min()) & (lp["timestamp"] <= armed_t.max())
t = (lp["timestamp"][m] - armed_t.min()) / 1e6
x, y, z, hd = lp["x"][m], lp["y"][m], -lp["z"][m], lp["heading"][m]
cx, cy = x + CAM_FWD * np.cos(hd), y + CAM_FWD * np.sin(hd)
dist = np.hypot(BOARD_NED[0] - cx, BOARD_NED[1] - cy)
air = z > 0.5
print(f"== ground truth ({KIND}) from {os.path.basename(f)}")
print(f"flight {t[-1]:.0f}s | max alt {z.max():.2f} m")
if air.any():
    print(f"camera->board distance while airborne: start {dist[air][0]:.2f} m, min {dist[air].min():.2f} m, "
          f"end {dist[air][-1]:.2f} m")
    for k in np.linspace(0, air.sum() - 1, 8).astype(int):
        i = np.where(air)[0][k]
        print(f"  t={t[i]:5.1f}s  dist={dist[i]:.2f} m  alt={z[i]:.2f} m  heading={np.degrees(hd[i]):6.1f} deg")
names = {14: "OFFBOARD", 18: "AUTO_LAND", 4: "AUTO_LOITER", 2: "POSCTL", 5: "AUTO_RTL"}
seq = []
for n in vs["nav_state"][(vs["timestamp"] >= armed_t.min() - 2e6)]:
    s = names.get(int(n), str(int(n)))
    if not seq or seq[-1] != s:
        seq.append(s)
print("nav_state:", " -> ".join(seq))
