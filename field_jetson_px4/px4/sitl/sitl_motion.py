import glob, os, numpy as np
from pyulog import ULog
f = max(glob.glob(os.path.expanduser("~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/*/*.ulg")), key=os.path.getmtime)
u = ULog(f, ["vehicle_local_position", "trajectory_setpoint", "vehicle_status"])
lp = u.get_dataset("vehicle_local_position").data
vs = u.get_dataset("vehicle_status").data
# last armed window
arm = vs["arming_state"] == 2
ts = vs["timestamp"][arm]
t0, t1 = ts.min(), ts.max()
# find the LAST contiguous armed flight
edges = np.where(np.diff(vs["arming_state"]) != 0)[0]
starts = [vs["timestamp"][i + 1] for i in edges if vs["arming_state"][i + 1] == 2]
t0 = starts[-1] if starts else t0
m = (lp["timestamp"] >= t0) & (lp["timestamp"] <= t1)
t = (lp["timestamp"][m] - t0) / 1e6
x, y, z, vx, vy, hd = lp["x"][m], lp["y"][m], -lp["z"][m], lp["vx"][m], lp["vy"][m], lp["heading"][m]
print(f"log {os.path.basename(f)} | last flight {t[-1]:.0f}s")
print(f"horizontal displacement start->end: dx {x[-1]-x[0]:+.2f} m, dy {y[-1]-y[0]:+.2f} m | max |v_xy| {np.hypot(vx, vy).max():.2f} m/s")
for a, b in ((0, 30), (30, 45), (45, 60), (60, 75), (75, 90)):
    s = (t >= a) & (t < b)
    if s.any():
        i, j = np.where(s)[0][[0, -1]]
        mv = np.hypot(x[j] - x[i], y[j] - y[i])
        # forward = projection of displacement on mean heading
        h = np.arctan2(np.sin(hd[s]).mean(), np.cos(hd[s]).mean())
        fwd = (x[j] - x[i]) * np.cos(h) + (y[j] - y[i]) * np.sin(h)
        print(f"  t {a:3d}-{b:3d}s: alt {z[s].mean():.2f} m | moved {mv:.2f} m (forward along heading {fwd:+.2f} m) | heading {np.degrees(h):.0f} deg")
tsp = u.get_dataset("trajectory_setpoint").data
mm = (tsp["timestamp"] >= t0) & (tsp["timestamp"] <= t1)
vxs, vys = tsp["velocity[0]"][mm], tsp["velocity[1]"][mm]
ok = np.isfinite(vxs)
if ok.any():
    sp = np.hypot(vxs[ok], vys[ok])
    print(f"velocity setpoints received: max {sp.max():.2f} m/s, nonzero (>0.01) samples {int((sp > 0.01).sum())}/{ok.sum()}")
