"""Which sources did EKF2 actually fuse in the latest Flytrap ulog? (GPS vs external vision)"""
import glob, os, numpy as np
from pyulog import ULog
f = max(glob.glob(os.path.expanduser("~/flytrap_sim/rootfs/log/*/*.ulg")), key=os.path.getmtime)
u = ULog(f, ["estimator_status_flags"])
d = u.get_dataset("estimator_status_flags").data
print("log", os.path.basename(f), "| samples", len(d["timestamp"]))
for k in ("cs_gnss_pos", "cs_gps", "cs_gps_hgt", "cs_ev_pos", "cs_ev_hgt", "cs_ev_yaw", "cs_mag_hdg", "cs_mag_3d", "cs_baro_hgt"):
    if k in d:
        v = d[k].astype(bool)
        print(f"  {k:12s} fused {100 * v.mean():5.1f}% of the time")
