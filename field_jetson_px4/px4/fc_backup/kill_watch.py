"""READ-ONLY: poll PX4 uORB manual_control_switches.kill_switch via the MAVLink shell (no arming).
kill_switch: 0=not mapped, 1=ON (KILL), 3=OFF (normal).  Usage: python3 -u kill_watch.py [--secs 45]
"""
import argparse, re, time
from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument("--dev", default="/dev/ttyACM0")
ap.add_argument("--secs", type=float, default=45)
a = ap.parse_args()
m = mavutil.mavlink_connection(a.dev, baud=115200, source_system=245)
m.wait_heartbeat(timeout=10)
DEV = mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL
FL = mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND | mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE

def shell(cmd, wait=1.0):
    b = (cmd + "\n").encode()
    for i in range(0, len(b), 70):
        chunk = b[i:i + 70]
        m.mav.serial_control_send(DEV, FL, 0, 0, len(chunk), list(chunk) + [0] * (70 - len(chunk)))
    out, t_end = b"", time.time() + wait
    while time.time() < t_end:
        r = m.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.2)
        if r and r.count:
            out += bytes(r.data[: r.count])
        else:  # poll for more output
            m.mav.serial_control_send(DEV, FL, 0, 0, 0, [0] * 70)
    return out.decode(errors="replace")

shell("", 0.5)
NAME = {0: "not mapped", 1: "ON  -> KILL", 2: "middle", 3: "OFF -> normal"}
last, t_end = None, time.time() + a.secs
print(f"watching {a.secs:.0f}s ...")
while time.time() < t_end:
    o = shell("listener manual_control_switches", 1.2)
    k = re.search(r"kill_switch:\s*(\d+)", o)
    if k:
        v = int(k.group(1))
        if v != last:
            print(f"  [{a.secs - (t_end - time.time()):5.1f}s] kill_switch = {v} ({NAME.get(v, '?')})")
            last = v
    elif last is None:
        print("  (no kill_switch field yet) raw:", o.strip()[-200:]); last = -1
m.mav.serial_control_send(DEV, 0, 0, 0, 0, [0] * 70)  # release shell
print("done")
