"""READ-ONLY recorder for the props-off kill test: armed state, motor outputs, SWA (ch7), FC messages.
Sends nothing but stream-rate requests. Usage: python3 -u arm_kill_log.py [--secs 120]
"""
import argparse, time
from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument("--dev", default="/dev/ttyACM0")
ap.add_argument("--secs", type=float, default=120)
a = ap.parse_args()
m = mavutil.mavlink_connection(a.dev, baud=115200, source_system=245)
m.wait_heartbeat(timeout=10)
for msg_id, us in ((36, 200000), (65, 200000), (1, 500000)):  # SERVO_OUTPUT_RAW, RC_CHANNELS, SYS_STATUS
    m.mav.command_long_send(m.target_system, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, us, 0, 0, 0, 0, 0)

t0 = time.time(); T = lambda: f"[{time.time() - t0:6.1f}s]"
armed = None; swa = None; motors = None; last_motor_print = 0
print(f"recording {a.secs:.0f}s ...")
while time.time() - t0 < a.secs:
    r = m.recv_match(blocking=True, timeout=1)
    if r is None:
        continue
    t = r.get_type()
    if t == "HEARTBEAT" and r.get_srcComponent() == 1 and r.type != mavutil.mavlink.MAV_TYPE_GCS:
        ar = bool(r.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if ar != armed:
            print(T(), "ARMED" if ar else "DISARMED"); armed = ar
    elif t == "RC_CHANNELS":
        s = "DOWN(kill)" if r.chan7_raw > 1750 else "UP" if r.chan7_raw < 1250 else f"{r.chan7_raw}"
        if s != swa:
            print(T(), f"SWA {s}  (throttle ch3={r.chan3_raw})"); swa = s
    elif t == "SERVO_OUTPUT_RAW" and r.port == 0:
        mo = (r.servo1_raw, r.servo2_raw, r.servo3_raw, r.servo4_raw)
        spinning = max(mo) > 1100 if max(mo) > 0 else False
        key = (spinning,)
        if motors is None or key != motors or (spinning and time.time() - last_motor_print > 2):
            print(T(), "motors", mo, "SPINNING" if spinning else "stopped"); motors = key; last_motor_print = time.time()
    elif t == "STATUSTEXT":
        print(T(), "FC:", r.text)
print("done")
