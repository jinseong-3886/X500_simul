"""READ-ONLY RC check: which transmitter switch is on which channel, and what the flight-mode switch does.
Never writes params. Usage (Jetson): python3 -u rc_check.py [--secs 0] [--dev /dev/ttyACM0]
  --secs 0  : snapshot (params + current channel values)
  --secs N  : also watch N seconds and print every channel that moves (flip switches one at a time)
"""
import argparse, struct, time
from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument("--dev", default="/dev/ttyACM0")
ap.add_argument("--baud", type=int, default=115200)
ap.add_argument("--secs", type=float, default=0)
a = ap.parse_args()

m = mavutil.mavlink_connection(a.dev, baud=a.baud, source_system=245)
hb = m.wait_heartbeat(timeout=10)
if not hb:
    raise SystemExit("no heartbeat from FC")
print(f"FC sys {m.target_system} ok")

MAIN = {1: "MANUAL", 2: "ALTCTL(Altitude)", 3: "POSCTL(Position)", 4: "AUTO", 5: "ACRO",
        6: "OFFBOARD", 7: "STABILIZED", 8: "RATTITUDE"}
FLT = {-1: "-", 0: "Manual", 1: "Altitude", 2: "Position", 3: "Mission", 4: "Hold", 5: "Return",
       6: "Acro", 7: "Offboard", 8: "Stabilized", 10: "Takeoff", 11: "Land", 12: "FollowMe", 13: "PrecLand"}

def param(name):
    for _ in range(3):
        m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
        t0 = time.time()
        while time.time() - t0 < 1.5:
            r = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if r and r.param_id.rstrip("\x00") == name:
                if r.param_type in (1, 2, 3, 4, 5, 6):  # integer types arrive bytewise in the float
                    return struct.unpack("<i", struct.pack("<f", r.param_value))[0]
                return r.param_value
    return None

names = ["RC_MAP_FLTMODE", "RC_MAP_KILL_SW", "RC_MAP_ARM_SW", "RC_MAP_RETURN_SW", "RC_MAP_OFFB_SW",
         "COM_FLTMODE1", "COM_FLTMODE2", "COM_FLTMODE3", "COM_FLTMODE4", "COM_FLTMODE5", "COM_FLTMODE6",
         "RC_CHAN_CNT", "COM_RC_IN_MODE"]
P = {n: param(n) for n in names}
for n in names:
    v = P[n]
    extra = f"  ({FLT.get(v, v)})" if n.startswith("COM_FLTMODE") and v is not None else ""
    print(f"  {n:16s} = {v}{extra}")
print("  flight-mode slot by switch PWM: ~1000us->slot1, ~1500us->slot4, ~2000us->slot6")

# ask for RC_CHANNELS (id 65) at 10 Hz
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 65, 100000, 0, 0, 0, 0, 0)

def chans(r):
    return [getattr(r, f"chan{i}_raw") for i in range(1, 19)][: max(r.chancount, 8)]

r = m.recv_match(type="RC_CHANNELS", blocking=True, timeout=5)
if not r:
    raise SystemExit("no RC_CHANNELS message from FC")
mode = MAIN.get((m.messages["HEARTBEAT"].custom_mode >> 16) & 0xFF, "?") if "HEARTBEAT" in m.messages else "?"
print(f"\nRC: chancount={r.chancount} rssi={r.rssi}  current mode={mode}")
if r.chancount == 0:
    print("  -> FC sees NO receiver signal (transmitter off / not bound / wiring).")
print("  now: " + "  ".join(f"ch{i+1}={v}" for i, v in enumerate(chans(r))))

if a.secs > 0:
    print(f"\nwatching {a.secs:.0f}s - flip ONE switch at a time ...")
    last = chans(r); last_mode = mode; t_end = time.time() + a.secs
    while time.time() < t_end:
        r = m.recv_match(type=["RC_CHANNELS", "HEARTBEAT"], blocking=True, timeout=1)
        if r is None:
            continue
        if r.get_type() == "HEARTBEAT":
            if r.get_srcComponent() != 1 or r.type == mavutil.mavlink.MAV_TYPE_GCS:
                continue
            md = MAIN.get((r.custom_mode >> 16) & 0xFF, "?")
            if md != last_mode:
                print(f"  [{a.secs - (t_end - time.time()):5.1f}s] FC mode -> {md}")
                last_mode = md
            continue
        c = chans(r)
        for i, (o, n) in enumerate(zip(last, c)):
            if abs(n - o) > 150:
                print(f"  [{a.secs - (t_end - time.time()):5.1f}s] ch{i+1}: {o} -> {n}")
                last[i] = n
    print("done")
