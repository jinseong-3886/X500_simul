"""READ-ONLY PX4 flight-controller check over MAVLink (no arming, no param writes, no mode changes).

Usage (on Jetson): python3 fc_check.py [--dev /dev/ttyACM0] [--baud 115200]
Reports: firmware/version, current mode & armed state, battery, GPS, estimator/local position,
RC link, and the PX4 parameters that matter for OFFBOARD control and failsafes.
"""
import argparse, struct, time
from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument("--dev", default="/dev/ttyACM0")
ap.add_argument("--baud", type=int, default=115200)
a = ap.parse_args()

PARAMS = [
    # offboard behaviour / failsafe
    "COM_OF_LOSS_T", "COM_OBL_RC_ACT", "COM_RC_IN_MODE", "COM_RCL_EXCEPT", "NAV_RCL_ACT", "NAV_DLL_ACT",
    "COM_FAIL_ACT_T", "GF_ACTION", "COM_LOW_BAT_ACT", "BAT_LOW_THR", "BAT_CRIT_THR", "BAT_N_CELLS",
    # arming
    "COM_ARM_WO_GPS", "COM_DISARM_PRFLT", "COM_DISARM_LAND", "CBRK_USB_CHK", "COM_PREARM_MODE",
    # position source (indoor vs GPS)
    "EKF2_GPS_CTRL", "EKF2_EV_CTRL", "EKF2_OF_CTRL", "EKF2_HGT_REF", "EKF2_BARO_CTRL", "EKF2_RNG_CTRL",
    # links
    "MAV_0_CONFIG", "MAV_0_MODE", "MAV_1_CONFIG", "SER_TEL1_BAUD", "SER_TEL2_BAUD",
    # speed limits that cap offboard velocity commands
    "MPC_XY_VEL_MAX", "MPC_Z_VEL_MAX_UP", "MPC_Z_VEL_MAX_DN", "MPC_TKO_SPEED", "MIS_TAKEOFF_ALT",
    # RC mode switches / kill switch
    "RC_MAP_FLTMODE", "RC_MAP_KILL_SW", "RC_MAP_OFFB_SW", "RC_MAP_ARM_SW", "RC_MAP_RETURN_SW",
    "COM_FLTMODE1", "COM_FLTMODE2", "COM_FLTMODE3", "COM_FLTMODE4", "COM_FLTMODE5", "COM_FLTMODE6",
    "SYS_AUTOSTART",
]

PX4_MAIN = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO", 5: "ACRO", 6: "OFFBOARD", 7: "STABILIZED", 8: "RATTITUDE"}
PX4_AUTO = {1: "READY", 2: "TAKEOFF", 3: "LOITER", 4: "MISSION", 5: "RTL", 6: "LAND", 8: "FOLLOW", 9: "PRECLAND"}

print(f"connecting {a.dev} @ {a.baud} ...")
m = mavutil.mavlink_connection(a.dev, baud=a.baud, source_system=245)
hb = m.wait_heartbeat(timeout=10)
if hb is None:
    raise SystemExit("NO HEARTBEAT - check cable (data cable?) / port")
ap_name = {mavutil.mavlink.MAV_AUTOPILOT_PX4: "PX4", mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA: "ArduPilot"}.get(hb.autopilot, hb.autopilot)
print(f"heartbeat: sys {m.target_system} comp {m.target_component} | autopilot {ap_name} | type {hb.type}")

# firmware version
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0, 148, 0, 0, 0, 0, 0, 0)
v = m.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=3)
if v:
    fw = v.flight_sw_version
    print(f"firmware: {fw >> 24 & 0xff}.{fw >> 16 & 0xff}.{fw >> 8 & 0xff} (type {fw & 0xff})")

# stream data for a few seconds
m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
seen = {}
t0 = time.time()
while time.time() - t0 < 4:
    msg = m.recv_match(blocking=True, timeout=1)
    if msg is None:
        continue
    seen[msg.get_type()] = msg

hb = seen.get("HEARTBEAT", hb)
cm = hb.custom_mode
main, sub = (cm >> 16) & 0xff, (cm >> 24) & 0xff
mode = PX4_MAIN.get(main, main) + (("." + PX4_AUTO.get(sub, str(sub))) if main == 4 else "")
armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
print(f"\n== state: mode {mode} | armed {armed}")

s = seen.get("SYS_STATUS")
if s:
    print(f"== battery: {s.voltage_battery / 1000:.2f} V, remaining {s.battery_remaining}%  (USB-only power shows ~0 V)")
g = seen.get("GPS_RAW_INT")
if g:
    print(f"== GPS: fix_type {g.fix_type} (0-1 none, 3 = 3D), sats {g.satellites_visible}")
lp = seen.get("LOCAL_POSITION_NED")
print("== local position:", f"x {lp.x:.2f} y {lp.y:.2f} z {lp.z:.2f}" if lp else "NOT AVAILABLE (OFFBOARD needs a valid local position)")
es = seen.get("ESTIMATOR_STATUS")
if es:
    F = mavutil.mavlink
    flags = es.flags
    names = [("ATTITUDE", F.ESTIMATOR_ATTITUDE), ("VEL_HORIZ", F.ESTIMATOR_VELOCITY_HORIZ),
             ("POS_HORIZ_REL", F.ESTIMATOR_POS_HORIZ_REL), ("POS_HORIZ_ABS", F.ESTIMATOR_POS_HORIZ_ABS),
             ("POS_VERT_ABS", F.ESTIMATOR_POS_VERT_ABS), ("PRED_POS_HORIZ_REL", F.ESTIMATOR_PRED_POS_HORIZ_REL),
             ("GPS_GLITCH", F.ESTIMATOR_GPS_GLITCH), ("ACCEL_ERROR", F.ESTIMATOR_ACCEL_ERROR)]
    print("== estimator flags:", ", ".join(n for n, b in names if flags & b) or "(none)",
          "  <- OFFBOARD/POSCTL need VEL_HORIZ + POS_HORIZ_REL")
rc = seen.get("RC_CHANNELS")
print("== RC:", f"{rc.chancount} channels, rssi {rc.rssi}" if rc else "no RC_CHANNELS (transmitter off?)")
ess = seen.get("EXTENDED_SYS_STATE")
if ess:
    print(f"== landed_state {ess.landed_state} (1 = on ground)")
for t in ("STATUSTEXT",):
    if t in seen:
        print("== last status text:", seen[t].text)
print("messages seen:", ", ".join(sorted(seen)))

print("\n== parameters")
for p in PARAMS:
    m.mav.param_request_read_send(m.target_system, m.target_component, p.encode(), -1)
    r = None
    t1 = time.time()
    while time.time() - t1 < 1.5:
        r = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if r and r.param_id.rstrip("\x00") == p:
            break
        r = None
    if r is None:
        val = "(not found)"
    elif r.param_type == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
        val = round(r.param_value, 3)
    else:
        # PX4 sends integer params byte-wise inside the float field -> reinterpret the bits
        val = struct.unpack("<i", struct.pack("<f", r.param_value))[0]
    print(f"  {p:18s} {val}")
print("\nREAD-ONLY check done (nothing was changed on the FC).")
