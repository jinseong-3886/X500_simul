#!/usr/bin/env python3
"""PX4 (OFFBOARD) port of sim_gazebo/fly_mission.py — the Jetson follow mission.

WHY: the real FC (KOA FC H743) runs PX4 v1.16, while fly_mission.py was written for ArduPilot
(GUIDED mode, set_mode_apm). See docs/JETSON_STATUS.md §6. Decision 2026-11-13: keep PX4,
port the FC-control layer to OFFBOARD. The state machine and control laws are UNCHANGED:

  TAKEOFF -> SEARCH (yaw-sweep until YOLO sees the target class)
          -> FOLLOW (yaw-centre the box + hold standoff D0 from box height)
          -> SAFETY_HALT (too close)

What changed vs. the ArduPilot version (FC layer only):
  * GUIDED            -> OFFBOARD (MAV_CMD_DO_SET_MODE, PX4 main mode 6)
  * PX4 rule: setpoints must stream continuously (>2 Hz) before AND during OFFBOARD, otherwise
    PX4 leaves OFFBOARD (failsafe). A background thread streams the latest setpoint at 20 Hz.
  * Takeoff: no NAV_TAKEOFF (that switches PX4 to AUTO.TAKEOFF and drops OFFBOARD); instead an
    OFFBOARD position setpoint at z = -HOVER_Z.
  * Follow commands: body-frame vx + yaw_rate are rotated into LOCAL_NED using the current yaw,
    and altitude is held with a z position setpoint (xy velocity + z position + yaw_rate).
  * Land: MAV_CMD_NAV_LAND (PX4 AUTO.LAND) instead of set_mode_apm("LAND").
  * If the pilot switches mode on the RC (leaves OFFBOARD), the script stops commanding at once
    and never tries to re-enter OFFBOARD.

Additions (logged, and one safety layer):
  * Camera: RealSense D435i (colour + aligned depth) or a frame directory (Gazebo /tmp/fpv_frames).
  * Independent depth: median D435i depth inside the target box is logged next to the box-based
    'perceived' distance; DEPTH-DECEIT flag when they disagree (the defense idea, see README).
  * Depth safety halt (on by default, --no-depth-halt to disable): SAFETY_HALT if the D435i depth
    to the target is below D_SAFE, so a patch that shrinks the box cannot pull the drone into it.

Modes:
  default      DRY RUN: no FC at all. Camera + YOLO + state machine, prints the commands it WOULD send.
  --live       real flight / SITL: connects, enters OFFBOARD, ARMS and flies. Props on = danger.

Examples (on the Jetson):
  python3 fly_mission_px4.py --target bus                                   # dry run, D435i
  python3 fly_mission_px4.py --target bus --live --conn /dev/ttyACM0        # real FC over USB
  python3 fly_mission_px4.py --target bus --live --conn udpin:0.0.0.0:14540 --cam dir:/tmp/fpv_frames  # PX4 SITL
"""
import argparse, glob, math, os, sys, threading, time
import numpy as np, cv2

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--target", default="bus", help="COCO class to follow (default bus)")
ap.add_argument("--live", action="store_true", help="actually connect, arm and fly (default: dry run)")
ap.add_argument("--conn", default="/dev/ttyACM0", help="MAVLink endpoint (serial dev or udpin:0.0.0.0:14540)")
ap.add_argument("--baud", type=int, default=115200)
ap.add_argument("--cam", default="realsense", help="'realsense', 'gz' (Flytrap Gazebo sim) or 'dir:/path/to/frames'")
ap.add_argument("--tgt-h", type=float, default=None,
                help="real target height [m] (A2 board = 0.396). If set, box distance = fy*tgt_h/box_h "
                     "(camera intrinsics) instead of the legacy 'first sighting = D0' calibration")
ap.add_argument("--fy", type=float, default=607.0, help="camera focal length [px] (D435i colour 640x480 = 607)")
ap.add_argument("--mocap", default="none",
                help="external position source sent to PX4 as VISION_POSITION_ESTIMATE: "
                     "'none' | 'gz:WORLD:MODEL' (sim ground truth as fake mocap)")
ap.add_argument("--board-ned", default="", help="target board position 'N,E' in the mocap/local NED frame [m] "
                                                 "-> mocap geofence: SAFETY_HALT when camera-board < D_SAFE")
ap.add_argument("--fence", default="", help="flight box 'Nmin,Nmax,Emin,Emax,Zmax_up' [m] in mocap NED frame "
                                             "-> SAFETY_HALT when outside")
ap.add_argument("--save-frames", default="", help="dir to save annotated frames every loop (for video/debug)")
ap.add_argument("--model", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "yolov8n.pt"))
ap.add_argument("--hover-z", type=float, default=1.0, help="takeoff/hold altitude [m]")
ap.add_argument("--d0", type=float, default=1.5, help="standoff distance to hold [m]")
ap.add_argument("--d-safe", type=float, default=0.7, help="safety halt distance [m]")
ap.add_argument("--t-max", type=float, default=90.0, help="mission time limit [s]")
ap.add_argument("--no-depth-halt", action="store_true", help="disable the D435i depth safety halt")
ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "outputs", "px4"))
args = ap.parse_args()

# --- control constants: identical to fly_mission.py (tuned for ~3 m mocap cube, ~0.6 m target) ---
TGT_H_M = 0.6; HOVER_Z = args.hover_z; D0 = args.d0; D_SAFE = args.d_safe
KP = 0.5; VMAX = 0.45; YAW_KP = 0.004; YAW_SEARCH = 0.45; YAW_MAX = 0.8
CENTER_TOL_PX = 60; DT = 0.25; T_MAX = args.t_max
DECEIT_TAU = 0.15            # |perceived/depth - 1| above this -> depth-deceit flag
SP_HZ = 20                   # setpoint streaming rate (PX4 needs > 2 Hz)
TARGET_CLASS = args.target
LAST_BOX = None
CAM_FWD = 0.12               # camera ahead of body centre [m] (mount)
APPROACH_K = 0.8             # near the board: forward speed <= APPROACH_K * (mocap distance - D_SAFE) [1/s]
BOARD_NED = tuple(float(v) for v in args.board_ned.split(",")) if args.board_ned else None
FENCE = tuple(float(v) for v in args.fence.split(",")) if args.fence else None
if args.save_frames:
    os.makedirs(args.save_frames, exist_ok=True)


# ============================== perception ==============================
class RealSenseCam:
    def __init__(self):
        import pyrealsense2 as rs
        self.rs = rs
        self.pipe, cfg = rs.pipeline(), rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        prof = self.pipe.start(cfg)
        self.scale = prof.get_device().first_depth_sensor().get_depth_scale()
        self.align = rs.align(rs.stream.color)
        for _ in range(15):
            self.pipe.wait_for_frames()

    def read(self):
        fs = self.align.process(self.pipe.wait_for_frames())
        c, d = fs.get_color_frame(), fs.get_depth_frame()
        if not c or not d:
            return None, None
        return np.asanyarray(c.get_data()), np.asanyarray(d.get_data()).astype(np.float32) * self.scale

    def close(self):
        self.pipe.stop()


class DirCam:
    """Newest JPEG in a directory (Gazebo FPV frames) — same as the original newest()."""
    def __init__(self, path):
        self.path = path

    def read(self):
        fs = glob.glob(self.path + "/*.jpg")
        if not fs:
            return None, None
        f = max(fs, key=os.path.getmtime)
        for _ in range(3):
            im = cv2.imread(f)
            if im is not None and im.size:
                return im, None
            time.sleep(0.02)
        return None, None

    def close(self):
        pass


class GzCam:
    """Gazebo Harmonic RGB-D camera via gz-transport (sim: x500_d435i model, topics flytrap/color + flytrap/depth)."""
    def __init__(self, color_topic="flytrap/color", depth_topic="flytrap/depth"):
        from gz.transport13 import Node
        from gz.msgs10.image_pb2 import Image
        self.lock = threading.Lock(); self.c = None; self.d = None
        self.node = Node()
        if not self.node.subscribe(Image, color_topic, self._on_color):
            sys.exit(f"cannot subscribe {color_topic}")
        self.node.subscribe(Image, depth_topic, self._on_depth)
        if not wait_for(lambda: self.c is not None, 10):
            sys.exit(f"no frames on {color_topic} — is the Flytrap gz world running?")

    def _on_color(self, msg):
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        with self.lock:
            self.c = cv2.cvtColor(a, cv2.COLOR_RGB2BGR)

    def _on_depth(self, msg):
        a = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width).copy()
        a[~np.isfinite(a)] = 0.0
        with self.lock:
            self.d = a

    def read(self):
        with self.lock:
            return (None if self.c is None else self.c.copy()), (None if self.d is None else self.d.copy())

    def close(self):
        pass


def detect(m, img, depth_m):
    """(seen, box_h_px, box_cx_px, conf, img_w, depth_m_at_target) for the best TARGET_CLASS box."""
    r = m(img, verbose=False)[0]; best = None
    for b in r.boxes:
        if r.names[int(b.cls)] == TARGET_CLASS:
            x1, y1, x2, y2 = [float(v) for v in b.xyxy[0]]; c = float(b.conf[0])
            if best is None or c > best[0]:
                best = (c, (x1, y1, x2, y2))
    w = img.shape[1]
    global LAST_BOX
    LAST_BOX = None if best is None else best[1]
    if best is None:
        return False, 0.0, w / 2, 0.0, w, None
    c, (x1, y1, x2, y2) = best
    z = None
    if depth_m is not None:
        bw, bh = x2 - x1, y2 - y1
        roi = depth_m[int(y1 + bh / 4):int(y2 - bh / 4), int(x1 + bw / 4):int(x2 - bw / 4)]
        v = roi[roi > 0]
        z = float(np.median(v)) if v.size else None
    return True, y2 - y1, (x1 + x2) / 2, c, w, z


# ============================== mocap (external position) ==============================
# ENU world / FLU body (Gazebo, most mocap setups after axis mapping)  ->  NED world / FRD body (PX4)
_ENU2NED = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float)
_FLU2FRD = np.diag([1.0, -1.0, -1.0])


def quat_to_R(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def enu_pose_to_ned(p_enu, R_enu):
    """position (3,) + rotation body(FLU)->world(ENU)  ->  NED position + (roll, pitch, yaw) of FRD body."""
    p = _ENU2NED @ np.asarray(p_enu, float)
    R = _ENU2NED @ R_enu @ _FLU2FRD
    roll = math.atan2(R[2, 1], R[2, 2])
    pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    yaw = math.atan2(R[1, 0], R[0, 0])
    return p, (roll, pitch, yaw)


class MocapGz:
    """Sim 'mocap': model pose from Gazebo (/world/W/pose/info), i.e. ground truth, like a mocap system gives."""
    def __init__(self, world, model):
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
        self.model = model; self.lock = threading.Lock(); self.pose = None; self.t = 0.0
        self.node = Node()
        if not self.node.subscribe(Pose_V, f"/world/{world}/pose/info", self._cb):
            sys.exit(f"cannot subscribe /world/{world}/pose/info")

    def _cb(self, msg):
        for p in msg.pose:
            if p.name == self.model:
                R = quat_to_R(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)
                ned = enu_pose_to_ned((p.position.x, p.position.y, p.position.z), R)
                with self.lock:
                    self.pose = ned; self.t = time.time()
                return

    def get(self):
        """((n, e, d), (roll, pitch, yaw)) in NED, or None if stale (> 0.2 s)."""
        with self.lock:
            return self.pose if (self.pose is not None and time.time() - self.t < 0.2) else None


def make_mocap(spec):
    if spec == "none":
        return None
    kind, *rest = spec.split(":")
    if kind == "gz" and len(rest) == 2:
        return MocapGz(rest[0], rest[1])
    sys.exit(f"unknown --mocap '{spec}' (lab mocap input is added once the system/brand is known)")


# ============================== PX4 link ==============================
class PX4Link:
    """Receiver thread (state) + sender thread (20 Hz setpoint stream + 1 Hz companion heartbeat)."""
    MASK_POS_YAW = 8 | 16 | 32 | 64 | 128 | 256 | 2048          # use x,y,z + yaw
    MASK_VXY_Z_YAWRATE = 1 | 2 | 32 | 64 | 128 | 256 | 1024      # use vx,vy + z + yaw_rate

    def __init__(self, conn, baud, mocap=None):
        from pymavlink import mavutil
        self.mu = mavutil
        self.mocap = mocap; self.t_start = time.time(); self.mocap_sent = 0
        self.tripped = None       # fast mocap geofence: reason string once tripped (latched)
        print(f"[link] connecting {conn}")
        self.m = mavutil.mavlink_connection(conn, baud=baud, source_system=245, source_component=191)
        hb = self.m.wait_heartbeat(timeout=10)
        if hb is None:
            sys.exit("[link] NO HEARTBEAT — check cable/port")
        if hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_PX4:
            sys.exit(f"[link] autopilot is not PX4 (autopilot={hb.autopilot}) — this script is PX4-only")
        print(f"[link] PX4 heartbeat sys {self.m.target_system}")
        self.lock = threading.Lock()
        self.pos = None; self.yaw = None; self.main_mode = None; self.armed = False
        self.pos_ok = False       # EKF has valid horizontal velocity + relative position
        self.sp = None            # (mask, x, y, z, vx, vy, yaw, yaw_rate)
        self.streaming = False
        self.alive = True
        self.last_text = ""
        threading.Thread(target=self._rx, daemon=True).start()
        for mid, hz in ((32, 20), (30, 20), (230, 2)):  # LOCAL_POSITION_NED, ATTITUDE, ESTIMATOR_STATUS
            self._cmd(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, mid, 1e6 / hz)
        threading.Thread(target=self._tx, daemon=True).start()
        if self.mocap is not None:
            threading.Thread(target=self._mocap_tx, daemon=True).start()

    def _mocap_tx(self):
        """Forward mocap pose to PX4 at 50 Hz as VISION_POSITION_ESTIMATE (NED / FRD)."""
        cov = [float("nan")] + [0.0] * 20      # NaN -> PX4 uses EKF2_EVP_NOISE / EKF2_EVA_NOISE
        while self.alive:
            pose = self.mocap.get()
            if pose is not None:
                (n, e, d), (r, p, y) = pose
                with self.lock:
                    self.m.mav.vision_position_estimate_send(int((time.time() - self.t_start) * 1e6),
                                                             n, e, d, r, p, y, cov, 0)
                self.mocap_sent += 1
                self._fast_fence(pose)
            time.sleep(0.02)

    def mocap_geometry(self, pose=None):
        """(camera->board distance or None, (n, e, d), yaw) from the latest mocap pose, or None."""
        pose = pose if pose is not None else (self.mocap.get() if self.mocap else None)
        if pose is None:
            return None
        (n, e, d), (_, _, yaw) = pose
        cn, ce = n + CAM_FWD * math.cos(yaw), e + CAM_FWD * math.sin(yaw)
        dist = math.hypot(BOARD_NED[0] - cn, BOARD_NED[1] - ce) if BOARD_NED is not None else None
        return dist, (n, e, d), yaw

    def _fast_fence(self, pose):
        """50 Hz geofence: on breach immediately replace the setpoint with 'zero velocity, hold altitude'
        (the 4 Hz mission loop + YOLO latency let the drone overshoot by ~0.2 m in the sim)."""
        if self.tripped or not self.streaming or self.sp is None:
            return
        dist, (n, e, d), _ = self.mocap_geometry(pose)
        why = None
        if dist is not None and dist < D_SAFE:
            why = f"camera-board {dist:.2f}m < {D_SAFE}m"
        elif FENCE is not None:
            nmin, nmax, emin, emax, zup = FENCE
            if not (nmin <= n <= nmax and emin <= e <= emax and -d <= zup):
                why = f"outside flight box at N{n:.2f} E{e:.2f} up{-d:.2f}"
        if why:
            self.tripped = why
            z = self.sp[3]
            self.sp = (self.MASK_VXY_Z_YAWRATE, 0, 0, z, 0, 0, 0, 0)
            return
        # 50 Hz approach limit: horizontal speed <= APPROACH_K * (distance - D_SAFE)
        if dist is not None and self.sp[0] == self.MASK_VXY_Z_YAWRATE:
            mask, x, y, z, vn, ve, yw, yr = self.sp
            vmax = max(0.0, APPROACH_K * (dist - D_SAFE))
            sp_speed = math.hypot(vn, ve)
            if sp_speed > vmax:
                s = vmax / sp_speed
                self.sp = (mask, x, y, z, vn * s, ve * s, yw, yr)

    def _cmd(self, cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
        with self.lock:
            self.m.mav.command_long_send(self.m.target_system, self.m.target_component, cmd, 0,
                                         p1, p2, p3, p4, p5, p6, p7)

    def _rx(self):
        while self.alive:
            msg = self.m.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            t = msg.get_type()
            if t == "LOCAL_POSITION_NED":
                self.pos = (msg.x, msg.y, msg.z)
            elif t == "ATTITUDE":
                self.yaw = msg.yaw
            elif t == "ESTIMATOR_STATUS":
                need = self.mu.mavlink.ESTIMATOR_VELOCITY_HORIZ | self.mu.mavlink.ESTIMATOR_POS_HORIZ_REL
                self.pos_ok = (msg.flags & need) == need
            elif t == "HEARTBEAT" and msg.get_srcSystem() == self.m.target_system and msg.get_srcComponent() == 1:
                self.main_mode = (msg.custom_mode >> 16) & 0xff
                self.armed = bool(msg.base_mode & self.mu.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif t == "STATUSTEXT":
                self.last_text = msg.text
                print(f"[px4] {msg.text}")

    def _tx(self):
        k = 0
        while self.alive:
            with self.lock:
                if k % SP_HZ == 0:
                    self.m.mav.heartbeat_send(self.mu.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                              self.mu.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                if self.streaming and self.sp is not None:
                    mask, x, y, z, vx, vy, yaw, yr = self.sp
                    self.m.mav.set_position_target_local_ned_send(
                        0, self.m.target_system, self.m.target_component,
                        self.mu.mavlink.MAV_FRAME_LOCAL_NED, mask,
                        x, y, z, vx, vy, 0, 0, 0, 0, yaw, yr)
            k += 1
            time.sleep(1.0 / SP_HZ)

    # --- setpoint helpers ---
    def hold_position(self, x, y, z, yaw):
        self.sp = (self.MASK_POS_YAW, x, y, z, 0, 0, yaw, 0)

    def body_velocity(self, vx_body, yaw_rate, z_hold):
        if self.tripped:                       # latched geofence: never move again, just hold
            vx_body, yaw_rate = 0.0, 0.0
        yaw = self.yaw or 0.0
        vn, ve = vx_body * math.cos(yaw), vx_body * math.sin(yaw)
        self.sp = (self.MASK_VXY_Z_YAWRATE, 0, 0, z_hold, vn, ve, 0, yaw_rate)

    def offboard(self):
        return self.main_mode == 6

    def set_offboard(self):
        self._cmd(self.mu.mavlink.MAV_CMD_DO_SET_MODE, 1, 6)   # custom mode enabled, PX4 main=OFFBOARD

    def arm(self, on=True):
        self._cmd(self.mu.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1 if on else 0)

    def land(self):
        self._cmd(self.mu.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, float("nan"), float("nan"), float("nan"), float("nan"))

    def close(self):
        self.alive = False


def wait_for(cond, timeout, every=0.1):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(every)
    return False


# ============================== mission ==============================
def main():
    os.makedirs(args.out, exist_ok=True)
    mode_txt = "LIVE (will ARM and FLY)" if args.live else "DRY RUN (no FC, nothing is sent)"
    print(f"=== fly_mission_px4 | {mode_txt} | target '{TARGET_CLASS}' | D0 {D0} D_SAFE {D_SAFE} HOVER_Z {HOVER_Z}")
    from ultralytics import YOLO
    print("load YOLO"); m = YOLO(args.model)
    m(np.zeros((480, 640, 3), np.uint8), verbose=False)   # CUDA warm-up (first call takes ~4 s)
    if args.cam == "realsense":
        cam = RealSenseCam()
    elif args.cam == "gz":
        cam = GzCam()
    else:
        cam = DirCam(args.cam.split(":", 1)[1])

    link = None; z_hold = -HOVER_Z
    try:
        if args.live:
            mocap = make_mocap(args.mocap)
            if mocap is not None:
                print(f"[mocap] source {args.mocap}: waiting for poses")
                if not wait_for(lambda: mocap.get() is not None, 10):
                    sys.exit("[abort] no mocap poses")
            link = PX4Link(args.conn, args.baud, mocap)
            # NOTE: PX4 publishes LOCAL_POSITION_NED even when x/y are NOT valid (e.g. indoors, no GPS),
            # so the estimator flags are the real check.
            if not wait_for(lambda: link.pos is not None and link.yaw is not None and link.pos_ok, 25):
                sys.exit("[abort] PX4 has no valid horizontal position/velocity (estimator flags) — "
                         "OFFBOARD impossible. Check position source (mocap / optical flow / GPS).")
            x0, y0, zg = link.pos; yaw0 = link.yaw
            z_hold = zg - HOVER_Z
            print(f"[link] local position ok: x {x0:.2f} y {y0:.2f} z {zg:.2f} yaw {math.degrees(yaw0):.0f} deg")
            # 1) stream a hold setpoint BEFORE switching (PX4 rejects OFFBOARD without a stream)
            link.hold_position(x0, y0, zg, yaw0); link.streaming = True; time.sleep(1.0)
            # 2) OFFBOARD
            for _ in range(5):
                link.set_offboard()
                if wait_for(link.offboard, 1.5):
                    break
            if not link.offboard():
                sys.exit(f"[abort] PX4 refused OFFBOARD (last msg: '{link.last_text}')")
            print("[link] OFFBOARD")
            # 3) ARM
            for _ in range(5):
                link.arm(True)
                if wait_for(lambda: link.armed, 2):
                    break
            if not link.armed:
                sys.exit(f"[abort] arming refused (last msg: '{link.last_text}')")
            print("[link] ARMED -> takeoff")
            # 4) takeoff = position setpoint at HOVER_Z above ground
            link.hold_position(x0, y0, z_hold, yaw0)
            ok = wait_for(lambda: link.pos and abs(link.pos[2] - z_hold) < 0.15, 15)
            if not link.offboard():
                print("[pilot] left OFFBOARD during takeoff -> handing over"); return
            print("[link] at hover altitude" if ok else "[warn] altitude not reached in 15 s, continuing")

        state = "SEARCH"; f_px = None; deceived_t = None; depth_deceit_t = None; log = []
        t_start = time.time()
        print(f"SEARCH: yaw-sweep for '{TARGET_CLASS}'")
        while time.time() - t_start < T_MAX:
            t = time.time() - t_start
            if link and not link.offboard():
                print(f"[{t:.1f}] [pilot] OFFBOARD left (RC override / failsafe) -> stop commanding")
                state = "PILOT"; break
            img, depth_m = cam.read()
            seen, bh, cx, conf, w, z_tgt = (False, 0, 0, 0, 640, None) if img is None else detect(m, img, depth_m)
            vx = 0.0; yaw_rate = 0.0; perceived = None

            if state == "SEARCH":
                yaw_rate = YAW_SEARCH
                if seen and bh > 4:
                    state = "FOLLOW"; print(f"[{t:.1f}] ACQUIRED ({TARGET_CLASS} conf {conf:.2f}) -> FOLLOW")
            elif state == "FOLLOW":
                if not seen or bh <= 4:
                    state = "SEARCH"
                else:
                    if f_px is None:
                        if args.tgt_h:   # intrinsics: works wherever the target is first seen
                            f_px = args.fy * args.tgt_h / TGT_H_M
                            print(f"[{t:.1f}] intrinsics: fy={args.fy:.0f}px, target {args.tgt_h}m")
                        else:            # legacy (fly_mission.py): assumes first sighting is at D0
                            f_px = D0 * bh / TGT_H_M; print(f"[{t:.1f}] calib f_px={f_px:.0f}")
                    perceived = f_px * TGT_H_M / max(bh, 1e-3)
                    err_px = cx - w / 2           # +ve: box right of centre
                    # FIX vs fly_mission.py: MAVLink/PX4 yaw_rate is NED (+ = clockwise = turn RIGHT).
                    # The original used -YAW_KP*err (turns away from the target); found in the PX4 gz sim.
                    yaw_rate = float(np.clip(YAW_KP * err_px, -YAW_MAX, YAW_MAX))
                    if abs(err_px) < CENTER_TOL_PX:
                        vx = float(np.clip(KP * (perceived - D0), -VMAX, VMAX))
                    if perceived > D0 * 1.6 and deceived_t is None:
                        deceived_t = t; print(f"[{t:.1f}] DECEIT FLAG: box says {perceived:.1f}m")
                    if z_tgt and abs(perceived / z_tgt - 1) > DECEIT_TAU and depth_deceit_t is None:
                        depth_deceit_t = t
                        print(f"[{t:.1f}] DEPTH-DECEIT: box says {perceived:.2f}m, D435i depth {z_tgt:.2f}m")
                    if conf > 0 and bh > (f_px * TGT_H_M / D_SAFE):
                        state = "SAFETY_HALT"; print(f"[{t:.1f}] SAFETY_HALT (too close by box size)")
                    elif z_tgt and z_tgt < D_SAFE and not args.no_depth_halt:
                        state = "SAFETY_HALT"; print(f"[{t:.1f}] SAFETY_HALT (D435i depth {z_tgt:.2f}m < {D_SAFE}m)")
            elif state == "SAFETY_HALT":
                vx = 0.0; yaw_rate = 0.0

            # ---- mocap geofence (README §7: 1st safety layer; kill switch is the 2nd) ----
            # (a) the 50 Hz guard in PX4Link already froze the setpoint on breach -> mirror it in the state
            # (b) approach speed limit near the board so there is nothing to overshoot with
            mc_dist = None
            geo = link.mocap_geometry() if (link and link.mocap) else None
            if geo is not None:
                mc_dist = geo[0]
                if link.tripped and state != "SAFETY_HALT":
                    state = "SAFETY_HALT"; vx = 0.0; yaw_rate = 0.0
                    print(f"[{t:.1f}] SAFETY_HALT (mocap geofence: {link.tripped})")
                if mc_dist is not None and vx > 0:
                    vx = min(vx, max(0.0, APPROACH_K * (mc_dist - D_SAFE)))

            if link:
                link.body_velocity(vx, yaw_rate, z_hold)
            if args.save_frames and img is not None:
                vis = img.copy()
                if LAST_BOX is not None:
                    x1, y1, x2, y2 = [int(v) for v in LAST_BOX]
                    cv2.rectangle(vis, (x1, y1), (x2, y2), (255, 0, 255), 2)
                txt = f"t={t:4.1f}s {state} vx={vx:+.2f} yaw={yaw_rate:+.2f}"
                txt2 = (f"box={'%.2f' % perceived if perceived else '-'}m depth={'%.2f' % z_tgt if z_tgt else '-'}m"
                        + ("  DECEIT" if deceived_t is not None else ""))
                for k, s in enumerate((txt, txt2)):
                    cv2.putText(vis, s, (8, 22 + 24 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
                    cv2.putText(vis, s, (8, 22 + 24 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                cv2.imwrite(os.path.join(args.save_frames, f"f{len(log):04d}.jpg"), vis)
            log.append((t, bh, cx, conf, vx, yaw_rate,
                        perceived if perceived else np.nan, z_tgt if z_tgt else np.nan, state))
            print(f"[{t:.1f}] {state:<11s} bh={bh:.0f} cx={cx:.0f} conf={conf:.2f} "
                  f"depth={'%.2f' % z_tgt if z_tgt else '-'} -> vx={vx:.2f} yaw_rate={yaw_rate:.2f}"
                  + ("" if link else "  (dry run: not sent)"))
            if state == "SAFETY_HALT" and len(log) >= 8 and all(r[8] == "SAFETY_HALT" for r in log[-8:]):
                break
            time.sleep(DT)

        if log:
            path = os.path.join(args.out, time.strftime("fly_px4_%Y%m%d_%H%M%S.csv"))
            np.savetxt(path, np.array([r[:8] for r in log], dtype=float), delimiter=",",
                       header="t,box_h,cx,conf,vx,yaw_rate,perceived,depth", comments="")
            print(f"log -> {path}")
            print(f"RESULT: final_state={state} deceit_flag={deceived_t is not None} "
                  f"depth_deceit={depth_deceit_t is not None}")
    except KeyboardInterrupt:
        print("\n[ctrl-c]")
    finally:
        if link:
            if link.offboard() and link.armed:
                print("[link] LAND"); link.land()
                wait_for(lambda: not link.offboard(), 3)
            link.streaming = False
            # keep the mocap stream alive until the vehicle is on the ground and disarmed —
            # stopping it mid-landing makes PX4 lose position and fall back to DESCEND (seen in the sim)
            if link.armed:
                # pilot took over on the RC -> they may fly for minutes in Position mode, which needs mocap:
                # keep streaming until disarm (Ctrl-C again to force quit). Auto LAND normally takes < 40 s.
                limit = 40 if link.mocap is None or link.offboard() or link.main_mode == 4 else 1800
                print(f"[link] waiting for touchdown/disarm (mocap keeps streaming, up to {limit} s)")
                if wait_for(lambda: not link.armed, limit):
                    print("[link] disarmed")
                else:
                    print("[link] WARNING: still armed after 40 s")
            time.sleep(0.5); link.close()
        cam.close()


if __name__ == "__main__":
    main()
    if args.cam == "gz":
        sys.stdout.flush()
        os._exit(0)   # skip gz-transport/pybind11 teardown (it aborts on interpreter exit)
