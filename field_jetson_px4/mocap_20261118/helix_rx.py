"""HELIX(XEROmotion HALO) 강체 수신 모듈 + 수신 테스트.

테스트 실행 (노트북/Jetson 공통):
    python3 helix_rx.py -s 192.168.0.66              # 모든 강체를 5 Hz로 화면 출력 + CSV 저장
    python3 helix_rx.py -s 192.168.0.66 --fwd +x     # 드론 기준 NED 좌표(N=앞, E=오른쪽, D=아래)도 같이 출력

좌표 규칙 (HELIX: Z가 위, 단위 mm):
    --fwd 는 '드론 앞(보드 쪽)'이 HELIX의 어느 축 방향인지. +x / -x / +y / -y 중 하나.
    왼쪽은 앞에서 반시계 90도 (Z가 위인 오른손 좌표계라 자동으로 정해짐).
    N = 앞 방향 성분, E = 오른쪽 성분, D = -Z  (단위 m)
"""
import argparse, csv, math, os, threading, time

FWD = {"+x": (1.0, 0.0), "-x": (-1.0, 0.0), "+y": (0.0, 1.0), "-y": (0.0, -1.0)}


def helix_yaw(qx, qy, qz, qw):
    """Z-up 좌표계에서 Z축 기준 회전각 [rad] (반시계 +)."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def to_ned(x_m, y_m, z_m, fwd):
    """HELIX(m, Z-up) -> NED(m). fwd = FWD['+x'] 등."""
    fx, fy = fwd
    lx, ly = -fy, fx                       # 왼쪽 = 앞을 반시계 90도
    n = x_m * fx + y_m * fy
    e = -(x_m * lx + y_m * ly)
    return n, e, -z_m


def heading_ned(yaw_helix, fwd, offset=0.0):
    """HELIX yaw -> NED yaw (시계 +, 앞 방향 = 0). offset은 시작 시 자동 0점 맞춤 값."""
    a_f = math.atan2(fwd[1], fwd[0])
    return wrap(-(yaw_helix - a_f) - offset)


class HelixRx:
    """HELIX 서버에서 강체를 받아 최신 값을 보관. get(name_or_id) 로 꺼냄."""

    def __init__(self, server):
        from ctypes import POINTER
        from helix.helixsdk import PySDKClient, DataDescriptions, DataDescriptors
        self.c = PySDKClient()
        ret = self.c.Initialize(bytes(server, encoding="utf8"))
        if ret != 0:
            raise RuntimeError(f"HELIX 연결 실패 [{ret}] server={server}  (같은 와이파이인지, HELIX 스트리밍이 켜져 있는지 확인)")
        self.lock = threading.Lock()
        self.bodies = {}            # id -> dict
        self.frames = 0
        self.t_last = 0.0
        self.names = {}             # id -> name
        self.c.PySetVerbosityLevel(0)
        self.c.PySetDataCallback(self._on_frame, None)
        try:
            pdds = POINTER(DataDescriptions)()
            self.c.PyGetDataDescriptions(pdds)
            d = pdds.contents
            for i in range(d.nDataDescriptions):
                dd = d.arrDataDescriptions[i]
                if dd.type == DataDescriptors.Descriptor_RigidBody.value:
                    rb = dd.Data.RigidBodyDescription.contents
                    self.names[rb.ID] = rb.szName.decode("utf-8", "replace")
        except Exception as ex:      # 이름 목록은 없어도 ID로 동작
            print("[경고] 강체 이름 목록을 못 받음:", ex)

    def _on_frame(self, pframe, puser):
        if not pframe:
            return
        f = pframe.contents
        now = time.monotonic()
        with self.lock:
            self.frames += 1
            self.t_last = now
            for i in range(f.nRigidBodies):
                b = f.RigidBodies[i]
                self.bodies[b.ID] = dict(t=now, frame=f.iFrame, id=b.ID,
                                         x=b.x / 1000.0, y=b.y / 1000.0, z=b.z / 1000.0,
                                         qx=b.qx, qy=b.qy, qz=b.qz, qw=b.qw,
                                         nmk=b.nMarkers, err=b.MeanError, params=b.params)

    def get(self, key=None):
        """key: 강체 이름 또는 ID(문자열/정수). None이면 첫 번째 강체."""
        with self.lock:
            if not self.bodies:
                return None
            if key is None:
                return dict(self.bodies[sorted(self.bodies)[0]])
            for bid, b in self.bodies.items():
                if str(bid) == str(key) or self.names.get(bid) == key:
                    return dict(b)
        return None

    def all(self):
        with self.lock:
            return {k: dict(v) for k, v in self.bodies.items()}


def valid(b):
    """추적이 살아 있는지: 값이 숫자이고 (0,0,0) 이 아니어야 함 (추적 끊기면 0으로 오는 경우 대비)."""
    if b is None:
        return False
    v = (b["x"], b["y"], b["z"], b["qw"])
    if not all(math.isfinite(t) for t in v):
        return False
    return not (b["x"] == 0.0 and b["y"] == 0.0 and b["z"] == 0.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-s", "--server", default="192.168.0.66")
    ap.add_argument("--fwd", choices=list(FWD), default=None, help="드론 앞이 HELIX의 어느 축인지 (NED 출력용)")
    ap.add_argument("--out", default=os.path.expanduser("~/mocap_logs"))
    ap.add_argument("--hz", type=float, default=5.0, help="화면 출력 주기")
    a = ap.parse_args()

    rx = HelixRx(a.server)
    print(f"HELIX 연결 성공: {a.server}   강체 목록: {rx.names or '(이름 없음)'}")
    os.makedirs(a.out, exist_ok=True)
    path = os.path.join(a.out, time.strftime("helix_rx_%Y%m%d_%H%M%S.csv"))
    fcsv = open(path, "w", newline="")
    w = csv.writer(fcsv)
    w.writerow(["t", "frame", "id", "name", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw",
                "yaw_deg", "nmarkers", "mean_err", "params"])
    print("CSV 저장:", path, "\nCtrl+C 로 종료\n")

    t0 = time.monotonic(); last_print = 0.0; last_frames = 0; seen = {}
    try:
        while True:
            time.sleep(0.005)
            now = time.monotonic()
            for bid, b in rx.all().items():
                if seen.get(bid) == b["frame"]:
                    continue
                seen[bid] = b["frame"]
                yaw = math.degrees(helix_yaw(b["qx"], b["qy"], b["qz"], b["qw"]))
                w.writerow([f"{b['t'] - t0:.4f}", b["frame"], bid, rx.names.get(bid, ""),
                            f"{b['x']:.4f}", f"{b['y']:.4f}", f"{b['z']:.4f}",
                            f"{b['qx']:.5f}", f"{b['qy']:.5f}", f"{b['qz']:.5f}", f"{b['qw']:.5f}",
                            f"{yaw:.2f}", b["nmk"], f"{b['err']:.3f}", b["params"]])
            if now - last_print >= 1.0 / a.hz:
                rate = (rx.frames - last_frames) / max(now - last_print, 1e-3)
                last_frames = rx.frames; last_print = now
                age = now - rx.t_last if rx.t_last else float("inf")
                print(f"[{now - t0:6.1f}s] 수신 {rate:5.1f} Hz  마지막 프레임 {age * 1000:5.0f} ms 전")
                if not rx.bodies:
                    print("    강체 없음 - HELIX에서 강체(Rigid Body) 등록/스트리밍 확인")
                for bid, b in rx.all().items():
                    yaw = math.degrees(helix_yaw(b["qx"], b["qy"], b["qz"], b["qw"]))
                    ok = "추적중" if valid(b) else "끊김!"
                    s = (f"    [{bid}] {rx.names.get(bid, ''):10s} {ok}  x={b['x']:+.3f} y={b['y']:+.3f} "
                         f"z={b['z']:+.3f} m  yaw={yaw:+7.1f}°  마커 {b['nmk']}  오차 {b['err']:.2f}  params={b['params']}")
                    if a.fwd:
                        n, e, d = to_ned(b["x"], b["y"], b["z"], FWD[a.fwd])
                        s += f"\n           -> NED  앞(N)={n:+.3f}  오른쪽(E)={e:+.3f}  위={-d:+.3f} m"
                    print(s)
    except KeyboardInterrupt:
        pass
    finally:
        fcsv.close()
        print("\n종료. CSV:", path)


if __name__ == "__main__":
    main()
