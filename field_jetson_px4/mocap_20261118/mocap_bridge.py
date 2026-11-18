"""HELIX 모캡 위치 -> PX4 (VISION_POSITION_ESTIMATE, 최대 50 Hz) 브리지.

FC 시리얼(/dev/ttyACM0)은 이 프로그램이 혼자 잡고, 다른 프로그램(미션 코드, 노트북 QGC)에는 UDP로 중계함.

    # 기본: 강체 'drone', 드론 앞 = HELIX +x 방향, 노트북 QGC(192.168.0.xx)에도 중계
    python3 mocap_bridge.py -s 192.168.0.66 --body drone --fwd +x --forward udpout:192.168.0.xx:14550

    # 미션 코드는 브리지를 통해 접속 (--mocap none 필수! 모캡은 브리지가 보냄)
    python3 fly_mission_px4.py --conn udpin:0.0.0.0:14551 --mocap none ...

    # PC 시뮬 시험용(HELIX 없이 가짜 원운동 데이터)
    python3 mocap_bridge.py --fake --conn udpin:0.0.0.0:14540

동작 규칙:
  - 시작할 때 드론은 '앞(보드 쪽)'을 보고 놓여 있어야 함 -> 그 방향을 yaw 0으로 자동 맞춤 (--no-yaw-zero로 끔)
  - 추적이 끊기거나(0,0,0 / NaN) 0.1초 넘게 새 데이터가 없으면 '보내지 않음'
    -> PX4가 스스로 모캡 끊김을 감지해서 비상 동작(착륙)을 함. 낡은 값을 보내 속이지 않음.
"""
import argparse, csv, math, os, sys, time
os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil
from helix_rx import FWD, to_ned, helix_yaw, heading_ned, wrap, valid


class FakeRx:
    """시뮬 시험용: 반지름 0.3 m 원을 20초에 한 바퀴, 높이 0, 몸체는 진행 방향을 봄."""
    names = {1: "fake"}

    def __init__(self):
        self.t0 = time.monotonic(); self.k = 0

    def get(self, key=None):
        t = time.monotonic() - self.t0; self.k += 1
        w = 2 * math.pi / 20
        yaw = w * t
        return dict(t=time.monotonic(), frame=self.k, id=1, x=0.3 * math.cos(w * t), y=0.3 * math.sin(w * t),
                    z=0.0001, qx=0.0, qy=0.0, qz=math.sin(yaw / 2), qw=math.cos(yaw / 2), nmk=4, err=0.0, params=0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-s", "--server", default="192.168.0.66")
    ap.add_argument("--body", default=None, help="HELIX 강체 이름 또는 ID (없으면 첫 번째 강체)")
    ap.add_argument("--fwd", choices=list(FWD), default="+x", help="드론 앞(보드 쪽)이 HELIX의 어느 축 방향인지")
    ap.add_argument("--conn", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--forward", action="append", default=[],
                    help="중계 대상 (여러 번 가능). 기본 udpout:127.0.0.1:14551 (미션 코드용)")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--stale", type=float, default=0.1, help="이 시간[s]보다 오래된 데이터는 안 보냄")
    ap.add_argument("--no-yaw-zero", action="store_true")
    ap.add_argument("--yaw-offset-deg", type=float, default=0.0, help="--no-yaw-zero 일 때 수동 보정값")
    ap.add_argument("--fake", action="store_true")
    ap.add_argument("--out", default=os.path.expanduser("~/mocap_logs"))
    a = ap.parse_args()
    fwd = FWD[a.fwd]
    fw_list = a.forward or ["udpout:127.0.0.1:14551"]

    # 1) 모캡
    if a.fake:
        rx = FakeRx()
    else:
        from helix_rx import HelixRx
        rx = HelixRx(a.server)
        print("HELIX 연결 성공. 강체:", rx.names)
    print("모캡 첫 데이터 기다리는 중...")
    t_wait = time.monotonic()
    while not valid(rx.get(a.body)):
        if time.monotonic() - t_wait > 10:
            sys.exit(f"10초 동안 강체 '{a.body}' 추적 데이터 없음. 강체 이름/스트리밍 확인")
        time.sleep(0.05)

    # 2) yaw 0점: 시작 시 드론이 '앞'을 보고 있다고 가정
    b = rx.get(a.body)
    if a.no_yaw_zero:
        offset = math.radians(a.yaw_offset_deg)
    else:
        samples = []
        t1 = time.monotonic()
        while time.monotonic() - t1 < 1.0:
            bb = rx.get(a.body)
            if valid(bb):
                samples.append(heading_ned(helix_yaw(bb["qx"], bb["qy"], bb["qz"], bb["qw"]), fwd, 0.0))
            time.sleep(0.02)
        offset = math.atan2(sum(math.sin(s) for s in samples), sum(math.cos(s) for s in samples))
    print(f"yaw 0점 보정값 {math.degrees(offset):+.1f}°  (지금 드론이 보는 방향 = 앞 = NED yaw 0)")

    # 3) FC + 중계
    fc = mavutil.mavlink_connection(a.conn, baud=a.baud, source_system=1, source_component=197)
    print("FC 연결 중:", a.conn)
    if fc.wait_heartbeat(timeout=10) is None:
        sys.exit("FC 하트비트 없음")
    print("FC 연결 OK. 중계:", fw_list)
    fws = [mavutil.mavlink_connection(u, source_system=1, source_component=197) for u in fw_list]

    os.makedirs(a.out, exist_ok=True)
    path = os.path.join(a.out, time.strftime("bridge_%Y%m%d_%H%M%S.csv"))
    fcsv = open(path, "w", newline=""); w = csv.writer(fcsv)
    w.writerow(["t", "frame", "sent", "n", "e", "d", "yaw_deg", "age_ms", "est_n", "est_e", "est_d"])
    print("로그:", path, "\nCtrl+C 로 종료 (비행 중에는 절대 끄지 말 것!)\n")

    t0 = time.monotonic(); period = 1.0 / a.rate
    next_tx = t0; last_frame = None; sent = drop = 0; last_print = t0
    est = (float("nan"),) * 3; reset_counter = 0; was_lost = False
    try:
        while True:
            # FC -> 중계 대상
            while True:
                msg = fc.recv_match(blocking=False)
                if msg is None:
                    break
                if msg.get_type() == "LOCAL_POSITION_NED":
                    est = (msg.x, msg.y, msg.z)
                if msg.get_type() != "BAD_DATA":
                    buf = msg.get_msgbuf()
                    for f in fws:
                        f.write(buf)
            # 중계 대상 -> FC
            for f in fws:
                while True:
                    m2 = f.recv_match(blocking=False)
                    if m2 is None:
                        break
                    if m2.get_type() != "BAD_DATA":
                        fc.write(m2.get_msgbuf())

            now = time.monotonic()
            if now >= next_tx:
                next_tx += period
                if next_tx < now:
                    next_tx = now + period
                b = rx.get(a.body)
                age = (now - b["t"]) if b else float("inf")
                ok = valid(b) and age < a.stale and b["frame"] != last_frame
                if ok:
                    if was_lost:
                        reset_counter = (reset_counter + 1) % 256; was_lost = False
                    last_frame = b["frame"]
                    n, e, d = to_ned(b["x"], b["y"], b["z"], fwd)
                    yaw = heading_ned(helix_yaw(b["qx"], b["qy"], b["qz"], b["qw"]), fwd, offset)
                    fc.mav.vision_position_estimate_send(int(now * 1e6), n, e, d, 0.0, 0.0, yaw,
                                                         [float("nan")] + [0.0] * 20, reset_counter)
                    sent += 1
                    w.writerow([f"{now - t0:.4f}", b["frame"], 1, f"{n:.4f}", f"{e:.4f}", f"{d:.4f}",
                                f"{math.degrees(yaw):.2f}", f"{age * 1000:.1f}",
                                f"{est[0]:.4f}", f"{est[1]:.4f}", f"{est[2]:.4f}"])
                else:
                    drop += 1; was_lost = True
                    w.writerow([f"{now - t0:.4f}", b["frame"] if b else -1, 0, "", "", "", "",
                                f"{age * 1000:.1f}", f"{est[0]:.4f}", f"{est[1]:.4f}", f"{est[2]:.4f}"])
            if now - last_print >= 0.5:
                b = rx.get(a.body)
                dt = now - last_print; last_print = now
                if valid(b):
                    n, e, d = to_ned(b["x"], b["y"], b["z"], fwd)
                    yaw = math.degrees(heading_ned(helix_yaw(b["qx"], b["qy"], b["qz"], b["qw"]), fwd, offset))
                    print(f"[{now - t0:6.1f}s] 송신 {sent / dt:4.0f} Hz  누락 {drop:3d} | 모캡 앞 {n:+.2f} 오른 {e:+.2f} "
                          f"위 {-d:+.2f} m yaw {yaw:+6.1f}° | FC추정 앞 {est[0]:+.2f} 오른 {est[1]:+.2f} 위 {-est[2]:+.2f}")
                else:
                    print(f"[{now - t0:6.1f}s] !!! 모캡 추적 끊김 - 송신 중단 중 (누락 {drop}) !!!")
                sent = drop = 0
            time.sleep(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        fcsv.close()
        print("\n종료. 로그:", path)


if __name__ == "__main__":
    main()
