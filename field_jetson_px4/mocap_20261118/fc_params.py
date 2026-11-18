"""PX4 FC 파라미터: 전체 백업 / 모캡용 설정 / 백업으로 되돌리기 / 확인.

    python3 fc_params.py backup                   # ~/fc_backup/params_YYYYmmdd_HHMMSS.txt 로 전체 저장
    python3 fc_params.py show                     # 모캡 관련 파라미터 현재 값만 출력
    python3 fc_params.py set-mocap                # 먼저 자동 백업 -> 모캡용 값 설정 -> 다시 읽어 확인
    python3 fc_params.py restore <백업파일>        # 모캡용으로 바꾼 파라미터만 백업 값으로 되돌림
    (바꾼 뒤에는 FC 재부팅 필요: QGC에서 Reboot Vehicle 또는 배터리 뽑았다 꽂기)

모캡용 값 (11-13 Gazebo 시뮬에서 검증한 조합과 동일):
    EKF2_EV_CTRL  = 11  (모캡 수평위치 1 + 수직위치 2 + yaw 8)
    EKF2_HGT_REF  = 3   (높이 기준 = 모캡)
    EKF2_GPS_CTRL = 0   (GPS 안 씀, 실내)
    EKF2_MAG_TYPE = 5   (나침반 안 씀, 방향은 모캡 yaw 사용 -> 'Found 0 compass' 문제도 피함)
"""
import os, struct, sys, time
from pymavlink import mavutil

MOCAP = {"EKF2_EV_CTRL": 11, "EKF2_HGT_REF": 3, "EKF2_GPS_CTRL": 0, "EKF2_MAG_TYPE": 5}
WATCH = list(MOCAP) + ["SYS_HAS_MAG", "EKF2_EV_DELAY", "EKF2_EVP_NOISE", "EKF2_EVA_NOISE",
                       "EKF2_OF_CTRL", "EKF2_RNG_CTRL", "EKF2_BARO_CTRL", "RC_MAP_KILL_SW", "COM_ARM_WO_GPS"]
INT_TYPES = {mavutil.mavlink.MAV_PARAM_TYPE_INT8, mavutil.mavlink.MAV_PARAM_TYPE_INT16,
             mavutil.mavlink.MAV_PARAM_TYPE_INT32, mavutil.mavlink.MAV_PARAM_TYPE_UINT8,
             mavutil.mavlink.MAV_PARAM_TYPE_UINT16, mavutil.mavlink.MAV_PARAM_TYPE_UINT32}


def connect(dev):
    m = mavutil.mavlink_connection(dev, baud=115200, source_system=255, source_component=190)
    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        sys.exit(f"FC 응답 없음 ({dev}). USB 케이블/브리지 실행 여부 확인")
    print(f"FC 연결: sys {m.target_system}")
    m.target_component = 1
    return m


def decode(msg):
    """PX4는 정수 파라미터를 float 비트로 담아 보냄 -> 정수로 풀기."""
    if msg.param_type in INT_TYPES:
        return struct.unpack("<i", struct.pack("<f", msg.param_value))[0]
    return msg.param_value


def read_all(m):
    m.mav.param_request_list_send(m.target_system, 1)
    vals, types, total, t_last = {}, {}, None, time.time()
    while time.time() - t_last < 3.0:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg is None or msg.get_srcComponent() != 1:
            continue
        name = msg.param_id.rstrip("\x00") if isinstance(msg.param_id, str) else msg.param_id.decode().rstrip("\x00")
        vals[name] = decode(msg); types[name] = msg.param_type
        total = msg.param_count; t_last = time.time()
        if total and len(vals) >= total:
            break
    print(f"파라미터 {len(vals)}/{total} 개 읽음")
    return vals, types


def read_one(m, name):
    for _ in range(3):
        m.mav.param_request_read_send(m.target_system, 1, name.encode(), -1)
        t0 = time.time()
        while time.time() - t0 < 1.5:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg and msg.get_srcComponent() == 1:
                pid = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode()
                if pid.rstrip("\x00") == name:
                    return decode(msg), msg.param_type
    return None, None


def write_one(m, name, value):
    cur, ptype = read_one(m, name)
    if ptype is None:
        print(f"  {name}: 이 FC에 없는 파라미터 - 건너뜀"); return False
    if ptype in INT_TYPES:
        raw = struct.unpack("<f", struct.pack("<i", int(value)))[0]
    else:
        raw = float(value)
    m.mav.param_set_send(m.target_system, 1, name.encode(), raw, ptype)
    time.sleep(0.3)
    new, _ = read_one(m, name)
    ok = new is not None and abs(float(new) - float(value)) < 1e-4
    print(f"  {name}: {cur} -> {new}  {'OK' if ok else '실패!'}")
    return ok


def backup(m):
    vals, types = read_all(m)
    if len(vals) < 500:
        sys.exit("파라미터를 다 못 읽음 - 백업 중단 (다시 실행)")
    d = os.path.expanduser("~/fc_backup"); os.makedirs(d, exist_ok=True)
    path = os.path.join(d, time.strftime("params_%Y%m%d_%H%M%S.txt"))
    with open(path, "w") as f:
        for k in sorted(vals):
            f.write(f"{k}\t{vals[k]}\t{types[k]}\n")
    print("백업 저장:", path)
    return path


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    cmd = sys.argv[1]
    dev = os.environ.get("FC_DEV", "/dev/ttyACM0")
    m = connect(dev)
    if cmd == "backup":
        backup(m)
    elif cmd == "show":
        for k in WATCH:
            v, _ = read_one(m, k)
            mark = "" if k not in MOCAP else ("  (모캡값)" if v == MOCAP[k] else f"  (모캡값 {MOCAP[k]})")
            print(f"  {k:16s} = {v}{mark}")
    elif cmd == "set-mocap":
        path = backup(m)
        print("모캡용 값 설정:")
        ok = all([write_one(m, k, v) for k, v in MOCAP.items()])
        print("\n완료." if ok else "\n일부 실패 - 위 내용 확인")
        print(f"되돌리기: python3 fc_params.py restore {path}")
        print("** FC 재부팅해야 적용됨 **")
    elif cmd == "restore":
        if len(sys.argv) < 3:
            sys.exit("백업 파일 경로를 주세요")
        old = {}
        for line in open(sys.argv[2]):
            k, v, _ = line.rstrip("\n").split("\t")
            old[k] = float(v)
        print("백업 값으로 되돌림:")
        for k in MOCAP:
            if k in old:
                write_one(m, k, old[k])
        print("** FC 재부팅해야 적용됨 **")
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
