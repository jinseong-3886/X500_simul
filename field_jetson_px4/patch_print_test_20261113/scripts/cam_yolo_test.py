"""D435i + YOLOv8n bench test on Jetson (no drone needed).

Uses the same YOLO call as sim_gazebo/fly_mission.py (m(img), default settings),
plus D435i depth at each box center (the 'independent depth' the defense relies on).
Usage: python3 cam_yolo_test.py [N_FRAMES] [OUT_JPG]
"""
import sys, time
import numpy as np, cv2, torch
import pyrealsense2 as rs
from ultralytics import YOLO

N = int(sys.argv[1]) if len(sys.argv) > 1 else 150
OUT = sys.argv[2] if len(sys.argv) > 2 else "cam_yolo_test.jpg"

print("torch", torch.__version__, "| cuda", torch.cuda.is_available(), "|", torch.cuda.get_device_name(0))
m = YOLO("/home/das/X500_simul/yolov8n.pt")

pipe, cfg = rs.pipeline(), rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
prof = pipe.start(cfg)
scale = prof.get_device().first_depth_sensor().get_depth_scale()
align = rs.align(rs.stream.color)

try:
    for _ in range(30):  # auto-exposure warm-up
        pipe.wait_for_frames()
    m(np.zeros((480, 640, 3), np.uint8), verbose=False)  # CUDA warm-up

    infer_ms, loop_t0, last = [], time.time(), None
    counts = {}
    for i in range(N):
        fs = align.process(pipe.wait_for_frames())
        c, d = fs.get_color_frame(), fs.get_depth_frame()
        img = np.asanyarray(c.get_data())
        t0 = time.time()
        r = m(img, verbose=False)[0]
        infer_ms.append((time.time() - t0) * 1000)
        for b in r.boxes:
            n = r.names[int(b.cls)]
            counts[n] = counts.get(n, 0) + 1
        last = (img, r, d)
    loop_fps = N / (time.time() - loop_t0)

    img, r, d = last
    out = r.plot()
    print("\n--- last frame detections ---")
    for b in r.boxes:
        x1, y1, x2, y2 = [int(v) for v in b.xyxy[0]]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        z = d.get_distance(cx, cy)
        name, conf = r.names[int(b.cls)], float(b.conf[0])
        print(f"{name:12s} conf={conf:.2f} box_h={y2 - y1}px  depth@center={z:.2f} m")
        cv2.putText(out, f"{z:.2f}m", (cx - 20, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    if len(r.boxes) == 0:
        print("(nothing detected in last frame)")
    cv2.imwrite(OUT, out)

    a = np.array(infer_ms)
    print("\n--- performance ---")
    print(f"frames={N}  YOLO infer: mean {a.mean():.1f} ms / p95 {np.percentile(a, 95):.1f} ms  (~{1000 / a.mean():.0f} fps)")
    print(f"end-to-end loop (camera+align+YOLO): {loop_fps:.1f} fps")
    print("detections over all frames:", dict(sorted(counts.items(), key=lambda x: -x[1])))
    print("saved", OUT)
finally:
    pipe.stop()
