"""Printed-patch bench measurement (clean vs patched bus print), D435i + YOLOv8n.

Usage (on Jetson):
  python3 patch_measure.py clean   [--n 60] [--h 0.198]
  python3 patch_measure.py patched [--n 60] [--h 0.198]
Each run saves measure_<label>.json + measure_<label>.jpg; when both exist, prints the comparison.
--h = printed bus height in metres (A4 test = 0.198, A2 = 0.396).
Camera must not be in use by another process (stop cam_stream.py first).
"""
import argparse, json, os, sys
import numpy as np, cv2
import pyrealsense2 as rs
from ultralytics import YOLO

ap = argparse.ArgumentParser()
ap.add_argument("label", choices=["clean", "patched"])
ap.add_argument("--n", type=int, default=60)
ap.add_argument("--h", type=float, default=0.198)
a = ap.parse_args()
HERE = os.path.dirname(os.path.abspath(__file__))

m = YOLO("/home/das/X500_simul/yolov8n.pt")
pipe, cfg = rs.pipeline(), rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
prof = pipe.start(cfg)
scale = prof.get_device().first_depth_sensor().get_depth_scale()
fy = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics().fy
align = rs.align(rs.stream.color)

rows, last = [], None
try:
    for _ in range(30):
        pipe.wait_for_frames()
    for i in range(a.n):
        fs = align.process(pipe.wait_for_frames())
        img = np.asanyarray(fs.get_color_frame().get_data())
        dm = np.asanyarray(fs.get_depth_frame().get_data()).astype(np.float32) * scale
        r = m(img, verbose=False)[0]
        # prefer bus boxes that do NOT touch the frame edge (spurious whole-scene boxes do),
        # then highest confidence
        best = None
        for b in r.boxes:
            if r.names[int(b.cls)] == "bus":
                c = float(b.conf[0]); xy = [int(v) for v in b.xyxy[0]]
                inside = not (xy[1] <= 3 or xy[3] >= 476 or xy[0] <= 3 or xy[2] >= 636)
                key = (inside, c)
                if best is None or key > best[2]:
                    best = (c, xy, key)
        if best:
            best = best[:2]
        # paper distance: median valid depth in the central 30% of the frame
        roi = dm[168:312, 224:416]; v = roi[roi > 0]
        dist = float(np.median(v)) if v.size else 0.0
        if best:
            c, (x1, y1, x2, y2) = best
            clip = y1 <= 3 or y2 >= 476 or x1 <= 3 or x2 >= 636
            rows.append(dict(conf=c, h=y2 - y1, w=x2 - x1, dist=dist, clip=clip))
        else:
            rows.append(dict(conf=0.0, h=0, w=0, dist=dist, clip=False))
        last = r
finally:
    pipe.stop()

cv2.imwrite(os.path.join(HERE, f"measure_{a.label}.jpg"), last.plot())
det = [x for x in rows if x["h"] > 0]
dists = [x["dist"] for x in rows if x["dist"] > 0]
d = float(np.median(dists)) if dists else 0.0
res = dict(label=a.label, n=a.n, det_rate=len(det) / a.n,
           h_med=float(np.median([x["h"] for x in det])) if det else 0.0,
           h_std=float(np.std([x["h"] for x in det])) if det else 0.0,
           conf_med=float(np.median([x["conf"] for x in det])) if det else 0.0,
           dist=d, expected_full_h=(fy * a.h / d) if d > 0 else 0.0, print_h=a.h,
           clip_rate=(sum(x["clip"] for x in det) / len(det)) if det else 0.0)
res["hd"] = res["h_med"] * d  # box height x distance: distance-independent for the same target
json.dump(res, open(os.path.join(HERE, f"measure_{a.label}.json"), "w"), indent=1)

print(f"[{a.label}] distance to paper {d:.2f} m | bus detected {len(det)}/{a.n} frames"
      f" | box_h median {res['h_med']:.0f}px (std {res['h_std']:.1f}) | conf {res['conf_med']:.2f}")
print(f"         full-height bus at this distance would be ~{res['expected_full_h']:.0f}px"
      f"  (A2-equivalent distance = {d * 0.396 / a.h:.2f} m)")
print(f"         box_h x distance = {res['hd']:.0f}")
if res["clip_rate"] > 0.2:
    print(f"   (!) bus box touches the frame edge in {res['clip_rate']:.0%} of frames -> move camera back / recentre")

other = "patched" if a.label == "clean" else "clean"
p = os.path.join(HERE, f"measure_{other}.json")
if os.path.exists(p):
    o = json.load(open(p))
    C, P = (o, res) if other == "clean" else (res, o)
    print("\n=== clean vs patched ===")
    print(f"distance: clean {C['dist']:.2f} m / patched {P['dist']:.2f} m"
          + ("   (!) differ by >10 cm — re-measure at the same spot" if abs(C['dist'] - P['dist']) > 0.10 else ""))
    print(f"detect rate: clean {C['det_rate']:.0%} / patched {P['det_rate']:.0%}")
    if C["h_med"] > 0 and P["h_med"] > 0:
        ratio = P["h_med"] / C["h_med"]
        if C.get("hd") and P.get("hd"):
            ratio = P["hd"] / C["hd"]
            print("(ratio below is distance-corrected: box_h x distance)")
        print(f"box_h: clean {C['h_med']:.0f}px -> patched {P['h_med']:.0f}px  => ratio r = {ratio:.2f}"
              f"  (apparent distance x{1 / ratio:.2f})")
        verdict = ("WORKS (sim-level shrink, r<=0.6)" if ratio <= 0.6 else
                   "PARTIAL (some shrink)" if ratio <= 0.85 else "NO EFFECT (no meaningful shrink)")
        print("verdict:", verdict, "| sim reference r~0.46")
    elif P["h_med"] == 0:
        print("patched bus NOT detected -> suppression (box vanished), not distance-pulling")
