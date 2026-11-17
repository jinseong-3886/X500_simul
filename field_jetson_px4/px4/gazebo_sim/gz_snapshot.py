"""Grab one colour + depth frame from the Flytrap gz camera and run YOLO on it. Usage: python gz_snapshot.py OUT.jpg"""
import sys, time, threading, numpy as np, cv2
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from ultralytics import YOLO

out = sys.argv[1] if len(sys.argv) > 1 else "snap.jpg"
got = {}
def oc(m): got["c"] = cv2.cvtColor(np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3), cv2.COLOR_RGB2BGR)
def od(m):
    a = np.frombuffer(m.data, np.float32).reshape(m.height, m.width).copy(); a[~np.isfinite(a)] = 0; got["d"] = a
n = Node(); n.subscribe(Image, "flytrap/color", oc); n.subscribe(Image, "flytrap/depth", od)
t0 = time.time()
while ("c" not in got or "d" not in got) and time.time() - t0 < 15: time.sleep(0.1)
if "c" not in got: sys.exit("no colour frames")
img, d = got["c"], got.get("d")
r = YOLO(sys.argv[2] if len(sys.argv) > 2 else "yolov8n.pt")(img, verbose=False)[0]
vis = r.plot()
for b in r.boxes:
    x1, y1, x2, y2 = [int(v) for v in b.xyxy[0]]
    roi = d[y1:y2, x1:x2] if d is not None else None
    z = float(np.median(roi[roi > 0])) if roi is not None and (roi > 0).any() else 0
    print(f"{r.names[int(b.cls)]:10s} conf={float(b.conf[0]):.2f} box_h={y2-y1}px cx={(x1+x2)//2} depth={z:.2f}m")
if len(r.boxes) == 0: print("nothing detected")
dcol = cv2.applyColorMap(cv2.convertScaleAbs(np.clip(d if d is not None else np.zeros(img.shape[:2]), 0, 4), alpha=255/4), cv2.COLORMAP_JET)
cv2.imwrite(out, np.hstack([vis, dcol]))
print("saved", out, "| depth centre px:", f"{d[240,320]:.2f} m" if d is not None else "-")
