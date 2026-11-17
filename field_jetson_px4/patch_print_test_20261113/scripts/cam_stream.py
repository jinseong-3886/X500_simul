"""Live D435i + YOLOv8n view in a browser (no drone needed).

Run on the Jetson:   python3 cam_stream.py [--port 8080] [--no-yolo]
Open on the PC:      http://<jetson-ip>:8080
Shows color with YOLO boxes + D435i depth (median inside box), and a depth colormap.
Stop with Ctrl+C (or `pkill -f cam_stream.py`).
"""
import argparse, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import numpy as np, cv2
import pyrealsense2 as rs

ap = argparse.ArgumentParser()
ap.add_argument("--port", type=int, default=8080)
ap.add_argument("--no-yolo", action="store_true")
args = ap.parse_args()

model = None
if not args.no_yolo:
    from ultralytics import YOLO
    model = YOLO("/home/das/X500_simul/yolov8n.pt")

latest = {"jpg": None}
lock = threading.Lock()


def box_depth(depth_m, x1, y1, x2, y2):
    """Median of valid depth in the inner half of the box (0 = no valid depth)."""
    w, h = x2 - x1, y2 - y1
    roi = depth_m[y1 + h // 4: y2 - h // 4, x1 + w // 4: x2 - w // 4]
    v = roi[roi > 0]
    return float(np.median(v)) if v.size else 0.0


def capture_loop():
    pipe, cfg = rs.pipeline(), rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    prof = pipe.start(cfg)
    scale = prof.get_device().first_depth_sensor().get_depth_scale()
    align = rs.align(rs.stream.color)
    t_prev, fps = time.time(), 0.0
    try:
        while True:
            fs = align.process(pipe.wait_for_frames())
            c, d = fs.get_color_frame(), fs.get_depth_frame()
            if not c or not d:
                continue
            img = np.asanyarray(c.get_data())
            depth_m = np.asanyarray(d.get_data()).astype(np.float32) * scale
            out = img.copy()
            if model is not None:
                r = model(img, verbose=False)[0]
                out = r.plot()
                for b in r.boxes:
                    x1, y1, x2, y2 = [int(v) for v in b.xyxy[0]]
                    z = box_depth(depth_m, x1, y1, x2, y2)
                    txt = f"{z:.2f}m" if z > 0 else "no depth"
                    cv2.putText(out, txt, ((x1 + x2) // 2 - 30, (y1 + y2) // 2),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            dcol = cv2.applyColorMap(
                cv2.convertScaleAbs(np.clip(depth_m, 0, 4.0), alpha=255 / 4.0), cv2.COLORMAP_JET)
            dcol[depth_m == 0] = 0
            now = time.time()
            fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_prev, 1e-6)); t_prev = now
            cv2.putText(out, f"{fps:.1f} fps", (10, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(dcol, "depth 0-4 m (black = no depth)", (10, 470),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            ok, jpg = cv2.imencode(".jpg", np.hstack([out, dcol]), [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                with lock:
                    latest["jpg"] = jpg.tobytes()
    finally:
        pipe.stop()


PAGE = b"""<!doctype html><html><head><meta charset="utf-8"><title>Jetson D435i Live</title>
<style>body{margin:0;background:#111;color:#ddd;font-family:sans-serif;text-align:center}
img{max-width:100%;height:auto}</style></head>
<body><h3>Jetson D435i + YOLOv8n live</h3><img src="/stream"></body></html>"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            try:
                while True:
                    with lock:
                        jpg = latest["jpg"]
                    if jpg:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                    time.sleep(1 / 30)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(PAGE)


threading.Thread(target=capture_loop, daemon=True).start()
import subprocess
ips = [i for i in subprocess.run(["hostname", "-I"], capture_output=True, text=True).stdout.split() if ":" not in i and not i.startswith("172.17.")]
for ip in ips:
    print(f"OPEN IN PC BROWSER ->  http://{ip}:{args.port}", flush=True)
print("(Ctrl+C to stop)", flush=True)
ThreadingHTTPServer(("0.0.0.0", args.port), H).serve_forever()
