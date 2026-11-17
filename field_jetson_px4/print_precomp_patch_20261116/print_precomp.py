"""Pre-compensated print variants of the patch (brighter / more saturated / more contrast, to fight a dark,
muddy print) + a DIGITAL check that each variant still shrinks the YOLO box. Same edit is applied to the clean
image so clean-vs-patched stays a fair pair.  ratio = patched box h / clean box h at the same simulated distance.
"""
import os, sys, numpy as np, cv2
from ultralytics import YOLO

src, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
clean = cv2.imread(os.path.join(src, "bus_clean.png")); patched = cv2.imread(os.path.join(src, "bus_patched.png"))
m = YOLO(os.path.expanduser("~/flytrap_sim/yolov8n.pt"))
BUS = 5

def edit(img, gamma=1.0, sat=1.0, contrast=1.0):
    x = img.astype(np.float32) / 255
    x = x ** gamma                                            # gamma < 1 = brighter mid-tones
    x = np.clip((x - 0.5) * contrast + 0.5, 0, 1)
    hsv = cv2.cvtColor((x * 255).astype(np.uint8), cv2.COLOR_BGR2HSV).astype(np.float32)
    hsv[..., 1] = np.clip(hsv[..., 1] * sat, 0, 255)
    return cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)

def bus_h(img, d, blur=0.0):
    """paste the board as the D435i would see it at distance d (640x480, fy 607, board h 0.396 m)."""
    h = int(round(607 * 0.396 / d)); w = int(round(h * img.shape[1] / img.shape[0]))
    s = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
    if blur > 0:
        s = cv2.GaussianBlur(s, (0, 0), blur)
    cv = np.full((480, 640, 3), 128, np.uint8)
    y0, x0 = (480 - h) // 2, (640 - w) // 2
    cv[y0:y0 + h, x0:x0 + w] = s
    r = m(cv, verbose=False, classes=[BUS])[0]
    if len(r.boxes) == 0:
        return 0.0
    b = r.boxes.xyxy.cpu().numpy(); i = int(np.argmax(b[:, 3] - b[:, 1]))
    return float(b[i, 3] - b[i, 1])

DISTS = (1.0, 1.5, 2.0, 2.5)
VARIANTS = {"original": {}, "bright_g0.8": dict(gamma=0.8), "bright_g0.65": dict(gamma=0.65),
            "sat1.3": dict(sat=1.3), "contrast1.2": dict(contrast=1.2),
            "g0.8_sat1.3": dict(gamma=0.8, sat=1.3), "g0.65_sat1.3_c1.15": dict(gamma=0.65, sat=1.3, contrast=1.15)}
print("ratio = patched/clean box height (lower = stronger patch; <0.6 good). blur=print/lens softness")
print(f"{'variant':22s}" + "".join(f"  {d}m" + " " * 4 for d in DISTS) + "  | same, blur 1.0px")
for name, kw in VARIANTS.items():
    c, p = edit(clean, **kw), edit(patched, **kw)
    row = []
    for blur in (0.0, 1.0):
        for d in DISTS:
            hc, hp = bus_h(c, d, blur), bus_h(p, d, blur)
            row.append(hp / hc if hc else float("nan"))
    print(f"{name:22s}" + "".join(f"  {r:5.2f}   " for r in row[:4]) + "  |" + "".join(f" {r:4.2f}" for r in row[4:]))
    if name != "original":
        cv2.imwrite(os.path.join(out, f"bus_patched_{name}.png"), p)
        cv2.imwrite(os.path.join(out, f"bus_clean_{name}.png"), c)
