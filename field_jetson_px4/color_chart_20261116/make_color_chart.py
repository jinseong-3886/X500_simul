"""Printer color-characterisation chart for print-aware patch retraining.
A3 landscape @300 dpi, 4 ArUco corner markers (auto-rectify the D435i photo), 242 color cells:
  125 = 5x5x5 RGB cube, 17 = gray ramp, 80 = k-means colors of the patch, 20 = k-means colors of the clean bus.
Writes chart PNG + PDF (exact physical size) + layout JSON (cell centres in mm + sRGB) for the analysis step.
Usage: python make_color_chart.py --patched bus_patched.png --clean bus_clean.png --out DIR
"""
import argparse, json, os
import numpy as np, cv2
from PIL import Image, ImageDraw, ImageFont

ap = argparse.ArgumentParser()
ap.add_argument("--patched", required=True); ap.add_argument("--clean", required=True)
ap.add_argument("--out", required=True)
a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)

DPI = 300; MM = DPI / 25.4
W_MM, H_MM = 420.0, 297.0                      # A3 landscape
W, H = round(W_MM * MM), round(H_MM * MM)
MARK_MM, MARGIN_MM = 35.0, 12.0
COLS, ROWS = 22, 11
GRID_X0, GRID_X1 = 15.0, W_MM - 15.0
GRID_Y0, GRID_Y1 = MARGIN_MM + MARK_MM + 6.0, H_MM - (MARGIN_MM + MARK_MM + 6.0)
CELL_W, CELL_H = (GRID_X1 - GRID_X0) / COLS, (GRID_Y1 - GRID_Y0) / ROWS
GAP_MM = 2.5

def kmeans_colors(path, k, crop=None):
    img = cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2RGB)
    if crop:
        h, w = img.shape[:2]; c = crop
        img = img[int(h * (0.5 - c / 2)):int(h * (0.5 + c / 2)), int(w * (0.5 - c / 2)):int(w * (0.5 + c / 2))]
    px = img.reshape(-1, 3).astype(np.float32)
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.5)
    _, _, centers = cv2.kmeans(px, k, None, crit, 3, cv2.KMEANS_PP_CENTERS)
    c = np.clip(np.round(centers), 0, 255).astype(int)
    return [tuple(map(int, x)) for x in c[np.argsort(c.sum(1))]]

lv = [0, 64, 128, 191, 255]
colors = [("cube", (r, g, b)) for r in lv for g in lv for b in lv]
colors += [("gray", (v, v, v)) for v in np.linspace(0, 255, 17).round().astype(int).tolist()]
colors += [("patch", c) for c in kmeans_colors(a.patched, 80, crop=0.6)]   # patch = centre 60% of the image
colors += [("clean", c) for c in kmeans_colors(a.clean, 20)]
assert len(colors) == COLS * ROWS, len(colors)
rng = np.random.default_rng(7)                 # shuffle so print non-uniformity doesn't correlate with color
order = rng.permutation(len(colors))
colors = [colors[i] for i in order]

img = Image.new("RGB", (W, H), (255, 255, 255)); d = ImageDraw.Draw(img)
px = lambda mm: int(round(mm * MM))
cells = []
for i, (kind, rgb) in enumerate(colors):
    r, c = divmod(i, COLS)
    x0 = GRID_X0 + c * CELL_W + GAP_MM / 2; y0 = GRID_Y0 + r * CELL_H + GAP_MM / 2
    x1 = x0 + CELL_W - GAP_MM;             y1 = y0 + CELL_H - GAP_MM
    d.rectangle([px(x0), px(y0), px(x1), px(y1)], fill=rgb)
    cells.append({"id": i, "row": r, "col": c, "kind": kind, "rgb": list(rgb),
                  "center_mm": [round((x0 + x1) / 2, 2), round((y0 + y1) / 2, 2)],
                  "size_mm": [round(x1 - x0, 2), round(y1 - y0, 2)]})

aruco = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
mk_px = px(MARK_MM)
corners_mm = {0: (MARGIN_MM, MARGIN_MM), 1: (W_MM - MARGIN_MM - MARK_MM, MARGIN_MM),
              2: (W_MM - MARGIN_MM - MARK_MM, H_MM - MARGIN_MM - MARK_MM), 3: (MARGIN_MM, H_MM - MARGIN_MM - MARK_MM)}
for mid, (x, y) in corners_mm.items():
    m = cv2.aruco.generateImageMarker(aruco, mid, mk_px, borderBits=1)
    img.paste(Image.fromarray(m).convert("RGB"), (px(x), px(y)))

try:
    font = ImageFont.truetype("DejaVuSans.ttf", px(4.2))
except OSError:
    font = ImageFont.load_default()
for k, line in enumerate(("FLYTRAP COLOR CHART v1  |  A3 420x297 mm  |  PRINT AT 100% (no scaling)",
                          "NO color correction / color management OFF  |  matte paper",
                          "Same printer & paper as the final board")):
    d.text((px(MARGIN_MM + MARK_MM + 6), px(MARGIN_MM + 2 + 7 * k)), line, fill=(0, 0, 0), font=font)
d.text((px(MARGIN_MM + MARK_MM + 6), px(H_MM - MARGIN_MM - 7)),
       "Do not crop. Keep the 4 black square markers. 100 mm scale bar ->", fill=(0, 0, 0), font=font)
sx = W_MM - MARGIN_MM - MARK_MM - 6 - 100
d.rectangle([px(sx), px(H_MM - MARGIN_MM - 6), px(sx + 100), px(H_MM - MARGIN_MM - 3)], fill=(0, 0, 0))

png = os.path.join(a.out, "flytrap_color_chart_A3.png"); pdf = os.path.join(a.out, "flytrap_color_chart_A3.pdf")
img.save(png, dpi=(DPI, DPI))
# PDF with the bitmap embedded LOSSLESSLY (Pillow's PDF writer would JPEG-compress = shift the colors)
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
fig = plt.figure(figsize=(W_MM / 25.4, H_MM / 25.4), dpi=DPI)
fig.figimage(np.asarray(img), xo=0, yo=0, origin="upper", resize=False)
fig.savefig(pdf, dpi=DPI); plt.close(fig)
json.dump({"paper_mm": [W_MM, H_MM], "dpi": DPI, "aruco_dict": "DICT_4X4_50", "marker_mm": MARK_MM,
           "markers_topleft_mm": {str(k): v for k, v in corners_mm.items()}, "cells": cells},
          open(os.path.join(a.out, "flytrap_color_chart_layout.json"), "w"), indent=1)
print(f"{W}x{H}px, {len(cells)} cells ({CELL_W - GAP_MM:.1f}x{CELL_H - GAP_MM:.1f} mm) -> {png}, {pdf}")
