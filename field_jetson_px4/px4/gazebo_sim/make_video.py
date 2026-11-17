"""Make mp4 videos (and a side-by-side) from the saved drone-camera frames of the clean/patched episodes."""
import glob, os, cv2, numpy as np
R = os.path.expanduser("~/flytrap_sim/out")
fps = 4  # mission loop ~ 4 Hz -> real time
seqs = {}
for kind in ("clean", "patched"):
    fs = sorted(glob.glob(f"{R}/{kind}/frames/*.jpg"))
    if not fs:
        continue
    imgs = [cv2.imread(f) for f in fs]
    for im in imgs:
        cv2.putText(im, kind.upper(), (520, 465), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
        cv2.putText(im, kind.upper(), (520, 465), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0) if kind == "clean" else (0, 0, 255), 2)
    seqs[kind] = imgs
    vw = cv2.VideoWriter(f"{R}/{kind}.mp4", cv2.VideoWriter_fourcc(*"mp4v"), fps, (640, 480))
    for im in imgs:
        vw.write(im)
    vw.release()
    print(kind, len(imgs), "frames ->", f"{R}/{kind}.mp4")
if len(seqs) == 2:
    n = max(len(s) for s in seqs.values())
    vw = cv2.VideoWriter(f"{R}/clean_vs_patched.mp4", cv2.VideoWriter_fourcc(*"mp4v"), fps, (1280, 480))
    for i in range(n):
        a = seqs["clean"][min(i, len(seqs["clean"]) - 1)]
        b = seqs["patched"][min(i, len(seqs["patched"]) - 1)]
        vw.write(np.hstack([a, b]))
    vw.release()
    print("side-by-side ->", f"{R}/clean_vs_patched.mp4")
    # key stills for the report
    for kind, idx in (("clean", -1), ("patched", None)):
        pass
