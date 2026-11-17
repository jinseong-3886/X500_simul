"""Key-frame montages from the saved episode frames (frame k = k-th mission loop line)."""
import cv2, glob, re, os, shutil, numpy as np
R = os.path.expanduser("~/flytrap_sim/out")
S = "/mnt/c/Users/default.DESKTOP-OA41OO2/AppData/Local/Temp/claude/1cd8287559c47995/097ab0e1-b4fa-4412-a7b5-e6d2f6e5f669/scratchpad"
log = open(f"{R}/patched/mission.log").read().splitlines()
lines = [l for l in log if re.match(r"^\[[0-9.]+\] (SEARCH|FOLLOW|SAFETY_HALT) ", l)]
fr = sorted(glob.glob(f"{R}/patched/frames/*.jpg"))


def first(pred):
    for k, l in enumerate(lines):
        if pred(l):
            return k
    return 0


k1 = first(lambda l: " FOLLOW" in l and "bh=16" in l)          # holding ~1.5 m, patch not yet biting
k2 = first(lambda l: " FOLLOW" in l and re.search(r"bh=7\d ", l))  # shrunk box
k3 = first(lambda l: "vx=0.45" in l)                            # pulled at max speed
k4 = len(fr) - 1                                                 # safety halt
ims = [cv2.resize(cv2.imread(fr[min(k, len(fr) - 1)]), (480, 360)) for k in (k1, k2, k3, k4)]
cv2.imwrite(f"{R}/patched_key.jpg", np.vstack([np.hstack(ims[:2]), np.hstack(ims[2:])]))
c = sorted(glob.glob(f"{R}/clean/frames/*.jpg"))
cv2.imwrite(f"{R}/clean_key.jpg", np.hstack([cv2.resize(cv2.imread(c[i]), (480, 360)) for i in (0, len(c) // 3, -1)]))
for f in ("patched_key.jpg", "clean_key.jpg"):
    pass
print("key frames:", k1, k2, k3, k4)
