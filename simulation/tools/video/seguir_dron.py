"""Track the drone in the flight video.

Reads frames from ffmpeg (rgb24, W x H) on stdin, one every 1/FPS s, and writes a
CSV with, per frame: time, drone centre (px), apparent width (px), detection
method, and the skyline row (lower edge of the sky) at the drone's column.

Detection, in order of reliability:
  verde:  compact cluster of lime-green props (nothing else in the scene is that green)
  cielo:  very dark blob (drone is black; eucalyptus leaves are grey-green) almost
          fully surrounded by blue sky; continuity only breaks ties
  plantilla: template match around the last position (short gaps only)
"""
import csv
import sys

import cv2
import numpy as np
from scipy import ndimage

W, H, FPS, OUT = int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
frame_bytes = W * H * 3


def verde(img, r, g, b):
    lime = (g > 140) & (g > r + 55) & (g > b + 55)
    ys, xs = np.nonzero(lime)
    if len(xs) < 4:
        return None
    cx, cy = np.median(xs), np.median(ys)
    keep = np.hypot(xs - cx, ys - cy) < 90
    xs, ys = xs[keep], ys[keep]
    if len(xs) < 4:
        return None
    return float(xs.mean()), float(ys.mean()), float(max(xs.max() - xs.min(), 4)), "verde"


def cielo(img, sky, prev, lost):
    holes = ndimage.binary_fill_holes(sky) & ~sky
    lab, n = ndimage.label(holes)
    v = img.mean(axis=2)
    cands = []
    for k, sl in enumerate(ndimage.find_objects(lab), start=1):
        hh, ww = sl[0].stop - sl[0].start, sl[1].stop - sl[1].start
        if not (2 <= ww <= 120 and 2 <= hh <= 120):
            continue
        y0, y1 = max(sl[0].start - 4, 0), min(sl[0].stop + 4, H)
        x0, x1 = max(sl[1].start - 4, 0), min(sl[1].stop + 4, W)
        blob = lab[y0:y1, x0:x1] == k
        if blob.sum() < 3 or v[y0:y1, x0:x1][blob].mean() > 75:
            continue
        ring = ndimage.binary_dilation(blob, iterations=3) & ~blob
        if sky[y0:y1, x0:x1][ring].mean() < 0.85:
            continue
        cx, cy = (sl[1].start + sl[1].stop) / 2, (sl[0].start + sl[0].stop) / 2
        oscuridad = v[y0:y1, x0:x1][blob].mean()
        d = np.hypot(cx - prev[0], cy - prev[1]) if (prev is not None and lost < 20) else 0.0
        cands.append((d + oscuridad, cx, cy, float(ww)))
    if not cands:
        return None
    _, cx, cy, ww = min(cands)
    return cx, cy, ww, "cielo"


prev, template, lost, rows, i = None, None, 0, [], 0
while True:
    buf = sys.stdin.buffer.read(frame_bytes)
    if len(buf) < frame_bytes:
        break
    img = np.frombuffer(buf, np.uint8).reshape(H, W, 3)
    r, g, b = (img[..., k].astype(np.int16) for k in range(3))
    t = i / FPS
    i += 1
    sky = (b > 110) & (b > r + 25) & (b > g - 5) & (r < 200)

    det = verde(img, r, g, b) or cielo(img, sky, prev, lost)
    if det is None and template is not None and prev is not None and lost < 10:
        th, tw = template.shape[:2]
        x0, y0 = int(max(prev[0] - 60, 0)), int(max(prev[1] - 60, 0))
        x1, y1 = int(min(prev[0] + 60 + tw, W)), int(min(prev[1] + 60 + th, H))
        roi = img[y0:y1, x0:x1]
        if roi.shape[0] > th and roi.shape[1] > tw:
            res = cv2.matchTemplate(roi, template, cv2.TM_CCOEFF_NORMED)
            _, mx, _, loc = cv2.minMaxLoc(res)
            if mx > 0.6:
                det = (x0 + loc[0] + tw / 2, y0 + loc[1] + th / 2, prev[2], "plantilla")

    if det is not None:
        prev, lost = det[:3], 0
        half = int(max(det[2], 8))
        ya, yb = int(max(det[1] - half, 0)), int(min(det[1] + half, H))
        xa, xb = int(max(det[0] - half, 0)), int(min(det[0] + half, W))
        if yb - ya > 4 and xb - xa > 4:
            template = img[ya:yb, xa:xb].copy()
    else:
        lost += 1

    col = int(prev[0]) if prev is not None else W // 2
    band = sky[:, max(col - 40, 0):min(col + 40, W)]
    skyrows = np.nonzero(band.mean(axis=1) > 0.5)[0]
    skyline = int(skyrows.max()) if len(skyrows) else -1
    if det is not None:
        rows.append([f"{t:.2f}", f"{det[0]:.1f}", f"{det[1]:.1f}", f"{det[2]:.1f}", det[3], skyline, f"{sky.mean():.3f}"])
    else:
        rows.append([f"{t:.2f}", "", "", "", "perdido", skyline, f"{sky.mean():.3f}"])

with open(OUT, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "x", "y", "ancho", "metodo", "horizonte", "frac_cielo"])
    w.writerows(rows)
metodos = [r[4] for r in rows]
print(f"{len(rows)} frames:", {m: metodos.count(m) for m in set(metodos)})
