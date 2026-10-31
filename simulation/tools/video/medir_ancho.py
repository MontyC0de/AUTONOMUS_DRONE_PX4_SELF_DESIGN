"""Drone width (tip to tip) at 1080x1920 around the tracked position (pista.csv).

The drone = lime props + pixels clearly darker than the local background,
connected to each other. Output anchos.csv: t, x, y, ancho, alto, ok.
"""
import csv
import sys

import cv2
import numpy as np
from scipy import ndimage

W, H = 1080, 1920
pista = list(csv.DictReader(open("pista.csv")))
nb = W * H * 3
filas, k = [], 0
while True:
    buf = sys.stdin.buffer.read(nb)
    if len(buf) < nb:
        break
    r = pista[k] if k < len(pista) else None
    t = k / 10.0
    k += 1
    if not (r and r["x"]) or r["metodo"] == "perdido":
        filas.append([f"{t:.1f}", "", "", "", "", 0])
        continue
    img = np.frombuffer(buf, np.uint8).reshape(H, W, 3)
    cx, cy = float(r["x"]) * 2, float(r["y"]) * 2
    w0 = float(r["ancho"]) * 2 if r["ancho"] else 30
    half = int(np.clip(1.8 * w0, 40, 420))
    x0, y0 = int(max(cx - half, 0)), int(max(cy - half, 0))
    x1, y1 = int(min(cx + half, W)), int(min(cy + half, H))
    c = img[y0:y1, x0:x1].astype(np.int16)
    R, G, B = c[..., 0], c[..., 1], c[..., 2]
    v = c.mean(axis=2)
    lima = (G > 110) & (G > R + 35) & (G > B + 40)
    kb = int(np.clip(half / 2, 15, 81)) | 1
    fondo = cv2.blur(v.astype(np.float32), (kb, kb))
    oscuro = (v < fondo - 40) & (v < 90)
    masc = lima | oscuro
    lab, n = ndimage.label(ndimage.binary_dilation(masc, iterations=int(np.clip(w0 / 25, 1, 6))))
    if n == 0:
        filas.append([f"{t:.1f}", f"{cx:.1f}", f"{cy:.1f}", "", "", 0])
        continue
    # component at (or nearest to) the tracked centre, preferring ones with lime
    pcx, pcy = cx - x0, cy - y0
    mejor, mejor_c = None, 1e9
    for j, sl in enumerate(ndimage.find_objects(lab), start=1):
        comp = (lab[sl] == j) & masc[sl]
        if comp.sum() < 3:
            continue
        ys, xs = np.nonzero(comp)
        bx0, bx1 = sl[1].start + xs.min(), sl[1].start + xs.max()
        by0, by1 = sl[0].start + ys.min(), sl[0].start + ys.max()
        dx = max(bx0 - pcx, 0, pcx - bx1)
        dy = max(by0 - pcy, 0, pcy - by1)
        tiene_lima = (lab[sl] == j)[lima[sl]].any()
        cst = np.hypot(dx, dy) - (8 if tiene_lima else 0)
        borde = bx0 <= 0 or by0 <= 0 or bx1 >= c.shape[1] - 1 or by1 >= c.shape[0] - 1
        if borde:
            cst += 50
        if cst < mejor_c:
            mejor_c, mejor = cst, (bx0, bx1, by0, by1)
    if mejor is None or mejor_c > 20:
        filas.append([f"{t:.1f}", f"{cx:.1f}", f"{cy:.1f}", "", "", 0])
        continue
    bx0, bx1, by0, by1 = mejor
    filas.append([f"{t:.1f}", f"{x0 + (bx0 + bx1) / 2:.1f}", f"{y0 + (by0 + by1) / 2:.1f}",
                  int(bx1 - bx0 + 1), int(by1 - by0 + 1), 1])
with open("anchos.csv", "w", newline="") as f:
    wr = csv.writer(f)
    wr.writerow(["t", "x", "y", "ancho", "alto", "ok"])
    wr.writerows(filas)
print("medidos", sum(1 for f in filas if f[5]), "de", len(filas))
