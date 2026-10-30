"""Integrate the phone camera yaw/pitch from background optical flow.

stdin: rgb24 frames W x H at FPS. Drone track (pista.csv, at the same FPS but
scaled to 540 px wide) is used to mask the drone. Output CSV: t, yaw_deg, pitch_deg
(cumulative, yaw positive = camera turned left / counter-clockwise from above).
"""
import csv
import sys

import cv2
import numpy as np

W, H, FPS, F, PISTA, OUT = int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]), sys.argv[5], sys.argv[6]
escala = W / 540.0
pista = list(csv.DictReader(open(PISTA)))
nbytes = W * H * 3
prev_g, yaw, pitch, i, rows = None, 0.0, 0.0, 0, []
while True:
    buf = sys.stdin.buffer.read(nbytes)
    if len(buf) < nbytes:
        break
    g = cv2.cvtColor(np.frombuffer(buf, np.uint8).reshape(H, W, 3), cv2.COLOR_RGB2GRAY)
    dyaw = dpitch = 0.0
    if prev_g is not None:
        mask = np.full_like(g, 255)
        r = pista[i] if i < len(pista) else None
        if r and r["x"]:
            cx, cy, ww = float(r["x"]) * escala, float(r["y"]) * escala, float(r["ancho"]) * escala
            cv2.circle(mask, (int(cx), int(cy)), int(max(3 * ww, 20)), 0, -1)
        p0 = cv2.goodFeaturesToTrack(prev_g, maxCorners=300, qualityLevel=0.01, minDistance=6, mask=mask)
        if p0 is not None and len(p0) >= 8:
            p1, st, _ = cv2.calcOpticalFlowPyrLK(prev_g, g, p0, None, winSize=(21, 21), maxLevel=3)
            good = st.ravel() == 1
            if good.sum() >= 8:
                d = (p1[good] - p0[good]).reshape(-1, 2)
                dx, dy = np.median(d[:, 0]), np.median(d[:, 1])
                # Background moves right when the camera turns left
                dyaw = np.degrees(np.arctan(dx / F))
                dpitch = np.degrees(np.arctan(dy / F))
    yaw += dyaw
    pitch += dpitch
    rows.append([f"{i / FPS:.2f}", f"{yaw:.2f}", f"{pitch:.2f}"])
    prev_g = g
    i += 1
with open(OUT, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "yaw", "pitch"])
    w.writerows(rows)
print(len(rows), "frames; final yaw %.1f deg, pitch %.1f deg" % (yaw, pitch))
