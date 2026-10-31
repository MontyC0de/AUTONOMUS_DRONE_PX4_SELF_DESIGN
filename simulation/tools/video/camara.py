"""Frame-to-frame camera rotation of the phone video (pure rotation model).

stdin: gray frames W x H (portrait) at FPS. pista.csv (540x960 coords) masks the drone.
Output npz: H (homographies k-1 -> k), inliers, keyframes (every KF s, 540x960 gray).
"""
import csv
import sys

import cv2
import numpy as np

W, H, FPS, PISTA, OUT = int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3]), sys.argv[4], sys.argv[5]
KF = 1.0
esc = W / 540.0
pista = list(csv.DictReader(open(PISTA)))
nbytes = W * H
prev, homos, inl, keys, kt = None, [], [], [], []
i = 0
while True:
    buf = sys.stdin.buffer.read(nbytes)
    if len(buf) < nbytes:
        break
    g = np.frombuffer(buf, np.uint8).reshape(H, W)
    if i % int(KF * FPS) == 0:
        keys.append(cv2.resize(g, (540, 960), interpolation=cv2.INTER_AREA))
        kt.append(i / FPS)
    Hm, n_in = np.full((3, 3), np.nan), 0
    if prev is not None:
        mask = np.full_like(prev, 255)
        r = pista[i - 1] if i - 1 < len(pista) else None
        if r and r["x"]:
            cv2.circle(mask, (int(float(r["x"]) * esc), int(float(r["y"]) * esc)),
                       int(max(4 * float(r["ancho"]) * esc, 60)), 0, -1)
        p0 = cv2.goodFeaturesToTrack(prev, maxCorners=800, qualityLevel=0.01, minDistance=10, mask=mask)
        if p0 is not None and len(p0) >= 15:
            p1, st, _ = cv2.calcOpticalFlowPyrLK(prev, g, p0, None, winSize=(25, 25), maxLevel=4)
            ok = st.ravel() == 1
            if ok.sum() >= 15:
                Hm_, m = cv2.findHomography(p0[ok], p1[ok], cv2.RANSAC, 2.0)
                if Hm_ is not None:
                    Hm, n_in = Hm_, int(m.sum())
    homos.append(Hm)
    inl.append(n_in)
    prev = g
    i += 1
    if i % 300 == 0:
        print(f"{i} fotogramas", flush=True)
np.savez_compressed(OUT, H=np.array(homos), inliers=np.array(inl), keys=np.array(keys), kt=np.array(kt), fps=FPS, W=W, Hh=H)
print(f"{i} fotogramas; sin homografía: {sum(1 for n in inl[1:] if n < 15)}")
