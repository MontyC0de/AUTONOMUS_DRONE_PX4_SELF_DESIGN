"""Per-frame poses (30 fps) for rendering the twin as the phone saw the real flight.

usage: poses.py replay_gemelo.csv pista.csv giro.csv poses.csv

Drone:   position and attitude logged by the twin (NED/FRD) -> Gazebo ENU/FLU.
Props:   four separate models on the motor axes; once armed they take a random
         angle every frame, as a fast phone shutter freezes spinning props.
Camera:  one fixed position per stretch of the video (the phone moved between
         the eucalyptus side and the field). Its distance to the drone is the one
         given by the drone's apparent size in the video; it is aimed so the
         drone sits on the same pixel as in the video.
"""
import csv
import json
import math
import sys

import numpy as np
from scipy.ndimage import median_filter, uniform_filter1d

REPLAY, PISTA, GIRO, OUT = sys.argv[1:5]
# "--camara-fija": phone fixed next to the takeoff point the whole flight (no
# jumps between stretches); otherwise one position per stretch
FIJA = "--camara-fija" in sys.argv[5:]
ARBOLES = next((a for a in sys.argv[5:] if not a.startswith("--")), None)
CAM_FIJA = (2.2, 2.2, 1.5)
FPS, T_FIN, T_ARMADO = 30.0, 272.96, 16.4
F, CX, CY = 690.0, 270.0, 480.0
Z_SUELO, ALTURA_MOVIL = 0.16, 1.5
ANCHO = {"verde": 0.42, "cielo": 0.32, "plantilla": 0.35}
ROTORES = [(0.1228, -0.1228, "ccw"), (-0.1228, 0.1228, "ccw"), (0.1228, 0.1228, "cw"), (-0.1228, -0.1228, "cw")]


def quat(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2),
                              math.sin(p / 2), math.cos(y / 2), math.sin(y / 2))
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def rotar(q, v):
    return qmul(qmul(q, np.array([0.0, *v])), q * np.array([1, -1, -1, -1]))[1:]


# --- Drone (twin log)
g = list(csv.DictReader(open(REPLAY)))
tg = np.array([float(r["t_video"]) for r in g])
col = lambda k: np.array([float(r[k]) for r in g])
t = np.arange(0, T_FIN, 1 / FPS)
ix = lambda a: np.interp(t, tg, a)
X, Y, Z = ix(col("e")), ix(col("n")), Z_SUELO - ix(col("d"))
R_, P_ = ix(np.radians(col("roll"))), ix(np.radians(col("pitch")))
Yw = ix(np.unwrap(np.radians(col("yaw"))))

# --- Video: drone pixel, range from apparent size, camera azimuth
p = list(csv.DictReader(open(PISTA)))
tp = np.array([float(r["t"]) for r in p])
met = np.array([r["metodo"] for r in p])
ok = np.array([bool(r["x"]) for r in p]) & (met != "perdido") & ~((tp > 233.5) & (tp < 236))
xp = np.array([float(r["x"]) if r["x"] else np.nan for r in p])
yp = np.array([float(r["y"]) if r["y"] else np.nan for r in p])
wp = np.array([float(r["ancho"]) if r["ancho"] else np.nan for r in p])
Rv = np.array([ANCHO.get(m, np.nan) for m in met]) * F / wp
fiable = ok & np.isfinite(Rv) & ((met == "cielo") | ((met == "verde") & (wp >= 20)))
logR = np.interp(tp, tp[fiable], np.log(Rv[fiable]))
k = (tp >= 100) & (tp <= 126)
logR[k] = np.log(np.interp(tp[k], [100, 113, 126], [45.0, 55.0, 50.0]))
Rv = np.exp(uniform_filter1d(median_filter(logR, size=31), 61))
yaw_cam = np.array([float(r["yaw"]) for r in csv.DictReader(open(GIRO))])[:len(tp)]
az_cam = 225.0 - yaw_cam
k = tp >= 237
az_cam[k] = 150.0 - (yaw_cam[k] - yaw_cam[np.searchsorted(tp, 240.0)])

xpx = uniform_filter1d(np.interp(t, tp[ok], xp[ok]), 9)
ypx = uniform_filter1d(np.interp(t, tp[ok], yp[ok]), 9)
az_drone = np.radians(np.interp(t, tp, az_cam) + np.degrees(np.arctan((xpx - CX) / F)))
Rt = np.interp(t, tp, Rv)

# --- Camera: fixed per stretch (view towards the eucalyptus vs towards the field)
campo = np.mod(np.interp(t, tp, az_cam), 360) < 200
cortes = np.flatnonzero(np.diff(campo.astype(int))) + 1
tramos = np.split(np.arange(len(t)), cortes)
CAM = np.zeros((len(t), 3))
arboles = json.load(open(ARBOLES)) if ARBOLES else []
for idx in tramos:
    # phone = drone - range * direction to drone; median over the stretch
    ce = X[idx] - Rt[idx] * np.sin(az_drone[idx])
    cn = Y[idx] - Rt[idx] * np.cos(az_drone[idx])
    if t[idx[0]] < 30:
        ce, cn = np.full_like(ce, 2.2), np.full_like(cn, 2.2)   # takeoff: next to the drone
    c = np.array([np.median(ce), np.median(cn)])
    if ARBOLES:
        # Keep the phone out of tree crowns: same distance to the drone, rotated
        # around it to the nearest clear spot
        centro = np.array([np.median(X[idx]), np.median(Y[idx])])
        radio, ang0 = np.hypot(*(c - centro)), math.atan2(*(c - centro)[::-1])
        for da in sorted(np.radians(np.arange(-90, 91, 2)), key=abs):
            cand = centro + radio * np.array([math.cos(ang0 + da), math.sin(ang0 + da)])
            if all(math.hypot(cand[0] - a["x"], cand[1] - a["y"]) > 5.3 * a["escala"] + 2.5 for a in arboles):
                c = cand
                break
    CAM[idx] = CAM_FIJA if FIJA else [c[0], c[1], ALTURA_MOVIL]
    c = CAM[idx[0]]
    print(f"tramo {t[idx[0]]:6.1f}-{t[idx[-1]]:6.1f} s  {'explanada ' if campo[idx[0]] else 'eucaliptos'}  "
          f"cámara ({c[0]:6.1f}, {c[1]:6.1f}) m")

v = np.stack([X, Y, Z], axis=1) - CAM
yaw_d = np.unwrap(np.arctan2(v[:, 1], v[:, 0]))
elev_d = np.arctan2(v[:, 2], np.hypot(v[:, 0], v[:, 1]))
cam_yaw = uniform_filter1d(yaw_d + np.arctan((xpx - CX) / F), 7)
cam_elev = uniform_filter1d(elev_d + np.arctan((ypx - CY) / F), 7)

rng = np.random.default_rng(3)
ang0 = rng.uniform(0, 2 * np.pi, 4)
with open(OUT, "w", newline="") as f:
    w = csv.writer(f)
    cab = ["frame", "t", "x", "y", "z", "qw", "qx", "qy", "qz", "cx", "cy", "cz", "cqw", "cqx", "cqy", "cqz"]
    for i in range(4):
        cab += [f"h{i}{c}" for c in ("x", "y", "z", "qw", "qx", "qy", "qz")]
    w.writerow(cab)
    for i in range(len(t)):
        qd = quat(R_[i], -P_[i], math.pi / 2 - Yw[i])        # FRD/NED -> FLU/ENU
        qc = quat(0.0, -cam_elev[i], cam_yaw[i])
        fila = [i, f"{t[i]:.4f}", f"{X[i]:.3f}", f"{Y[i]:.3f}", f"{Z[i]:.3f}", *(f"{q:.5f}" for q in qd),
                *(f"{c:.3f}" for c in CAM[i]), *(f"{q:.5f}" for q in qc)]
        giro = rng.uniform(0, 2 * np.pi, 4) if t[i] >= T_ARMADO else ang0
        for j, (ox, oy, _) in enumerate(ROTORES):
            pos = np.array([X[i], Y[i], Z[i]]) + rotar(qd, (ox, oy, 0.0))
            qh = qmul(qd, quat(0.0, 0.0, giro[j]))
            fila += [*(f"{c:.4f}" for c in pos), *(f"{q:.5f}" for q in qh)]
        w.writerow(fila)
print(len(t), "fotogramas")
