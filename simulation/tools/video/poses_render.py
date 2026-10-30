"""Per-frame poses (30 fps) for rendering the twin from the phone's position.

Drone: position and attitude logged by the twin (NED/FRD) -> Gazebo ENU/FLU.
Camera: fixed at the phone position, aimed so the drone lands on the same pixel
as in the real video (tracked at 540x960, focal 690 px), lightly smoothed.
"""
import csv
import math
import sys

import numpy as np
from scipy.ndimage import uniform_filter1d

REPLAY, PISTA, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
FPS, T_FIN = 30.0, 272.96
CAM = np.array([2.2, 2.2, 1.5])
F, CX, CY = 690.0, 270.0, 480.0
Z_SUELO = 0.16                         # base_link height with the skids on the ground

g = list(csv.DictReader(open(REPLAY)))
tg = np.array([float(r["t_video"]) for r in g])
col = lambda k: np.array([float(r[k]) for r in g])
n, e, d = col("n"), col("e"), col("d")
roll, pitch, yaw = np.radians(col("roll")), np.radians(col("pitch")), np.unwrap(np.radians(col("yaw")))

p = list(csv.DictReader(open(PISTA)))
tp = np.array([float(r["t"]) for r in p])
okp = np.array([bool(r["x"]) and r["metodo"] != "perdido" for r in p])
okp &= ~((tp > 233.5) & (tp < 236))            # arm in front of the lens
xp = np.array([float(r["x"]) if r["x"] else np.nan for r in p])
yp = np.array([float(r["y"]) if r["y"] else np.nan for r in p])

t = np.arange(0, T_FIN, 1 / FPS)
ix = lambda a: np.interp(t, tg, a)
X, Y, Z = ix(e), ix(n), Z_SUELO - ix(d)
R_, P_, Yw = ix(roll), ix(pitch), ix(yaw)
xpx = uniform_filter1d(np.interp(t, tp[okp], xp[okp]), 9)
ypx = uniform_filter1d(np.interp(t, tp[okp], yp[okp]), 9)


def quat(r, p_, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p_ / 2),
                              math.sin(p_ / 2), math.cos(y / 2), math.sin(y / 2))
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


# Camera aim: direction to the drone, offset by the drone's pixel offset in the video
v = np.stack([X, Y, Z]) - CAM[:, None]
yaw_d = np.unwrap(np.arctan2(v[1], v[0]))
elev_d = np.arctan2(v[2], np.hypot(v[0], v[1]))
cam_yaw = uniform_filter1d(yaw_d + np.arctan((xpx - CX) / F), 7)
cam_elev = uniform_filter1d(elev_d + np.arctan((ypx - CY) / F), 7)

with open(OUT, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["frame", "t", "x", "y", "z", "qw", "qx", "qy", "qz", "cqw", "cqx", "cqy", "cqz"])
    for i in range(len(t)):
        # FRD/NED -> FLU/ENU: roll same, pitch and yaw sign flip, yaw from East
        qd = quat(R_[i], -P_[i], math.pi / 2 - Yw[i])
        qc = quat(0.0, -cam_elev[i], cam_yaw[i])
        w.writerow([i, f"{t[i]:.4f}", f"{X[i]:.3f}", f"{Y[i]:.3f}", f"{Z[i]:.3f}",
                    *(f"{q:.5f}" for q in qd), *(f"{q:.5f}" for q in qc)])
print(len(t), "frames; drone z range %.2f..%.2f m" % (Z.min(), Z.max()))
