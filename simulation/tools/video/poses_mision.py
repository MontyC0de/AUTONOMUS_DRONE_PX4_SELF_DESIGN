"""Per-frame poses (30 fps) to render a logged mission of the twin (mision_avanzada.py).

usage: poses_mision.py vuelos/mision_<fecha>.csv poses_mision.csv [velocidad] [fija|seguimiento]

main  : fixed, south-east of the take-off point, high enough to see the four figures
follow: "fija" (default) a static camera at eye height, 21 m south-east of the take-off
        point, framing all four figures, as someone filming from the ground;
        "seguimiento" 2.6 m from the drone towards the main camera, 0.6 m up, smoothed
props : four separate models on the motor axes, a new angle every frame
The output also carries the reference and flown position of every frame, so the
trail can be drawn on top of the render (montaje_mision.py).
"""
import csv
import sys

import numpy as np
from scipy.ndimage import uniform_filter1d
from scipy.spatial.transform import Rotation

LOG, OUT = sys.argv[1], sys.argv[2]
VEL = float(sys.argv[3]) if len(sys.argv) > 3 else 4.0
MODO = sys.argv[4] if len(sys.argv) > 4 else "fija"
FPS, Z_SUELO = 30.0, 0.16
ROTORES = [(0.1228, -0.1228), (-0.1228, 0.1228), (0.1228, 0.1228), (-0.1228, -0.1228)]
CAM = np.array([11.3, -9.9, 15.2])            # ENU, metres from the take-off point
MIRA = np.array([0.0, 0.0, 5.8])
FIJA = np.array([14.85, -14.85, 1.6])          # eye height; with hfov 0.8 every figure stays in frame
MIRA_FIJA = np.array([0.0, 0.0, 5.0])


def mirando(desde, hacia):
    """Quaternion (x, y, z, w) of a Gazebo camera link (x forward) at `desde` looking at `hacia`."""
    v = hacia - desde
    yaw = np.arctan2(v[..., 1], v[..., 0])
    pitch = np.arctan2(v[..., 2], np.hypot(v[..., 0], v[..., 1]))
    return Rotation.from_euler("ZY", np.column_stack([np.atleast_1d(yaw), -np.atleast_1d(pitch)]))


g = list(csv.DictReader(open(LOG)))
tg = np.array([float(r["t"]) for r in g])
col = lambda k: np.array([float(r[k]) for r in g])
t = np.arange(0, tg[-1], VEL / FPS)
ix = lambda a: np.interp(t, tg, a)
X, Y, Z = ix(col("e")), ix(col("n")), Z_SUELO - ix(col("d"))
RX, RY, RZ = ix(col("sp_e")), ix(col("sp_n")), Z_SUELO - ix(col("sp_d"))
roll, pitch = ix(np.radians(col("roll"))), ix(np.radians(col("pitch")))
yaw = ix(np.unwrap(np.radians(col("yaw"))))
q_dron = Rotation.from_euler("xyz", np.column_stack([roll, -pitch, np.pi / 2 - yaw]))   # FRD/NED -> FLU/ENU
tramo = [g[min(np.searchsorted(tg, ti, side="right") - 1, len(g) - 1)]["tramo"] for ti in t]

P = np.column_stack([X, Y, Z])
q_main = mirando(CAM, MIRA)[0].as_quat()
hacia_cam = CAM[:2] - P[:, :2]
hacia_cam /= np.linalg.norm(hacia_cam, axis=1, keepdims=True)
Ps = uniform_filter1d(P, 9, axis=0)
F = Ps + np.column_stack([2.6 * hacia_cam, np.full(len(t), 0.6)])
F = uniform_filter1d(F, 15, axis=0)
q_seg = mirando(F, Ps)
if MODO == "fija":
    F = np.tile(FIJA, (len(t), 1))
    q_seg = Rotation.from_quat(np.tile(mirando(FIJA, MIRA_FIJA)[0].as_quat(), (len(t), 1)))

rng = np.random.default_rng(3)
with open(OUT, "w", newline="") as f:
    w = csv.writer(f)
    cab = ["frame", "t", "tramo", "x", "y", "z", "qw", "qx", "qy", "qz", "rx", "ry", "rz",
           "mx", "my", "mz", "mqw", "mqx", "mqy", "mqz", "fx", "fy", "fz", "fqw", "fqx", "fqy", "fqz"]
    for i in range(4):
        cab += [f"h{i}{k}" for k in ("x", "y", "z", "qw", "qx", "qy", "qz")]
    w.writerow(cab)
    for i in range(len(t)):
        qd = q_dron[i]
        qdx, qdy, qdz, qdw = qd.as_quat()
        fxq, fyq, fzq, fwq = q_seg[i].as_quat()
        fila = [i, f"{t[i]:.3f}", tramo[i], f"{X[i]:.3f}", f"{Y[i]:.3f}", f"{Z[i]:.3f}",
                f"{qdw:.5f}", f"{qdx:.5f}", f"{qdy:.5f}", f"{qdz:.5f}", f"{RX[i]:.3f}", f"{RY[i]:.3f}", f"{RZ[i]:.3f}",
                *(f"{v:.3f}" for v in CAM), f"{q_main[3]:.5f}", f"{q_main[0]:.5f}", f"{q_main[1]:.5f}", f"{q_main[2]:.5f}",
                *(f"{v:.3f}" for v in F[i]), f"{fwq:.5f}", f"{fxq:.5f}", f"{fyq:.5f}", f"{fzq:.5f}"]
        giro = rng.uniform(0, 2 * np.pi, 4)
        for j, (ox, oy) in enumerate(ROTORES):
            pos = P[i] + qd.apply([ox, oy, 0.0])
            qh = (qd * Rotation.from_euler("z", giro[j])).as_quat()
            fila += [*(f"{v:.4f}" for v in pos), f"{qh[3]:.5f}", f"{qh[0]:.5f}", f"{qh[1]:.5f}", f"{qh[2]:.5f}"]
        w.writerow(fila)
print(len(t), f"fotogramas a x{VEL:g} ({len(t) / FPS:.1f} s de vídeo)")
