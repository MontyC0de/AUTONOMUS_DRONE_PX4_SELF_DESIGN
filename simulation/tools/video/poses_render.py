"""Per-frame poses (30 fps) for the two twin views.

usage: poses3.py replay_gemelo.csv poses3.csv

phone : fixed at the calibrated phone position, with the REAL camera orientation
        of every video frame (orientacion.npz + ejes.npz + calib.npz), i.e. the
        same pans and tilts as the person filming. The drone is never aimed at.
follow: behind the drone (opposite to its nose), 3.5 m back and 1.4 m up,
        smoothed over ~1 s, looking at the drone.
props : four separate models on the motor axes; once armed, a new angle every
        frame (a fast phone shutter freezes spinning props).
"""
import csv
import math
import sys

import numpy as np
from scipy.ndimage import uniform_filter1d
from scipy.spatial.transform import Rotation, Slerp

REPLAY, OUT = sys.argv[1], sys.argv[2]
FPS, T_FIN, T_ARMADO = 30.0, 272.96, 16.4
Z_SUELO = 0.16
ROTORES = [(0.1228, -0.1228), (-0.1228, 0.1228), (0.1228, 0.1228), (-0.1228, -0.1228)]

o = np.load("orientacion.npz"); Rv = o["R"]; fps_v = float(o["fps"])
e = np.load("ejes.npz"); up, fwd0, east0 = e["up"], e["fwd0"], e["east0"]
c = np.load("calib.npz"); psi, hc = float(c["psi"]), float(c["hc"])
CAM = np.array([-c["despegue"][0], -c["despegue"][1], hc])
# reference frame -> world ENU
M = np.array([np.cos(psi) * east0 + np.sin(psi) * fwd0, -np.sin(psi) * east0 + np.cos(psi) * fwd0, up])

t = np.arange(0, T_FIN, 1 / FPS)
tv = np.arange(len(Rv)) / fps_v
cam_rot = Slerp(tv, Rotation.from_matrix(Rv))(np.clip(t, tv[0], tv[-1])).as_matrix()


def gz_cam(Rc):
    """Camera (x right, y down, z fwd) -> Gazebo camera link (x fwd, y left, z up)."""
    W = M @ Rc
    return Rotation.from_matrix(np.column_stack([W[:, 2], -W[:, 0], -W[:, 1]])).as_quat()  # x,y,z,w


g = list(csv.DictReader(open(REPLAY)))
tg = np.array([float(r["t_video"]) for r in g])
col = lambda k: np.array([float(r[k]) for r in g])
ix = lambda a: np.interp(t, tg, a)
X, Y = ix(col("e")), ix(col("n"))
Zd = -col("d")
Zd -= Zd[np.searchsorted(tg, T_ARMADO)]                 # barometer offset on the ground
Z = Z_SUELO + np.maximum(ix(Zd), 0.0)
roll, pitch = ix(np.radians(col("roll"))), ix(np.radians(col("pitch")))
yaw = ix(np.unwrap(np.radians(col("yaw"))))
q_dron = Rotation.from_euler("xyz", np.column_stack([roll, -pitch, np.pi / 2 - yaw]))   # FRD/NED -> FLU/ENU

# Follow camera: behind the nose (smoothed heading), 3.5 m back, 1.4 m up
yaw_s = uniform_filter1d(yaw, 61)
P = np.column_stack([X, Y, Z])
detras = -np.column_stack([np.sin(yaw_s), np.cos(yaw_s), np.zeros_like(yaw_s)])
F = uniform_filter1d(P + 3.5 * detras + [0, 0, 1.4], 31, axis=0)
F[:, 2] = np.maximum(F[:, 2], 1.0)
v = P - F
fy = np.arctan2(v[:, 1], v[:, 0])
fp = np.arctan2(v[:, 2], np.hypot(v[:, 0], v[:, 1]))
q_seg = Rotation.from_euler("ZY", np.column_stack([fy, -fp]))

rng = np.random.default_rng(3)
ang0 = rng.uniform(0, 2 * np.pi, 4)
with open(OUT, "w", newline="") as f:
    w = csv.writer(f)
    cab = ["frame", "t", "x", "y", "z", "qw", "qx", "qy", "qz",
           "cx", "cy", "cz", "cqw", "cqx", "cqy", "cqz", "fx", "fy", "fz", "fqw", "fqx", "fqy", "fqz"]
    for i in range(4):
        cab += [f"h{i}{k}" for k in ("x", "y", "z", "qw", "qx", "qy", "qz")]
    w.writerow(cab)
    for i in range(len(t)):
        qd = q_dron[i]
        qdx, qdy, qdz, qdw = qd.as_quat()
        cxq, cyq, czq, cwq = gz_cam(cam_rot[i])
        fxq, fyq, fzq, fwq = q_seg[i].as_quat()
        fila = [i, f"{t[i]:.4f}", f"{X[i]:.3f}", f"{Y[i]:.3f}", f"{Z[i]:.3f}",
                f"{qdw:.5f}", f"{qdx:.5f}", f"{qdy:.5f}", f"{qdz:.5f}",
                *(f"{v:.3f}" for v in CAM), f"{cwq:.5f}", f"{cxq:.5f}", f"{cyq:.5f}", f"{czq:.5f}",
                *(f"{v:.3f}" for v in F[i]), f"{fwq:.5f}", f"{fxq:.5f}", f"{fyq:.5f}", f"{fzq:.5f}"]
        giro = rng.uniform(0, 2 * np.pi, 4) if t[i] >= T_ARMADO else ang0
        for j, (ox, oy) in enumerate(ROTORES):
            pos = P[i] + qd.apply([ox, oy, 0.0])
            qh = (qd * Rotation.from_euler("z", giro[j])).as_quat()
            fila += [*(f"{v:.4f}" for v in pos), f"{qh[3]:.5f}", f"{qh[0]:.5f}", f"{qh[1]:.5f}", f"{qh[2]:.5f}"]
        w.writerow(fila)
print(len(t), "fotogramas; cámara del móvil en", CAM.round(2))
