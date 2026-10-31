"""Camera orientation for every frame: odometry from homographies + loop closures
between keyframes that see the same scene, solved as a rotation pose graph.

Camera frame: x right, y down, z forward. R[k] maps camera -> reference (first
keyframe) coordinates. Output npz: R (N x 3 x 3) at FPS.

usage: orientacion.py camara.npz orientacion.npz [f1080]
"""
import os
import sys
from multiprocessing import Pool

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation, Slerp

IN, OUT = sys.argv[1], sys.argv[2]
F1080 = float(sys.argv[3]) if len(sys.argv) > 3 else 1680.0
MAX_CIERRES_POR_KF = 8

d = np.load(IN)
Hs, inl, keys, kt, fps = d["H"], d["inliers"], d["keys"], d["kt"], float(d["fps"])
N = len(Hs)
K = np.array([[F1080, 0, 540.0], [0, F1080, 960.0], [0, 0, 1]])
K540 = np.array([[F1080 / 2, 0, 270.0], [0, F1080 / 2, 480.0], [0, 0, 1]])


def rot_de_h(H, Km):
    M = np.linalg.inv(Km) @ H @ Km
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    return R if np.linalg.det(R) > 0 else -R


# Frame odometry: d_k = Rrel[k] d_{k-1}  ->  R_k = R_{k-1} Rrel[k]^T
Rodo = np.empty((N, 3, 3))
Rodo[0] = np.eye(3)
for k in range(1, N):
    ok = np.isfinite(Hs[k]).all() and inl[k] >= 15
    Rodo[k] = Rodo[k - 1] @ (rot_de_h(Hs[k], K).T if ok else np.eye(3))
kidx = np.round(kt * fps).astype(int)
M = len(kidx)

# Loop closures, all keyframe pairs, in parallel
_sift = None
_feats = None


def _init():
    global _sift, _feats
    _sift = cv2.SIFT_create(nfeatures=1500)
    _feats = [_sift.detectAndCompute(k, None) for k in keys]


def _emparejar(i):
    bf = cv2.BFMatcher(cv2.NORM_L2)
    Kinv = np.linalg.inv(K540)
    out = []
    ki, di = _feats[i]
    if di is None or len(ki) < 30:
        return out
    for j in range(i + 5, M):
        kj, dj = _feats[j]
        if dj is None or len(kj) < 30:
            continue
        m = [a for a, b in bf.knnMatch(di, dj, k=2) if a.distance < 0.7 * b.distance]
        if len(m) < 40:
            continue
        p0 = np.float32([ki[a.queryIdx].pt for a in m])
        p1 = np.float32([kj[a.trainIdx].pt for a in m])
        H, msk = cv2.findHomography(p0, p1, cv2.RANSAC, 2.5)
        if H is None or msk.sum() < 40:
            continue
        Mh = Kinv @ H @ K540
        Mh = Mh / np.cbrt(np.linalg.det(Mh))
        if np.linalg.norm(Mh @ Mh.T - np.eye(3)) > 0.06:
            continue
        out.append((i, j, rot_de_h(H, K540).T, int(msk.sum())))
    return out


cache = OUT.replace(".npz", "_cierres.npz")
if os.path.exists(cache):
    c = np.load(cache)
    cierres = [(int(a), int(b), Rm, int(n)) for (a, b, n), Rm in zip(c["ijn"], c["R"])]
else:
    with Pool(os.cpu_count(), initializer=_init) as pool:
        cierres = [x for lista in pool.map(_emparejar, range(M)) for x in lista]
    np.savez(cache, ijn=np.array([(i, j, n) for i, j, _, n in cierres]), R=np.array([Rm for _, _, Rm, _ in cierres]))
print(f"{M} fotogramas clave, {len(cierres)} cierres de bucle encontrados")

# Keep the strongest closures per keyframe
cierres.sort(key=lambda c: -c[3])
cuenta = np.zeros(M, int)
sel = []
for c in cierres:
    if cuenta[c[0]] < MAX_CIERRES_POR_KF or cuenta[c[1]] < MAX_CIERRES_POR_KF:
        sel.append(c)
        cuenta[c[0]] += 1
        cuenta[c[1]] += 1
print(f"{len(sel)} cierres usados")

aristas = [(i, i + 1, Rodo[kidx[i]].T @ Rodo[kidx[i + 1]], 1.0) for i in range(M - 1)]
aristas += [(i, j, Rm, 3.0) for i, j, Rm, _ in sel]
I = np.array([a[0] for a in aristas])
J = np.array([a[1] for a in aristas])
Rm_inv = Rotation.from_matrix(np.array([a[2] for a in aristas])).inv()
w = np.array([a[3] for a in aristas])


def residuos(x):
    R = Rotation.from_rotvec(x.reshape(-1, 3))
    err = Rm_inv * R[I].inv() * R[J]
    return np.concatenate([x[:3] * 100, (err.as_rotvec() * w[:, None]).ravel()])


spars = lil_matrix((3 + 3 * len(aristas), 3 * M), dtype=int)
spars[0:3, 0:3] = 1
for e, (i, j) in enumerate(zip(I, J)):
    spars[3 + 3 * e:6 + 3 * e, 3 * i:3 * i + 3] = 1
    spars[3 + 3 * e:6 + 3 * e, 3 * j:3 * j + 3] = 1

x0 = Rotation.from_matrix(Rodo[kidx]).as_rotvec().ravel()
# Robust loss: a broken odometry link (arm in front of the lens) must not drag the closures
sol = least_squares(residuos, x0, jac_sparsity=spars, loss="soft_l1", f_scale=0.05, x_scale="jac", max_nfev=200)
print("coste %.2f -> %.2f" % (np.sum(residuos(x0) ** 2), np.sum(sol.fun ** 2)))
Rk = Rotation.from_rotvec(sol.x.reshape(-1, 3)).as_matrix()

# Per frame: odometry between keyframes with the keyframe correction slerped in
corr = Rotation.from_matrix(np.einsum("kij,kjl->kil", Rk, np.transpose(Rodo[kidx], (0, 2, 1))))
interp = Slerp(kidx, corr)
Rf = np.array([interp(min(max(k, kidx[0]), kidx[-1])).as_matrix() @ Rodo[k] for k in range(N)])
np.savez(OUT, R=Rf, Rodo=Rodo, fps=fps, f1080=F1080)
ang = np.degrees(Rotation.from_matrix(np.einsum("kji,kjl->kil", Rodo[kidx], Rk)).magnitude())
print("corrección máxima de deriva: %.1f°" % ang.max())
