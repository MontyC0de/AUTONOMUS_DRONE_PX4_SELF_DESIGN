"""Drone trajectory from the video with the camera's real motion removed.

Per frame (10 fps):
  direction : drone pixel (pista.csv, 540 -> 1080) through the camera orientation
              R (orientacion.npz) and the absolute heading psi from the shadow
              calibration (calib.npz): rays in world ENU.
  range     : apparent tip-to-tip width (anchos.csv auto + manual keyframes +
              old lime extent), with the real width for the viewing angle
              W = 2a(|cos th| + |sin th|) + prop diameter.
  position  : camera + range * direction; on the ground until the takeoff.
Output trayectoria_video.csv: t_video, este, norte, altura, yaw (heading, deg).
"""
import csv

import numpy as np
from scipy.ndimage import median_filter, uniform_filter1d

FPS, F = 10.0, 1680.0
A, PROP = 0.1228, 0.127
DESPEGUE, T_FIN = 20.4, 272.96
RUMBO = 287.0                        # drone heading, constant except the spin
GIRO = (160.3, 166.1)                # 360 deg yaw spin seen at 2:41-2:47, 0.6 s early for PX4 lag
# Tip-to-tip width read by hand on 1080 px crops with a ruler (px)
MANUAL = {19: 170, 20.6: 187, 21.6: 189, 22.6: 162, 23.6: 175, 24.6: 161, 25.6: 154, 26.6: 146,
          27.6: 150, 28.6: 140, 29.6: 122, 35: 121, 50: 125, 56: 123, 58: 133, 60: 94, 62: 107,
          64: 115, 66: 108, 68: 102, 70: 137, 72: 117, 74: 119, 75: 121, 76: 112, 78: 106, 80: 104,
          82: 104, 84: 115, 86: 146, 88: 165, 90: 174, 92: 178, 94: 207, 96: 134, 98: 82, 100: 63,
          110: 47, 120: 50, 165: 35, 174: 43, 176: 60, 178: 108, 180: 111, 182: 118, 184: 120,
          186: 105, 188: 100, 190: 113, 192: 108, 194: 92, 196: 65, 198: 43, 200: 29, 210: 18,
          222: 70, 224: 127, 226: 143, 228: 151, 230: 161, 232: 158, 234: 153, 236: 133, 238: 130,
          240: 113, 242: 106, 244: 110, 246: 106, 248: 120, 250: 110, 252: 114, 254: 117, 256: 120,
          258: 138, 260: 115, 262: 120, 264: 111, 266: 112, 268: 114, 270: 118, 272: 120,
          101: 55, 102: 55, 106: 47, 107: 47, 108: 47, 109: 48, 112: 56, 115: 49, 130: 48,
          155: 36, 160: 35, 170: 37, 203: 18, 206: 17, 209: 18, 212: 17, 215: 19, 218: 29}
# Drone position read by hand (1080 px) where the old track locked onto the lamp
# post / trees: frames in these windows use only these anchors
POS_MANUAL = {103: (470, 794), 103.5: (480, 794), 104: (468, 806), 104.5: (450, 818),
              105: (432, 826), 105.5: (412, 830), 139: (386, 752), 140: (412, 762),
              141: (436, 762), 142: (406, 752), 143: (398, 732), 144: (390, 716),
              145: (380, 724), 146: (364, 730), 147: (350, 740), 148: (370, 736),
              149: (392, 736), 150: (400, 738), 151: (396, 742), 152: (402, 746),
              153: (360, 732)}
VENTANAS_POS = [(102.6, 105.9), (138.6, 153.6)]
# Stretches where the automatic width is unreliable (ground / tree background):
# only the hand readings are used there
SOLO_MANUAL = [(20.4, 30.5), (55, 101), (102.6, 105.9), (138.6, 153.6), (173, 195), (200, 218), (221, 273)]

o = np.load("orientacion.npz"); Rcam = o["R"]
e = np.load("ejes.npz"); up, fwd0, east0 = e["up"], e["fwd0"], e["east0"]
c = np.load("calib.npz"); psi, hc = float(c["psi"]), float(c["hc"])
desp = c["despegue"]                         # takeoff relative to the phone (E, N)
CAM = np.array([-desp[0], -desp[1], hc])     # phone relative to the takeoff point
Ki = np.linalg.inv(np.array([[F, 0, 540.0], [0, F, 960.0], [0, 0, 1]]))

pista = list(csv.DictReader(open("pista.csv")))
anch = list(csv.DictReader(open("anchos.csv")))
N = min(len(pista), len(anch), len(Rcam))
t = np.arange(N) / FPS
ok = np.array([bool(pista[k]["x"]) and pista[k]["metodo"] != "perdido" for k in range(N)])
ok &= ~((t > 233.5) & (t < 236.0))          # arm in front of the lens
u = np.array([float(pista[k]["x"]) * 2 if pista[k]["x"] else np.nan for k in range(N)])
v = np.array([float(pista[k]["y"]) * 2 if pista[k]["y"] else np.nan for k in range(N)])
for a, b in VENTANAS_POS:
    ok[(t >= a) & (t <= b)] = False
for T, (px, py) in POS_MANUAL.items():
    k = int(round(T * FPS))
    u[k], v[k], ok[k] = px, py, True


def a_mundo(d):
    le, lf, lu = d @ east0, d @ fwd0, d @ up
    return np.array([le * np.cos(psi) + lf * np.sin(psi), -le * np.sin(psi) + lf * np.cos(psi), lu])


D = np.full((N, 3), np.nan)
for k in np.flatnonzero(ok):
    d = Rcam[k] @ Ki @ np.array([u[k], v[k], 1.0])
    D[k] = a_mundo(d / np.linalg.norm(d))

# Heading per frame (constant + spin) and the real width for the viewing angle
rumbo = np.full(N, RUMBO)
k = (t >= GIRO[0]) & (t <= GIRO[1])
s = (t[k] - GIRO[0]) / (GIRO[1] - GIRO[0])
rumbo[k] = RUMBO + 360.0 * s ** 3 * (10 - 15 * s + 6 * s ** 2)
rumbo[t > GIRO[1]] = RUMBO + 360.0
los = np.degrees(np.arctan2(D[:, 0], D[:, 1]))
th = np.radians(los - rumbo)
W_motores = 2 * A * (np.abs(np.cos(th)) + np.abs(np.sin(th)))   # motor axes extent

# Width sources (1080 px)
w_auto = np.array([float(a["ancho"]) if a["ancho"] else np.nan for a in anch[:N]])
w_viejo = np.array([float(p["ancho"]) * 2 if p["ancho"] and p["metodo"] == "verde" else np.nan for p in pista[:N]])
w_viejo[w_viejo < 40] = np.nan
w = np.where(np.isfinite(w_auto), w_auto, w_viejo)
w[~ok] = np.nan
# outlier rejection against a 3 s running median in log space
lw = np.log(w)
idx = np.flatnonzero(np.isfinite(lw))
med = median_filter(np.interp(np.arange(N), idx, lw[idx]), size=31)
w[np.abs(lw - med) > np.log(1.35)] = np.nan
# automatic width vs hand readings in the sky: correct its small bias
pares = [(w[int(round(T * FPS))], m) for T, m in MANUAL.items()
         if np.isfinite(w[int(round(T * FPS))]) and not any(a <= T <= b for a, b in SOLO_MANUAL)]
sesgo = np.median([a / m for a, m in pares]) if pares else 1.0
print(f"sesgo de la medida automática: {sesgo:.2f} ({len(pares)} pares)")
w = w / sesgo
for a, b in SOLO_MANUAL:
    w[(t >= a) & (t <= b)] = np.nan
for T, m in MANUAL.items():
    w[np.abs(t - T) <= 0.15] = m

# Spinning props are invisible on a small drone (< ~20 px): only motors (bells
# 28 mm) count; fully visible from ~35 px
vis = np.clip((w - 20.0) / 15.0, 0.0, 1.0)
W_real = W_motores + 0.028 + (PROP - 0.028) * np.where(np.isfinite(vis), vis, 1.0)
rho = W_real * F / w
idx = np.flatnonzero(np.isfinite(rho) & ok)
lr = np.interp(np.arange(N), idx, np.log(rho[idx]))
lr = uniform_filter1d(median_filter(lr, size=11), 15)        # 1 s median, 1.5 s mean
rho = np.exp(lr)

# Directions for lost frames: interpolate the unit vectors
for j in range(3):
    D[:, j] = np.interp(np.arange(N), np.flatnonzero(ok), D[ok, j])
# 0.5 s median: removes single-frame tracking glitches, keeps real motion
D = median_filter(D, size=(5, 1))
D /= np.linalg.norm(D, axis=1, keepdims=True)
X = CAM + rho[:, None] * D

# Ground segment and takeoff
X[t < DESPEGUE] = [0.0, 0.0, 0.0]
X[:, 2] = np.maximum(X[:, 2], 0.0)
# light smoothing of the flight part (0.5 s), keeping the takeoff instant
vuelo = t >= DESPEGUE
for j in range(3):
    X[vuelo, j] = uniform_filter1d(X[vuelo, j], 5)

# Speed limit (5 m/s) with a forward-backward pass, then 0.5 s smoothing
V_MAX = 5.0
for paso in (range(1, N), range(N - 2, -1, -1)):
    for i in paso:
        j = i - 1 if paso.step == 1 else i + 1
        dlt = X[i] - X[j]
        n = np.linalg.norm(dlt)
        if n > V_MAX / FPS:
            X[i] = X[j] + dlt * (V_MAX / FPS / n)
for j in range(3):
    X[vuelo, j] = uniform_filter1d(X[vuelo, j], 5)
X[t < DESPEGUE] = [0.0, 0.0, 0.0]

keep = t <= T_FIN
with open("trayectoria_video.csv", "w", newline="") as f:
    wr = csv.writer(f)
    wr.writerow(["t_video", "este", "norte", "altura", "yaw"])
    for k in np.flatnonzero(keep)[::2]:
        wr.writerow([f"{t[k]:.1f}", f"{X[k, 0]:.2f}", f"{X[k, 1]:.2f}", f"{X[k, 2]:.2f}", f"{rumbo[k]:.1f}"])
    k = np.flatnonzero(keep)[-1]
    wr.writerow([f"{T_FIN:.2f}", f"{X[k, 0]:.2f}", f"{X[k, 1]:.2f}", f"{X[k, 2]:.2f}", f"{rumbo[k]:.1f}"])
np.savez("tray2.npz", t=t, X=X, rho=rho, D=D, rumbo=rumbo, cam=CAM)
dist = np.hypot(X[:, 0], X[:, 1])
vel = np.linalg.norm(np.diff(X, axis=0), axis=1) * FPS
print("cámara en (%.2f, %.2f, %.2f) m" % tuple(CAM))
print("distancia máx al despegue %.1f m, altura máx %.1f m, vel. máx %.1f m/s (p95 %.1f)"
      % (dist.max(), X[:, 2].max(), vel.max(), np.percentile(vel, 95)))
for T in (25, 40, 60, 80, 95, 105, 115, 125, 140, 155, 165, 180, 195, 210, 225, 240, 255, 270):
    k = int(T * FPS)
    print(f"{T:4d}s  E {X[k,0]:6.1f} N {X[k,1]:6.1f} h {X[k,2]:5.1f}  dist {dist[k]:5.1f}  rango {rho[k]:5.1f}")
