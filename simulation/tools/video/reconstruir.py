"""Reconstruct the drone trajectory (ENU, metres from the takeoff point) from the
tracked pixels, apparent size and camera yaw.

Model:
  camera at CAM (fixed, 1.5 m above ground), absolute azimuth anchored per segment
  range      R = real_width * F / pixel_width, median-filtered
  azimuth    az = az_cam + atan((x - cx) / F)
  elevation  el = atan((skyline - y) / F) + skyline elevation of that view
  position   CAM + R cos(el) * (sin az, cos az),  h = 1.5 + R sin(el)
"""
import csv
import math

import numpy as np
from scipy.ndimage import median_filter, uniform_filter1d

F, CX = 690.0, 270.0
CAM = np.array([3.0, 3.0])          # m E, N of the takeoff point (opposite to the eucalyptus)
ANCHO = {"verde": 0.42, "cielo": 0.32, "plantilla": 0.35}
# Measured on the 60 fps frames: legs leave the shadow at 20.4 s; the video ends
# at 272.96 s with the drone still airborne
DESPEGUE, T_FIN = 20.4, 272.96      # s in the video

pista = list(csv.DictReader(open("pista.csv")))
t = np.array([float(r["t"]) for r in pista])
x = np.array([float(r["x"]) if r["x"] else np.nan for r in pista])
y = np.array([float(r["y"]) if r["y"] else np.nan for r in pista])
w_px = np.array([float(r["ancho"]) if r["ancho"] else np.nan for r in pista])
met = np.array([r["metodo"] for r in pista])
hz = np.array([float(r["horizonte"]) for r in pista])
hz[hz < 0] = np.nan
R = np.array([ANCHO.get(m, np.nan) for m in met]) * F / w_px
giro = list(csv.DictReader(open("giro.csv")))
yaw = np.array([float(r["yaw"]) for r in giro])[:len(t)]

# Absolute camera azimuth: starts looking SW at the takeoff point (225 deg).
# Yaw drift after the occlusion at 3:54 is re-anchored to the field view (same
# planks / olive hill landmarks as 1:36-2:54, azimuth ~150 deg).
az_cam = 225.0 - yaw
k = t >= 237
az_cam[k] = 150.0 - (yaw[k] - yaw[np.searchsorted(t, 240.0)])
vista_eucaliptos = (np.mod(az_cam, 360) > 185) & (np.mod(az_cam, 360) < 300)
offset_el = np.where(vista_eucaliptos, 22.0, 1.5)

ok = np.isfinite(x) & (met != "perdido")
# Drop the occluded stretch (arm in front of the lens)
ok &= ~((t > 233.5) & (t < 236))
# Range only from trustworthy sizes: template matches inherit the last width and
# the lime-prop spread is noisy when the drone is small
ancho_px = w_px
fiable = ok & np.isfinite(R) & ((met == "cielo") | ((met == "verde") & (ancho_px >= 20)))
logR = np.interp(t, t[fiable], np.log(R[fiable]))
# Manual correction after reviewing the frames: from 1:42 to 2:04 the drone sits
# on the far tree line (no sky around it, almost no reliable sizes) and is clearly
# far away, ~45-55 m from the camera
k = (t >= 100) & (t <= 126)
logR[k] = np.log(np.interp(t[k], [100, 113, 126], [45.0, 55.0, 50.0]))
logR = uniform_filter1d(median_filter(logR, size=31), 61)   # 3 s median, 6 s mean
Rf = np.clip(np.exp(logR), 2.0, 90.0)
xf = np.interp(t, t[ok], x[ok])
yf = np.interp(t, t[ok], y[ok])
hzf = np.where(np.isfinite(hz), hz, np.nan)

az = np.radians(az_cam + np.degrees(np.arctan((xf - CX) / F)))
el_rel = np.degrees(np.arctan((hzf - yf) / F))
el = el_rel + offset_el
h = 1.5 + Rf * np.sin(np.radians(el))
d = Rf * np.cos(np.radians(el))
# Frames without sky (camera looking down) or before takeoff: interpolate height
h = np.where(np.isfinite(h), h, np.nan)
d = np.where(np.isfinite(d), d, Rf)
h[t < DESPEGUE] = 0.0
hv = np.isfinite(h)
h = np.interp(t, t[hv], h[hv])
h = np.clip(uniform_filter1d(h, 31), 0.0, 15.0)
# Once airborne the drone never touched down in the video: at least 1.5 m
# (it sits on the horizon line, i.e. roughly at the camera's eye height)
en_vuelo = t >= DESPEGUE + 4
h[en_vuelo] = np.maximum(h[en_vuelo], 1.5)
h = uniform_filter1d(h, 11)

e = CAM[0] + d * np.sin(az)
n = CAM[1] + d * np.cos(az)
e[t < DESPEGUE], n[t < DESPEGUE] = 0.0, 0.0
# Blend smoothly from the takeoff spot during the climb (first 6 s)
w = np.clip((t - DESPEGUE) / 6.0, 0, 1)
e, n = e * w, n * w
e, n = uniform_filter1d(e, 31), uniform_filter1d(n, 31)

# Physical limits: the offboard flight was gentle; cap horizontal speed at V_MAX with a
# forward-backward rate limiter (keeps the shape, removes impossible jumps)
V_MAX, dt = 6.0, t[1] - t[0]
def limitar(a):
    a = a.copy()
    for rng in (range(1, len(a)), range(len(a) - 2, -1, -1)):
        for i in rng:
            j = i - 1 if rng.step == 1 else i + 1
            a[i] = a[j] + np.clip(a[i] - a[j], -V_MAX * dt / np.sqrt(2), V_MAX * dt / np.sqrt(2))
    return a
e, n = limitar(e), limitar(n)
e, n = uniform_filter1d(e, 21), uniform_filter1d(n, 21)

# Exact takeoff: on the ground until DESPEGUE, then the fast climb seen in the
# frames (~2 m in 1.5 s), a minimum-jerk ramp up to the reconstructed height at
# DESPEGUE + 2.5 s
suelo = t < DESPEGUE
e[suelo], n[suelo], h[suelo] = 0.0, 0.0, 0.0
k = (t >= DESPEGUE) & (t < DESPEGUE + 2.5)
u = (t[k] - DESPEGUE) / 2.5
h_obj = h[np.searchsorted(t, DESPEGUE + 2.5)]
h[k] = h_obj * u ** 3 * (10 - 15 * u + 6 * u ** 2)

keep = t <= T_FIN
with open("trayectoria_video.csv", "w", newline="") as f:
    wr = csv.writer(f)
    wr.writerow(["t_video", "este", "norte", "altura"])
    idx = np.nonzero(keep)[0][::2]              # 5 Hz keyframes
    for i in idx:
        wr.writerow([f"{t[i]:.1f}", f"{e[i]:.2f}", f"{n[i]:.2f}", f"{h[i]:.2f}"])
    # Close exactly at the end of the video
    i = idx[-1]
    wr.writerow([f"{T_FIN:.2f}", f"{e[i]:.2f}", f"{n[i]:.2f}", f"{h[i]:.2f}"])
np.savez("trayectoria_video.npz", t=t[keep], e=e[keep], n=n[keep], h=h[keep])
print("max dist %.1f m, max h %.1f m" % (np.hypot(e, n)[keep].max(), h[keep].max()))
vel = np.hypot(np.diff(e), np.diff(n)) / np.diff(t)
print("max horizontal speed %.1f m/s, p95 %.1f m/s" % (vel.max(), np.percentile(vel, 95)))
