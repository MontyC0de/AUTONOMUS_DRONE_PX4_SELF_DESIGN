"""Absolute heading and phone height from the drone's shadow at takeoff.

Frames at 1080x1920 (f=1680). Reference frame from orientacion.npz + ejes.npz.
"""
import numpy as np
from scipy.optimize import least_squares

o = np.load("orientacion.npz"); R = o["R"]; fps = float(o["fps"]); f = float(o["f1080"])
e = np.load("ejes.npz"); up, fwd0, east0 = e["up"], e["fwd0"], e["east0"]
K_inv = np.linalg.inv(np.array([[f, 0, 540.0], [0, f, 960.0], [0, 0, 1]]))
SUN_AZ, SUN_EL = np.radians(163.5), np.radians(60.95)


def rayo(t, u, v):
    d = R[int(round(t * fps))] @ K_inv @ np.array([u, v, 1.0])
    d = d / np.linalg.norm(d)
    # to local (east0, fwd0, up) coordinates
    return np.array([d @ east0, d @ fwd0, d @ up])


def sol_local(psi):
    # psi = world azimuth of fwd0 (clockwise from North). Sun azimuth in local frame:
    a = SUN_AZ - psi
    return np.array([np.sin(a) * np.cos(SUN_EL), np.cos(a) * np.cos(SUN_EL), np.sin(SUN_EL)])


# Observations (t, drone body centre pixel, shadow centre pixel, prop span px)
obs = [(21.8, (497, 745), (603, 1070), 175.0), (22.2, (460, 690), (596, 1117), 177.0)]
suelo = (19.0, (556, 970))
W_FRONTAL = 0.373     # m: motor spacing (side) + prop diameter, front view
H_CUERPO = 0.12       # m: body centre above the skids


def residuos(x):
    psi, hc = x
    s = sol_local(psi)
    r = []
    for t, pd, ps, wpx in obs:
        d, g = rayo(t, *pd), rayo(t, *ps)
        G = g * (-hc / g[2])                     # shadow on ground (z = -hc)
        rho = W_FRONTAL * f / wpx                 # range from apparent size
        X = d * rho
        Gp = X - ((X[2] + hc) / s[2]) * s        # where X's shadow should fall
        r += list((G - Gp)[:2])
    return np.array(r)


best = None
for psi0 in np.radians(np.arange(0, 360, 20)):
    sol = least_squares(residuos, [psi0, 1.4], bounds=([-10, 0.6], [10, 2.2]))
    if best is None or sol.cost < best.cost:
        best = sol
psi, hc = best.x
print("rumbo inicial de la cámara (fwd0): %.1f°   altura del móvil: %.2f m   residuo %.3f m"
      % (np.degrees(psi) % 360, hc, np.sqrt(np.mean(best.fun ** 2))))
# Takeoff point relative to the camera
g = rayo(*suelo[:1], *suelo[1])
P = g * (-hc / g[2])
az = np.arctan2(P[0], P[1]) + psi
dist = np.hypot(P[0], P[1])
print("despegue a %.2f m de la cámara, azimut %.1f°" % (dist, np.degrees(az) % 360))
for t, pd, ps, wpx in obs:
    d = rayo(t, *pd); rho = W_FRONTAL * f / wpx; X = d * rho
    print(f"t={t}: dron a {rho:.2f} m, altura sobre el suelo {X[2] + hc:.2f} m")
np.savez("calib.npz", psi=psi, hc=hc, despegue=np.array([dist * np.sin(az), dist * np.cos(az)]))
