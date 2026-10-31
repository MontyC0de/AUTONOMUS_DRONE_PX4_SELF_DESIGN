"""Vertical and horizontal axes of the camera reference frame -> ejes.npz.

The phone is held roughly level: its right axis stays horizontal, so the vertical
is the direction least seen by the right axis over the whole video.
"""
import numpy as np

o = np.load("orientacion.npz")
R = o["R"]
x_cam, y_cam, z_cam = R[:, :, 0], R[:, :, 1], R[:, :, 2]
_, V = np.linalg.eigh(x_cam.T @ x_cam)
up = V[:, 0]
if np.mean(y_cam @ up) > 0:          # image y points down
    up = -up
fwd0 = z_cam[0] - (z_cam[0] @ up) * up
fwd0 /= np.linalg.norm(fwd0)
east0 = np.cross(fwd0, up)
np.savez("ejes.npz", up=up, fwd0=fwd0, east0=east0)
pitch = np.degrees(np.arcsin(np.clip(z_cam @ up, -1, 1)))
print(f"cabeceo inicial {pitch[0]:.1f}°, rango {pitch.min():.0f}..{pitch.max():.0f}°")
