"""Final mission video: follow-camera render + figure label + top-view mini-map with the flown trail.

usage: montaje_mision.py poses_mision.csv seguimiento.mp4 salida.mp4 FFMPEG FUENTES [velocidad]

FUENTES is a folder with Inter-300.ttf, Inter-400.ttf and Inter-500.ttf.
"""
import csv
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

POSES, VIDEO, OUT, FFMPEG, FUENTES = sys.argv[1:6]
VEL = sys.argv[6] if len(sys.argv) > 6 else "4"
W, H, FPS = 1920, 1080, 30
VERDE, BLANCO, FONDO = (166, 227, 74), (242, 242, 240), (10, 10, 10)

FIGURAS = [("Cuadrado", "Cuadrado", "8 × 8 m"), ("Orbita", "Órbita", "radio 6 m, mirando al centro"),
           ("Ocho", "Ocho", "14 × 7 m"), ("Helice", "Hélice", "radio 4 m, sube 4 m"),
           ("Regreso", "Regreso", "vuelta al punto de despegue"), ("Aproximacion", "Regreso", "vuelta al punto de despegue")]


def figura(tramo):
    for clave, nombre, detalle in FIGURAS:
        if tramo.startswith(clave):
            return nombre, detalle
    return "", ""


f = lambda peso, size: ImageFont.truetype(f"{FUENTES}/Inter-{peso}.ttf", size)
F_EYE, F_TIT, F_DET, F_PIE = f(500, 24), f(300, 76), f(400, 28), f(400, 24)

filas = list(csv.DictReader(open(POSES)))
n = len(filas)
P = np.array([[float(r["x"]), float(r["y"]), float(r["z"])] for r in filas])
R = np.array([[float(r["rx"]), float(r["ry"]), float(r["rz"])] for r in filas])
nombres = [figura(r["tramo"]) for r in filas]
grupo = np.array([[nm for _, nm, _ in FIGURAS].index(nombres[i][0]) if nombres[i][0] else -1 for i in range(n)])

# mini-map: top view, north up, 1 m = ESC px
MAPA, ESC, MARGEN = 400, 22.0, 64
cx_m, cy_m = W - MARGEN - MAPA / 2, H - MARGEN - MAPA / 2       # bottom right: the drone never goes there


def a_mapa(x, y):
    return cx_m + x * ESC, cy_m - y * ESC


def capa_mapa(i):
    im = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    x0, y0 = cx_m - MAPA / 2, cy_m - MAPA / 2
    d.rounded_rectangle((x0, y0, x0 + MAPA, y0 + MAPA), radius=18, fill=FONDO + (170,))
    for k in range(-8, 9, 2):                                    # 2 m grid
        a, b = a_mapa(k, -8.5), a_mapa(k, 8.5)
        d.line([a, b], fill=BLANCO + (18,), width=1)
        a, b = a_mapa(-8.5, k), a_mapa(8.5, k)
        d.line([a, b], fill=BLANCO + (18,), width=1)
    g = grupo[i]
    ref = [a_mapa(*R[j, :2]) for j in range(n) if grupo[j] == g]
    if len(ref) > 1:
        d.line(ref, fill=BLANCO + (90,), width=2)
    for gg in sorted(set(grupo[:i + 1])):                         # flown trail, current figure brighter
        pts = [a_mapa(*P[j, :2]) for j in range(i + 1) if grupo[j] == gg]
        if len(pts) > 1:
            d.line(pts, fill=VERDE + ((255,) if gg == g else (70,)), width=4 if gg == g else 2, joint="curve")
    px, py = a_mapa(*P[i, :2])
    d.ellipse((px - 9, py - 9, px + 9, py + 9), fill=BLANCO + (255,), outline=VERDE + (255,), width=3)
    d.text((x0 + 20, y0 + 16), "N ↑", font=F_PIE, fill=BLANCO + (150,))
    d.text((x0 + MAPA - 20, y0 + MAPA - 16), "Vista en planta", font=F_PIE, fill=BLANCO + (150,), anchor="rd")
    return im


def capa_texto(i, alfa):
    im = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    nombre, detalle = nombres[i]
    a = int(255 * alfa)
    d.text((128, H - 280), "MISIÓN AUTÓNOMA · GEMELO DIGITAL", font=F_EYE, fill=VERDE + (a,), spacing=0)
    d.text((124, H - 244), nombre, font=F_TIT, fill=BLANCO + (a,))
    d.text((128, H - 144), detalle, font=F_DET, fill=BLANCO + (int(a * 0.8),))
    altura = f"Altura {P[i, 2] - 0.16:4.1f} m".replace(".", ",")
    d.text((128, H - 88), f"{altura}   ·   PX4 SITL + Gazebo · ×{VEL}", font=F_PIE, fill=BLANCO + (200,))
    return im


def sombra_superior():
    """Soft dark band at the bottom so the white text and the map read over the ground."""
    yy = np.mgrid[0:H, 0:W][0]
    a = np.clip((yy - (H - 560)) / 560, 0, 1) ** 1.1 * 0.8
    im = Image.new("RGBA", (W, H), FONDO + (0,))
    im.putalpha(Image.fromarray((a * 255).astype(np.uint8)))
    return im


SOMBRA = sombra_superior()
dec = subprocess.Popen([FFMPEG, "-hide_banner", "-loglevel", "error", "-i", VIDEO, "-f", "rawvideo", "-pix_fmt", "rgb24", "-"],
                       stdout=subprocess.PIPE)
enc = subprocess.Popen([FFMPEG, "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
                        "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-preset", "slow", "-crf", "18",
                        "-pix_fmt", "yuv420p", "-movflags", "+faststart", OUT], stdin=subprocess.PIPE)
cambio = [0] + [i for i in range(1, n) if nombres[i][0] != nombres[i - 1][0]]
for i in range(n):
    raw = dec.stdout.read(W * H * 3)
    if len(raw) < W * H * 3:
        break
    fr = Image.frombuffer("RGB", (W, H), raw).convert("RGBA")
    desde = max(c for c in cambio if c <= i)
    alfa = min(1.0, (i - desde + 1) / 12)                       # label fades in on every new figure
    fr = Image.alpha_composite(fr, SOMBRA)
    fr = Image.alpha_composite(fr, capa_mapa(i))
    fr = Image.alpha_composite(fr, capa_texto(i, alfa))
    fundido = min(1.0, (i + 1) / 15, (n - i) / 20)               # fade in / out of the whole clip
    if fundido < 1:
        fr = Image.blend(Image.new("RGBA", (W, H), FONDO + (255,)), fr, fundido)
    enc.stdin.write(fr.convert("RGB").tobytes())
    if i % 300 == 0:
        print(f"{i}/{n}", flush=True)
enc.stdin.close()
enc.wait()
dec.kill()
print("listo", OUT)
