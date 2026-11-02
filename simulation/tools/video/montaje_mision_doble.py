"""Mission video with both views filling the screen: fixed camera (left half) and follow camera (right half).

usage: montaje_mision_doble.py poses_fija.csv fija.mp4 seguimiento.mp4 salida.mp4 FFMPEG FUENTES [velocidad]

Both renders come from render_mision.py on the same flight log (poses_mision.py ... fija / seguimiento),
so frame i of each video is the same instant. Each half is a 960x1080 strip of its 1920x1080 render:
the follow view keeps its centre (the drone is always there); the fixed view's strip turns slowly
with the drone, like someone turning their head from the same spot (the camera itself never moves).
FUENTES holds Inter-300/400/500.ttf.
"""
import csv
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.ndimage import uniform_filter1d
from scipy.spatial.transform import Rotation

POSES, FIJA, SEG, OUT, FFMPEG, FUENTES = sys.argv[1:7]
VEL = sys.argv[7] if len(sys.argv) > 7 else "4"
W, H, FPS = 1920, 1080, 30
MITAD = W // 2
HFOV = 0.8                                      # cam_seg in the render world
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
F_EYE, F_TIT, F_DET, F_PIE = f(500, 24), f(300, 64), f(400, 28), f(500, 22)

filas = list(csv.DictReader(open(POSES)))
n = len(filas)
P = np.array([[float(r["x"]), float(r["y"]), float(r["z"])] for r in filas])
R = np.array([[float(r["rx"]), float(r["ry"]), float(r["rz"])] for r in filas])
nombres = [figura(r["tramo"]) for r in filas]
etiquetas = [nm for _, nm, _ in FIGURAS]
grupo = np.array([etiquetas.index(nombres[i][0]) if nombres[i][0] else -1 for i in range(n)])

# where the drone is in the fixed render -> left edge of the strip, smoothed over ~2 s
r0 = filas[0]
C = np.array([float(r0[k]) for k in ("fx", "fy", "fz")])
Rc = Rotation.from_quat([float(r0[k]) for k in ("fqx", "fqy", "fqz", "fqw")]).as_matrix()
d = (P - C) @ Rc                                  # camera link: x forward, y left, z up
u = W / 2 - (W / 2) / np.tan(HFOV / 2) * d[:, 1] / d[:, 0]
centro = np.clip(uniform_filter1d(u, 61, mode="nearest"), MITAD / 2, W - MITAD / 2)
x_fija = np.round(centro - MITAD / 2).astype(int)
x_seg = W // 4

# mini-map over the follow half, bottom right (the drone stays in the middle of that view)
MAPA = 280
ESC = MAPA / 17.0
cx_m, cy_m = W - 40 - MAPA / 2, H - 40 - MAPA / 2


def a_mapa(x, y):
    return cx_m + x * ESC, cy_m - y * ESC


def bandas():
    yy = np.mgrid[0:H, 0:W][0].astype(np.float32)
    arriba = np.clip(1 - yy / 170, 0, 1) * 0.5
    abajo = np.clip((yy - (H - 440)) / 440, 0, 1) ** 1.1 * 0.88
    im = Image.new("RGBA", (W, H), FONDO + (0,))
    im.putalpha(Image.fromarray((np.maximum(arriba, abajo) * 255).astype(np.uint8)))
    return im


BANDAS = bandas()


def capa(i, alfa):
    im = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    d.line([(MITAD, 0), (MITAD, H)], fill=FONDO + (255,), width=4)
    d.text((40, 36), "CÁMARA FIJA · DESDE EL SUELO", font=F_PIE, fill=BLANCO + (215,))
    d.text((MITAD + 40, 36), "CÁMARA DE SEGUIMIENTO", font=F_PIE, fill=BLANCO + (215,))
    nombre, detalle = nombres[i]
    a = int(255 * alfa)
    altura = f"{P[i, 2] - 0.16:.1f}".replace(".", ",")
    d.text((40, H - 196), "MISIÓN AUTÓNOMA · GEMELO DIGITAL", font=F_EYE, fill=VERDE + (255,))
    d.text((37, H - 164), nombre, font=F_TIT, fill=BLANCO + (a,))
    d.text((40, H - 76), f"{detalle}   ·   altura {altura} m   ·   ×{VEL}", font=F_DET, fill=BLANCO + (int(a * 0.85),))
    # mini-map
    x0, y0 = cx_m - MAPA / 2, cy_m - MAPA / 2
    d.rounded_rectangle((x0, y0, x0 + MAPA, y0 + MAPA), radius=14, fill=FONDO + (175,))
    for k in range(-8, 9, 2):
        d.line([a_mapa(k, -8.5), a_mapa(k, 8.5)], fill=BLANCO + (16,), width=1)
        d.line([a_mapa(-8.5, k), a_mapa(8.5, k)], fill=BLANCO + (16,), width=1)
    g = grupo[i]
    ref = [a_mapa(*R[j, :2]) for j in range(n) if grupo[j] == g]
    if len(ref) > 1:
        d.line(ref, fill=BLANCO + (90,), width=2)
    for gg in sorted(set(grupo[:i + 1])):
        pts = [a_mapa(*P[j, :2]) for j in range(i + 1) if grupo[j] == gg]
        if len(pts) > 1:
            d.line(pts, fill=VERDE + ((255,) if gg == g else (70,)), width=3 if gg == g else 2, joint="curve")
    px, py = a_mapa(*P[i, :2])
    d.ellipse((px - 7, py - 7, px + 7, py + 7), fill=BLANCO + (255,), outline=VERDE + (255,), width=3)
    d.text((x0 + 14, y0 + 10), "N ↑", font=F_PIE, fill=BLANCO + (150,))
    d.text((x0 + MAPA - 14, y0 + MAPA - 10), "planta", font=F_PIE, fill=BLANCO + (150,), anchor="rd")
    return im


def lector(path):
    return subprocess.Popen([FFMPEG, "-hide_banner", "-loglevel", "error", "-i", path,
                             "-f", "rawvideo", "-pix_fmt", "rgb24", "-"], stdout=subprocess.PIPE)


dec = [lector(FIJA), lector(SEG)]
enc = subprocess.Popen([FFMPEG, "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
                        "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-preset", "slow", "-crf", "18",
                        "-pix_fmt", "yuv420p", "-movflags", "+faststart", OUT], stdin=subprocess.PIPE)
cambio = [0] + [i for i in range(1, n) if nombres[i][0] != nombres[i - 1][0]]
for i in range(n):
    raw = [p.stdout.read(W * H * 3) for p in dec]
    if any(len(b) < W * H * 3 for b in raw):
        break
    fija, seg = (Image.frombuffer("RGB", (W, H), b) for b in raw)
    fr = Image.new("RGB", (W, H))
    fr.paste(fija.crop((x_fija[i], 0, x_fija[i] + MITAD, H)), (0, 0))
    fr.paste(seg.crop((x_seg, 0, x_seg + MITAD, H)), (MITAD, 0))
    desde = max(c for c in cambio if c <= i)
    fr = Image.alpha_composite(Image.alpha_composite(fr.convert("RGBA"), BANDAS), capa(i, min(1.0, (i - desde + 1) / 12)))
    fundido = min(1.0, (i + 1) / 15, (n - i) / 20)
    if fundido < 1:
        fr = Image.blend(Image.new("RGBA", (W, H), FONDO + (255,)), fr, fundido)
    enc.stdin.write(fr.convert("RGB").tobytes())
    if i % 300 == 0:
        print(f"{i}/{n}", flush=True)
enc.stdin.close()
enc.wait()
for p in dec:
    p.kill()
print("listo", OUT)
