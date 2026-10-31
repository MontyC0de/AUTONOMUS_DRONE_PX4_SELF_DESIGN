"""Three-panel video: real flight | twin, same camera | twin, follow camera.

usage: montaje3.py REAL.mp4 MOVIL.mp4 SEGUIMIENTO.mp4 OUT.mp4 FFMPEG [start_s] [dur_s]
"""
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

REAL, MOVIL, SEG, OUT, FF = sys.argv[1:6]
INICIO = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0
DUR = float(sys.argv[7]) if len(sys.argv) > 7 else 272.96
W, H, GAP, FPS = 720, 1280, 8, 30
FONT = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 32)
PANELES = [("Vuelo real", REAL, INICIO), ("Gemelo digital", MOVIL, INICIO), ("Gemelo · seguimiento", SEG, INICIO)]


def lector(ruta, inicio):
    cmd = [FF, "-hide_banner", "-loglevel", "error", "-ss", f"{inicio:.3f}", "-i", ruta, "-t", f"{DUR:.3f}",
           "-vf", f"fps={FPS},scale={W}:{H}", "-f", "rawvideo", "-pix_fmt", "rgb24", "-"]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE)


def etiqueta(img, texto, t=None):
    d = ImageDraw.Draw(img, "RGBA")
    linea = texto if t is None else f"{texto}   {int(t) // 60}:{int(t) % 60:02d}"
    x0, y0, x1, y1 = d.textbbox((0, 0), linea, font=FONT)
    d.rounded_rectangle((22, 22, 22 + (x1 - x0) + 34, 22 + (y1 - y0) + 24), radius=14, fill=(0, 0, 0, 150))
    d.text((22 + 17, 22 + 12 - y0), linea, font=FONT, fill=(255, 255, 255, 255))


lectores = [lector(r, i) for _, r, i in PANELES]
anchura = 3 * W + 2 * GAP
enc = subprocess.Popen([FF, "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
                        "-s", f"{anchura}x{H}", "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-crf", "21",
                        "-preset", "medium", "-pix_fmt", "yuv420p", "-movflags", "+faststart", OUT],
                       stdin=subprocess.PIPE)
tam = W * H * 3
ultimo = [np.zeros((H, W, 3), np.uint8) for _ in PANELES]
n = 0
while True:
    datos = [lr.stdout.read(tam) for lr in lectores]
    if len(datos[0]) < tam:
        break
    t = INICIO + n / FPS
    lienzo = Image.new("RGB", (anchura, H), (12, 12, 12))
    for j, ((nombre, _, _), b) in enumerate(zip(PANELES, datos)):
        if len(b) == tam:
            ultimo[j] = np.frombuffer(b, np.uint8).reshape(H, W, 3)
        im = Image.fromarray(ultimo[j])
        etiqueta(im, nombre, t if j == 0 else None)
        lienzo.paste(im, (j * (W + GAP), 0))
    enc.stdin.write(lienzo.tobytes())
    n += 1
    if n % 1800 == 0:
        print(f"{n} fotogramas ({n / FPS:.0f} s)", flush=True)
enc.stdin.close()
enc.wait()
print(f"listo: {n} fotogramas -> {OUT}")
