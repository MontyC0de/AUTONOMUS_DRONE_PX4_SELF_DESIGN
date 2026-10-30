"""Side-by-side video: real flight (left) and digital twin (right), synchronised.

usage: montaje.py REAL.mp4 GEMELO.mp4 OUT.mp4 FFMPEG [start_s] [dur_s] [gemelo_start_s]
"""
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

REAL, GEMELO, OUT, FF = sys.argv[1:5]
INICIO = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
DUR = float(sys.argv[6]) if len(sys.argv) > 6 else 272.96
INICIO_G = float(sys.argv[7]) if len(sys.argv) > 7 else INICIO
W, H, GAP, FPS = 720, 1280, 8, 30
FONT = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 34)


def lector(ruta, inicio):
    cmd = [FF, "-hide_banner", "-loglevel", "error", "-ss", f"{inicio:.3f}", "-i", ruta, "-t", f"{DUR:.3f}",
           "-vf", f"fps={FPS},scale={W}:{H}", "-f", "rawvideo", "-pix_fmt", "rgb24", "-"]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE)


def etiqueta(img, texto, t):
    d = ImageDraw.Draw(img, "RGBA")
    linea = f"{texto}   {int(t) // 60}:{int(t) % 60:02d}"
    x0, y0, x1, y1 = d.textbbox((0, 0), linea, font=FONT)
    d.rounded_rectangle((24, 24, 24 + (x1 - x0) + 36, 24 + (y1 - y0) + 26), radius=14, fill=(0, 0, 0, 150))
    d.text((24 + 18, 24 + 13 - y0), linea, font=FONT, fill=(255, 255, 255, 255))


real, gem = lector(REAL, INICIO), lector(GEMELO, INICIO_G)
enc = subprocess.Popen([FF, "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
                        "-s", f"{2 * W + GAP}x{H}", "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-crf", "20",
                        "-preset", "medium", "-pix_fmt", "yuv420p", "-movflags", "+faststart", OUT],
                       stdin=subprocess.PIPE)
n, tam = 0, W * H * 3
ultimo_g = np.zeros((H, W, 3), np.uint8)
while True:
    br = real.stdout.read(tam)
    if len(br) < tam:
        break
    bg = gem.stdout.read(tam)
    if len(bg) == tam:
        ultimo_g = np.frombuffer(bg, np.uint8).reshape(H, W, 3)
    t = INICIO + n / FPS
    lienzo = Image.new("RGB", (2 * W + GAP, H), (12, 12, 12))
    izq = Image.fromarray(np.frombuffer(br, np.uint8).reshape(H, W, 3))
    der = Image.fromarray(ultimo_g)
    etiqueta(izq, "Vuelo real", t)
    etiqueta(der, "Gemelo digital", t)
    lienzo.paste(izq, (0, 0))
    lienzo.paste(der, (W + GAP, 0))
    enc.stdin.write(lienzo.tobytes())
    n += 1
    if n % 900 == 0:
        print(f"{n} fotogramas ({n / FPS:.0f} s)", flush=True)
enc.stdin.close()
enc.wait()
print(f"listo: {n} fotogramas -> {OUT}")
