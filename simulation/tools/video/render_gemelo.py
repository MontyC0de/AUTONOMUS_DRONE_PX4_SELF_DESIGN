"""Render the two twin views frame by frame from poses3.csv (paused Gazebo).

usage: python3 render3.py poses3.csv movil.mp4 seguimiento.mp4 FFMPEG [first] [n]
"""
import csv
import subprocess
import sys
import threading
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.world_control_pb2 import WorldControl
from gz.transport13 import Node

POSES, OUT1, OUT2, FFMPEG = sys.argv[1:5]
FIRST = int(sys.argv[5]) if len(sys.argv) > 5 else 0
N = int(sys.argv[6]) if len(sys.argv) > 6 else 10 ** 9
W, H, WORLD = 720, 1280, "render"
node = Node()
cond = threading.Condition()
img = {"cam_movil": [-1.0, None], "cam_seguimiento": [-1.0, None]}


def suscribir(topic):
    def cb(msg):
        with cond:
            img[topic] = [msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9, msg.data]
            cond.notify_all()
    node.subscribe(Image, f"/{topic}", cb)


for k in img:
    suscribir(k)


def set_pose(name, x, y, z, qw, qx, qy, qz):
    p = Pose()
    p.name = name
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z = qw, qx, qy, qz
    node.request(f"/world/{WORLD}/set_pose", p, Pose, Boolean, 2000)


def step():
    c = WorldControl()
    c.pause = True
    c.multi_step = 1
    node.request(f"/world/{WORLD}/control", c, WorldControl, Boolean, 2000)


def codificador(out):
    return subprocess.Popen([FFMPEG, "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
                             "-s", f"{W}x{H}", "-r", "30", "-i", "-", "-c:v", "libx264", "-crf", "18",
                             "-preset", "medium", "-pix_fmt", "yuv420p", out], stdin=subprocess.PIPE)


filas = list(csv.DictReader(open(POSES)))[FIRST:FIRST + N]
enc = {"cam_movil": codificador(OUT1), "cam_seguimiento": codificador(OUT2)}
for _ in range(3):
    step()
time.sleep(1.0)
t0 = time.time()
for k, r in enumerate(filas):
    f = lambda c: float(r[c])
    set_pose("dron", *(f(c) for c in ("x", "y", "z", "qw", "qx", "qy", "qz")))
    for j in range(4):
        set_pose(f"h{j}", *(f(f"h{j}{c}") for c in ("x", "y", "z", "qw", "qx", "qy", "qz")))
    set_pose("cam_movil", *(f(c) for c in ("cx", "cy", "cz", "cqw", "cqx", "cqy", "cqz")))
    set_pose("cam_seguimiento", *(f(c) for c in ("fx", "fy", "fz", "fqw", "fqx", "fqy", "fqz")))
    with cond:
        antes = {kk: v[0] for kk, v in img.items()}
    step()
    step()
    with cond:
        if not cond.wait_for(lambda: all(img[kk][0] > antes[kk] for kk in img), timeout=8.0):
            print("sin imagen en el fotograma", FIRST + k, file=sys.stderr)
        datos = {kk: v[1] for kk, v in img.items()}
    for kk, e in enc.items():
        e.stdin.write(datos[kk])
    if k % 300 == 0:
        el = time.time() - t0
        print(f"fotograma {FIRST + k}/{FIRST + len(filas)}  {el:.0f} s  ({(k + 1) / max(el, 1e-6):.1f} fps)", flush=True)
for e in enc.values():
    e.stdin.close()
    e.wait()
print(f"listo: {len(filas)} fotogramas en {time.time() - t0:.0f} s")
