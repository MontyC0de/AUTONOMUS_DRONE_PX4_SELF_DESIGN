"""Render the twin frame by frame from poses.csv into an mp4 (paused gz server).

usage: python3 render.py poses.csv out.mp4 FFMPEG [first_frame] [n_frames]
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

POSES, OUT, FFMPEG = sys.argv[1], sys.argv[2], sys.argv[3]
FIRST = int(sys.argv[4]) if len(sys.argv) > 4 else 0
N = int(sys.argv[5]) if len(sys.argv) > 5 else 10 ** 9
W, H, WORLD = 720, 1280, "render"

node = Node()
lock = threading.Condition()
ultima = {"stamp": -1.0, "data": None}


def on_image(msg):
    with lock:
        ultima["stamp"] = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        ultima["data"] = msg.data
        lock.notify_all()


node.subscribe(Image, "/cam_movil", on_image)


def set_pose(name, x, y, z, qw, qx, qy, qz):
    p = Pose()
    p.name = name
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z = qw, qx, qy, qz
    ok, _ = node.request(f"/world/{WORLD}/set_pose", p, Pose, Boolean, 2000)
    return ok


def step():
    c = WorldControl()
    c.pause = True
    c.multi_step = 1
    ok, _ = node.request(f"/world/{WORLD}/control", c, WorldControl, Boolean, 2000)
    return ok


filas = list(csv.DictReader(open(POSES)))[FIRST:FIRST + N]
enc = subprocess.Popen([FFMPEG, "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo",
                        "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", "30", "-i", "-",
                        "-c:v", "libx264", "-crf", "18", "-preset", "medium", "-pix_fmt", "yuv420p", OUT],
                       stdin=subprocess.PIPE)
t0 = time.time()
# Warm-up so the first frame has the scene loaded
for _ in range(3):
    step()
time.sleep(1.0)
for k, r in enumerate(filas):
    f = lambda c: float(r[c])
    set_pose("dron", f("x"), f("y"), f("z"), f("qw"), f("qx"), f("qy"), f("qz"))
    for j in range(4):
        set_pose(f"h{j}", *(f(f"h{j}{c}") for c in ("x", "y", "z", "qw", "qx", "qy", "qz")))
    set_pose("cam_movil", f("cx"), f("cy"), f("cz"), f("cqw"), f("cqx"), f("cqy"), f("cqz"))
    with lock:
        antes = ultima["stamp"]
    step()   # applies the poses
    step()   # renders with them
    with lock:
        if not lock.wait_for(lambda: ultima["stamp"] > antes, timeout=5.0):
            print("sin imagen en el fotograma", FIRST + k, file=sys.stderr)
        data = ultima["data"]
    enc.stdin.write(data)
    if k % 300 == 0:
        el = time.time() - t0
        print(f"fotograma {FIRST + k}/{FIRST + len(filas)}  {el:.0f} s  ({(k + 1) / max(el, 1e-6):.1f} fps)", flush=True)
enc.stdin.close()
enc.wait()
print(f"listo: {len(filas)} fotogramas en {time.time() - t0:.0f} s -> {OUT}")
