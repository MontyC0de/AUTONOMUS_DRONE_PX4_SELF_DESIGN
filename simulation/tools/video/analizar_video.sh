#!/usr/bin/env bash
# Trajectory of the drone from a phone video, removing the camera's own motion.
#
#   ./simulation/tools/video/analizar_video.sh VIDEO.mp4 vuelos/real_YYYYMMDD
#
# Steps (10 fps):
#   1. seguir_dron.py     -> pista.csv        drone pixel in every frame (540x960)
#   2. camara.py          -> camara.npz       frame-to-frame camera rotation (1080x1920)
#   3. orientacion.py     -> orientacion.npz  camera orientation per frame, drift removed
#                                             with loop closures between repeated views
#   4. ejes.py            -> ejes.npz         vertical of the scene
#   5. calibrar_sombra.py -> calib.npz        north and phone height from the drone's
#                                             shadow at takeoff (sun position at the
#                                             video's date and place)
#   6. medir_ancho.py     -> anchos.csv       apparent drone width (1080 px)
#   7. reconstruir.py     -> trayectoria_video.csv  (t_video, east, north, height, yaw)
# Dependencies: ffmpeg, numpy, scipy, opencv-python-headless
set -euo pipefail
VIDEO="$(realpath "$1")"; OUT="$2"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$OUT"; cd "$OUT"
frames() { ffmpeg -hide_banner -loglevel error -i "$VIDEO" -vf "fps=10,scale=$1" -f rawvideo -pix_fmt "$2" -; }
frames 540:960 rgb24 | python3 "$HERE/seguir_dron.py" 540 960 10 pista.csv
frames 1080:1920 gray | python3 "$HERE/camara.py" 1080 1920 10 pista.csv camara.npz
python3 "$HERE/orientacion.py" camara.npz orientacion.npz 1680
python3 "$HERE/ejes.py"
python3 "$HERE/calibrar_sombra.py"
frames 1080:1920 rgb24 | python3 "$HERE/medir_ancho.py"
python3 "$HERE/reconstruir.py"
