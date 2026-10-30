#!/usr/bin/env bash
# Reconstruye la trayectoria aproximada del dron a partir de un vídeo grabado con
# el móvil (Nothing Phone 2, 24 mm eq., ~690 px de focal a 540 px de ancho).
#
#   ./simulation/tools/video/analizar_video.sh VIDEO.mp4 vuelos/real_YYYYMMDD
#
# Pasos (todo a 10 fps, 540x960):
#   1. seguir_dron.py  -> pista.csv   posición, tamaño aparente y horizonte del dron
#   2. giro_camara.py  -> giro.csv    giro de la cámara por flujo óptico del fondo
#   3. reconstruir.py  -> trayectoria_video.csv  (t_video, este, norte, altura)
# Dependencias: ffmpeg, numpy, scipy, opencv-python-headless
set -euo pipefail
VIDEO="$1"; OUT="$2"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$OUT"; cd "$OUT"
ffmpeg -hide_banner -loglevel error -i "$VIDEO" -vf "fps=10,scale=540:960" -f rawvideo -pix_fmt rgb24 - \
	| python3 "$HERE/seguir_dron.py" 540 960 10 pista.csv
ffmpeg -hide_banner -loglevel error -i "$VIDEO" -vf "fps=10,scale=540:960" -f rawvideo -pix_fmt rgb24 - \
	| python3 "$HERE/giro_camara.py" 540 960 10 690 pista.csv giro.csv
python3 "$HERE/reconstruir.py"
