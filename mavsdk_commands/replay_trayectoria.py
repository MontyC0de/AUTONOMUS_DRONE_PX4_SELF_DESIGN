#!/usr/bin/env python3
"""
Reproduce en el dron (gemelo o real) una trayectoria con marcas de tiempo.

Pensado para el vuelo real del 31/08/2025, reconstruido a partir del vídeo
(vuelos/real_20250831/trayectoria_video.csv): mismas posiciones, mismas alturas
y la misma cronología que el vídeo, para ponerlos lado a lado.

El CSV tiene columnas t_video, este, norte, altura (m, relativas al despegue).
Entre muestras se interpola con Catmull-Rom; a PX4 se le envía posición,
velocidad y aceleración. El dron despega en OFFBOARD en el instante del vídeo
en que despega el real, mira hacia donde avanza (como un piloto en vuelo
manual) y, al acabar la trayectoria, vuelve sobre el despegue y aterriza.

Uso:
    python3 mavsdk_commands/replay_trayectoria.py vuelos/real_20250831/trayectoria_video.csv
    python3 mavsdk_commands/replay_trayectoria.py traj.csv --conexion serial:///dev/serial0:921600
"""
import argparse
import asyncio
import csv
import math
import os
import signal
from datetime import datetime

from mavsdk import System
from mavsdk.offboard import AccelerationNed, PositionNedYaw, VelocityNedYaw
from mavsdk.telemetry import FlightMode

from mision_avanzada import (CARPETA_VUELOS, Abortar, aterrizar, esperar_listo,
                             minimo_jerk, angulo_corto)

FRECUENCIA = 20
GIRO_MAX = 45.0     # °/s


class Trayectoria:
    """Muestras (t, este, norte, altura) interpoladas con Catmull-Rom."""

    def __init__(self, ruta):
        filas = list(csv.DictReader(open(ruta)))
        self.t = [float(f["t_video"]) for f in filas]
        # NED: norte, este, abajo
        self.p = [(float(f["norte"]), float(f["este"]), -float(f["altura"])) for f in filas]
        self.despegue = next(t for t, p in zip(self.t, self.p) if -p[2] > 0.05) - 0.5
        self.fin = self.t[-1]

    def muestra(self, t):
        t = min(max(t, self.t[0]), self.t[-1] - 1e-6)
        i = max(1, min(len(self.t) - 3, next(k for k in range(len(self.t) - 1) if self.t[k + 1] > t)))
        h = self.t[i + 1] - self.t[i]
        u = (t - self.t[i]) / h
        p0, p1, p2, p3 = self.p[i - 1], self.p[i], self.p[i + 1], self.p[i + 2]
        pos, vel, acc = [], [], []
        for a, b, c, d in zip(p0, p1, p2, p3):
            m1, m2 = (c - a) / 2, (d - b) / 2
            h00, h10, h01, h11 = 2*u**3 - 3*u**2 + 1, u**3 - 2*u**2 + u, -2*u**3 + 3*u**2, u**3 - u**2
            dh00, dh10, dh01, dh11 = 6*u**2 - 6*u, 3*u**2 - 4*u + 1, -6*u**2 + 6*u, 3*u**2 - 2*u
            ddh00, ddh10, ddh01, ddh11 = 12*u - 6, 6*u - 4, -12*u + 6, 6*u - 2
            pos.append(h00 * b + h10 * m1 + h01 * c + h11 * m2)
            vel.append((dh00 * b + dh10 * m1 + dh01 * c + dh11 * m2) / h)
            acc.append((ddh00 * b + ddh10 * m1 + ddh01 * c + ddh11 * m2) / h ** 2)
        return pos, vel, acc


async def volar(drone, tray, args, estado, casa, registro, parar):
    periodo = 1.0 / FRECUENCIA
    reloj = asyncio.get_running_loop().time
    yaw = 0.0
    offboard_visto = False
    inicio = reloj()
    t = tray.despegue
    print(f"▶ Reproduciendo t_video {tray.despegue:.1f} → {tray.fin:.1f} s")
    ultimo_aviso = 0
    while t <= tray.fin:
        p, v, a = tray.muestra(t)
        # Morro hacia donde avanza; quieto, mantiene el último rumbo
        if math.hypot(v[0], v[1]) > 1.0:
            objetivo = math.degrees(math.atan2(v[1], v[0]))
            yaw += max(-GIRO_MAX * periodo, min(GIRO_MAX * periodo, angulo_corto(yaw, objetivo)))
        await drone.offboard.set_position_velocity_acceleration_ned(
            PositionNedYaw(p[0] + casa[0], p[1] + casa[1], p[2] + casa[2], yaw),
            VelocityNedYaw(*v, yaw), AccelerationNed(*a))

        if parar.is_set():
            raise Abortar("interrumpida por el operador (Ctrl+C)")
        modo = estado.get("modo")
        if modo == FlightMode.OFFBOARD:
            offboard_visto = True
        elif offboard_visto:
            raise Abortar(f"modo cambiado a {modo}: el piloto tiene el control", piloto=True)
        rel = [pi - ci for pi, ci in zip(estado["p"], casa)]
        if math.hypot(rel[0], rel[1]) > args.geovalla or -rel[2] > args.techo:
            raise Abortar(f"fuera de límites: {math.hypot(rel[0], rel[1]):.0f} m, {-rel[2]:.1f} m de altura")
        pct = estado.get("bat", (0, 100))[1]
        if not math.isnan(pct) and pct < args.bateria_min:
            raise Abortar(f"batería baja: {pct:.0f} %")

        registro.writerow([f"{t:.2f}", *(f"{x:.3f}" for x in p), f"{yaw:.1f}",
                           *(f"{x:.3f}" for x in rel), *(f"{x:.3f}" for x in estado["v"]),
                           *(f"{x:.1f}" for x in estado.get("att", (0, 0, 0)))])
        if int(t) // 15 != ultimo_aviso:
            ultimo_aviso = int(t) // 15
            print(f"  t_video {int(t) // 60}:{int(t) % 60:02d}  dist {math.hypot(rel[0], rel[1]):5.1f} m  "
                  f"altura {-rel[2]:4.1f} m")
        await asyncio.sleep(max(0.0, inicio + (t - tray.despegue) + periodo - reloj()))
        t = tray.despegue + reloj() - inicio

    # Vuelta suave sobre el despegue (mínimo jerk) y aterrizaje
    p_fin, _, _ = tray.muestra(tray.fin)
    dist = math.hypot(p_fin[0], p_fin[1])
    T = max(3.0, 1.875 * dist / 2.0)
    print(f"▶ Regreso al despegue ({dist:.0f} m, {T:.0f} s)")
    t0 = reloj()
    while reloj() - t0 <= T:
        s, _, _ = minimo_jerk((reloj() - t0) / T)
        await drone.offboard.set_position_ned(PositionNedYaw(
            p_fin[0] * (1 - s) + casa[0], p_fin[1] * (1 - s) + casa[1], p_fin[2] + casa[2], yaw))
        if parar.is_set():
            raise Abortar("interrumpida por el operador (Ctrl+C)")
        await asyncio.sleep(periodo)


async def run(args):
    tray = Trayectoria(args.csv)
    alcance = max(math.hypot(p[0], p[1]) for p in tray.p)
    techo = max(-p[2] for p in tray.p)
    print(f"Trayectoria: {tray.fin - tray.despegue:.0f} s de vuelo, alcance {alcance:.0f} m, "
          f"altura máx. {techo:.1f} m, despegue en t_video {tray.despegue:.1f} s")
    if alcance > args.geovalla or techo > args.techo:
        raise SystemExit("La trayectoria se sale de la geovalla o del techo")

    drone = System()
    try:
        estado = {}
        try:
            tareas = await esperar_listo(drone, args, estado)
        except Abortar as e:
            print(f"⛔ No se despega: {e}")
            return
        casa = estado["p"]
        parar = asyncio.Event()
        bucle = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            bucle.add_signal_handler(sig, parar.set)

        os.makedirs(CARPETA_VUELOS, exist_ok=True)
        ruta = os.path.join(CARPETA_VUELOS, f"replay_{datetime.now():%Y%m%d_%H%M%S}.csv")
        with open(ruta, "w", newline="") as f:
            registro = csv.writer(f)
            registro.writerow(["t_video", "sp_n", "sp_e", "sp_d", "sp_yaw", "n", "e", "d",
                               "vn", "ve", "vd", "roll", "pitch", "yaw"])
            try:
                # Despegue en OFFBOARD: consigna en el suelo, OFFBOARD, armar
                await drone.offboard.set_position_ned(PositionNedYaw(*casa, 0.0))
                await drone.offboard.start()
                await drone.action.arm()
                print("🛫 Armado en OFFBOARD")
                await volar(drone, tray, args, estado, casa, registro, parar)
                await aterrizar(drone)
            except Abortar as e:
                print(f"⚠️ Abortado: {e}")
                if not e.piloto:
                    await aterrizar(drone)
            finally:
                print(f"📄 Telemetría guardada en {os.path.relpath(ruta)}")
                for tarea in tareas:
                    tarea.cancel()
                for sig in (signal.SIGINT, signal.SIGTERM):
                    bucle.remove_signal_handler(sig)
    finally:
        drone._stop_mavsdk_server()


def main():
    p = argparse.ArgumentParser(description="Reproduce una trayectoria con marcas de tiempo")
    p.add_argument("csv", help="t_video, este, norte, altura")
    p.add_argument("--conexion", default="udpin://0.0.0.0:14540")
    p.add_argument("--geovalla", type=float, default=100.0, help="radio máximo [m]")
    p.add_argument("--techo", type=float, default=15.0, help="altura máxima [m]")
    p.add_argument("--bateria-min", type=float, default=30.0)
    asyncio.run(run(p.parse_args()))


if __name__ == "__main__":
    main()
