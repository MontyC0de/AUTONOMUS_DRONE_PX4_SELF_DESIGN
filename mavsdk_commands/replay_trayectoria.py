#!/usr/bin/env python3
"""
Reproduce en el dron (gemelo o real) una trayectoria con marcas de tiempo,
con la misma cronología que el vídeo del vuelo real.

Pensado para el vuelo de prueba real (offboard desde la Raspberry), cuya
trayectoria se ha estimado a partir del vídeo (vuelos/real_20250831/
trayectoria_video.csv): mismas posiciones, alturas y tiempos, para poner los
dos vídeos lado a lado.

Reloj: tras los chequeos previos el script marca "t_video 0:00" (arranca ahí la
grabación o el vídeo real). El dron espera en tierra, arma 3 s antes del
despegue, despega en el mismo segundo que el real, sigue la trayectoria hasta
el último instante del vídeo y aterriza en el sitio. Al final imprime los
tiempos medidos en el gemelo frente a los del vídeo.

El CSV tiene columnas t_video, este, norte, altura (m, relativas al despegue).
Entre muestras se interpola con Catmull-Rom; a PX4 se le envía posición,
velocidad y aceleración. El dron mira hacia donde avanza.

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

from mision_avanzada import (CARPETA_VUELOS, Abortar, angulo_corto, aterrizar, esperar_listo,
                             minimo_jerk)

FRECUENCIA = 20
GIRO_MAX = 45.0       # °/s
ARMAR_ANTES = 4.0     # s antes del despegue (antes de que empiece la rampa)
RAMPA = 2.5           # s de subida inicial, igual que en la reconstrucción del vídeo


def mmss(t):
    return f"{int(t) // 60}:{t % 60:05.2f}"


class Trayectoria:
    """Muestras (t, este, norte, altura) interpoladas con Catmull-Rom."""

    def __init__(self, ruta):
        filas = list(csv.DictReader(open(ruta)))
        self.t = [float(f["t_video"]) for f in filas]
        # NED: norte, este, abajo
        self.p = [(float(f["norte"]), float(f["este"]), -float(f["altura"])) for f in filas]
        # Despegue = último instante en el suelo antes de empezar a subir
        primero = next(i for i, p in enumerate(self.p) if -p[2] > 0.05)
        self.despegue = max(t for t, p in zip(self.t[:primero], self.p[:primero]) if -p[2] <= 0.001)
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


async def volar(drone, tray, args, estado, casa, registro, parar, tiempos):
    """Sigue la trayectoria con el reloj del vídeo; t_video 0 = inicio de esta función."""
    periodo = 1.0 / FRECUENCIA
    reloj = asyncio.get_running_loop().time
    yaw = 0.0
    offboard_visto = False
    armado = False
    inicio = reloj()
    tiempos["inicio_reloj"] = inicio
    print(f"⏱  t_video 0:00  ← arranca aquí el vídeo real / la grabación")
    print(f"   armado en {mmss(tray.despegue - ARMAR_ANTES)}, despegue en {mmss(tray.despegue)}, "
          f"fin del vídeo en {mmss(tray.fin)}")
    ultimo_aviso = -1
    # PX4 despega con una rampa de empuje (MPC_TKO_RAMP_T): el dron se separa del
    # suelo ~2 s después de que la consigna empiece a subir. La subida se adelanta
    # ADELANTO s para que el despegue físico caiga en el mismo instante que en el
    # vídeo; a partir de despegue + RAMPA la trayectoria es la original.
    t_rampa = tray.despegue - args.adelanto_despegue
    fin_rampa = tray.despegue + RAMPA
    h_rampa = -tray.muestra(fin_rampa)[0][2]
    t = 0.0
    while t <= tray.fin:
        if t < t_rampa:
            p, v, a = [0.0, 0.0, 0.0], [0.0] * 3, [0.0] * 3
        elif t < fin_rampa:
            T = fin_rampa - t_rampa
            s, ds, dds = minimo_jerk((t - t_rampa) / T)
            p, v, a = [0.0, 0.0, -h_rampa * s], [0.0, 0.0, -h_rampa * ds / T], [0.0, 0.0, -h_rampa * dds / T ** 2]
        else:
            p, v, a = tray.muestra(t)
        if math.hypot(v[0], v[1]) > 1.0:
            objetivo = math.degrees(math.atan2(v[1], v[0]))
            yaw += max(-GIRO_MAX * periodo, min(GIRO_MAX * periodo, angulo_corto(yaw, objetivo)))
        await drone.offboard.set_position_velocity_acceleration_ned(
            PositionNedYaw(p[0] + casa[0], p[1] + casa[1], p[2] + casa[2], yaw),
            VelocityNedYaw(*v, yaw), AccelerationNed(*a))

        if not armado and t >= tray.despegue - ARMAR_ANTES:
            await drone.action.arm()
            armado = True
            tiempos["armado"] = reloj() - inicio
            print(f"  t_video {mmss(tiempos['armado'])}  🔓 armado")
        altura = -(estado["p"][2] - casa[2])
        if "despegue" not in tiempos and armado and altura > 0.05:
            tiempos["despegue"] = reloj() - inicio
            print(f"  t_video {mmss(tiempos['despegue'])}  🛫 se separa del suelo (5 cm)")
        if "en_aire" not in tiempos and estado.get("en_aire"):
            tiempos["en_aire"] = reloj() - inicio
            print(f"  t_video {mmss(tiempos['en_aire'])}  PX4 declara el despegue")

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
        if t >= tray.despegue and int(t) // 15 != ultimo_aviso:
            ultimo_aviso = int(t) // 15
            print(f"  t_video {mmss(t)}  dist {math.hypot(rel[0], rel[1]):5.1f} m  altura {-rel[2]:4.1f} m")
        await asyncio.sleep(max(0.0, inicio + t + periodo - reloj()))
        t = reloj() - inicio
    tiempos["fin"] = reloj() - inicio
    print(f"  t_video {mmss(tiempos['fin'])}  ⏹  fin del vídeo")


def informe(tray, tiempos):
    filas = [("Despegue", tray.despegue, tiempos.get("despegue")),
             ("Fin del vídeo", tray.fin, tiempos.get("fin")),
             ("Vuelo (despegue → fin)", tray.fin - tray.despegue,
              tiempos["fin"] - tiempos["despegue"] if "fin" in tiempos and "despegue" in tiempos else None)]
    print("\n                          vídeo real    gemelo     diferencia")
    for nombre, real, gemelo in filas:
        if gemelo is None:
            print(f"{nombre:24s} {mmss(real):>10s}   {'—':>8s}")
        else:
            print(f"{nombre:24s} {mmss(real):>10s}   {mmss(gemelo):>8s}   {gemelo - real:+6.2f} s")


async def run(args):
    tray = Trayectoria(args.csv)
    alcance = max(math.hypot(p[0], p[1]) for p in tray.p)
    techo = max(-p[2] for p in tray.p)
    print(f"Trayectoria: despegue {mmss(tray.despegue)}, fin {mmss(tray.fin)} "
          f"({tray.fin - tray.despegue:.1f} s de vuelo), alcance {alcance:.0f} m, altura máx. {techo:.1f} m")
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

        async def en_aire():
            async for valor in drone.telemetry.in_air():
                estado["en_aire"] = valor
        tareas.append(asyncio.ensure_future(en_aire()))

        casa = estado["p"]
        parar = asyncio.Event()
        bucle = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            bucle.add_signal_handler(sig, parar.set)

        os.makedirs(CARPETA_VUELOS, exist_ok=True)
        ruta = os.path.join(CARPETA_VUELOS, f"replay_{datetime.now():%Y%m%d_%H%M%S}.csv")
        tiempos = {}
        with open(ruta, "w", newline="") as f:
            registro = csv.writer(f)
            registro.writerow(["t_video", "sp_n", "sp_e", "sp_d", "sp_yaw", "n", "e", "d",
                               "vn", "ve", "vd", "roll", "pitch", "yaw"])
            try:
                # OFFBOARD con consigna en el suelo; se arma justo antes del despegue
                await drone.offboard.set_position_ned(PositionNedYaw(*casa, 0.0))
                await drone.offboard.start()
                await volar(drone, tray, args, estado, casa, registro, parar, tiempos)
                await aterrizar(drone)
            except Abortar as e:
                print(f"⚠️ Abortado: {e}")
                if not e.piloto:
                    await aterrizar(drone)
            finally:
                informe(tray, tiempos)
                print(f"📄 Telemetría guardada en {os.path.relpath(ruta)}")
                for tarea in tareas:
                    tarea.cancel()
                for sig in (signal.SIGINT, signal.SIGTERM):
                    bucle.remove_signal_handler(sig)
    finally:
        drone._stop_mavsdk_server()


def main():
    p = argparse.ArgumentParser(description="Reproduce una trayectoria con la cronología del vídeo")
    p.add_argument("csv", help="t_video, este, norte, altura")
    p.add_argument("--conexion", default="udpin://0.0.0.0:14540")
    p.add_argument("--geovalla", type=float, default=100.0, help="radio máximo [m]")
    p.add_argument("--techo", type=float, default=15.0, help="altura máxima [m]")
    p.add_argument("--bateria-min", type=float, default=30.0)
    p.add_argument("--adelanto-despegue", type=float, default=1.7,
                   help="s que se adelanta la subida para compensar la rampa de despegue de PX4")
    asyncio.run(run(p.parse_args()))


if __name__ == "__main__":
    main()
