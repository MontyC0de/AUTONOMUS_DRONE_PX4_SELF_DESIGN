#!/usr/bin/env python3
"""
Misión autónoma de demostración: cuadrado, órbita, ocho y hélice.

Funciona igual en el gemelo digital (PX4 SITL + Gazebo) que en el dron real
(Raspberry Pi conectada a la Pixhawk 4), porque toda la trayectoria se define
en metros relativos al punto de despegue, en el marco NED de PX4
(x = Norte, y = Este, z = Abajo).

Cada tramo se recorre con un perfil de mínimo jerk,
    s(τ) = 10τ³ − 15τ⁴ + 6τ⁵,   τ = t / T,
que empieza y termina con velocidad y aceleración nulas. A PX4 se le envía la
posición de referencia junto con la velocidad y la aceleración de
prealimentación (feedforward), de modo que el controlador de posición sigue
la curva sin ir por detrás de ella.

Uso:
    # Gemelo digital (con `make px4_sitl gz_mi_drone` corriendo)
    python3 mavsdk_commands/mision_avanzada.py

    # Ver el plan (tramos, duración, alcance) sin conectar al dron
    python3 mavsdk_commands/mision_avanzada.py --plan

    # Dron real, desde la Raspberry Pi
    python3 mavsdk_commands/mision_avanzada.py --conexion serial:///dev/serial0:921600

    # Versión reducida para la primera prueba en real
    python3 mavsdk_commands/mision_avanzada.py --escala 0.5 --velocidad 1.0

La telemetría se guarda en vuelos/mision_<fecha>.csv y las gráficas se
generan con graficas_vuelo.py.
"""
import argparse
import asyncio
import csv
import math
import os
import signal
from datetime import datetime

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import (AccelerationNed, OffboardError, PositionNedYaw,
                             VelocityNedYaw)
from mavsdk.telemetry import FlightMode

FRECUENCIA = 20          # Hz de envío de consignas (PX4 exige más de 2 Hz)
ACELERACION_MAX = 1.5    # m/s², límite para dimensionar la duración de cada tramo
GIRO_MAX = 45.0          # °/s, velocidad máxima de guiñada
SUBIDA_HELICE = 4.0      # m que gana la hélice
CARPETA_VUELOS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "vuelos")


class Abortar(Exception):
    """Condición de seguridad: se interrumpe la misión."""

    def __init__(self, motivo, piloto=False):
        super().__init__(motivo)
        self.piloto = piloto   # True si el piloto ha tomado el control con la emisora


# ------------------------------------------------------------- trayectorias

def minimo_jerk(tau):
    """s, ds/dτ y d²s/dτ² del perfil de mínimo jerk, τ ∈ [0, 1]."""
    tau = min(max(tau, 0.0), 1.0)
    s = tau ** 3 * (10 - 15 * tau + 6 * tau ** 2)
    ds = 30 * tau ** 2 * (1 - tau) ** 2
    dds = 60 * tau * (1 - tau) * (1 - 2 * tau)
    return s, ds, dds


def angulo_corto(desde, hasta):
    """Diferencia de guiñada más corta, en grados, en (-180, 180]."""
    return (hasta - desde + 180.0) % 360.0 - 180.0


def distancia(p, q):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(p, q)))


class Tramo:
    """Curva P(θ), θ ∈ [0, Θ], con guiñada Y(θ), recorrida con mínimo jerk.

    La duración T se elige para que el pico de velocidad (1.875·Θ·max|P'|/T),
    el de aceleración (5.77·Θ·max|P'|/T²) y el de giro no superen los límites.
    Se usa max|P'| y no la longitud media porque curvas como el ocho no se
    recorren a velocidad uniforme en θ.
    """

    def __init__(self, nombre, curva, theta_total, guinada, v_max):
        self.nombre = nombre
        self.curva = curva
        self.theta_total = theta_total
        self.guinada = guinada
        self.h = theta_total * 1e-4

        muestras = [i * theta_total / 200 for i in range(201)]
        puntos = [curva(th) for th in muestras]
        self.longitud = sum(distancia(a, b) for a, b in zip(puntos, puntos[1:]))
        recorrido = theta_total * max(
            distancia(curva(th + self.h), curva(th - self.h)) / (2 * self.h) for th in muestras)
        yaws = [guinada(th) for th in muestras]
        giro = sum(abs(angulo_corto(a, b)) for a, b in zip(yaws, yaws[1:]))
        self.alcance = max(math.hypot(p[0], p[1]) for p in puntos)
        self.altura_max = max(-p[2] for p in puntos)

        self.duracion = max(2.0,
                            1.875 * recorrido / v_max,
                            math.sqrt(5.77 * recorrido / ACELERACION_MAX),
                            1.875 * giro / GIRO_MAX)

    def muestra(self, t):
        """Posición, velocidad, aceleración (NED) y guiñada en el instante t."""
        s, ds, dds = minimo_jerk(t / self.duracion)
        th = self.theta_total * s
        th_p = self.theta_total * ds / self.duracion
        th_pp = self.theta_total * dds / self.duracion ** 2

        p = self.curva(th)
        p_mas, p_menos = self.curva(th + self.h), self.curva(th - self.h)
        d1 = [(a - b) / (2 * self.h) for a, b in zip(p_mas, p_menos)]
        d2 = [(a - 2 * c + b) / self.h ** 2 for a, b, c in zip(p_mas, p_menos, p)]
        v = [d * th_p for d in d1]
        a = [dd * th_p ** 2 + d * th_pp for d, dd in zip(d1, d2)]
        return p, v, a, self.guinada(th)


class Plan:
    """Encadena tramos y añade automáticamente las transiciones entre ellos."""

    def __init__(self, altura, v_max):
        self.v_max = v_max
        self.tramos = []
        self.pos = (0.0, 0.0, -altura)
        self.yaw = 0.0

    def ir_a(self, nombre, destino, yaw=None):
        """Línea recta (o giro en el sitio) desde la posición actual."""
        p0, y0 = self.pos, self.yaw
        y1 = y0 if yaw is None else yaw
        dy = angulo_corto(y0, y1)
        self.tramos.append(Tramo(
            nombre,
            lambda th, p0=p0, p1=destino: tuple(a + (b - a) * th for a, b in zip(p0, p1)),
            1.0,
            lambda th, y0=y0, dy=dy: y0 + dy * th,
            self.v_max))
        self.pos, self.yaw = tuple(destino), y1

    def girar(self, nombre, yaw):
        self.ir_a(nombre, self.pos, yaw)

    def curva(self, nombre, curva, theta_total, guinada):
        inicio, yaw_inicio = curva(0.0), guinada(0.0)
        if distancia(self.pos, inicio) > 0.05 or abs(angulo_corto(self.yaw, yaw_inicio)) > 2:
            self.ir_a(f"{nombre}: entrada", inicio, yaw_inicio)
        self.tramos.append(Tramo(nombre, curva, theta_total, guinada, self.v_max))
        self.pos, self.yaw = curva(theta_total), guinada(theta_total) % 360.0

    @property
    def duracion(self):
        return sum(t.duracion for t in self.tramos)


def tangente(curva, h=1e-4):
    """Guiñada mirando en la dirección de avance de la curva."""
    def yaw(th):
        a, b = curva(th - h), curva(th + h)
        return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
    return yaw


def construir_plan(altura, v_max, escala):
    """Cuadrado → órbita de inspección → ocho → hélice → regreso."""
    plan = Plan(altura, v_max)
    z = -altura

    # 1. Cuadrado de 8 m: gira en cada esquina y avanza mirando al frente
    lado = 8.0 * escala
    esquinas = [(lado / 2, lado / 2), (lado / 2, -lado / 2),
                (-lado / 2, -lado / 2), (-lado / 2, lado / 2)]
    plan.ir_a("Cuadrado: a la esquina", (*esquinas[0], z))
    for i in range(4):
        a, b = esquinas[i], esquinas[(i + 1) % 4]
        rumbo = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
        plan.girar("Cuadrado: giro", rumbo)
        plan.ir_a("Cuadrado", (*b, z))

    # 2. Órbita de inspección de 6 m alrededor del despegue, cámara al centro
    r = 6.0 * escala
    orbita = lambda th: (r * math.cos(th), r * math.sin(th), z)  # noqa: E731
    plan.curva("Orbita", orbita, 2 * math.pi,
               lambda th: math.degrees(math.atan2(-orbita(th)[1], -orbita(th)[0])))

    # 3. Ocho (lemniscata de Gerono) de 14 x 7 m mirando hacia donde avanza
    amp = 7.0 * escala
    ocho = lambda th: (amp * math.sin(th), amp / 2 * math.sin(2 * th), z)  # noqa: E731
    plan.curva("Ocho", ocho, 2 * math.pi, tangente(ocho))

    # 4. Hélice: dos vueltas de 4 m de radio subiendo SUBIDA_HELICE metros
    rh = 4.0 * escala
    helice = lambda th: (rh * math.cos(th), rh * math.sin(th),  # noqa: E731
                         z - SUBIDA_HELICE * th / (4 * math.pi))
    plan.curva("Helice", helice, 4 * math.pi, tangente(helice))

    # 5. Regreso sobre el punto de despegue y aproximación a 1.5 m
    plan.ir_a("Regreso", (0.0, 0.0, z), 0.0)
    plan.ir_a("Aproximacion", (0.0, 0.0, -1.5))
    return plan


def imprimir_plan(plan):
    print(f"{'Tramo':28s} {'long [m]':>9s} {'T [s]':>7s}")
    for t in plan.tramos:
        print(f"{t.nombre:28s} {t.longitud:9.1f} {t.duracion:7.1f}")
    alcance = max(t.alcance for t in plan.tramos)
    techo = max(t.altura_max for t in plan.tramos)
    print(f"Duración total {plan.duracion:.0f} s, alcance horizontal {alcance:.1f} m, "
          f"altura máxima {techo:.1f} m")
    return alcance, techo


# ------------------------------------------------------------- telemetría

def lanzar_telemetria(drone, estado):
    async def posicion():
        async for pv in drone.telemetry.position_velocity_ned():
            estado["p"] = (pv.position.north_m, pv.position.east_m, pv.position.down_m)
            estado["v"] = (pv.velocity.north_m_s, pv.velocity.east_m_s, pv.velocity.down_m_s)

    async def actitud():
        async for att in drone.telemetry.attitude_euler():
            estado["att"] = (att.roll_deg, att.pitch_deg, att.yaw_deg)

    async def bateria():
        async for bat in drone.telemetry.battery():
            estado["bat"] = (bat.voltage_v, bat.remaining_percent)

    async def modo():
        async for m in drone.telemetry.flight_mode():
            estado["modo"] = m

    return [asyncio.ensure_future(f()) for f in (posicion, actitud, bateria, modo)]


# ------------------------------------------------------------- vuelo

async def esperar_listo(drone, args, estado):
    """Conecta, arranca la telemetría y comprueba que se puede volar. Devuelve las tareas."""
    print(f"Conectando a {args.conexion} ...")
    # connect() lanza mavsdk_server como proceso hijo. Con SIGINT ignorado al
    # crearlo, lo hereda: un Ctrl+C en la terminal ya no lo mata y el script
    # puede seguir enviando la orden de aterrizar.
    anterior = signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        await drone.connect(system_address=args.conexion)
    finally:
        signal.signal(signal.SIGINT, anterior)
    async for s in drone.core.connection_state():
        if s.is_connected:
            print("✅ Conectado a PX4")
            break
    tareas = lanzar_telemetria(drone, estado)

    print("Esperando GPS y posición de casa...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            break

    # Si un vuelo anterior se cortó, PX4 puede seguir en OFFBOARD, modo en el
    # que no deja armar sin consignas: se pasa a HOLD antes de nada
    try:
        await drone.action.hold()
    except ActionError:
        pass

    # is_armable recoge los chequeos previos de PX4 (sensores, modo, enlace
    # con la estación de tierra, que es este script)
    async def armable():
        async for health in drone.telemetry.health():
            if health.is_armable:
                return
    try:
        await asyncio.wait_for(armable(), timeout=30)
    except asyncio.TimeoutError:
        raise Abortar("PX4 no permite armar; revisa los avisos en QGroundControl")

    for _ in range(50):
        if "p" in estado and "bat" in estado:
            break
        await asyncio.sleep(0.1)
    if "p" not in estado:
        raise Abortar("no llega la posición local de PX4")
    voltaje, pct = estado.get("bat", (float("nan"), float("nan")))
    print(f"🔋 Batería {voltaje:.2f} V, {pct:.0f} %")
    if math.isnan(pct) or pct < args.bateria_min + 20:
        raise Abortar(f"batería insuficiente o sin lectura ({pct:.0f} %)")
    return tareas


async def volar_plan(drone, plan, args, estado, casa, registro, parar):
    periodo = 1.0 / FRECUENCIA
    reloj = asyncio.get_running_loop().time
    t_mision = 0.0
    offboard_visto = False

    for tramo in plan.tramos:
        print(f"▶ {tramo.nombre} ({tramo.duracion:.1f} s)")
        inicio = reloj()
        t = 0.0
        while t <= tramo.duracion:
            p, v, a, yaw = tramo.muestra(t)
            p_abs = [pi + ci for pi, ci in zip(p, casa)]
            await drone.offboard.set_position_velocity_acceleration_ned(
                PositionNedYaw(*p_abs, yaw),
                VelocityNedYaw(*v, yaw),
                AccelerationNed(*a))

            # Seguridad
            if parar.is_set():
                raise Abortar("interrumpida por el operador (Ctrl+C)")
            modo = estado.get("modo")
            if modo == FlightMode.OFFBOARD:
                offboard_visto = True
            elif offboard_visto:
                raise Abortar(f"modo cambiado a {modo}: el piloto tiene el control", piloto=True)
            rel = [pi - ci for pi, ci in zip(estado["p"], casa)]
            if math.hypot(rel[0], rel[1]) > args.geovalla:
                raise Abortar(f"geovalla: {math.hypot(rel[0], rel[1]):.1f} m del despegue")
            if -rel[2] > args.altura + SUBIDA_HELICE + 2.0:
                raise Abortar(f"altura máxima superada: {-rel[2]:.1f} m")
            pct = estado.get("bat", (0, 100))[1]
            if not math.isnan(pct) and pct < args.bateria_min:
                raise Abortar(f"batería baja: {pct:.0f} %")

            registro.writerow([f"{t_mision + t:.3f}", tramo.nombre,
                               *(f"{x:.3f}" for x in p), f"{yaw:.1f}",
                               *(f"{x:.3f}" for x in rel),
                               *(f"{x:.3f}" for x in estado["v"]),
                               *(f"{x:.1f}" for x in estado.get("att", (0, 0, 0))),
                               f"{estado.get('bat', (0, 0))[0]:.2f}", f"{pct:.1f}",
                               str(modo).split(".")[-1]])

            await asyncio.sleep(max(0.0, inicio + t + periodo - reloj()))
            t = reloj() - inicio
        t_mision += tramo.duracion


async def aterrizar(drone):
    print("🛬 Aterrizando")
    try:
        await drone.offboard.stop()
    except OffboardError:
        pass
    await drone.action.land()
    async for en_aire in drone.telemetry.in_air():
        if not en_aire:
            print("✅ En tierra")
            break


async def run(args):
    plan = construir_plan(args.altura, args.velocidad, args.escala)
    alcance, techo = imprimir_plan(plan)
    if alcance > args.geovalla:
        raise SystemExit(f"El plan llega a {alcance:.1f} m y la geovalla es {args.geovalla} m")
    if args.plan:
        return

    drone = System()
    try:
        await mision(drone, plan, args)
    finally:
        # mavsdk_server ignora Ctrl+C (ver esperar_listo), así que se cierra aquí
        drone._stop_mavsdk_server()


async def mision(drone, plan, args):
    estado = {}
    try:
        tareas = await esperar_listo(drone, args, estado)
    except Abortar as e:
        print(f"⛔ No se despega: {e}")
        return

    casa = estado["p"]
    print(f"🏠 Despegue en NED local ({casa[0]:.2f}, {casa[1]:.2f}, {casa[2]:.2f})")

    # Ctrl+C no cancela las tareas (eso dejaría a PX4 sin consignas y sin orden
    # de aterrizar): solo marca el evento y la misión aborta por la vía normal.
    parar = asyncio.Event()

    def al_interrumpir():
        if parar.is_set():
            print("⏳ Ya se está aterrizando; para tomar el control usa la emisora")
        parar.set()

    bucle = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        bucle.add_signal_handler(sig, al_interrumpir)

    os.makedirs(CARPETA_VUELOS, exist_ok=True)
    ruta = os.path.join(CARPETA_VUELOS, f"mision_{datetime.now():%Y%m%d_%H%M%S}.csv")
    with open(ruta, "w", newline="") as f:
        registro = csv.writer(f)
        registro.writerow(["t", "tramo", "sp_n", "sp_e", "sp_d", "sp_yaw",
                           "n", "e", "d", "vn", "ve", "vd",
                           "roll", "pitch", "yaw", "bat_v", "bat_pct", "modo"])
        try:
            print(f"🛫 Armando y despegando a {args.altura} m")
            await drone.action.set_takeoff_altitude(args.altura)
            await drone.action.arm()
            await drone.action.takeoff()
            while -(estado["p"][2] - casa[2]) < args.altura * 0.95:
                if parar.is_set():
                    raise Abortar("interrumpida por el operador (Ctrl+C)")
                await asyncio.sleep(0.1)

            # Primera consigna = punto actual, después se activa OFFBOARD
            await drone.offboard.set_position_ned(
                PositionNedYaw(casa[0], casa[1], casa[2] - args.altura, 0.0))
            await drone.offboard.start()
            print("🧠 OFFBOARD activo, empieza la misión")
            await volar_plan(drone, plan, args, estado, casa, registro, parar)
            await aterrizar(drone)
        except Abortar as e:
            print(f"⚠️ Misión abortada: {e}")
            if not e.piloto:
                await aterrizar(drone)
        except (ActionError, OffboardError) as e:
            print(f"⛔ PX4 ha rechazado la orden: {e}")
            async for armado in drone.telemetry.armed():
                if armado:
                    await aterrizar(drone)
                break
        finally:
            print(f"📄 Telemetría guardada en {os.path.relpath(ruta)}")
            for tarea in tareas:
                tarea.cancel()
            for sig in (signal.SIGINT, signal.SIGTERM):
                bucle.remove_signal_handler(sig)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1],
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--conexion", default="udpin://0.0.0.0:14540",
                        help="SITL: udpin://0.0.0.0:14540 | Raspberry: serial:///dev/serial0:921600")
    parser.add_argument("--altura", type=float, default=5.0, help="altura de crucero [m]")
    parser.add_argument("--velocidad", type=float, default=2.0, help="velocidad máxima [m/s]")
    parser.add_argument("--escala", type=float, default=1.0, help="factor de tamaño de las figuras")
    parser.add_argument("--geovalla", type=float, default=15.0, help="radio máximo [m]")
    parser.add_argument("--bateria-min", type=float, default=30.0, help="aterriza por debajo de [%%]")
    parser.add_argument("--plan", action="store_true", help="solo muestra el plan, no vuela")
    asyncio.run(run(parser.parse_args()))


if __name__ == "__main__":
    main()
