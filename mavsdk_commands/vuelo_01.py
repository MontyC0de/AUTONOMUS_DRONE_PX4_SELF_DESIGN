from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityNedYaw)
import asyncio

ALTITUD_DESPEGUE = 5.0  # m


async def run():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Conectando...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Conectado a PX4")
            break

    print("Esperando estado de sensores...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Listo para volar")
            break

    await drone.action.arm()

    # Offboard con velocidad 0 no despega: primero subimos con TAKEOFF
    print(f"🛫 Despegando a {ALTITUD_DESPEGUE} m")
    await drone.action.set_takeoff_altitude(ALTITUD_DESPEGUE)
    await drone.action.takeoff()
    async for position in drone.telemetry.position():
        if position.relative_altitude_m > ALTITUD_DESPEGUE * 0.95:
            break

    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
        print("🧠 OFFBOARD iniciado")
    except OffboardError as error:
        print(f"⚠️ Error: {error._result.result}")
        await drone.action.land()
        return

    # NED: x = Norte, y = Este. Con yaw 0 (morro al norte), Este es la derecha.
    print("⬆️ Norte (adelante) 5s")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(1.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(5)

    print("➡️ Este (derecha) 5s")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 1.0, 0.0, 0.0))
    await asyncio.sleep(5)

    print("⬇️ Sur (atrás) 5s")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(-1.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(5)

    print("⬅️ Oeste (izquierda) 5s")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, -1.0, 0.0, 0.0))
    await asyncio.sleep(5)

    print("🛬 Parando y aterrizando")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(1)
    await drone.offboard.stop()
    await drone.action.land()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("✅ En tierra")
            break

asyncio.run(run())
