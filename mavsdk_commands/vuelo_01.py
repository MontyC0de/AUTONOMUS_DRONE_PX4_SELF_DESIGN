from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityNedYaw)
import asyncio

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Conectando...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Conectado a PX4")
            break

    print("Esperando estado de sensores...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("Listo para volar")
            break

    await drone.action.arm()

    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
        print("🧠 OFFBOARD iniciado")
    except OffboardError as error:
        print(f"⚠️ Error: {error._result.result}")
        await drone.action.disarm()
        return

    print("➡️ Adelante 5s")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(1.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(5)

    print("⬅️ Izquierda 5s")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 1.0, 0.0, 0.0))
    await asyncio.sleep(5)

    print("↘️ Atrás 5s")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(-1.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(5)

    print("↙️ Derecha 5s")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, -1.0, 0.0, 0.0))
    await asyncio.sleep(5)

    print("🛬 Parando y aterrizando")
    await drone.offboard.stop()
    await drone.action.land()

asyncio.run(run())
