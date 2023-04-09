from mavsdk import System
import asyncio
from time import sleep
from mavsdk.offboard import (OffboardError, PositionNedYaw)

async def  main():
    print("Connecting")
    drone = System()
    await drone.connect(system_address="udp://:14540")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break
    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break
    print("now arming")
    await drone.action.arm()
    print("-- Setting startpoint to takeoff position!")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard mode")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed \
                with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return
    

    print("Now taking off")
    await drone.action.takeoff()
    print("Taking off")
    sleep(15)
    print("-- Go 5m North, 0m East, 5m Up within local coordinate system, turn to face East and move up")
    await drone.offboard.set_position_ned(
            PositionNedYaw(5.0, 0.0, 5.0, 90.0))
    
    await drone.manual_control.
    sleep(15)
    print("landing")
    await drone.action.land()
    return

if __name__ == "__main__":
    asyncio.run(main())
    