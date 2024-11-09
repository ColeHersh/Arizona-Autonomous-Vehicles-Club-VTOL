#!/usr/bin/env python3

import asyncio

from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)
from mavsdk.telemetry import FlightMode

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print_mission_progress_task = asyncio.ensure_future(
        print_mission_progress(drone))

    running_tasks = [print_mission_progress_task]
    termination_task = asyncio.ensure_future(
        observe_is_in_air(drone, running_tasks))

    mission_items = []
    with open("bayland.txt", "r") as handler:
        text = handler.read()
        lines = text.split("\n")
        #print(lines)
        for line in lines:
            latLong = line.split(", ")
            print(latLong)
            mission_items.append(MissionItem((float)(latLong[0].strip()),
                                        (float)(latLong[1].strip()),
                                        25,
                                        10,
                                        True,
                                        float('0'),
                                        float('0'),
                                        MissionItem.CameraAction.NONE,
                                        float('1'),
                                        float('0'),
                                        float('4'),
                                        float('0'),
                                        float('0')))
            
        
    
    mission_plan = MissionPlan(mission_items)
    fl = FlightMode(4)
    await drone.mission.set_return_to_launch_after_mission(True)
    #await drone.telemetry.FlightMode('MISSION')
    print("-- Uploading mission")
    await drone.mission.upload_mission(mission_plan)

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Starting mission")
    await drone.mission.start_mission()

    await termination_task


async def print_mission_progress(drone):
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: "
              f"{mission_progress.current}/"
              f"{mission_progress.total}")
        if mission_progress.current == mission_progress.total:
            #await drone.telemetry.FlightMode('')
            print("-- Landing")
            await drone.action.land()
async def observe_is_in_air(drone, running_tasks):
    """ Monitors whether the drone is flying or not and
    returns after landing """

    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()

            return


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())