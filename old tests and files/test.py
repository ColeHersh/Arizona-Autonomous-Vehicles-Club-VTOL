#!/usr/bin/env python3

import asyncio
from mavsdk import System


async def run():

    drone = System()
    await drone.connect(system_address="udp://:14540")

    status_text_task = asyncio.ensure_future(print_status_text(drone))

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break
    #asyncio.ensure_future(print_position(drone))
    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(10)

    print("-- Creating Mission --")
    mission_items = []
    with open("bayland.txt", "r") as handler:
        text = handler.read()
        lines = text.split("\n")
        #print(lines)
        for line in lines:
            latLong = line.split(", ")
            #print(latLong)
            if latLong[0] != None:
                mission_items.append([(float)(latLong[0].strip()), (float)(latLong[1].strip())])

    print('-- Mission Created --')
    print(mission_items)

    while len(mission_items) > 0:
        await drone.action.goto_location(mission_items[0][0], mission_items[0][1], 50 , 0)
        #await next_item(mission_items)
        #print('HHHERRRRRREEEE')
        #currLat =  drone.telemetry.position.latitude_deg()
        #currLong = drone.telemetry.position.longitude_deg()
        #print(currLat, currLong)
        async for position_info in drone.telemetry.position():
            current_latitude = position_info.latitude_deg
            current_longitude = position_info.longitude_deg
            break
        #currLat = drone.telemetry.latitude_deg()
        #current_longitude =  drone.telemetry.position.longitude_deg()
        #print('yooo',  current_latitude, current_longitude)

        # Waits until the vtol has reached the point to move to the next one
        if current_latitude == mission_items[0][0] and current_longitude ==  mission_items[0][1]:
            #exit()
            mission_items = mission_items[1:]
            #drone.action.hold()
    print("-- Landing")
    await drone.action.land()

    status_text_task.cancel()
    
async def next_item(i):
    i = i[1:]
async def print_status_text(drone):
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"Status: {status_text.type}: {status_text.text}")
    except asyncio.CancelledError:
        return
   
#async def print_position(drone):
    async for position in drone.telemetry.position():
        print('yoo',position)

if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())