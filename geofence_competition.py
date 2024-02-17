#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.geofence import Point, Polygon

"""
This example shows how to use the geofence plugin.
Note: The behavior when your vehicle hits the geofence is NOT configured
in this example.
"""


async def set_geofence_comp(drone):
    # Define your geofence boundary
    file = open('coords.txt', 'r')
    fence = []
    for line in file:
        m = line.split(',')
        fence.append(Point(float(m[0]), float(m[1].strip())))

    # Create a polygon object using your points
    polygon = Polygon(fence, Polygon.FenceType.INCLUSION)

    # Upload the geofence to your vehicle
    print("Uploading geofence...")
    await drone.geofence.upload_geofence([polygon])

    print("Geofence uploaded!")


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())