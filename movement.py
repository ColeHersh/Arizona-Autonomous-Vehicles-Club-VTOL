from pymavlink import mavutil
from plane import *
from mission_item import *

the_connection = mavutil.mavlink_connection('udpin:localhost:14550')
plane = Plane(the_connection)

plane.get_heartbeat()
plane.arm()
plane.takeoff()
plane.get_global_info()
mission_items = []
mission_items.append(mission_item(0, 0, plane.get_lat() + 0.0001, plane.get_lon() + 0.0001, 50))
mission_items.append(mission_item(1, 0, plane.get_lat() + 0.0003, plane.get_lon() + 0.0001, 50))
mission_items.append(mission_item(2, 0, plane.get_lat() + 0.0001, plane.get_lon() - 0.0001, 50))
mission_items.append(mission_item(3, 0, plane.get_lat() - 0.0003, plane.get_lon() - 0.0001, 50, mavutil.mavlink.MAV_CMD_NAV_LAND))

plane.upload_mission(mission_items)

plane.start_mission()

# print("Attempting go to")
# plane.get_global_info()
# plane.goto(plane.get_lat() + 0.0001 , plane.get_lon()  + 0.0001,50)