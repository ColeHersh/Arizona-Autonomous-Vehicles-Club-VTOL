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
mission_items.append(mission_item(1, 0, plane.get_lat() + 0.0003, plane.get_lon() + 0.001, 50))
#mission_items.append(mission_item(2, 0, plane.get_lat() + 0.0001, plane.get_lon() - .0001, 50, mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM))
mission_items.append(mission_item(2, 0, plane.get_lat(), plane.get_lon(), 50, mavutil.mavlink.MAV_CMD_NAV_LAND))

plane.upload_mission(mission_items)

plane.start_mission()

# Upload interrup mission item, set the sequence number
inter = True
while(1):
    print(plane.get_curr_item_squence())
    if(plane.get_curr_item_squence() == 1 and inter):
        print('Abort')
        plane.abort()
       # plane.upload_mission([mission_item(1, 0, plane.get_lat(), plane.get_lon() + 0.0001, 50, mavutil.mavlink.MAV_CMD_NAV_LAND)])
       # plane.start_mission()
        #plane.set_curr_waypoint(0)
        inter = False
# print("Attempting go to")
# plane.get_global_info()
# plane.goto(plane.get_lat() + 0.0001 , plane.get_lon()  + 0.0001,50)
