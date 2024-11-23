from pymavlink import mavutil
from plane import *
the_connection = mavutil.mavlink_connection('com3', baud=57600)
#udpin:localhost:14550'
plane = Plane(the_connection)

plane.get_heartbeat()
print("getting global info")
plane.get_global_info()
print(plane._global_info)
#plane.arm()
#plane.takeoff()
#plane.rtl()