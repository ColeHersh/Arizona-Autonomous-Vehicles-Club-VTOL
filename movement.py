from pymavlink import mavutil
from plane import *

the_connection = mavutil.mavlink_connection('udpin:localhost:14550')
plane = Plane(the_connection)

plane.get_heartbeat()
plane.arm()
plane.takeoff()
# plane.rtl()
plane.go_to(10,0,10)