from pymavlink import mavutil
from plane import *
from mission_item import *
from time import sleep

the_connection = mavutil.mavlink_connection('udpin:localhost:14550')
plane = Plane(the_connection)

plane.arm()
plane.takeoff
plane.transition_to_htol()
sleep(2)
plane.avoid()
#.mav.collision_send