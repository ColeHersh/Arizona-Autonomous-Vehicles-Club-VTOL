from pymavlink import mavutil
from plane import *
the_connection = mavutil.mavlink_connection('udpin:localhost:14550')
plane = Plane(the_connection)
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))
# Arms
plane.arm()
# msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
# print(msg

#Takes Off
#Lat test  37.41337964719044
#Long Test -121.9966774838903

plane.takeoff()
# This gets the current lat and lon

plane.rtl()