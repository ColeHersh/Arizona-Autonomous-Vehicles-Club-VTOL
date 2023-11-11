from pymavlink import mavutil
from commands import *
global the_connection
the_connection = mavutil.mavlink_connection('udpin:localhost:14550')
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))
# Arms
arm(the_connection)
# msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
# print(msg

#Takes Off
#Lat test  37.41337964719044
#Long Test -121.9966774838903

takeoff(the_connection)
# This gets the current lat and lon

rtl(the_connection)