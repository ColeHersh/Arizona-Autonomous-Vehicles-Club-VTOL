from pymavlink import mavutil
from plane import *
from mission_item import *
from time import sleep

the_connection = mavutil.mavlink_connection('com4', baud=57600)
print("Connection established")
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))