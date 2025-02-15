from pymavlink import mavutil
from plane import *
from mission_item import *
from time import sleep

the_connection = mavutil.mavlink_connection('com3', baud=57600)
print("Connection established")
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

while True:

        msg = the_connection.recv_match(type='GPS_RAW_INT', blocking=True)
        print(msg)
        msg = str(msg).split()
        print(msg)
        lat = msg[9]
        lat = float(lat[:len(lat) - 1])
        
        lon = msg[12]
        lon = float(lon[:len(lon) - 1])
        
        lat /= 10 ** 7
        lon /= 10 ** 7
        
        alt = msg[15]
        alt = int(alt[:len(alt) - 1])/1000
        print(lat, lon, alt)
        break
        