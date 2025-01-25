from pymavlink import mavutil
from plane import *
from mission_item import *
from time import sleep
from threading import Thread

the_connection = mavutil.mavlink_connection('udpin:localhost:14550')
plane = Plane(the_connection)
plane.get_heartbeat()
plane.arm()
dict = {}

f = open("gazeebMsg.txt", 'w')

for i in range(100000):
    msg = str(plane.rcv())
    if "None" not in msg:
        # makes dictionay entry
        #print(msg)
        msg = msg.split()
        if msg[0] not in dict:
            dict[msg[0]] = " ".join(msg[1:])
for key in dict.keys():
    str = 1
    f.write("Message:" + key +"\n\n")
    f.write("Info:" + dict[key] + "\n\n\n\n\n")
f.close()
print(dict.keys())