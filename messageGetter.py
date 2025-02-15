from pymavlink import mavutil
from plane import *
from mission_item import *
from time import sleep
from threading import Thread

the_connection = mavutil.mavlink_connection('com4', baud=57600)
plane = Plane(the_connection)
print("made plane")
plane.get_heartbeat()
print("got heart")
dict = {}

f = open("actualMsg.txt", 'w')

for i in range(10000000):
    msg = str(plane.rcv())
    if "None" not in msg:
        # makes dictionay entry
        #print(msg)
        msg = msg.split()
        if msg[0] not in dict:
            dict[msg[0]] = " ".join(msg[1:])
        print(i)

for key in sorted(dict.keys()):
    str = 1
    f.write("Message:" + key +"\n\n")
    f.write("Info:" + dict[key] + "\n\n\n\n\n")
f.close()

print(dict.keys())