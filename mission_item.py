from math import nan
from pymavlink import mavutil
class mission_item:
    def __init__(self, i, current, x, y, z, auto, command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT):
        self.seq = i
        self.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        self.command = command
        self.current = current
        self.auto = auto
        self.param1 = 0.0
        self.param2 = 2.00
        self.param3 = 20.00
        self.param4 = nan
        self.param5 = x
        self.param6 = y
        self.param7 = z
        self.mission_type = 0