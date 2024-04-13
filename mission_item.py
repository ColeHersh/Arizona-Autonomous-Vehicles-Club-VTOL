from math import nan
from pymavlink import mavutil
class mission_item:
    """
    Holds relevant information for a new mission item
    Uploads to mission as an array of these
    Possible Commands are:
    MAV_CMD_NAV_WAYPOINT - Tested and confirmed works
    MAV_CMD_NAV_LOITER_UNLIM
    MAV_CMD_NAV_LOITER_TURNS
    MAV_CMD_NAV_LOITER_TIME
    MAV_CMD_NAV_RETURN_TO_LAUNCH
    MAV_CMD_NAV_LAND - Tested and confirmed works
    MAV_CMD_NAV_TAKEOFF
    MAV_CMD_NAV_LAND_LOCAL
    MAV_CMD_NAV_TAKEOFF_LOCAL
    MAV_CMD_NAV_FOLLOW
    MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT
    MAV_CMD_NAV_LOITER_TO_ALT 
    MAV_CMD_NAV_ROI 
    MAV_CMD_NAV_PATHPLANNING
    MAV_CMD_NAV_SPLINE_WAYPOINT
    MAV_CMD_NAV_VTOL_TAKEOFF
    MAV_CMD_NAV_VTOL_LAND
    MAV_CMD_NAV_GUIDED_ENABLE
    MAV_CMD_NAV_DELAY
    MAV_CMD_NAV_PAYLOAD_PLACE
    MAV_CMD_NAV_LAST
    MAV_CMD_NAV_SET_YAW_SPEED
    MAV_CMD_NAV_FENCE_RETURN_POINT
    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
    MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
    MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION
    MAV_CMD_NAV_RALLY_POINT
    Check https://mavlink.io/en/messages/common.html for parameters
    """
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