from pymavlink import mavutil
from time import sleep
from geofence_competition import set_geofence_comp
import asyncio

class Plane:
    def __init__(self, connection):
        self._the_connection = connection
        self._global_info = []
        self._altitude = 0
        self.get_global_info()
        # Since global alt is used, need to get the diff
        self._altitude = self.get_alt()
    
    def get_heartbeat(self):
        self._the_connection.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (self._the_connection.target_system, self._the_connection.target_component))
        
    def arm(self):
        self._the_connection.mav.command_long_send(self._the_connection.target_system, self._the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    
    def upload_mission(self, mission_items):
        n = len(mission_items)
        print("Sending mission")
        self._the_connection.mav.mission_count_send(self._the_connection.target_system, self._the_connection.target_component, n, 0)
        #self.acknowledge('MISSION_REQUEST')
        for waypoint in mission_items:
            print("Adding a waypoint")
            self._the_connection.mav.mission_item_send(self._the_connection.target_system,
                                                       self._the_connection.target_component,
                                                       waypoint.seq,
                                                       waypoint.frame,
                                                       waypoint.command,
                                                       waypoint.current,
                                                       waypoint.auto,
                                                       waypoint.param1,
                                                       waypoint.param2,
                                                       waypoint.param3,
                                                       waypoint.param4,
                                                       waypoint.param5,
                                                       waypoint.param6,
                                                       waypoint.param7)
            if waypoint != mission_items[n-1]:
                self.acknowledge('MISSION_REQUEST')
        self.acknowledge('MISSION_ACK')
    




    def goto(self,lat,lon,alt, FLIGHT_VELOCITY_X = 0, FLIGHT_VELOCITY_Y = 0, FLIGHT_VELOCITY_Z = 0):
        # self._the_connection.mav.command_int_send(self._the_connection.target_system, 
        #                                           self._the_connection.target_component, 
        #                                           mavutil.mavlink.MAV_FRAME_GLOBAL_INT, 
        #                                           mavutil.mavlink.MAV_CMD_DO_SET_HOME, 
        #                                           0, 0, 
        #                                           0, 0, 
        #                                           0, 0, 
        #                                           int(lat*1e7), int(lon*1e7), alt)
        
        # self._the_connection.mav.command_int_send(self._the_connection.target_system, 
        #                                           self._the_connection.target_component, 
        #                                           mavutil.mavlink.MAV_FRAME_GLOBAL, 
        #                                           mavutil.mavlink.MAV_CMD_DO_REPOSITION, 
        #                                           0,
        #                                           0, float(-1), 
        #                                           mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE, 
        #                                           0.0, float("nan"), 
        #                                           int(lat*1e7), int(lon*1e7), alt)
        
        # could use MAV_CMD_DO_REPOSITION for this
        self._the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
            10, self._the_connection.target_system, self._the_connection.target_component, 
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT, 
            int(0b111000000000), int(lat*10**7), int(lon*10**7), alt, FLIGHT_VELOCITY_X, FLIGHT_VELOCITY_Y, FLIGHT_VELOCITY_Z, 
            0, 0, 0, 0, 0))
        msg = self._the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)
        
    def acknowledge(self, command = 'COMMAND_ACK'):
        msg = self._the_connection.recv_match(type=command, blocking=True)
        print(msg)

    def get_curr_item_squence(self):
        return self._the_connection.waypoint_current()
        
    def interrupt(self, mission_item):
        # Use MISSION_CURRENT  to check curr mission item
        # an Overdie? MAV_CMD_OVERRIDE_GOTO
        MISSION_SET_CURRENT = 0
    
    def takeoff(self):
        lat = self.get_lat()
        lon = self.get_lon()
        self._the_connection.mav.command_long_send(self._the_connection.target_system, self._the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, lat, lon, 50)
        msg = self._the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        # This runs until the desired Altitude is reached
        run = True
        while run: 
            self.get_global_info()
            alt = self.get_alt()
            # altitude is off by less than 2 meters
            if(alt >= 48):
                run = False
                
    def rtl(self):
        self._the_connection.mav.command_long_send(self._the_connection.target_system, self._the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)
    
    def set_curr_waypoint(self, seq_num):
        self._the_connection.waypoint_set_current_send(seq_num)
        
    def start_mission(self):
        print("Starting mission")
        self._the_connection.mav.command_long_send(self._the_connection.target_system,
                                                   self._the_connection.target_component,
                                                   mavutil.mavlink.MAV_CMD_MISSION_START,
                                                   0, 0, 0, 0, 0, 0, 0, 0)
        
        #self._the_connection.mav.command_long_send(self._the_connection.target_system,
                                                   #self._the_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,  3, 0, 0, 0, 0, 0, 0, 0)
        #self._the_connection.waypoint_set_current_send(2)
        # gets curre waypoint - tries to keep it at first one
       # while (1):
           # way = self._the_connection.waypoint_current()
           # print(way)
            #if(way == 2):
                 #self._the_connection.waypoint_set_current_send(0)
           # if way != 2:
              #  self._the_connection.mav.command_long_send(self._the_connection.target_system,
                #                                   self._the_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,  0, 0, 0, 0, 0, 0, 0, 0)
                #self._the_connection.waypoint_set_current_send(1)

    '''
    Lands plane at the current GPS Cords
    '''
    def land(self):
        self.get_global_info()
        lat = self.get_lat()
        lon = self.get_lon()
        self._the_connection.mav.command_long_send(self._the_connection.target_system, self._the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, lat, lon, self._altitude)
        
    def get_global_info(self):
        msg = self._the_connection.recv_match(
            type='GLOBAL_POSITION_INT', blocking=True)
        msg = str(msg).split()
        
        lat = msg[6]
        lat = float(lat[:len(lat) - 1])
        
        lon = msg[9]
        lon = float(lon[:len(lon) - 1])
        
        lat /= 10 ** 7
        lon /= 10 ** 7
        
        alt = msg[12]
        alt = (float(alt[:len(alt) - 1]) - abs(self._altitude))/1000
        
        velo_x = msg[15]
        velo_x = float(velo_x[:len(velo_x) - 1])
        
        velo_y = msg[18]
        velo_y = float(velo_y[:len(velo_y) - 1])
        
        velo_z = msg[21]
        velo_z = float(velo_z[:len(velo_z) - 1])
        # do vx, vy, vy and getters for all
        self._global_info = [lat, lon, alt, velo_x, velo_y, velo_z]
    
    def get_lat(self):
        return self._global_info[0]
    
    def get_lon(self):
        return self._global_info[1]
    
    def get_alt(self):
        return self._global_info[2]

    def get_velo_x(self):
        return self._global_info[3]
    
    def get_velo_y(self):
        return self._global_info[4]
    
    def get_velo_z(self):
        return self._global_info[5] 
        
    def set_geofence():
        asyncio.run(set_geofence_comp)
