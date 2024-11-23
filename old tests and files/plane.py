from pymavlink import mavutil
from time import sleep
from geofence_competition import set_geofence_comp
import asyncio

class Plane:
    """
    This class holds the commands and variables
    for the aircraft
    """
    
    def __init__(self, connection):
        """
        This intializes the plan
        :param connection - the connection to the aircraft
        """
        self._the_connection = connection
        self._global_info = []
        self._altitude = 0
        self.get_global_info()
        # Since global alt is used, need to get the diff
        self._altitude = self.get_alt()
        self.landed = False
    
    def get_heartbeat(self):
        """
        This method prints the connection and its info.
        Used to confirm connection to the aircraft
        """
        self._the_connection.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (self._the_connection.target_system, self._the_connection.target_component))
        
    def arm(self):
        """
        Arms the aricraft so it is ready to recive commands
        """
        self._the_connection.mav.command_long_send(self._the_connection.target_system, self._the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    
    def disarm(self):
        """
        disarms the aircraft so it is safe
        """
        self._the_connection.mav.command_long_send(self._the_connection.target_system, self._the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    
    
    def upload_mission(self, mission_items):
        """
        Takes in mission and uploads it to the plane
        
        :param mission_items - an array of mission_item objects
        """
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
    
        
    def acknowledge(self, command = 'COMMAND_ACK'):
        """
        Prints the command acknownledgement and result
        
        :param command - the command to be acknowledged
        """
        msg = self._the_connection.recv_match(type=command, blocking=True)
        print(msg)

    def get_curr_item_squence(self):
        """
        Gets the current waypoint's sequence number
        
        :return an integer representing the current waypont's postion in the sequence
        """
        return self._the_connection.waypoint_current()
        
    def get_state(self):
        # Use MISSION_CURRENT  to check curr mission item
        # an Overdie? MAV_CMD_OVERRIDE_GOTO
        msg = str(self._the_connection.recv_match(type='MISSION_STATE', blocking=True))
        print(msg)
        msg.split()[3]
        return msg
        
    def abort(self):
        """
        Pauses the current mission for interrupt.
        Allows for CV to operate and direct aircraft
        """
        self.get_global_info()
        self.move()
        msg = self._the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)

    def resume(self):
        self._the_connection.mav.command_long_send(self._the_connection.target_system, self._the_connection.target_component,  mavutil.mavlink.MAV_CMD_DO_PAUSE_CONTINUE, 0,
                                                     0, 0, 0, 0, 0, 0, 0)
        msg = self._the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)

    def pause(self):
        self._the_connection.mav.command_long_send(self._the_connection.target_system, self._the_connection.target_component,  mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT, -1,
                                                     0, 0, 0, 0, 0, 0, 0)
        msg = self._the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)
    
    
    def unpause(self):
        #self._the_connection.mav.command_long_send(self._the_connection.target_system, self._the_connection.target_component,  mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT, self.get_curr_item_squence(),
                                                     #0, 0, 0, 0, 0, 0, 0)
        self._the_connection.mav.command_long_send(self._the_connection.target_system,
                                                   self._the_connection.target_component,
                                                   mavutil.mavlink.MAV_CMD_MISSION_START,
                                                   0, 0, 0, 0, 0, 0, 0, 0)

        msg = self._the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)
        print("unpausing")
  
    def move(self):
        """
        Moves the aircraft to the desired position
        """
             
        self._the_connection.mav.command_long_send(self._the_connection.target_system, self._the_connection.target_component,  mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 
                                                  -1, 0x00000001, 0, float("NaN"),  self.get_lat() + 0.0004, float("NaN"), 10)
    
    
    def takeoff(self, height = 0):
        """
        Takes off at the current location
        """,
        lat = self.get_lat()
        lon = self.get_lon()
        self._the_connection.mav.command_long_send(self._the_connection.target_system, self._the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, lat, lon, height)
        msg = self._the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        # This runs until the desired Altitude is reached
        run = True
        while run: 
            self.get_global_info()
            alt = self.get_alt()
            # altitude is off by less than 2 meters
            if(alt >= height):
                run = False
                
    def rtl(self):
        """
        Lands at the takeoff location
        """
        self._the_connection.mav.command_long_send(self._the_connection.target_system, self._the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)
        self.landed = True
        
    def set_curr_waypoint(self, seq_num):
        """
        Sets the current waypoint to move to in the mission
        
        :param the desired waypoint's sequence number
        """
        self._the_connection.waypoint_set_current_send(seq_num)
        
    def start_mission(self, start_val = 0):
        """
        Starts the mission.  Requirement- a mission must be uploaded
        """
        print("Starting mission")
        self._the_connection.mav.command_long_send(self._the_connection.target_system,
                                                   self._the_connection.target_component,
                                                   mavutil.mavlink.MAV_CMD_MISSION_START,
                                                   start_val, 0, 0, 0, 0, 0, 0, 0)


    def land(self):
        """
        Lands at the current location
        """
        self.get_global_info()
        lat = self.get_lat()
        lon = self.get_lon()
        self._the_connection.mav.command_long_send(self._the_connection.target_system, self._the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, lat, lon, self._altitude)
        self.landed = True
        
    def get_global_info(self):
        """
        Pulls the information of the plane of the getters
        """
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
        """
        Get the current lattitude
        
        :return the current lattiude as a float
        """
        return self._global_info[0]
    
    def get_lon(self):
        """
        Get the current longitude
        
        :return the current longitude as a float
        """
        return self._global_info[1]
    
    def get_alt(self):
        """
        Gets the current altitude
        
        :return the current altitde as an int
        """
        
        return self._global_info[2]

    def get_velo_x(self):
        """
        Get the current velocity onn the x plane
        
        :return the current x velocity
        """
        return self._global_info[3]
    
    def get_velo_y(self):
        """
        Get the current velocity on the y plane
        
        :return the current y velocity
        """
        return self._global_info[4]
    
    def get_velo_z(self):
        """
        Get the current velocity onn the z plane
        
        :return the current z velocity
        """
        return self._global_info[5] 
        
    def set_geofence():
        """
        Sets up the geofence(s)
        """
        asyncio.run(set_geofence_comp)
