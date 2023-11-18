from pymavlink import mavutil
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
        
        
    def takeoff(self):
        lat = self.get_lat()
        lon = self.get_lon()
        self._the_connection.mav.command_long_send(self._the_connection.target_system, self._the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, lat, lon, 50)
        msg = self._the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        #print(msg)
        # This runs until the desired Altitude is reached
        run = True
        while run: 
            self.get_global_info()
            alt = self.get_alt()
            #print(alt)
            #print(temp)
            # altitude is off by less than 2 meters
            if(alt >= 48):
                run = False
                
    def rtl(self):
        self._the_connection.mav.command_long_send(self._the_connection.target_system, self._the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)
        
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
        
