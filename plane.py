from pymavlink import mavutil
class Plane:
    def __init__(self, connection):
        self._the_connection = connection
        self._global_info = self.get_global_info()
    
    def get_heartbeat(self):
        self._the_connection.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (self._the_connection.target_system, self._the_connection.target_component))
        
    def arm(self):
        self._the_connection.mav.command_long_send(self._the_connection.target_system, self._the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        
        
    def takeoff(self):
        msg = self._the_connection.recv_match(
            type='GLOBAL_POSITION_INT', blocking=True)
        msg = str(msg).split()
        #print(msg)
        lat = msg[6]
        lat = float(lat[:len(lat) - 1])
        lon = msg[9]
        lon = float(lon[:len(lon) - 1])
        lat /= 10 ** 7
        lon /= 10 ** 7

        self._the_connection.mav.command_long_send(self._the_connection.target_system, self._the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, lat, lon, 50)
        msg = self._the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        #print(msg)
        # This runs until the desired Altitude is reached
        run = True
        while run:
            msg = self._the_connection.recv_match(
                type='LOCAL_POSITION_NED', blocking=True)
            #print(msg)
            temp = str(msg).split()
            temp = temp[12]
            temp = float(temp[:len(temp) - 1])
            #print(temp)
            # altitude is off by less than 2 meters
            if(temp <= -48):
                run = False
                
    def rtl(self):
        self._the_connection.mav.command_long_send(self._the_connection.target_system, self._the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)
        
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
        alt = float(alt[:len(alt) - 1])
        
        velo_x = msg[15]
        velo_x = float(velo_x[:len(velo_x) - 1])
        
        velo_y = msg[18]
        velo_y = float(velo_y[:len(velo_y) - 1])
        
        velo_z = msg[21]
        velo_z = float(velo_z[:len(velo_z) - 1])
        # do vx, vy, vy and getters for all
        return[lat, lon, alt, velo_x, velo_y, velo_z]
        
        
