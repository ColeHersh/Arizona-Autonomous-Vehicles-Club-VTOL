from pymavlink import mavutil
class Plane:
    def __init__(self, connection):
        self.the_connection = connection
    
    def arm(self):
        self.the_connection.mav.command_long_send(self.the_connection.target_system, self.the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        
        
    def takeoff(self):
        msg = self.the_connection.recv_match(
            type='GLOBAL_POSITION_INT', blocking=True)
        msg = str(msg).split()
        print(msg)
        lat = msg[6]
        lat = float(lat[:len(lat) - 1])
        lon = msg[9]
        lon = float(lon[:len(lon) - 1])
        lat /= 10 ** 7
        lon /= 10 ** 7

        self.the_connection.mav.command_long_send(self.the_connection.target_system, self.the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, lat, lon, 50)
        msg = self.the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        #print(msg)
        # This runs until the desired Altitude is reached
        run = True
        while run:
            msg = self.the_connection.recv_match(
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
        self.the_connection.mav.command_long_send(self.the_connection.target_system, self.the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)
