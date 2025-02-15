from pymavlink import mavutil
def arm(the_connection):
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
def takeoff(the_connection):
    msg = the_connection.recv_match(
        type='GLOBAL_POSITION_INT', blocking=True)
    msg = str(msg).split()
    print(msg)
    lat = msg[6]
    lat = float(lat[:len(lat) - 1])
    lon = msg[9]
    lon = float(lon[:len(lon) - 1])
    lat /= 10 ** 7
    lon /= 10 ** 7

    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, lat, lon, 50)
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    #print(msg)
    # This runs until the desired Altitude is reached
    run = True
    while run:
        msg = the_connection.recv_match(
            type='LOCAL_POSITION_NED', blocking=True)
        #print(msg)
        temp = str(msg).split()
        temp = temp[12]
        temp = float(temp[:len(temp) - 1])
        #print(temp)
        # altitude is off by less than 2 meters
        if(temp <= -48):
            run = False
            
def rtl(the_connection):
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)
