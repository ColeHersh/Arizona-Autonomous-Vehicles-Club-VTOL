import math

class conv:
    r_earth = 6.371E6 #meters
    """Class for simplifying movement commands"""
    def move_lat(current_lat, dy):
        """Takes a current latitude and moves it by dy meters"""
        return current_lat + (dy / conv.r_earth) * (180 / math.pi)

    def move_long(current_long, dx, latitude):
        """Takes a current longitude and moves it by dx meters"""
        return current_long + (dx / conv.r_earth) * (180 / math.pi) / math.cos(latitude * math.pi / 180)

    def move_long_lat(current_long, current_lat, dx, dy):
        new_long = conv.move_long(current_long, dx, current_lat)
        new_lat = conv.move_lat(current_lat, dy)
        return new_long, new_lat
    
    def move_heading(current_long, current_lat, heading, m):
        """Move some distance along a heading in degrees, with 0 being north"""
        heading = heading * math.pi / 180
        dx = math.cos(heading + math.pi/2) * m
        dy = math.sin(heading + math.pi/2) * m
        return conv.move_long_lat(current_long, current_lat, dx, dy)