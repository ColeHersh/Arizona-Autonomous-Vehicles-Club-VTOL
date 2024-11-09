import math
r_earth = 6.371E6 #meters

def move_lat(current_lat, dy):
    """Takes a current latitude and moves it by dy meters"""
    return current_lat + (dy / r_earth) * (180 / math.pi)

def move_long(current_long, dx, latitude):
    """Takes a current longitude and moves it by dx meters"""
    return current_long + (dx / r_earth) * (180 / math.pi) / math.cos(latitude * math.pi / 180)
