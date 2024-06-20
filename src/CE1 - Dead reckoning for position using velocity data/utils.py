import numpy as np
import math

def quaternion_from_euler(ai, aj, ak): 
    '''
    Arguments:
        ai, aj, ak: Euler angles in RPY order (Roll, Pitch and Yaw) (float)
    Returns:
        q: The corresponding quaternion in format [qx, qy, qz, qw] (python list)
    '''
    
    sin_r = math.sin(ai * 0.5)
    cos_r = math.cos(ai * 0.5)
    sin_p = math.sin(aj * 0.5)
    cos_p = math.cos(aj * 0.5)
    sin_y = math.sin(ak * 0.5)
    cos_y = math.cos(ak * 0.5)

    # Quaternion components
    qx = sin_r * cos_p * cos_y - cos_r * sin_p * sin_y
    qy = cos_r * sin_p * cos_y + sin_r * cos_p * sin_y
    qz = cos_r * cos_p * sin_y - sin_r * sin_p * cos_y
    qw = cos_r * cos_p * cos_y + sin_r * sin_p * sin_y

    return [qx, qy, qz, qw]


def lonlat2xyz(lat, lon, lat0, lon0): 
    # WGS84 ellipsoid constants:
    a = 6378137
    b = 6356752.3142
    e = math.sqrt(1-b**2/a**2)
    
    x = a*math.cos(math.radians(lat0))*math.radians(lon-lon0)/math.pow(1-e**2*(math.sin(math.radians(lat0)))**2,0.5)
    y = a*(1 - e**2)*math.radians(lat-lat0)/math.pow(1-e**2*(math.sin(math.radians(lat0)))**2,1.5)
    
    return x, y # x and y coordinates in a reference frame with the origin in lat0, lon0
