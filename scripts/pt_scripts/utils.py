import numpy as np


def distance(x1, y1, x2, y2):
        return np.sqrt((x1 -x2)**2 + (y1 -y2)**2)

def mod_angle(theta):
    """
    The function mod_angle takes an angle in radians and returns the equivalent angle in the range [-pi,
    pi].
    """
    angle = np.arctan2(np.sin(theta), np.cos(theta))
    return angle

def angle_diff(a1, a2):
    da = a1 - a2
    return np.arctan2(np.sin(da), np.cos(da))

def find_nearest_ind(way_points, pose):
    dists = [distance(pose[0], pose[1], way_points[i][0], way_points[i][1]) for i in range(len(way_points))]
    dists = np.array(dists)
    idx = np.argmin(dists)
    return idx, dists