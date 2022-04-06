import math
import random
import numpy as np
import numpy.linalg as LA

class Vector2(object):
    """
    Represents a bidimensional geometric vector.
    """
    def __init__(self, x, y):
        """
        Creates a bidimensional geometric vector.

        :param x: x coordinate.
        :type x: float
        :param y: y coordinate.
        :type y: float
        """
        self.x = x
        self.y = y


class Pose(object):
    """
    Represents a pose on the plane, i.e. a (x, y) position plus a rotation.
    """
    def __init__(self, x, y, rotation):
        """
        Creates a pose on the plane.

        :param x: x coordinate.
        :type x: float
        :param y: y coordinate.
        :type y: float
        :param rotation: rotation around z axis.
        :type rotation: float
        """
        self.position = Vector2(x, y)
        self.rotation = rotation

def dist(p1,p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def rand_num(a,b):
    return random.uniform(a, b)

def ang_vector_deg(vec1,vec2):
    a = vec1
    b = vec2

    inner = np.inner(a, b)
    norms = LA.norm(a) * LA.norm(b)

    cos_t = inner / norms

    rad = np.arccos(np.clip(cos_t, -1.0, 1.0))
    deg = np.rad2deg(rad)
    return deg