import numpy as np
from math import pi


class Servo(object):
    def __init__(self, neutral, min, max):
        self.neutral = neutral
        self.min = min
        self.max = max

    def period_for_angle(self, angle, degrees=True):
        period = np.interp(angle, [-pi/2., 0, pi/2.], [self.min, self.neutral, self.max])
        return period
