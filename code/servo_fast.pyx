import numpy as np
from math import pi


class ServoFast(object):
    def __init__(self, double neutral, double min, double max):
        self.neutral = neutral
        self.min = min
        self.max = max

        cdef double pi_div_2 = 1.5707963267

        self.m_neg = (neutral - min) / (pi_div_2)
        self.b_neg = neutral

        self.m_pos = (max - neutral) / (pi_div_2)
        self.b_pos = neutral

    def period_for_angle(self, angle):
        cdef double period = 0
        angle = -angle  # inverted to match logic in Servo

        if angle >= 0:
            period = self.m_pos * angle + self.b_pos
        else:
            period = self.m_neg * angle + self.b_neg

        return period
