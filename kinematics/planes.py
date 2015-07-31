from sympy import *
from sympy.mpmath import *
from sympy.plotting import plot3d

v_init = matrix([0, 0, 1])

angle_x = 20.0 * pi/180.0
angle_y = 10.0 * pi/180.0
height = 5.0

rot_x = matrix([
    [1, 0, 0],
    [0, cos(angle_x), -sin(angle_x)],
    [0, sin(angle_x), cos(angle_x)]
])

rot_y = matrix([
    [cos(angle_y), 0, sin(angle_y)],
    [0, 1, 0],
    [-sin(angle_y), 0, cos(angle_y)]
])

v = rot_y * rot_x * v_init
p = matrix([0, 0, height])
d = v.T * p

print d