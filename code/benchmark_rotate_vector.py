import cProfile as profile
import numpy
import sys
from numpy import radians, array
from kinematics.inverse.platform_ik import rotate_vector
import pyximport
pyximport.install(setup_args={'include_dirs': numpy.get_include()})
from kinematics.inverse.utilities import rotate_vector_fast

if __name__ == "__main__":
    e_v = array([1., 0., 0.])
    gy_v = array([0., 1., 0.])
    angle = radians(10.)

    if len(sys.argv) > 1:
        runs = int(sys.argv[1])
    else:
        runs = 1

    profile.run("for i in range(runs): rotate_vector(e_v, gy_v, angle)")
    profile.run("for i in range(runs): rotate_vector_fast(e_v, gy_v, angle)")
