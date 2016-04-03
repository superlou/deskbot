import numpy as np
from numpy import radians
from kinematics.utilities import rotate_vector
from kinematics.inverse.platform_ik import PlatformIK
from kinematics.inverse.platform_ik_fast import PlatformIK as PlatformIKFast
import cProfile as profile

if __name__ == "__main__":
    gx_v = np.array([1., 0., 0.])
    gy_v = np.array([0., 1., 0.])
    gz_v = np.array([0., 0., 1.])

    e_v = np.array([1., 0., 0.])
    f_v = np.array([0., 1., 0.])

    e_v = rotate_vector(e_v, gz_v, radians(0))
    f_v = rotate_vector(f_v, gz_v, radians(0))

    e_v = rotate_vector(e_v, gy_v, radians(-10))
    f_v = rotate_vector(f_v, gy_v, radians(-10))

    e_v = rotate_vector(e_v, gx_v, radians(-10))
    f_v = rotate_vector(f_v, gx_v, radians(-10))

    pik = PlatformIK(l=1, n=0.1)
    pikf = PlatformIKFast(l=1, n=0.1)
    profile.run("for i in range(1000): theta, centroid, points = pik.solve(e_v, f_v, 1)")
    profile.run("for i in range(1000): theta, centroid, points = pikf.solve(e_v, f_v, 1)")
