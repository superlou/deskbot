import numpy as np
from numpy import degrees
from kinematics.inverse.leg_ik import LegIK
from kinematics.inverse.leg_ik_fast import LegIK as LegIKFast
import cProfile as profile


if __name__ == "__main__":
    lik = LegIK(np.array([0, 0, 0]), np.array([1, 1, 1]), 1, 1)
    likf = LegIKFast(np.array([0, 0, 0]), np.array([1, 1, 1]), 1, 1)
    profile.run("for i in range(10000): root_angle = lik.solve()")
    profile.run("for i in range(10000): root_angle = likf.solve()")
