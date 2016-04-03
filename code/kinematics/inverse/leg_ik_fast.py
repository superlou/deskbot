import numpy as np
import pyximport
pyximport.install()
from leg_ik_cython import solve_fast


class LegIK(object):
    def __init__(self, p0, p1, l, m):
        self.l = float(l)
        self.m = float(m)

        self.p0 = p0.astype(float)
        self.p1 = p1.astype(float)

    def solve(self):
        return solve_fast(self.p0, self.p1, self.m, self.l)


if __name__ == "__main__":
    np.set_printoptions(suppress=True)

    lik = LegIK(np.array([0, 0, 0]), np.array([1, 1, 1]), 1, 1)
    root_angle = lik.solve()
    print np.degrees(root_angle)
