import numpy as np


class LegIK(object):
    def __init__(self, p0, p1, l, m):
        self.l = l
        self.m = m

        self.p0 = p0
        self.p1 = p1

    def solve(self):
        p0 = self.p0
        p1 = self.p1

        # Translate to origin
        p0p = np.zeros(3)
        p1p = p1 - p0

        # Rotate to X-Z plane
        angle = -np.arctan2(p1p[1], p1p[0])

        rot_z = np.array([
            [np.cos(angle), -np.sin(angle), 0],
            [np.sin(angle), np.cos(angle), 0],
            [0, 0, 1]
        ])

        p1p = np.dot(rot_z, p1p)

        # Determine root actuator angle
        alpha1, alpha2 = self.two_link_from_origin_ik(p1p[0], p1p[2])
        alpha = min(alpha1, alpha2)

        return alpha

    def two_link_from_origin_ik(self, x, z):
        m = self.m
        l = self.l

        a = m**2 - l**2 - x**2 - z**2
        b = 2 * x * l
        c = 2 * z * l

        t1 = (-2 * c + np.sqrt(4 * (c**2 - a**2 + b**2))) / (2 * (a - b))
        t2 = (-2 * c - np.sqrt(4 * (c**2 - a**2 + b**2))) / (2 * (a - b))

        alpha1 = 2 * np.arctan(t1)
        alpha2 = 2 * np.arctan(t2)

        return alpha1, alpha2

if __name__ == "__main__":
    np.set_printoptions(suppress=True)

    lik = LegIK(np.array([0, 0, 0]), np.array([1, 1, 1]), 1, 1)
    root_angle = lik.solve()
    print np.degrees(root_angle)
