import numpy as np
import scipy as sp
import scipy.optimize as optimize
import cProfile as profile


class DeskbotModel(object):
    def __init__(self, theta_b, r_b, l, m, d, h):
        self.theta_b = np.array(theta_b)
        self.r_b = np.array(r_b)
        self.d = np.array(d)
        self.h = h

        # Determine position of base points
        self.b = np.array([
            r_b * np.cos(self.theta_b),
            r_b * np.sin(self.theta_b),
            np.array([0, 0, 0])
        ])

        self.l = l
        self.m = m
        self.alpha = np.zeros(3)
        self.gamma = np.zeros(3)
        self.k = np.zeros(3)
        self.p = np.zeros(3)

    def solve(self, alpha, alpha_h):
        self.alpha = np.array(alpha)
        self.alpha_h = alpha_h

        # Determine positions of knee points
        self.k = np.array([
            self.b[0] + self.l * np.cos(np.pi + self.theta_b) * np.cos(self.alpha),
            self.b[1] + self.l * np.sin(np.pi + self.theta_b) * np.cos(self.alpha),
            self.b[2] + self.l * np.sin(self.alpha),
        ])

        gamma = optimize.fsolve(self.errors, (1.57, 1.57, 1.57))

        # Determine positions of platform points
        self.p = np.array([
            self.k[0] + self.m * np.cos(np.pi + self.theta_b) * np.cos(gamma),
            self.k[1] + self.m * np.sin(np.pi + self.theta_b) * np.cos(gamma),
            self.k[2] + self.m * np.sin(gamma),
        ])

        # Determine position of head centroid above platform
        p_c = np.average(self.p, axis=1)        # platform centroid
        self.p_c = p_c

        p01 = self.p[:, 1] - self.p[:, 0]
        p02 = self.p[:, 2] - self.p[:, 0]

        p_v = np.cross(p01, p02)
        p_v = p_v / np.linalg.norm(p_v)         # platform normal vecto

        h_c = self.h * p_v + self.p_c           # center of head
        self.h_c = h_c

        # Determine head vectors
        h_v = self.p[:, 0] - p_c
        h_v = h_v / np.linalg.norm(h_v)

        h_v = self.rotate_vector(h_v, p_v, alpha_h)
        self.h_v = h_v                          # eye line vector

        e_p = h_v + h_c
        self.e_p = e_p

        return gamma

    def rotate_vector(self, vector, axis, angle):
        """
        Uses Rodrigues rotation formula
        axis must be a normal vector
        """
        k = axis
        v = vector
        v_rot = (v * np.cos(angle) + np.cross(k, v) * np.sin(angle) +
                 k * (k * v) * (1 - np.cos(angle)))

        return v_rot

    def errors(self, p):
        gamma_0, gamma_1, gamma_2 = p
        gamma = np.array([gamma_0, gamma_1, gamma_2])

        # Determine positions of platform points
        self.p = np.array([
            self.k[0] + self.m * np.cos(np.pi + self.theta_b) * np.cos(gamma),
            self.k[1] + self.m * np.sin(np.pi + self.theta_b) * np.cos(gamma),
            self.k[2] + self.m * np.sin(gamma),
        ])

        d01 = np.linalg.norm(self.p[:, 0] - self.p[:, 1])
        d12 = np.linalg.norm(self.p[:, 1] - self.p[:, 2])
        d20 = np.linalg.norm(self.p[:, 2] - self.p[:, 0])

        return (d01 - self.d[0], d12 - self.d[1], d20 - self.d[2])

    def distance(p1, p2):
        np.linalg.norm(p1 - p2)

if __name__ == "__main__":
    np.set_printoptions(suppress=True)

    dm = DeskbotModel(
        theta_b=[0 * np.pi / 180, 120 * np.pi / 180, 240 * np.pi / 180],
        r_b=[2, 2, 2],
        l=[1, 1, 1],
        m=[1, 1, 1],
        d=[1.414, 1.414, 1.414],
        h=0.5
    )

    dm.solve(np.array([0, 0, 0]) * np.pi / 180, 0 * np.pi / 180)

    print "Base:\n", dm.b
    print "Knees:\n", dm.k
    print "Platform:\n", dm.p
    print "Head vector:", dm.h_v
