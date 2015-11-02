import numpy as np
from numpy import radians, degrees
import scipy as sp
import scipy.optimize as optimize


def rotate_vector(vector, axis, angle):
    """
    Uses Rodrigues rotation formula
    axis must be a normal vector
    """
    k = axis
    v = vector
    v_rot = (v * np.cos(angle) + np.cross(k, v) * np.sin(angle) +
             k * (np.dot(k, v)) * (1 - np.cos(angle)))

    return v_rot


def point_line_distance(point, line_slope):
    """
    Based on https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    Assumes line passes through zero
    Let b = 1
    """
    x0 = point[0]
    y0 = point[1]
    a = -line_slope

    distance = np.abs(a * x0 + y0) / np.sqrt(a ** 2 + 1)
    return distance


class PlatformIK(object):
    def __init__(self, l):
        self.l = l

    def solve(self, e_v, f_v, h):
        self.e_v = e_v
        self.f_v = f_v
        self.h = h

        x = optimize.fsolve(self.errors, [radians(0), 0, 0, self.h])

        return x

    def errors(self, x):
        e_v = self.e_v
        f_v = self.f_v

        theta, px, py, pz = x
        # print "x:", x

        n_v = np.cross(e_v, f_v)
        n_v = n_v / np.linalg.norm(n_v)
        # print "n_v", n_v

        p0_v = rotate_vector(e_v, n_v, -theta)
        p1_v = rotate_vector(p0_v, n_v, radians(120))
        p2_v = rotate_vector(p0_v, n_v, radians(240))

        # print "cross:", np.dot(p0_v, n_v)
        # print "vectors"
        # print p0_v, p1_v, p2_v

        pc = np.array([px, py, pz])
        p0 = self.l * p0_v + pc
        p1 = self.l * p1_v + pc
        p2 = self.l * p2_v + pc

        self.p = np.array([p0, p1, p2]).T

        # print "p1 error:", point_line_distance(p1, np.tan(radians(-60)))
        # print "pz error:", pz - self.h
        # print p2_v

        return (point_line_distance(p0, 0),
                point_line_distance(p1, np.tan(radians(-60))),
                point_line_distance(p2, np.tan(radians(60))),
                pz - self.h)


if __name__ == "__main__":
    np.set_printoptions(suppress=True)

    gx_v = np.array([1, 0, 0])
    gy_v = np.array([0, 1, 0])
    gz_v = np.array([0, 0, 1])

    e_v = np.array([1, 0, 0])
    f_v = np.array([0, 1, 0])

    e_v = rotate_vector(e_v, gz_v, radians(10))
    f_v = rotate_vector(f_v, gz_v, radians(10))

    e_v = rotate_vector(e_v, gy_v, radians(-20))
    f_v = rotate_vector(f_v, gy_v, radians(-20))

    e_v = rotate_vector(e_v, gx_v, radians(-20))
    f_v = rotate_vector(f_v, gx_v, radians(-20))

    pik = PlatformIK(l=1)
    x = pik.solve(e_v, f_v, 1)

    print "Solution:"
    print "theta:", degrees(x[0])
    print "p_c:", [x[1], x[2], x[3]]
    print "p:"
    print pik.p
