import numpy as np
from numpy import radians, degrees
import scipy as sp
import scipy.optimize as optimize
import cProfile as profile
import math
import pyximport
pyximport.install()
from utilities import rotate_vector_fast, point_line_distance_fast


def rotation_matrix(axis, theta):
    """
    From http://stackoverflow.com/questions/6802577/python-rotation-of-3d-vector

    Calculating the rotation matrix and applying it to the vector is slower
    than the rotate_vector function, but simply applying a precalculated
    rotation matrix is much faster.
    """
    axis = np.asarray(axis)
    theta = np.asarray(theta)
    axis = axis/math.sqrt(np.dot(axis, axis))
    a = math.cos(theta/2)
    b, c, d = -axis*math.sin(theta/2)
    aa, bb, cc, dd = a*a, b*b, c*c, d*d
    bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
    return np.array([[aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac)],
                     [2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab)],
                     [2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc]])


class PlatformIK(object):
    def __init__(self, l, n):
        self.l = l
        self.n = n

        self.p_c = None
        self.theta = None

    def solve(self, e_v, f_v, h):
        self.e_v = e_v
        self.f_v = f_v
        self.h = h

        n_v = np.cross(e_v, f_v)
        n_v = n_v / np.linalg.norm(n_v)
        self.n_v = n_v

        theta_init = np.arctan2(e_v[1], e_v[0])
        pz_init = self.h - self.n

        estimate = [theta_init, 0, 0, pz_init]

        rot_mat = rotation_matrix(n_v, radians(120))
        tan60 = np.tan(radians(60))
        x = optimize.fsolve(self.errors, estimate, args=(
            rot_mat, tan60, e_v, n_v, self.h, self.l, self.n
        ))

        theta = x[0]
        self.theta = theta
        platform_centroid = np.array([x[1], x[2], x[3]])
        self.p_c = platform_centroid

        self.n_p = self.p_c + self.n * n_v
        self.e_p = self.n_p + self.e_v

        # If necessary, these points are calculated like they
        # are in the errors function.
        p0_v = rotate_vector_fast(self.l * self.e_v, self.n_v, -self.theta)
        p1_v = np.dot(rot_mat, p0_v)
        p2_v = np.dot(rot_mat, p1_v)
        p0 = p0_v + self.p_c
        p1 = p1_v + self.p_c
        p2 = p2_v + self.p_c

        self.platform_points = np.array([p0, p1, p2]).T

        return theta, platform_centroid, self.platform_points

    def errors(self, x, rot_mat, tan60, e_v, n_v, h, l, n):
        theta, px, py, pz = x

        # Because n_v is fixed, after finding p0, a precomputed
        # rotation matrix can be used to find p1 and p2.
        p0_v = rotate_vector_fast(l * e_v, n_v, -theta)
        p1_v = np.dot(rot_mat, p0_v)
        p2_v = np.dot(rot_mat, p1_v)

        pc = np.array([px, py, pz])
        p0 = p0_v + pc
        p1 = p1_v + pc
        p2 = p2_v + pc

        n_p = pc + n * n_v

        return (point_line_distance_fast(p0, 0),
                point_line_distance_fast(p1, -tan60),
                point_line_distance_fast(p2, tan60),
                n_p[2] - h)


if __name__ == "__main__":
    np.set_printoptions(suppress=True)

    gx_v = np.array([1., 0., 0.])
    gy_v = np.array([0., 1., 0.])
    gz_v = np.array([0., 0., 1.])

    e_v = np.array([1., 0., 0.])
    f_v = np.array([0., 1., 0.])

    e_v = rotate_vector_fast(e_v, gz_v, radians(0))
    f_v = rotate_vector_fast(f_v, gz_v, radians(0))

    e_v = rotate_vector_fast(e_v, gy_v, radians(-10))
    f_v = rotate_vector_fast(f_v, gy_v, radians(-10))

    e_v = rotate_vector_fast(e_v, gx_v, radians(-10))
    f_v = rotate_vector_fast(f_v, gx_v, radians(-10))

    pik = PlatformIK(l=1, n=0.1)
    theta, centroid, points = pik.solve(e_v, f_v, 1)

    print "Solution:"
    print "theta:", degrees(theta)
    print "p_c:", centroid
    print "p:"
    print points
