import cython
import numpy as np
cimport numpy as np


cdef extern from "math.h":
    double sin(double x)
    double cos(double x)
    double sqrt(double x)
    double atan(double x)
    double atan2(double y, double x)


ctypedef np.float64_t npfloat


cdef npfloat cmin(npfloat a, npfloat b):
  if a <= b:
    return a
  else:
    return b


@cython.boundscheck(False)
@cython.wraparound(False)
cdef void vsubtract(npfloat[:] out, npfloat[:] a, npfloat[:] b, int n):
  for i in range(n):
    out[i] = a[i] - b[i]


@cython.boundscheck(False)
@cython.wraparound(False)
cdef void rotate3(npfloat[:] m, npfloat[:] p, npfloat[:] out):
  cdef npfloat out0 = m[0]*p[0] + m[1]*p[1] + m[2]*p[2]
  cdef npfloat out1 = m[3]*p[0] + m[4]*p[1] + m[5]*p[2]
  cdef npfloat out2 = m[6]*p[0] + m[7]*p[1] + m[8]*p[2]

  out[0] = out0
  out[1] = out1
  out[2] = out2

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.initializedcheck(False)
def solve_fast(npfloat[:] p0, npfloat[:] p1, npfloat m, npfloat l):
    cdef npfloat[3] p0p
    cdef npfloat[3] p1p
    cdef npfloat[:] _p0p = p0p
    cdef npfloat[:] _p1p = p1p

    # Translate to origin
    vsubtract(_p1p, p1, p0, 3)

    # Rotate to X-Z plane
    cdef npfloat angle = -atan2(_p1p[1], _p1p[0])

    cdef npfloat[9] rot_matrix
    rot_matrix[:] = [
        cos(angle), -sin(angle), 0.,
        sin(angle),  cos(angle), 0.,
        0.,          0.,         1.
    ]
    cdef npfloat[:] _rot_matrix = rot_matrix
    rotate3(_rot_matrix, _p1p, _p1p)

    # Determine root actuator angle
    cdef npfloat alpha1, alpha2
    two_link_from_origin_ik(_p1p[0], _p1p[2], m, l, &alpha1, &alpha2)
    cdef npfloat alpha = cmin(alpha1, alpha2)

    return alpha


@cython.cdivision(True)
cdef void two_link_from_origin_ik(npfloat x, npfloat z,
                                  npfloat m, npfloat l,
                                  npfloat* angle1, npfloat* angle2):
    cdef npfloat a = m**2 - l**2 - x**2 - z**2
    cdef npfloat b = 2 * x * l
    cdef npfloat c = 2 * z * l

    cdef npfloat t1 = (-2 * c + sqrt(4 * (c**2 - a**2 + b**2))) / (2 * (a - b))
    cdef npfloat t2 = (-2 * c - sqrt(4 * (c**2 - a**2 + b**2))) / (2 * (a - b))

    angle1[0] = 2 * atan(t1)
    angle2[0] = 2 * atan(t2)
