import numpy as np
cimport numpy as np


cdef extern from "math.h":
    double sin(double x)
    double cos(double x)


def rotate_vector_fast(np.ndarray[np.float64_t, ndim=1] vector,
                       np.ndarray[np.float64_t, ndim=1] axis,
                       np.float64_t angle):
    """
    Uses Rodrigues rotation formula
    axis must be a normal vector

    Implements:
    v_rot = (v * np.cos(angle) + np.cross(k, v) * np.sin(angle) +
             k * (np.dot(k, v)) * (1 - np.cos(angle)))
    """

    v = <np.float64_t*> vector.data
    k = <np.float64_t*> axis.data

    # Cos-scaled component
    cdef np.float64_t[3] cos_component
    copy3v(cos_component, v)
    scale3v(cos_component, cos(angle))

    # Cross component
    cdef np.float64_t[3] k_cross_v
    cross3v(k, v, k_cross_v)
    scale3v(k_cross_v, sin(angle))

    # Last component
    cdef np.float64_t[3] last_component
    copy3v(last_component, k)
    scale3v(last_component, dot3v(k, v))
    scale3v(last_component, 1 - cos(angle))

    # Result
    cdef np.float64_t[3] result
    copy3v(result, cos_component)
    add3v(result, k_cross_v)
    add3v(result, last_component)

    v_rot = np.zeros(3)
    v_rot[0] = result[0]
    v_rot[1] = result[1]
    v_rot[2] = result[2]

    return v_rot


cdef np.float64_t dot3v(np.float64_t* a, np.float64_t* b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

cdef void cross3v(np.float64_t* a, np.float64_t* b, np.float64_t* out):
    out[0] = a[1] * b[2] - a[2] * b[1]
    out[1] = a[2] * b[0] - a[0] * b[2]
    out[2] = a[0] * b[1] - a[1] * b[0]

cdef void scale3v(np.float64_t* v, np.float64_t gain):
    v[0] = v[0] * gain
    v[1] = v[1] * gain
    v[2] = v[2] * gain

cdef void copy3v(np.float64_t* dest, np.float64_t* src):
    dest[0] = src[0]
    dest[1] = src[1]
    dest[2] = src[2]

cdef void add3v(np.float64_t* accum, np.float64_t* delta):
    accum[0] = accum[0] + delta[0]
    accum[1] = accum[1] + delta[1]
    accum[2] = accum[2] + delta[2]
