from __future__ import print_function

import numpy

from numba import cuda
from numba import unittest_support as unittest
from numba.cuda.testing import captured_cuda_stdout


def cuhello():
    i = cuda.grid(1)
    print(i, 999)


def printfloat():
    i = cuda.grid(1)
    print(i, 23, 34.75, 321)


def cuprintary(A):
    i = cuda.grid(1)
    print("A[", i, "]", A[i])


def printempty():
    print()


class TestPrint(unittest.TestCase):
    @unittest.skipIf(True, "Temporarily disable test")
    def test_cuhello(self):
        jcuhello = cuda.jit('void()', debug=False)(cuhello)
        with captured_cuda_stdout() as stdout:
            jcuhello[2, 3]()
        # The output of GPU threads is intermingled, just sanity check it
        out = stdout.getvalue()
        expected = ''.join('%d 999\n' % i for i in range(6))
        self.assertEqual(sorted(out), sorted(expected))

    def test_printfloat(self):
        jprintfloat = cuda.jit('void()', debug=False)(printfloat)
        with captured_cuda_stdout() as stdout:
            jprintfloat()
        # CUDA and the simulator use different formats for float formatting
        self.assertIn(stdout.getvalue(), ["0 23 34.750000 321\n",
                                          "0 23 34.75 321\n"])

    def test_printempty(self):
        cufunc = cuda.jit('void()', debug=False)(printempty)
        with captured_cuda_stdout() as stdout:
            cufunc()
        self.assertEqual(stdout.getvalue(), "\n")

    @unittest.skipIf(True, "Print string not implemented yet")
    def test_print_array(self):
        """
        Eyeballing required
        """
        jcuprintary = cuda.jit('void(float32[:])')(cuprintary)
        A = numpy.arange(10, dtype=numpy.float32)
        jcuprintary[2, 5](A)
        cuda.synchronize()


if __name__ == '__main__':
    unittest.main()
