#
# test_sg03ad.py - test suite for stability margin commands
# RvP, 15 Jun 2017
from __future__ import print_function

import unittest
from slycot import synthesis
import numpy as np

from numpy.testing import assert_almost_equal

# test cases from
# Penzl T., Numerical Solution of Generalized Lyapunov Equations
# http://www.qucosa.de/fileadmin/data/qucosa/documents/4168/data/b002.pdf


class test_sg03ad(unittest.TestCase):

    def test_sg03ad_ex1c(self):
        """ Example 1 continuous case"""
        n = 100
        Xref = np.ones((n, n))
        U = np.tril(Xref)
        for t in range(0, 50, 10):
            A = (2**(-t) - 1) * np.eye(n) + np.diag(np.arange(1., n+1.)) + U.T
            E = np.eye(n) + 2**(-t) * U
            Y = A.T.dot(Xref).dot(E) + E.T.dot(Xref).dot(A)
            Q = np.zeros((n, n))
            Z = np.zeros((n, n))
            A, E, Q, Z, X, scale, sep, ferr, alphar, alphai, beta = \
                synthesis.sg03ad('C', 'B', 'N', 'N', 'L', n, A, E, Q, Z, Y)
            assert_almost_equal(X, Xref)

    def test_sg03ad_ex1d(self):
        """ Example 1 discrete case"""
        n = 100
        Xref = np.ones((n, n))
        U = np.tril(Xref)
        for t in range(0, 50, 10):
            A = 2**(-t) * np.eye(n) + np.diag(np.arange(1., n+1.)) + U.T
            E = np.eye(n) + 2**(-t) * U
            Y = A.T.dot(Xref).dot(A) - E.T.dot(Xref).dot(E)
            Q = np.zeros((n, n))
            Z = np.zeros((n, n))
            A, E, Q, Z, X, scale, sep, ferr, alphar, alphai, beta = \
                synthesis.sg03ad('D', 'B', 'N', 'N', 'L', n, A, E, Q, Z, Y)
            assert_almost_equal(X, Xref)

    def test_sg03ad_b1(self):
        """ SLICOT doc example / Penzl B.1 """
        n = 3
        A = np.array([[3.0, 1.0, 1.0],
                      [1.0, 3.0, 0.0],
                      [1.0, 0.0, 2.0]])
        E = np.array([[1.0, 3.0, 0.0],
                      [3.0, 2.0, 1.0],
                      [1.0, 0.0, 1.0]])
        Y = np.array([[64.0, 73.0, 28.0],
                      [73.0, 70.0, 25.0],
                      [28.0, 25.0, 18.0]])
        Xref = np.array([[-2.0000, -1.0000, 0.0000],
                         [-1.0000, -3.0000, -1.0000],
                         [0.0000, -1.0000, -3.0000]])
        Q = np.zeros((3, 3))
        Z = np.zeros((3, 3))
        A, E, Q, Z, X, scale, sep, ferr, alphar, alphai, beta = \
            synthesis.sg03ad('C', 'B', 'N', 'N', 'L', n, A, E, Q, Z, -Y)
        # print(A, E, Q, Z, X, scale, sep)
        assert_almost_equal(X, Xref)


if __name__ == "__main__":
    unittest.main()
