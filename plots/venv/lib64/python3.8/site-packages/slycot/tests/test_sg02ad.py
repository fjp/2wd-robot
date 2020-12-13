#
# test_sg02ad.py - test suite for ricatti equation solving
# RvP, 19 Jun 2017
from __future__ import print_function

import unittest
from slycot import synthesis
import numpy as np

from numpy.testing import assert_almost_equal


class test_sg02ad(unittest.TestCase):

    def test_sg02ad_case1(self):
        n = 3
        m = 1
        # from a discussion here:
        # https://github.com/scipy/scipy/issues/2251
        A = np.array([[ 0.63399379,  0.54906824,  0.76253406],
                      [ 0.5404729 ,  0.53745766,  0.08731853],
                      [ 0.27524045,  0.84922129,  0.4681622 ]])
        B = np.array([[ 0.96861695],
                      [ 0.05532739],
                      [ 0.78934047]])
        Q = np.eye(3)
        E = np.eye(3)
        R = np.ones((1,1), dtype=float)
        S = np.array([[-2.67522766, -5.39447418,  2.19128542],
                      [-1.94918951, -3.15480639,  5.24379117],
                      [ 4.29133973,  8.10585767, -5.88895897]])
        L = np.array(np.zeros((3,1)))
        rcondu, X, alphar, alphai, beta, S, T, U, iwarn = \
            synthesis.sg02ad('D', 'B', 'N', 'U', 'Z', 'N', 'S', 'R',
                             n, m, 1,
                             A, E, B, Q, R, L)
        LATXB = L + A.T.dot(X).dot(B)
        assert_almost_equal(
            A.T.dot(X).dot(A) -
            E.T.dot(X).dot(E) -
            LATXB.dot(np.linalg.solve(R+B.T.dot(X).dot(B), LATXB.T)) + Q,
            np.zeros((n, n)))


if __name__ == "__main__":
    unittest.main()
