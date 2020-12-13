#
# test_td04ad.py - test suite for tf -> ss conversion
# RvP, 04 Jun 2018

from __future__ import print_function, division

import unittest
from slycot import transform
import numpy as np


class TestTf2SS(unittest.TestCase):

    def test_td04ad_c(self):
        """td04ad: Convert with 'C' option"""

        # for octave:
        """
        num = { [0.0,  0.0, 1.0 ], [ 1.0, 0.0 ];
                [3.0, -1.0, 1.0 ], [ 0.0, 1.0 ];
                [0.0, 0.0, 1.0],   [ 0.0, 2.0 ] };
        den = { [1.0,  0.4, 3.0],  [ 1.0, 1.0 ];
                [1.0,  0.4, 3.0],  [ 1.0, 1.0 ];
                [1.0,  0.4, 3.0],  [ 1.0, 1.0 ]};
        """

        m = 2
        p = 3
        d = 3
        num = np.array([
            [ [0.0,  0.0, 1.0], [1.0, 0.0, 0.0] ],
            [ [3.0, -1.0, 1.0], [0.0, 1.0, 0.0] ],
            [ [0.0,  0.0, 1.0], [0.0, 2.0, 0.0] ] ])

        numc = np.zeros((max(1, m, p), max(1, m, p), d), dtype=float)
        numc[:p,:m,:] = num
        denc = np.array(
            [ [1.0,  0.4, 3.0],  [ 1.0, 1.0, 0.0 ] ])
        indc = np.array(
            [ 2, 1 ], dtype=int)

        nref = 3
        Aref = np.array([ [-1,    0,    0],
                          [ 0, -0.4, -0.3],
                          [ 0,   10,    0] ])
        Bref = np.array([ [0, -1],
                          [1,  0],
                          [0,  0] ])
        Cref = np.array([ [1,     0,  0.1],
                          [-1, -2.2, -0.8],
                          [-2,    0,  0.1] ])
        Dref = np.array([ [0, 1],
                          [3, 0],
                          [0, 0] ])

        nr, A, B, C, D = transform.td04ad('C', m, p, indc, denc, numc)
        #print('A=\n', A, '\nB=\n', B, '\nC=\n', C, '\nD=\n', D)
        np.testing.assert_equal(nref, nr)
        # the returned state space representation is not guaranteed to
        # be of one form for all architectures, so we transform back
        # to tf and check for equality then
        _, _, _, _, _, dcoeff, ucoeff = transform.tb04ad(
                nr, m, p, A, B, C, D)
        _, _, _, _, _, dcoeffref, ucoeffref = transform.tb04ad(
                nref, m, p, Aref, Bref, Cref, Dref)
        np.testing.assert_array_almost_equal(dcoeff,dcoeffref)
        np.testing.assert_array_almost_equal(ucoeff,ucoeffref)

    def test_td04ad_r(self):
        """td04ad: Convert with 'R' option

        example program from
        http://slicot.org/objects/software/shared/doc/TD04AD.html
        """

        m = 2
        p = 2
        rowcol = 'R'
        index = [3, 3]
        dcoeff = np.array([ [1.0, 6.0, 11.0, 6.0], [1.0, 6.0, 11.0, 6.0] ])

        ucoeff = np.array([ [[1.0, 6.0, 12.0, 7.0], [0.0, 1.0,  4.0,  3.0]],
                            [[0.0, 0.0, 1.0,  1.0], [1.0, 8.0, 20.0, 15.0]] ])

        nref = 3

        Aref = np.array([ [ 0.5000,  -0.8028,   0.9387],
                          [ 4.4047,  -2.3380,   2.5076],
                          [-5.5541,   1.6872,  -4.1620] ])
        Bref = np.array([ [-0.2000,  -1.2500],
                          [ 0.0000,  -0.6097],
                          [ 0.0000,   2.2217] ])
        Cref = np.array([ [0.0000,  -0.8679,   0.2119],
                          [0.0000,   0.0000,   0.9002] ])
        Dref = np.array([ [1.0000,   0.0000],
                          [0.0000,   1.0000] ])

        nr, A, B, C, D = transform.td04ad(rowcol, m, p, index, dcoeff, ucoeff)
        #print('A=\n', A, '\nB=\n', B, '\nC=\n', C, '\nD=\n', D)
        np.testing.assert_equal(nref, nr)
        # order of states is not guaranteed, so we reorder the reference
        rindex = np.flip(np.argsort(np.diag(A)))
        Arref = Aref[rindex, :][:, rindex]
        Brref = Bref[rindex, :]
        Crref = Cref[:, rindex]
        Drref = Dref
        np.testing.assert_array_almost_equal(A, Arref,decimal=4)
        np.testing.assert_array_almost_equal(B, Brref,decimal=4)
        np.testing.assert_array_almost_equal(C, Crref,decimal=4)
        np.testing.assert_array_almost_equal(D, Drref,decimal=4)


    def test_staticgain(self):
        """td04ad: Convert a transferfunction to SS with only static gain"""

        # 2 inputs, 3 outputs? columns share a denominator
        num = np.array([ [ [1.0], [2.0] ],
                         [ [0.2], [4.3] ],
                         [ [1.2], [3.2] ] ])
        p, m, d = num.shape
        numc = np.zeros((max(1, m, p), max(1, m, p), d), dtype=float)
        numc[:p,:m,:] = num

        # denc, columns share a common denominator
        denc = np.array([ [ 1.0], [0.5] ])
        Dc = (num / denc).reshape((3,2))
        idxc = np.zeros((2,), dtype=int)

        # denr, rows share a common denominator
        denr = np.array([ [1.0], [0.5], [3.0] ])
        idxr = np.zeros((3,), dtype=int)
        Dr = (num / denr[:, np.newaxis]).reshape((3,2))

        # fails with:
        # On entry to TB01XD parameter number  5 had an illegal value

        n, A, B, C, D = transform.td04ad('C', 2, 3, idxc, denc, numc)
        #print('A=\n', A, '\nB=\n', B, '\nC=\n', C, '\nD=\n', D)
        self.assertEqual(A.shape, (0,0))
        self.assertEqual(B.shape, (0,2))
        self.assertEqual(C.shape, (3,0))
        np.testing.assert_array_almost_equal(D, Dc)

        n, A, B, C, D = transform.td04ad('R', 2, 3, idxr, denr, num)
        #print('A=\n', A, '\nB=\n', B, '\nC=\n', C, '\nD=\n', D)
        self.assertEqual(A.shape, (0,0))
        self.assertEqual(B.shape, (0,2))
        self.assertEqual(C.shape, (3,0))
        np.testing.assert_array_almost_equal(D, Dr)

    def test_td04ad_static(self):
        """Regression: td04ad (TFM -> SS transformation) for static TFM"""
        from itertools import product
        for nout, nin, rc in product(range(1, 6), range(1, 6), ['R', 'C']):
            Dref = np.zeros((nout, nin))
            if rc == 'R':
                num = np.reshape(np.arange(nout * nin), (nout, nin, 1))
                den = np.reshape(np.arange(1, 1 + nout), (nout, 1))
                index = np.repeat(0, nout)
                Dref = num[:nout, :nin, 0] / np.broadcast_to(den, (nout, nin))
            else:
                maxn = max(nout, nin)
                num = np.zeros((maxn, maxn, 1))
                num[:nout, :nin, 0] = np.reshape(
                        np.arange(nout * nin), (nout, nin))
                den = np.reshape(np.arange(1, 1 + nin), (nin, 1))
                index = np.repeat(0, nin)
                Dref = num[:nout, :nin, 0] / np.broadcast_to(den.T, (nout, nin))
            nr, A, B, C, D = transform.td04ad(rc, nin, nout, index, den, num)
            np.testing.assert_equal(nr, 0)
            for M in [A, B, C]:
                np.testing.assert_equal(M, np.zeros_like(M))
            np.testing.assert_almost_equal(D, Dref)

    def test_mixfeedthrough(self):
        """Test case popping up from control testing

        a mix of feedthrough and dynamics. The problem from the control
        package was somewhere else
        """
        num = np.array([ [ [ 0.0,  0.0 ], [ 0.0, -0.2 ] ],
                         [ [ -0.1,  0.0 ], [ 0.0,  0.0 ] ] ])
        p, m, d = num.shape
        numc = np.zeros((max(1, m, p), max(1, m, p), d), dtype=float)
        numc[:p,:m,:] = num
        denc = np.array([[1.0, 1.1],
                         [1.0, 0.0]])
        idxc = np.array([1, 0])
        n, A, B, C, D = transform.td04ad('C', 2, 2, idxc, denc, numc)
        np.testing.assert_array_almost_equal(D, np.array([[0,  0],[-0.1, 0]]))

    def test_toandfrom(self):
        A = np.array([[-3.0]])
        B = np.array([[0.1, 0.0]])
        C = np.array([[1.0],
                      [0.0]])
        D = np.array([[0.0, 0.0],
                      [0.0, 1.0]])

        tfout = transform.tb04ad(1, 2, 2, A, B, C, D)

        num = tfout[6]
        den = tfout[5]
        idxc = np.array([1, 0])
        n, At, Bt, Ct, Dt = transform.td04ad('R', 2, 2, idxc, den, num)
        np.testing.assert_array_almost_equal(D, Dt)
        np.testing.assert_array_almost_equal(A, At)

    def test_tfm2ss_6(self):
        """Python version of Fortran test program from
        -- Bug in TD04AD when ROWCOL='C' #6
           This bug was fixed in PR #27"""
        m = 1
        p = 1
        index = np.array([0])
        dcoeff = np.array([[0.5]])
        ucoeff = np.array([[[32]]])
        n, A, B, C, D = transform.td04ad('R', m, p, index, dcoeff, ucoeff)
        self.assertEqual(n, 0)
        np.testing.assert_array_almost_equal(D, np.array([[64]]))
        n, A, B, C, D = transform.td04ad('C', m, p, index, dcoeff, ucoeff)
        self.assertEqual(n, 0)
        np.testing.assert_array_almost_equal(D, np.array([[64]]))


if __name__ == "__main__":
    unittest.main()
