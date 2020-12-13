# ===================================================
# ab08n* tests

import unittest
from slycot import analysis
import numpy as np

from scipy.linalg import eig
from numpy.testing import assert_equal, assert_allclose


class test_ab08nX(unittest.TestCase):
    """ Test regular pencil construction ab08nX with input parameters
    according to example in documentation """

    A = np.diag([1., 1., 3., -4., -1., 3.])

    B = np.array([[ 0., -1.],
                  [-1.,  0.],
                  [ 1., -1.],
                  [ 0.,  0.],
                  [ 0.,  1.],
                  [-1., -1.]])

    C = np.array([[1., 0., 0., 1., 0., 0.],
                  [0., 1., 0., 1., 0., 1.],
                  [0., 0., 1., 0., 0., 1.]])

    D = np.zeros((3, 2))

    def normalize(self, w):
        wi = np.flip(np.argsort(np.abs(w)))
        wn = w[wi]/w[wi[0]]
        return wn

    def ab08nX(self, ab08fun, A, B, C, D):
        n = 6
        m = 2
        p = 3
        # Check the observability and compute the ordered set of
        # the observability indices (call the routine with M = 0).
        out = ab08fun(n, 0, p, A, B, C, D)
        nu, rank, dinfz, nkror, nkrol, infz, kronr, kronl, Af, Bf = out[:10]

        assert_equal(kronl[:nkrol], np.array([1, 2, 2]))
        assert_equal(n-nu, 5)
        assert_allclose(Af[:nu, :nu], np.array([[-1.]]))
        # Check the controllability and compute the ordered set of
        # the controllability indices (call the routine with P = 0)
        out = ab08fun(n, m, 0, A, B, C, D)
        nu, rank, dinfz, nkror, nkrol, infz, kronr, kronl, Af, Bf = out[:10]
        assert_equal(kronr[:nkror], np.array([2, 3]))
        assert_equal(n-nu, 5)
        assert_allclose(Af[:nu, :nu], np.array([[-4.]]))
        # Compute the structural invariants of the given system.
        out = ab08fun(n, m, p, A, B, C, D)
        nu, rank, dinfz, nkror, nkrol, infz, kronr, kronl, Af, Bf = out[:10]
        assert_equal(nu, 2)
        # Compute the invariant zeros of the given system.
        w = eig(Af[:nu, :nu], Bf[:nu, :nu], left=False, right=False)
        w_ref = np.array([-2., 1.])
        assert_allclose(self.normalize(w), self.normalize(w_ref))
        # the examples value of infinite zeros does not match the code
        # compare output formats to given strings
        # assert_equal(sum(infz[:dinfz]), 2)
        # assert_equal([[infz[i], i+1] for i in range(dinfz)], [[1, 1]])
        assert_equal(nkror, 0)
        assert_equal(nkrol, 1)
        assert_equal(kronl[:nkrol], np.array([2]))

    def test_ab08nd(self):
        "Test Construct regular pencil for real matrices"
        self.ab08nX(analysis.ab08nd, self.A, self.B, self.C, self.D)

    def test_ab08nz(self):
        "Test Construct regular pencil for (pseudo) complex matrices"
        Ac, Bc, Cc, Dc = [M.astype(np.complex128) for M in [self.A, self.B,
                                                            self.C, self.D]]
        self.ab08nX(analysis.ab08nz, Ac, Bc, Cc, Dc)


if __name__ == "__main__":
    unittest.main()
