# ===================================================
# tb05ad tests

from slycot import transform
from slycot.exceptions import SlycotArithmeticError, SlycotParameterError
import sys
import numpy as np
from scipy.linalg import matrix_balance, eig
import unittest
from numpy.testing import assert_almost_equal

# set the random seed so we can get consistent results.
np.random.seed(40)
CASES = {}

# This was (pre 2020) a known failure for tb05ad when running job 'AG'
CASES['known'] = {'A': np.array([[-0.5,  0.,  0.,  0.],
                                 [ 0.,  -1.,  0.,  0.],
                                 [ 1.,   0., -0.5, 0.],
                                 [ 0.,   1.,  0., -1.]]),
                  'B': np.array([[ 1.,  0.],
                                 [ 0.,  1.],
                                 [ 0.,  0.],
                                 [ 0.,  0.]]),
                  'C': np.array([[ 0.,  1.,  1.,  0.],
                                 [ 0.,  1.,  0.,  1.],
                                 [ 0.,  1.,  1.,  1.]])}

n = 20
p = 10
m = 14

CASES['pass1'] = {'A': np.random.randn(n, n),
                  'B':  np.random.randn(n, m),
                  'C':  np.random.randn(p, n)}


class test_tb05ad(unittest.TestCase):

    def test_tb05ad_ng(self):
        """
        Test that tb05ad with job 'NG' computes the correct
        frequency response.
        """
        for key in CASES:
            sys = CASES[key]
            self.check_tb05ad_AG_NG(sys, 10*1j, 'NG')

    def test_tb05ad_ag(self):
        """
        Test that tb05ad with job 'AG' computes the correct
        frequency response.
        """
        for key in CASES:
            sys = CASES[key]
            self.check_tb05ad_AG_NG(sys, 10*1j, 'AG')

    def test_tb05ad_nh(self):
        """Test that tb05ad with job = 'NH' computes the correct
        frequency response after conversion to Hessenberg form.

        First call tb05ad with job='NH' to transform to upper Hessenberg
        form which outputs the transformed system.
        Subsequently, call tb05ad with job='NH' using this transformed system.
        """
        jomega = 10*1j
        for key in CASES:
            sys = CASES[key]
            sys_transformed = self.check_tb05ad_AG_NG(sys, jomega, 'NG')
            self.check_tb05ad_NH(sys_transformed, sys, jomega)

    def test_tb05ad_errors(self):
        """
        Test tb05ad error handling. We give wrong inputs and
        and check that this raises an error.
        """
        self.check_tb05ad_errors(CASES['pass1'])

    def check_tb05ad_AG_NG(self, sys, jomega, job):
        """
        Check that tb05ad computes the correct frequency response when
        running jobs 'AG' and/or 'NG'.

        Inputs
        ------

        sys: A a dict of system matrices with keys 'A', 'B', and 'C'.
        jomega: A complex scalar, which is the frequency we are
                evaluating the system at.
        job: A string, either 'AG' or 'NH'

        Returns
        -------
        sys_transformed: A dict of the system matrices which have been
                         transformed according the job.
        """
        n, m = sys['B'].shape
        p = sys['C'].shape[0]
        result = transform.tb05ad(n, m, p, jomega,
                                  sys['A'], sys['B'], sys['C'], job=job)
        g_i = result[3]
        hinvb = np.linalg.solve(np.eye(n) * jomega - sys['A'], sys['B'])
        g_i_solve = sys['C'].dot(hinvb)
        assert_almost_equal(g_i_solve, g_i)
        sys_transformed = {'A': result[0], 'B': result[1], 'C': result[2]}
        return sys_transformed

    def check_tb05ad_NH(self, sys_transformed, sys, jomega):
        """
        Check tb05ad, computes the correct frequency response when
        job='NH' and we supply system matrices 'A', 'B', and 'C'
        which have been transformed by a previous call to tb05ad.
        We check we get the same result as computing C(sI - A)^-1B
        with the original system.

        Inputs
        ------

        sys_transformed: A a dict of the transformed (A in upper
                         hessenberg form) system matrices with keys
                         'A', 'B', and 'C'.

        sys: A dict of the original un-transformed system matrices.

        jomega: A complex scalar, which is the frequency to evaluate at.

        """

        n, m = sys_transformed['B'].shape
        p = sys_transformed['C'].shape[0]
        result = transform.tb05ad(n, m, p, jomega, sys_transformed['A'],
                                  sys_transformed['B'], sys_transformed['C'],
                                  job='NH')
        g_i = result[0]
        hinvb = np.linalg.solve(np.eye(n) * jomega - sys['A'], sys['B'])
        g_i_solve = sys['C'].dot(hinvb)
        assert_almost_equal(g_i_solve, g_i)

    def check_tb05ad_errors(self, sys):
        """
        Check the error handling of tb05ad. We give wrong inputs and
        and check that this raises an error.
        """
        n, m = sys['B'].shape
        p = sys['C'].shape[0]
        jomega = 10*1j
        # test error handling
        # wrong size A
        with self.assertRaises(SlycotParameterError) as cm:
            transform.tb05ad(
                n+1, m, p, jomega, sys['A'], sys['B'], sys['C'], job='NH')
        assert cm.exception.info == -7
        # wrong size B
        with self.assertRaises(SlycotParameterError) as cm:
            transform.tb05ad(
                n, m+1, p, jomega, sys['A'], sys['B'], sys['C'], job='NH')
        assert cm.exception.info == -9
        # wrong size C
        with self.assertRaises(SlycotParameterError) as cm:
            transform.tb05ad(
                n, m, p+1, jomega, sys['A'], sys['B'], sys['C'], job='NH')
        assert cm.exception.info == -11
        # unrecognized job
        with self.assertRaises(SlycotParameterError) as cm:
            transform.tb05ad(
                n, m, p, jomega, sys['A'], sys['B'], sys['C'], job='a')
        assert cm.exception.info == -1

    @unittest.skipIf(sys.version < "3", "no assertRaisesRegex in old Python")
    def test_tb05ad_resonance(self):
        """ Test tb05ad resonance failure.

        Actually test one of the exception messages. These
        are parsed from the docstring, tests both the info index and the
        message
        """
        A = np.array([[0, -1],
                      [1, 0]])
        B = np.array([[1],
                      [0]])
        C = np.array([[0, 1]])
        jomega = 1j
        with self.assertRaisesRegex(
                SlycotArithmeticError,
                r"Either `freq`.* is too near to an eigenvalue of A,\n"
                r"or `rcond` is less than the machine precision EPS.") as cm:
            transform.tb05ad(2, 1, 1, jomega, A, B, C, job='NH')
        assert cm.exception.info == 2

    def test_tb05ad_balance(self):
        """Test balancing in tb05ad.

        Tests for the cause of the problem reported in issue #11
        balancing permutations were not correctly applied to the
        C and D matrix.
        """

        # find a good test case. Some sparsity,
        # some zero eigenvalues, some non-zero eigenvalues,
        # and proof that the 1st step, with dgebal, does some
        # permutation and some scaling
        crit = False
        n = 8
        while not crit:
            A = np.random.randn(n, n)
            A[np.random.uniform(size=(n, n)) > 0.35] = 0.0

            Aeig = eig(A)[0]
            neig0 = np.sum(np.abs(Aeig) == 0)
            As, T = matrix_balance(A)
            nperm = np.sum(np.diag(T == 0))
            nscale = n - np.sum(T == 1.0)
            crit = nperm < n and nperm >= n//2 and \
                neig0 > 1 and neig0 <= 3 and nscale > 0

        # print("number of permutations", nperm, "eigenvalues=0", neig0)
        B = np.random.randn(8, 4)
        C = np.random.randn(3, 8)

        # do a run
        jomega = 1.0
        At, Bt, Ct, rcond, g_jw, ev, hinvb, info = transform.tb05ad(
            8, 4, 3, jomega, A, B, C, job='AG')

        # remove information on Q, in lower sub-triangle part of A
        At = np.triu(At, k=-1)

        # now after the balancing in DGEBAL, and conversion to
        # upper Hessenberg form:
        # At = Q^T * (P^-1 * A * P ) * Q
        # with Q orthogonal
        # Ct = C * P * Q
        # Bt = Q^T * P^-1 * B
        # so test with Ct * At * Bt  ==  C * A * B
        # and verify that eigenvalues of both A matrices are close
        assert_almost_equal(np.dot(np.dot(Ct, At), Bt),
                            np.dot(np.dot(C, A), B))
        # uses a sort, there is no guarantee on the order of eigenvalues
        eigAt = eig(At)[0]
        idxAt = np.argsort(eigAt)
        eigA = eig(A)[0]
        idxA = np.argsort(eigA)
        assert_almost_equal(eigA[idxA], eigAt[idxAt])


if __name__ == "__main__":
    unittest.main()
