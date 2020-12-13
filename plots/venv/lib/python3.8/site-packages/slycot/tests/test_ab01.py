"""
Test ab01 wrappers

@author: bnavigator
"""

from numpy import array
from numpy.testing import assert_allclose, assert_equal

from scipy.linalg.lapack import dorgqr

from slycot.analysis import ab01nd


def test_ab01nd():
    """SLICOT doc example

    http://slicot.org/objects/software/shared/doc/AB01ND.html"""

    # Example program data
    n = 3
    m = 2
    tol = 0.0

    A = array([[-1., 0., 0.],
               [-2., -2., -2.],
               [-1., 0., -3.]])
    B = array([[1., 0.],
               [0,  2.],
               [0., 1.]])

    for jobz in ['N', 'I', 'F']:
        Ac, Bc, ncont, indcon, nblk, Z, tau = ab01nd(n, m, A, B,
                                                     jobz=jobz, tol=tol)

        # The transformed state dynamics matrix of a controllable realization
        assert_allclose(Ac[:ncont, :ncont], array([[-3.0000,  2.2361],
                                                   [ 0.0000, -1.0000]]),
                        atol=0.0001)

        # and the dimensions of its diagonal blocks are
        assert_equal(nblk[:indcon], array([2]))

        # The transformed input/state matrix B of a controllable realization
        assert_allclose(Bc[:ncont, :],array([[ 0.0000, -2.2361],
                                             [ 1.0000,  0.0000]]),
                        atol=0.0001)

        # The controllability index of the transformed system representation
        assert indcon == 1

        if jobz == 'N':
            assert Z is None
            continue
        elif jobz == 'I':
            Z_ = Z
        elif jobz == 'F':
            Z_, _, info = dorgqr(Z, tau)
            assert info == 0

        # The similarity transformation matrix Z
        assert_allclose(Z_, array([[ 0.0000,  1.0000,  0.0000],
                                  [-0.8944,  0.0000, -0.4472],
                                  [-0.4472,  0.0000,  0.8944]]),
                        atol=0.0001)

