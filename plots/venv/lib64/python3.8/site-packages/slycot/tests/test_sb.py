# ===================================================
# sb* synthesis tests

from slycot import synthesis
from slycot.exceptions import raise_if_slycot_error, \
                              SlycotParameterError, SlycotArithmeticError, \
                              SlycotResultWarning

from numpy import array, eye, zeros
from numpy.testing import assert_allclose, assert_raises
import pytest
from .test_exceptions import assert_docstring_parse


def test_sb02mt():
    """Test if sb02mt is callable

    This is a dummy test, not really checking the wrapper of the FORTRAN
    function
    """
    out = synthesis.sb02mt(1, 1, 1., 1.)
    assert(len(out) == 8)


def test_sb10ad():
    """Test sb10ad, Hinf synthesis"""
    a = array([[-1]])
    b = array([[1, 1]])
    c = array([[1],
               [1]])
    d = array([[0, 1],
               [1, 0]])

    n = 1
    m = 2
    np = 2
    ncon = 1
    nmeas = 1
    gamma = 10

    gamma_est, Ak, Bk, Ck, Dk, Ac, Bc, Cc, Dc, rcond = synthesis.sb10ad(
        n, m, np, ncon, nmeas, gamma, a, b, c, d)
    # from Octave, which also uses SB10AD:
    #   a= -1; b1= 1; b2= 1; c1= 1; c2= 1; d11= 0; d12= 1; d21= 1; d22= 0;
    #   g = ss(a,[b1,b2],[c1;c2],[d11,d12;d21,d22]);
    #   [k,cl] = hinfsyn(g,1,1);
    # k.a is Ak, cl.a is Ac
    # gamma values don't match; not sure that's critical
    # this is a bit fragile
    # a simpler, more robust check might be to check stability of Ac
    assert_allclose(Ak, array([[-3.]]))
    assert_allclose(Ac, array([[-1., -1.],
                               [1., -3.]]))


def test_sb10jd():
    """ verify the output of sb10jd for a descriptor system """

    # test1 input parameters
    n = 6
    m = 1
    np = 6

    A = array([[ 0,  0,  0, -1,  1,  0],
               [ 0, 32,  0,  0, -1,  1],
               [ 0,  0,  1,  0,  0,  0],
               [ 0,  0,  0,  1,  0,  0],
               [-1,  1,  0,  0,  0,  0],
               [ 0, -1,  1,  0,  0,  0]])
    E = array([[  0,   0,   0,   0,   0,   0],
               [  0,   0,   0,   0,   0,   0],
               [  0,   0,   0, -10,   0,  10],
               [  0,   0,   0,   0,   0,   0],
               [  0,   0,   0,   0,   0,   0],
               [  0,   0,   0,   0,   0,   0]])
    B = array([[-7.1],
               [ 0. ],
               [ 0. ],
               [ 0. ],
               [ 0. ],
               [ 0. ]])
    C = eye(6)
    D = zeros((7, 1))

    # test1 expected results
    Aexp = array([[-0.003125]])
    Bexp = array([[ 0.059000]])
    Cexp = array([[-1.17519e-02],
                  [-1.17519e-02],
                  [-1.17519e-02],
                  [ 0.         ],
                  [ 0.         ],
                  [ 3.76060e-01]])
    Dexp = array([[ 2.21875e-01],
                  [ 2.21875e-01],
                  [ 2.21875e-01],
                  [ 0.         ],
                  [ 7.100000+00],
                  [ 0.         ]])

    A_r, B_r, C_r, D_r = synthesis.sb10jd(n, m, np, A, B, C, D, E)
    assert_allclose(A_r, Aexp, atol=1e-5)
    assert_allclose(B_r, Bexp, atol=1e-5)
    assert_allclose(C_r, Cexp, atol=1e-5)
    assert_allclose(D_r, Dexp, atol=1e-5)


def test_sb10fd():
    A = array(((-1.0,  0.0,  4.0,  5.0, -3.0, -2.0),
               (-2.0,  4.0, -7.0, -2.0,  0.0,  3.0),
               (-6.0,  9.0, -5.0,  0.0,  2.0, -1.0),
               (-8.0,  4.0,  7.0, -1.0, -3.0,  0.0),
               ( 2.0,  5.0,  8.0, -9.0,  1.0, -4.0),
               ( 3.0, -5.0,  8.0,  0.0,  2.0, -6.0)))
    B = array(((-3.0, -4.0, -2.0,  1.0,  0.0),
               ( 2.0,  0.0,  1.0, -5.0,  2.0),
               (-5.0, -7.0,  0.0,  7.0, -2.0),
               ( 4.0, -6.0,  1.0,  1.0, -2.0),
               (-3.0,  9.0, -8.0,  0.0,  5.0),
               ( 1.0, -2.0,  3.0, -6.0, -2.0)))
    C = array(((1.0, -1.0,  2.0, -4.0,  0.0, -3.0),
               (-3.0,  0.0,  5.0, -1.0,  1.0,  1.0),
               (-7.0,  5.0,  0.0, -8.0,  2.0, -2.0),
               ( 9.0, -3.0,  4.0,  0.0,  3.0,  7.0),
               ( 0.0,  1.0, -2.0,  1.0, -6.0, -2.0)))
    D = array((( 1.0, -2.0, -3.0,  0.0,  0.0),
               ( 0.0,  4.0,  0.0,  1.0,  0.0),
               ( 5.0, -3.0, -4.0,  0.0,  1.0),
               ( 0.0,  1.0,  0.0,  1.0, -3.0),
               ( 0.0,  0.0,  1.0,  7.0,  1.0)))

    gamma, tol = 15.0, 0.00000001
    n, m, np, ncon, nmeas = 6, 5, 5, 2, 2

    # ldwork too small
    assert_raises(SlycotParameterError, synthesis.sb10fd,
        n, m, np, ncon, nmeas, gamma, A, B, C, D, tol, 1)
    Ak, Bk, Ck, Dk, rcond = synthesis.sb10fd(
        n, m, np, ncon, nmeas, gamma, A, B, C, D, tol, 900)
    Ak, Bk, Ck, Dk, rcond = synthesis.sb10fd(
        n, m, np, ncon, nmeas, gamma, A, B, C, D, tol, 0)
    Ak, Bk, Ck, Dk, rcond = synthesis.sb10fd(
        n, m, np, ncon, nmeas, gamma, A, B, C, D, tol)

    Ak_ref = array((
        ( -2.8043,  14.7367,   4.6658,   8.1596,   0.0848,   2.5290),
        (  4.6609,   3.2756,  -3.5754,  -2.8941,   0.2393,   8.2920),
        (-15.3127,  23.5592,  -7.1229,   2.7599,   5.9775,  -2.0285),
        (-22.0691,  16.4758,  12.5523, -16.3602,   4.4300,  -3.3168),
        ( 30.6789,  -3.9026,  -1.3868,  26.2357,  -8.8267,  10.4860),
        ( -5.7429,   0.0577,  10.8216, -11.2275,   1.5074, -10.7244)))
    Bk_ref = array((
        ( -0.1581,  -0.0793),
        ( -0.9237,  -0.5718),
        (  0.7984,   0.6627),
        (  0.1145,   0.1496),
        ( -0.6743,  -0.2376),
        (  0.0196,  -0.7598)))
    Ck_ref = array((
        ( -0.2480,  -0.1713,  -0.0880,   0.1534,   0.5016,  -0.0730),
        (  2.8810,  -0.3658,   1.3007,   0.3945,   1.2244,   2.5690)))
    Dk_ref = array((
        (  0.0554,   0.1334),
        ( -0.3195,   0.0333)))

    assert_allclose(Ak, Ak_ref, rtol=1e-3)
    assert_allclose(Bk, Bk_ref, rtol=1e-3)
    assert_allclose(Ck, Ck_ref, rtol=1e-3)
    assert_allclose(Dk, Dk_ref, rtol=1e-3, atol=1e-4)
    assert_allclose(rcond, (1.0, 1.0, 0.011241, 0.80492e-3), rtol=1e-4)


def test_sb10fd_2():
    """ fails, from python-control issue #367"""
    A = array([[-1, 0, 0], [0, -12, -5], [0, 4, 0]])
    B = array([[2, 0], [0, 0.5], [0, 0]])
    C = array([[-0.5, 0, -0.5], [0, 0, 0], [-0.5, 0, -0.5]])
    D = array([[0, 0],  [0, 1], [0, 0]], dtype=float)
    assert_raises(SlycotArithmeticError, synthesis.sb10fd,
                  3, 2, 3, 1, 1, 100, A, B, C, D, 1e-7, None)


@pytest.mark.parametrize(
    'fun,               exception_class,       erange,         checkvars',
    ((synthesis.sb01bd, SlycotArithmeticError, 2,              {}),
     (synthesis.sb01bd, SlycotResultWarning,   [3, 4, [1, 0]], {'nap': '1'}),
     (synthesis.sb02md, SlycotArithmeticError, 5,              {}),
     (synthesis.sb02od, SlycotArithmeticError, 6,              {}),
     (synthesis.sb03md, SlycotResultWarning,   3,              {'n': 2,
                                                                'dico': 'D'}),
     (synthesis.sb03md, SlycotResultWarning,   3,              {'n': 2,
                                                                'dico': 'C'}),
     (synthesis.sb03od, SlycotResultWarning,   [1, 2],         {'dico': 'C',
                                                                'fact': 'N'}),
     (synthesis.sb03od, SlycotResultWarning,   [1, 2],         {'dico': 'D',
                                                                'fact': 'N'}),
     (synthesis.sb03od, SlycotArithmeticError, [3, 4, 5, 6],   {'dico': 'D',
                                                                'fact': 'F'}),
     (synthesis.sb04md, SlycotArithmeticError, 2,              {'m': 1}),
     (synthesis.sb04qd, SlycotArithmeticError, 3,              {'m': 2}),
     (synthesis.sb10ad, SlycotArithmeticError, 12,             {}),
     (synthesis.sb10dd, SlycotArithmeticError, 9,              {}),
     (synthesis.sb10hd, SlycotArithmeticError, 4,              {}),
     (synthesis.sb10jd, SlycotArithmeticError, 0,              {}),
     (synthesis.sg02ad, SlycotArithmeticError, 7,              {}),
     (synthesis.sg02ad, SlycotResultWarning,   [[1, 0]],       {'dico': 'C'}),
     (synthesis.sg02ad, SlycotResultWarning,   [[1, 0]],       {'dico': 'D'}),
     (synthesis.sg03ad, SlycotArithmeticError, 2,              {}),
     (synthesis.sg03ad, SlycotResultWarning,   [3, 4],         {}),
     (synthesis.sg03bd, SlycotResultWarning,   1,              {}),
     (synthesis.sg03bd, SlycotArithmeticError, range(2, 8),    {}),
     (synthesis.sb10fd, SlycotArithmeticError, 9,              {}),
     (synthesis.sb10fd, SlycotParameterError,  (-27, ),        {})))
def test_sb_docparse(fun, exception_class, erange, checkvars):
    assert_docstring_parse(fun.__doc__,  exception_class, erange, checkvars)
