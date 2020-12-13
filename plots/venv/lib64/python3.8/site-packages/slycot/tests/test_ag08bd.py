# ===================================================
# ag08bd tests

import unittest
from slycot import analysis
import numpy as np

from numpy.testing import assert_almost_equal, assert_equal

# test1 input parameters

test1_l = 9
test1_n = 9
test1_m = 3
test1_p = 3
test1_tol = 1.0e-7
test1_equil = 'N'

test1_A = np.eye(9, dtype=int)

test1_E = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [1, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 1, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 1, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 1, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 1, 0]])

test1_B = np.array([[-1,  0,  0],
                    [ 0,  0,  0],
                    [ 0,  0,  0],
                    [ 0, -1,  0],
                    [ 0,  0,  0],
                    [ 0,  0,  0],
                    [ 0,  0, -1],
                    [ 0,  0,  0],
                    [ 0,  0,  0]])

test1_C = np.array([[ 0,  1,  1,  0,  3,  4,  0,  0,  2],
                    [ 0,  1,  0,  0,  4,  0,  0,  2,  0],
                    [ 0,  0,  1,  0, -1,  4,  0, -2,  2]])

test1_D = np.array([[ 1,  2, -2],
                    [ 0, -1, -2],
                    [ 0,  0,  0]])


class test_tg01fd(unittest.TestCase):
    """ Verify ag08bd with input parameters according to example in documentation """

    def test1_ag08bd(self):
        """test [A-lambda*E]

        B,C,D must have correct dimensions according to l,n,m and p, but cannot
        have zero length in any dimenstion. Then the wrapper will complain.
        The length is then set to one.
        """

        Af,Ef,nrank,niz,infz,kronr,infe,kronl = analysis.ag08bd(l=test1_l,n=test1_n,m=0,p=0,A=test1_A,E=test1_E,B=np.zeros((test1_l,1)),C=np.zeros((1,test1_n)),D=np.zeros((1,1)),equil=test1_equil, tol=test1_tol)

        assert_equal(Af, np.zeros((0,0)))
        assert_equal(Ef, np.zeros((0,0)))
        assert_equal(nrank, 9)
        assert_equal(niz, 6)
        assert_equal(infz, [0,3])
        assert_equal(kronr, [])
        assert_equal(infe, [3,3,3])
        assert_equal(kronl, [])

    def test2_ag08bd(self):
        """test [A-lambda*E;C]

        B,D must have correct dimensions as before
        """

        Af,Ef,nrank,niz,infz,kronr,infe,kronl = analysis.ag08bd(l=test1_l,n=test1_n,m=0,p=test1_p,A=test1_A,E=test1_E,B=np.zeros((test1_l,1)),C=test1_C,D=np.zeros((test1_p,1)),equil=test1_equil, tol=test1_tol)

        assert_equal(Af, np.zeros((0,0)))
        assert_equal(Ef, np.zeros((0,0)))
        assert_equal(nrank, 9)
        assert_equal(niz, 4)
        assert_equal(infz, [0,2])
        assert_equal(kronr, [])
        assert_equal(infe, [1,3,3])
        assert_equal(kronl, [0,1,1])

    def test3_ag08bd(self):
        """test [A-lambda*E,B]

        C,D must have correct dimensions as before
        """

        Af,Ef,nrank,niz,infz,kronr,infe,kronl = analysis.ag08bd(l=test1_l,n=test1_n,m=test1_m,p=0,A=test1_A,E=test1_E,B=test1_B,C=np.zeros((1,test1_n)),D=np.zeros((1,test1_m)),equil=test1_equil, tol=test1_tol)

        assert_equal(Af, np.zeros((0,0)))
        assert_equal(Ef, np.zeros((0,0)))
        assert_equal(nrank, 9)
        assert_equal(niz, 0)
        assert_equal(infz, [])
        assert_equal(kronr, [2,2,2])
        assert_equal(infe, [1,1,1])
        assert_equal(kronl, [])

    def test4_ag08bd(self):
        """test [A-lambda*E,B;C,D]"""

        Af,Ef,nrank,niz,infz,kronr,infe,kronl = analysis.ag08bd(l=test1_l,n=test1_n,m=test1_m,p=test1_p,A=test1_A,E=test1_E,B=test1_B,C=test1_C,D=test1_D,equil=test1_equil, tol=test1_tol)

        assert_almost_equal(Af, [[0.77045021]])
        assert_almost_equal(Ef, [[0.77045021]])
        assert_equal(nrank, 11)
        assert_equal(niz, 2)
        assert_equal(infz, [0,1])
        assert_equal(kronr, [2])
        assert_equal(infe, [1,1,1,1,3])
        assert_equal(kronl, [1])


if __name__ == "__main__":
    unittest.main()
