# ===================================================
# tg01fd tests

import unittest
from slycot import transform
import numpy as np

from numpy.testing import assert_raises, assert_almost_equal, assert_equal

# test1 input parameters
test1_l = 4
test1_n = 4
test1_m = 2
test1_p = 2
test1_tol = 0.0
test1_A = np.array([[-1,  0,  0,  3],
                    [ 0,  0,  1,  2],
                    [ 1,  1,  0,  4],
                    [ 0,  0,  0,  0]])

test1_E = np.array([[1, 2, 0, 0],
                    [0, 1, 0, 1],
                    [3, 9, 6, 3],
                    [0, 0, 2, 0]])

test1_B = np.array([[1, 0],
                    [0, 0],
                    [0, 1],
                    [1, 1]])

test1_C = np.array([[-1,  0,  1,  0],
                    [ 0,  1, -1,  1]])

#test1 expected output
test1_Aexp = np.array([[ 2.02781052,  0.10783277,  3.90616686, -2.15710472],
                       [-0.09804588,  0.25437761,  1.60529591, -0.12692683],
                       [ 0.27131089,  0.77603837, -0.36920735, -0.48533567],
                       [ 0.06900656, -0.56694671, -2.19740106,  0.3086067 ]])

test1_Eexp  = np.array([[10.15874008,  5.82296975,  1.30205562,  0.        ],
                        [ 0.        , -2.468405  , -0.18960188,  0.        ],
                        [ 0.        ,  0.        ,  1.03378058,  0.        ],
                        [ 0.        ,  0.        ,  0.        ,  0.        ]])

test1_Bexp = np.array([[-0.21566555, -0.97049496],
                       [ 0.30148458,  0.95156071],
                       [ 0.75952691,  0.09906873],
                       [ 1.13389342,  0.37796447]])

test1_Cexp = np.array([[ 3.65148372e-01, -1.00000000e+00, -4.47213595e-01, -8.16496581e-01],
                       [-1.09544512e+00,  1.00000000e+00, -8.94427191e-01,  2.22044605e-16]])

test1_Qexp = np.array([[-0.21566555, -0.50875523,  0.61092382,  0.56694671],
                       [-0.10783277, -0.25437761, -0.77603837,  0.56694671],
                       [-0.97049496,  0.1413209 , -0.04953436, -0.18898224],
                       [ 0.        ,  0.81023981,  0.14860309,  0.56694671]])
test1_Zexp = np.array([[-3.65148372e-01, -1.35772740e-16,  4.47213595e-01,  8.16496581e-01],
                       [-9.12870929e-01,  0.00000000e+00,  0.00000000e+00, -4.08248290e-01],
                       [ 6.19714937e-17, -1.00000000e+00,  0.00000000e+00, -1.38572473e-16],
                       [-1.82574186e-01, -6.78863700e-17, -8.94427191e-01,  4.08248290e-01]])

test1_ranke_exp = 3
test1_rnka22_exp = 1

class test_tg01fd(unittest.TestCase):

    def test1_tg01fd(self):
        """ test1: Verify from tg01fd with input parameters according to test in documentation """
        A,E,B,C,ranke,rnka22,Q,Z = transform.tg01fd(l=test1_l,n=test1_n,m=test1_m,p=test1_p,A=test1_A,E=test1_E,B=test1_B,C=test1_C,compq='I',compz='I',joba='T',tol=test1_tol)
        assert_almost_equal(A, test1_Aexp)
        assert_almost_equal(E, test1_Eexp)
        assert_almost_equal(B, test1_Bexp)
        assert_almost_equal(C, test1_Cexp)
        assert_almost_equal(Q, test1_Qexp)
        assert_almost_equal(Z, test1_Zexp)
        assert_equal(test1_ranke_exp, ranke)
        assert_equal(test1_rnka22_exp, rnka22)

    def test2_tg01fd(self):
        """ verify that Q and Z output with compq and compz set to 'U' equals the dot product of Q and Z input and Q and Z output with compq and compz set to 'I' """

        l = 30
        n = 30
        m = 70
        p = 44

        np.random.seed(0)

        Ain = np.random.rand(l, n)
        Ein = np.random.rand(l, n)
        Bin = np.random.rand(n, m)
        Cin = np.random.rand(p, n)
        Qin = np.random.randn(l,l)
        Zin = np.random.randn(n,n)

        A_1,E_1,B_1,C_1,ranke_1,rnka22_1,Q_1,Z_1= transform.tg01fd(l=l,n=n,m=m,p=p,A=Ain,E=Ein,B=Bin,C=Cin,compq='I', compz='I', joba='T', tol=0.0)

        A_2,E_2,B_2,C_2,ranke_2,rnka22_2,Q_2,Z_2= transform.tg01fd(l=l,n=n,m=m,p=p,A=Ain,E=Ein,B=Bin,C=Cin,Q=Qin,Z=Zin,compq='U', compz='U', joba='T', tol=0.0)

        assert_equal(A_1, A_2)
        assert_equal(E_1, E_2)
        assert_equal(B_1, B_2)
        assert_equal(C_1, C_2)
        assert_equal(ranke_1, ranke_2)
        assert_equal(rnka22_1, rnka22_2)

        assert_almost_equal(np.dot(Qin, Q_1), Q_2)
        assert_almost_equal(np.dot(Zin, Z_1), Z_2)


if __name__ == "__main__":
    unittest.main()
