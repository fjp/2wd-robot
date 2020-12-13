# ===================================================
# tg01ad tests

import unittest
from slycot import transform
import numpy as np

from numpy.testing import assert_raises, assert_almost_equal, assert_equal

# test1 input parameters

test1_l = 4
test1_n = 4
test1_m = 2
test1_p = 2
test1_job = 'A'
test1_thresh = 0.0

test1_A = \
    np.array([[-1.0,    0.0,    0.0,    3e-3   ],
              [ 0.0,    0.0,    0.1,    2e-2   ],
              [ 1e2,    10.0,   0.0,    0.4    ],
              [ 0.0,    0.0,    0.0,    0.0    ]])

test1_E = \
    np.array([[ 1.0,    0.2,    0.0,    0.0    ],
              [ 0.0,    1.0,    0.0,    1e-2   ],
              [ 3e2,    90.0,   6.0,    0.3    ],
              [ 0.0,    0.0,    20.0,   0.0    ]])

test1_B = \
    np.array([[ 10.0,   0.0    ],
              [ 0.0,    0.0    ],
              [ 0.0,    1e3    ],
              [ 1e4,    1e4    ]])

test1_C = \
    np.array([[-0.1,    0.0,    1e-3,    0.0   ],
              [ 0.0,    1e-2,  -1e-3,    1e-4  ]])

test1_A_desired = \
    np.array([[-1.0,    0.0,    0.0,     0.3   ],
              [ 0.0,    0.0,    1.0,     2.0   ],
              [ 1.0,    0.1,    0.0,     0.4   ],
              [ 0.0,    0.0,    0.0,     0.0   ]])

test1_E_desired = \
    np.array([[ 1.0,    0.2,    0.0,     0.0   ],
              [ 0.0,    1.0,    0.0,     1.0   ],
              [ 3.0,    0.9,    0.6,     0.3   ],
              [ 0.0,    0.0,    0.2,     0.0   ]])

test1_B_desired = \
    np.array([[ 1e2,    0.0    ],
              [ 0.0,    0.0    ],
              [ 0.0,    1e2    ],
              [ 1e2,    1e2    ]])

test1_C_desired = \
    np.array([[-1e-2,   0.0,    1e-3,    0.0   ],
              [ 0.0,    1e-3,  -1e-3,    1e-3  ]])

test1_lscale_desired = \
    np.array([  10.0,   10.0,   0.1,     1e-2  ])

test1_rscale_desired = \
    np.array([  0.1,    0.1,    1.0,     10.0  ])

class test_tg01ad(unittest.TestCase):
    """ test1: Verify tg01ad with input parameters according to example in documentation """

    def test1_tg01ad(self):

        A,E,B,C,lscale,rscale = transform.tg01ad(l=test1_l,n=test1_n,m=test1_m,p=test1_p,A=test1_A,E=test1_E,B=test1_B,C=test1_C,job=test1_job, thresh=test1_thresh)

        assert_almost_equal(A, test1_A_desired)
        assert_almost_equal(E, test1_E_desired)
        assert_almost_equal(B, test1_B_desired)
        assert_almost_equal(C, test1_C_desired)
        assert_almost_equal(lscale, test1_lscale_desired)
        assert_almost_equal(rscale, test1_rscale_desired)


if __name__ == "__main__":
    unittest.main()
