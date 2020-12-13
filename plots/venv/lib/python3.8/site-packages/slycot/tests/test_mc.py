#
# test_mc.py - test suite for polynomial and rational function manipulation
# bnavigator <code@bnavigator.de>, Aug 2019

import pytest
import re

from slycot import mc01td
from slycot.exceptions import SlycotResultWarning


def test_mc01td():
    """ test_mc01td: doc example
    data from http://slicot.org/objects/software/shared/doc/MC01TD.html
    """
    (dp, stable, nz) = mc01td('C', 4, [2, 0, 1, -1, 1])
    assert dp == 4
    assert stable == 0
    assert nz == 2

def test_mc01td_D():
    """ test_mc01td_D: test discrete option """
    (dp, stable, nz) = mc01td('D', 3, [1, 2, 3, 4])
    assert dp == 3
    assert stable == 1
    assert nz == 0
    (dp, stable, nz) = mc01td('D', 3, [4, 3, 2, 1])
    assert dp == 3
    assert stable == 0
    assert nz == 3

def test_mc01td_warnings():
    """ test_mc01td_warnings: Test warnings """
    T = [([0, 0], "Entry ``P(x)`` is the zero polynomial."),
         ([0, 1], "The polynomial ``P(x)`` is most probably unstable,\n"
                  "although it may be stable with one or more zeros\n"
                  "very close to the imaginary axis.\n"
                  "The number of unstable zeros (NZ) is not determined."),
         ([1, 0], "The degree of the polynomial ``P(x)`` has been\n"
                  "reduced to ``(DB - 1)`` because\n"
                  "``P(DB+1-j) = 0.0`` on entry\n"
                  "for ``j = 0, 1,..., k-1`` and ``P(DB+1-k) <> 0.0``.")]
    for P, m in T:
        with pytest.warns(SlycotResultWarning, match=re.escape(m)):
            (dp, stable, nz) = mc01td('C', len(P)-1, P)
