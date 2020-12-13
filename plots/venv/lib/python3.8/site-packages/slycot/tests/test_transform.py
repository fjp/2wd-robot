#
# test_transform.py - generic tests for transform routines
# repagh <rene.vanpaassen@gmail.com, May 2020

import pytest
from .test_exceptions import assert_docstring_parse
from slycot import transform as tf
from slycot.exceptions import SlycotArithmeticError


@pytest.mark.parametrize(
    'fun,        exception_class,       erange,         checkvars',
    ((tf.tb03ad, SlycotArithmeticError, 2,              {}),
     (tf.tb05ad, SlycotArithmeticError, 2,              {'n30': 90,
                                                         'jomega': 2.0,
                                                         'rcond': 1e-12}),
     (tf.td04ad, SlycotArithmeticError, (3,),           {}),
     (tf.tc04ad, SlycotArithmeticError, 1,              {'leri': 'L'})))
def test_transform_docparse(fun, exception_class, erange, checkvars):
    assert_docstring_parse(fun.__doc__,  exception_class, erange, checkvars)
