#
# test_ab.py - generic tests for analysis programs
# repagh <rene.vanpaassen@gmail.com, May 2020

import pytest
from .test_exceptions import assert_docstring_parse
from slycot import analysis
from slycot.exceptions import SlycotArithmeticError, SlycotResultWarning


@pytest.mark.parametrize(
    'fun,              exception_class,       erange,         checkvars',
    ((analysis.ab05nd, SlycotArithmeticError, 1,              {'p1': 1}),
     (analysis.ab07nd, SlycotResultWarning,   2,              {'m': 1}),
     (analysis.ab09ad, SlycotArithmeticError, 3,              {'dico': 'C'}),
     (analysis.ab09ad, SlycotArithmeticError, (2,),           {'dico': 'D'}),
     (analysis.ab09ad, SlycotResultWarning,   ((1, 0), ),     {'nr': 3,
                                                               'Nr': 2}),
     (analysis.ab09ax, SlycotArithmeticError, 2,              {'dico': 'C'}),
     (analysis.ab09ax, SlycotResultWarning,   ((1, 0), ),     {'nr': 3,
                                                               'Nr': 2}),
     (analysis.ab09ad, SlycotArithmeticError, 3,              {'dico': 'C'}),
     (analysis.ab09ad, SlycotResultWarning,   ((1, 0), ),     {'nr': 3,
                                                               'Nr': 2}),
     (analysis.ab09md, SlycotArithmeticError, 3,              {'alpha': -0.1}),
     (analysis.ab09md, SlycotResultWarning,   ((1, 0), (2, 0)), {'nr': 3,
                                                               'Nr': 2,
                                                               'alpha': -0.1}),
     (analysis.ab09nd, SlycotArithmeticError, 3,              {'alpha': -0.1}),
     (analysis.ab09nd, SlycotResultWarning,   ((1, 0), (2, 0)), {'nr': 3,
                                                               'Nr': 2,
                                                               'alpha': -0.1}),
     (analysis.ab13bd, SlycotArithmeticError, 6,              {'dico': 'C'}),
     (analysis.ab13bd, SlycotResultWarning,   ((1, 0),),      {}),
     (analysis.ab13dd, SlycotArithmeticError, 4,              {}),
     (analysis.ab13ed, SlycotArithmeticError, 1,              {}),
     (analysis.ab13fd, SlycotArithmeticError, (2,),           {}),
     (analysis.ab13fd, SlycotResultWarning,   (1,),           {})))
def test_ab_docparse(fun, exception_class, erange, checkvars):
    assert_docstring_parse(fun.__doc__,  exception_class, erange, checkvars)


