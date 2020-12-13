"""
docstring_check.py

Copyright 2020 Slycot team

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License version 2 as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
MA 02110-1301, USA.
"""

import pytest
import subprocess
import sys

from slycot.exceptions import raise_if_slycot_error, \
                              SlycotError, SlycotWarning, SlycotParameterError
from slycot import _wrapper


def assert_docstring_parse(docstring, exception_class, erange, checkvars={}):
    """To check that a docstring can be parsed into exceptions
    See also raise_if_slycot_error

    Parameters
    ----------
    docstring: str
        Documentation string with exception definitions
    exception_class: SlycotError or SlycotWarning
        Subclass of Slycot specific Errors or Warnings expected to raise
    erange: int or iterable with int or iterable of two-element iterables
        of [IWARN, INFO] for which the documentation should have exception
        text
    checkvars: dict, optional
        dict of variables for evaluation of <infospec> and formatting the
        exception message
    """

    # if erange is a simple integer, assume a continous range of errors
    try:
        erange = range(1, erange+1)
    except TypeError:
        pass

    for e in erange:
        try:
            iwarn, info = e
        except TypeError:
            iwarn = None
            info = e
        if issubclass(exception_class, SlycotError):
            with pytest.raises(exception_class) as ex_info:
                raise_if_slycot_error(e, [], docstring, checkvars)
            assert ex_info.value.info == info
        elif issubclass(exception_class, SlycotWarning):
            with pytest.warns(exception_class) as wm:
                raise_if_slycot_error(e, [], docstring, checkvars)
            assert wm[0].message.iwarn == iwarn
            assert wm[0].message.info == info
        else:
            raise RuntimeError("Invalid test exception")


def test_standard_info_error():
    """Test the standard case of illegal arguments"""
    with pytest.raises(SlycotParameterError) as ex_info:
        raise_if_slycot_error(-2, ["a", "b"])
    assert ex_info.value.info == -2


def test_unhandled_info_iwarn():
    with pytest.raises(SlycotError) as ex_info:
        raise_if_slycot_error(100, [], docstring="no valid docstring")
    assert ex_info.value.info == 100
    with pytest.warns(SlycotWarning) as wm:
        raise_if_slycot_error([101, 0], [], docstring="no valid docstring")
        raise_if_slycot_error(0, [], docstring="no valid docstring",
                              checkvars={'iwarn': 102})
    assert wm[0].message.iwarn == 101
    assert wm[0].message.info == 0
    assert wm[1].message.iwarn == 102
    assert wm[1].message.info == 0


# Test code for test_xerbla_override
CODE = """
import sys
sys.path.pop(0)  # do not import from current directory ('')
from slycot._wrapper import __file__, ab08nd
print(__file__)
# equil='X' is invalid
out = ab08nd(1, 1, 1, [1], [1], [1], [1], equil='X')
print("INFO={}".format(out[-1]))
"""


def test_xerbla_override():
    """Test that Fortran routines calling XERBLA do not print to stdout."""

    try:
        out = subprocess.check_output([sys.executable, '-c', CODE],
                                       stderr=subprocess.STDOUT,
                                       universal_newlines=True)
    except subprocess.CalledProcessError as cpe:
        raise RuntimeError("Trying to call _wrapper.ab08nd() failed with "
                           "returncode {}.\n"
                           "Captured STDOUT: \n {}\n"
                           "Captured STDERR: \n {}\n"
                           "".format(cpe.returncode, cpe.stdout, cpe.stderr))

    outlines = out.splitlines()
    assert len(outlines) == 2
    assert outlines[0] == _wrapper.__file__
    assert outlines[1] == "INFO=-1"
