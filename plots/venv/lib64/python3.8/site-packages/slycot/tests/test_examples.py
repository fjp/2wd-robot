"""

test_examples.py


"""

from inspect import getmembers, isfunction
import pytest

from slycot import examples

examplefunctions = [fun for (fname, fun) in getmembers(examples)
                    if isfunction(fun) and "_example" in fname]

# Exempt certain functions to produce warnings with attributes (iwarn, info)
#
# Failed to compute beta(A) within the specified tolerance
examples.ab13fd_example.allow_iwarninfo = [(None, 1)]


def check_warn(recwarn, examplefun):
    """Returns True if a warning occurs that is not exempt"""
    for w in recwarn:
        try:
            if (w.message.iwarn, w.message.info) in examplefun.allow_iwarninfo:
                continue
        except AttributeError:
            pass
        return True


@pytest.mark.parametrize('examplefun', examplefunctions)
#ignore numpy ABI change warnings https://github.com/numpy/numpy/pull/432
@pytest.mark.filterwarnings("ignore:numpy.dtype size changed")
@pytest.mark.filterwarnings("ignore:numpy.ufunc size changed")
def test_example(examplefun, capsys, recwarn):
    """
    Test the examples.

    Test that all the examples work, produce some (unchecked) output but no
    exceptions or warnings.
    """
    examplefun()
    captured = capsys.readouterr()

    # fail for first in order
    failconditions = [
        ((not len(captured.out) > 0), "Example {} did not print any results\n"),
        (captured.err, "Example {} wrote to stderr\n"),
        (check_warn(recwarn, examplefun), "Example {} produced a warning.\n")]
    for failed, msgfmt  in failconditions:
        if failed:
            pytest.fail(msgfmt.format(examplefun.__name__) +
                        "Captured output:\n{}\n"
                        "Captured stderr:\n{}\n"
                        "Captured warnings:\n{}\n"
                        "".format(captured.out,
                                  captured.err,
                                  [w.message for w in recwarn]))
