"""
exceptions.py

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

import re

from warnings import warn


class SlycotError(RuntimeError):
    """Slycot exception base class"""

    def __init__(self, message, info):
        super(SlycotError, self).__init__(message)
        self.info = info


class SlycotParameterError(SlycotError, ValueError):
    """A Slycot input parameter had an illegal value.

    In case of a wrong input value, the SLICOT routines return a negative
    info parameter indicating which parameter was illegal.
    """

    pass


class SlycotArithmeticError(SlycotError, ArithmeticError):
    """A Slycot computation failed"""

    pass


class SlycotWarning(UserWarning):
    """Slycot Warning"""

    def __init__(self, message, iwarn, info):
        super(SlycotWarning, self).__init__(message)
        self.info = info
        self.iwarn = iwarn


class SlycotResultWarning(SlycotWarning):
    """Slycot computation result warning

    A Slycot routine returned a nonzero info parameter that warns about the
    returned results, but the results might still be usable.
    """

    pass


def _parse_docsection(section_name, docstring, checkvars):
    slycot_error = None
    message = None
    docline = iter(docstring.splitlines())
    try:

        info_eval = False
        while section_name not in next(docline):
            continue
        section_indent = next(docline).index("-")

        for l in docline:
            # ignore blank lines
            if not l.strip():
                continue

            # reached next section without match
            if l[section_indent] == "-":
                break

            # Exception Type
            ematch = re.match(
                r'(\s*)(Slycot.*(Error|Warning))', l)
            if ematch:
                error_indent = len(ematch.group(1))
                slycot_error = ematch.group(2)

            # new infospec
            if slycot_error:
                imatch = re.match(
                    r'(\s{' + str(error_indent + 1) + r',}):(.+):\s*(.*)', l)
                if imatch:
                    infospec_indent = len(imatch.group(1))
                    infospec = imatch.group(2)
                    # Don't handle the standard case unless we have i
                    if infospec == "info = -i":
                        if 'i' not in checkvars.keys():
                            continue
                    infospec_ = infospec.replace(" = ", " == ")
                    try:
                        info_eval = eval(infospec_, checkvars)
                    except NameError:
                        raise RuntimeError("Unknown variable in infospec: "
                                           + infospec)
                    except SyntaxError:
                        raise RuntimeError("Invalid infospec: " + infospec)
                    if info_eval:
                        message = imatch.group(3).strip() + '\n'
                        mmatch = re.match(
                            r'(\s{' + str(infospec_indent+1) + r',})(.*)',
                            next(docline))
                        if not mmatch:
                            break  # docstring
                        body_indent = len(mmatch.group(1))
                        message += mmatch.group(2) + '\n'
                        for l in docline:
                            if l and not l[:body_indent].isspace():
                                break  # message body
                            message += l[body_indent:] + '\n'
                        break  # docstring
    except StopIteration:
        pass
    return (slycot_error, message)


def raise_if_slycot_error(info, arg_list=None, docstring=None, checkvars=None):
    """Raise exceptions or warnings if slycot info returned is non-zero.

    Parameters
    ----------
    info: int or list of int
        The parameter INFO or [IWARN, INFO] returned by the SLICOT subroutine
    arg_list: list of str, optional
        A list of arguments (possibly hidden by the wrapper) of the SLICOT
        subroutine
    docstring: str, optional
        The docstring of the Slycot function
    checkvars: dict, optional
        dict of variables for evaluation of <infospec> and formatting the
        exception message

    Notes
    -----
    If the numpydoc compliant docstring has a "Raises" section with one or
    multiple definition terms ``SlycotError`` or a subclass of it,
    the matching exception text is used.

    To raise warnings, define a "Warns" section using a ``SlycotWarning``
    definition or a subclass of it.

    The definition body must contain a reST compliant field list with
    ':<infospec>:' as field name, where <infospec> is a python parseable
    expression using the arguments `iwarn`, `info` and any additional variables
    provided in `checkvars` (usually obtained by calling `locals()`.
    A single " = " is treated as " == ".

    The body of the field list contains the exception or warning message and
    can contain replacement fields in format string syntax using the variables
    in `checkvars`.

    For negative info, the argument as indicated in arg_list was erroneous and
    a generic SlycotParameterError is raised if matching infospec was defined.

    Example
    -------
    >>> def fun(info):
    ...     '''Example function
    ...
    ...     Raises
    ...     ------
    ...     SlycotArithmeticError
    ...         :info = 1: INFO is 1
    ...         :info > 1 and info < n:
    ...             INFO is {info}, which is between 1 and {n}
    ...         :n <= info < m:
    ...             {info} is in [{n}, {m:10.2g})!
    ...
    ...     Warns
    ...     -----
    ...     SlycotResultWarning
    ...         :info >= 120: {info} is too large
    ...     SlycotResultWarning
    ...         :iwarn == 1: IWARN is 1
    ...     '''
    ...     n, m = 4, 120.
    ...     raise_if_slycot_error(info,
    ...                           arg_list=["a", "b", "c"],
    ...                           docstring=(fun.__doc__ if type(info) is list
    ...                                          else fun.__doc__[:-60]),
    ...                           checkvars=locals())
    ...
    >>> fun(0)
    >>> fun(-1)
    SlycotParameterError:
    The following argument had an illegal value: a
    >>> fun(1)
    SlycotArithmeticError:
    INFO is 1
    >>> fun(2)
    SlycotArithmeticError:
    INFO is 2, which is between 1 and 4
    >>> fun(4)
    SlycotArithmeticError:
    4 is in [4,    1.2e+02)!
    >>> fun(120)
    SlycotResultWarning:
    120 is too large
    >>> fun([1,0])
    SlycotResultWarning:
    IWARN is 1
    """
    try:
        iwarn, info = info
    except TypeError:
        iwarn = None
    if not checkvars:
        checkvars = {}
    if docstring and (iwarn or info):
        # possibly override info with mandatory argument
        checkvars['info'] = info
        # do not possibly override iwarn if not provided
        if iwarn is not None:
            checkvars['iwarn'] = iwarn

        exception, message = _parse_docsection("Raises", docstring, checkvars)
        if exception and message:
            fmessage = '\n' + message.format(**checkvars).strip()
            raise globals()[exception](fmessage, info)

        warning, message = _parse_docsection("Warns", docstring, checkvars)
        if warning and message:
            fmessage = '\n' + message.format(**checkvars).strip()
            warn(globals()[warning](fmessage, iwarn, info))
            return

    if info < 0 and arg_list:
        message = ("The following argument had an illegal value: {}"
                   "".format(arg_list[-info-1]))
        raise SlycotParameterError(message, info)

    # catch all
    if info > 0:
        raise SlycotError("Caught unhandled nonzero INFO value {}"
                          "".format(info),
                          info)
    if iwarn is None and 'iwarn' in checkvars:
        iwarn = checkvars['iwarn']
    if iwarn:
        warn(SlycotWarning("Caught unhandled nonzero IWARN value {}"
                           "".format(iwarn),
                           iwarn, info))
