from . import pango, ffi


def pango_version() -> int:
    """Return the pango version number as a single integer, such as 14204 for
    ``1.42.4``.
    Major, minor and micro versions are "worth" 10000, 100 and 1 respectively.

    Can be useful as a guard for method not available in older pango versions::

        if pango_version() >= 11000:
            ...

    """
    return pango.pango_version()


def pango_version_string() -> str:
    """Return the pango version number as a string, such as ``1.42.4``."""
    return ffi.string(pango.pango_version_string()).decode('ascii')
