"""
    pangocffi
    ~~~~~~~~~

    CFFI-based pangocairo bindings for Python
    See README for details.

"""

import ctypes.util
from ._generated.ffi import ffi
import cairocffi


def _dlopen(generated_ffi, *names):
    """Try various names for the same library, for different platforms."""
    for name in names:
        for lib_name in (name, 'lib' + name):
            try:
                path = ctypes.util.find_library(lib_name)
                lib = generated_ffi.dlopen(path or lib_name)
                if lib:
                    return lib
            except OSError:
                pass
    raise OSError("dlopen() failed to load a library: %s" % ' / '.join(names))


pangocairo = _dlopen(ffi, 'pangocairo-1.0', 'pangocairo-1.0-0')


# Imports are normally always put at the top of the file.
# But the wrapper API requires that the pango library be loaded first.
# Therefore, we have to disable linting rules for these lines.
# from .constants import *  # noqa

# from .pango_cairo_font_map import PangoCairoFontMap

def _get_cairo_t_from_cairo_ctx(cairo_context: cairocffi.Context):
    # Cairo does not give access to the pointer publicly. So this is the best
    # we can do.
    # noinspection PyProtectedMember
    return cairo_context._pointer  # noqa


from .render_functions import *  # noqa
from .create_update_functions import *  # noqa
from .font_functions import *  # noqa
from .font_map import PangoCairoFontMap  # noqa
