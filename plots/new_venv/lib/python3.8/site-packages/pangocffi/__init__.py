import ctypes.util
from ._generated.ffi import ffi


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


pango = _dlopen(ffi, 'pango', 'pango-1', 'pango-1.0', 'pango-1.0-0')
gobject = _dlopen(ffi, 'gobject-2.0', 'gobject-2.0-0')


# Imports are normally always put at the top of the file.
# But the wrapper API requires that the pango library be loaded first.
# Therefore, we have to disable linting rules for these lines.
from .version import *  # noqa
from .enums import *  # noqa
from .convert import *  # noqa
from .font_description import FontDescription  # noqa
from .rectangle import Rectangle  # noqa
from .item import Item  # noqa
from .context import Context  # noqa
from .glyph_item import GlyphItem  # noqa
from .glyph_item_iter import GlyphItemIter  # noqa
from .layout_run import LayoutRun  # noqa
from .layout_iter import LayoutIter  # noqa
from .layout import Layout  # noqa
