import pangocffi
from pangocffi import Context
from pangocffi.ffi_build import ffi as ffi_builder
from cffi import FFI
import ctypes


def _dlopen(ffi, *names):
    """Try various names for the same library, for different platforms."""
    for name in names:
        for lib_name in (name, 'lib' + name):
            try:
                path = ctypes.util.find_library(lib_name)
                lib = ffi.dlopen(path or lib_name)
                if lib:
                    return lib
            except OSError:
                pass
    raise OSError("dlopen() failed to load a library: %s" % ' / '.join(names))


class ContextCreator(object):
    cairo = None
    cairo_surface_t = None
    cairo_t = None
    pango_context = None

    @staticmethod
    def get_pango_context() -> Context:
        ffi = FFI()
        ffi.include(ffi_builder)
        ffi.cdef('''
            /* Cairo */
            typedef void cairo_t;
            typedef struct _cairo_surface cairo_surface_t;

            typedef enum _cairo_status {
                CAIRO_STATUS_SUCCESS = 0,

                CAIRO_STATUS_NO_MEMORY,
                CAIRO_STATUS_INVALID_RESTORE,
                CAIRO_STATUS_INVALID_POP_GROUP,
                CAIRO_STATUS_NO_CURRENT_POINT,
                CAIRO_STATUS_INVALID_MATRIX,
                CAIRO_STATUS_INVALID_STATUS,
                CAIRO_STATUS_NULL_POINTER,
                CAIRO_STATUS_INVALID_STRING,
                CAIRO_STATUS_INVALID_PATH_DATA,
                CAIRO_STATUS_READ_ERROR,
                CAIRO_STATUS_WRITE_ERROR,
                CAIRO_STATUS_SURFACE_FINISHED,
                CAIRO_STATUS_SURFACE_TYPE_MISMATCH,
                CAIRO_STATUS_PATTERN_TYPE_MISMATCH,
                CAIRO_STATUS_INVALID_CONTENT,
                CAIRO_STATUS_INVALID_FORMAT,
                CAIRO_STATUS_INVALID_VISUAL,
                CAIRO_STATUS_FILE_NOT_FOUND,
                CAIRO_STATUS_INVALID_DASH,
                CAIRO_STATUS_INVALID_DSC_COMMENT,
                CAIRO_STATUS_INVALID_INDEX,
                CAIRO_STATUS_CLIP_NOT_REPRESENTABLE,
                CAIRO_STATUS_TEMP_FILE_ERROR,
                CAIRO_STATUS_INVALID_STRIDE,
                CAIRO_STATUS_FONT_TYPE_MISMATCH,
                CAIRO_STATUS_USER_FONT_IMMUTABLE,
                CAIRO_STATUS_USER_FONT_ERROR,
                CAIRO_STATUS_NEGATIVE_COUNT,
                CAIRO_STATUS_INVALID_CLUSTERS,
                CAIRO_STATUS_INVALID_SLANT,
                CAIRO_STATUS_INVALID_WEIGHT,
                CAIRO_STATUS_INVALID_SIZE,
                CAIRO_STATUS_USER_FONT_NOT_IMPLEMENTED,
                CAIRO_STATUS_DEVICE_TYPE_MISMATCH,
                CAIRO_STATUS_DEVICE_ERROR,
                CAIRO_STATUS_INVALID_MESH_CONSTRUCTION,
                CAIRO_STATUS_DEVICE_FINISHED,
                CAIRO_STATUS_JBIG2_GLOBAL_MISSING,
                CAIRO_STATUS_PNG_ERROR,
                CAIRO_STATUS_FREETYPE_ERROR,
                CAIRO_STATUS_WIN32_GDI_ERROR,
                CAIRO_STATUS_TAG_ERROR,

                CAIRO_STATUS_LAST_STATUS
            } cairo_status_t;

            typedef cairo_status_t (*cairo_write_func_t) (
                void * closure,
                const unsigned char *data,
                unsigned int length
            );
            cairo_surface_t * cairo_pdf_surface_create_for_stream (
                cairo_write_func_t write_func,
                void *closure,
                double width_in_points,
                double height_in_points
            );
            void cairo_surface_destroy (cairo_surface_t *surface);
            cairo_t * cairo_create (cairo_surface_t *target);
            void cairo_destroy (cairo_t *cr);

            PangoContext * pango_cairo_create_context (cairo_t *cr);
        ''')
        ffi.set_source('pangocffi._generated.ffi', None)
        cairo = _dlopen(ffi, 'cairo', 'cairo-2', 'cairo-gobject-2',
                        'cairo.so.2')
        pangocairo = _dlopen(ffi, 'pangocairo-1.0', 'pangocairo-1.0-0')

        cairo_surface_t = cairo.cairo_pdf_surface_create_for_stream(
            ffi.NULL,
            ffi.NULL,
            10,
            10
        )
        cairo_t = cairo.cairo_create(cairo_surface_t)

        pango_pointer = pangocairo.pango_cairo_create_context(cairo_t)
        pango_pointer = pangocffi.ffi.cast('PangoContext *', pango_pointer)
        pango_pointer = pangocffi.ffi.gc(
            pango_pointer,
            pangocffi.gobject.g_object_unref
        )

        ContextCreator.cairo = cairo
        ContextCreator.cairo_surface_t = cairo_surface_t
        ContextCreator.cairo_t = cairo_t
        ContextCreator.pango_context = Context.from_pointer(pango_pointer)

        return ContextCreator.pango_context

    @staticmethod
    def free():
        ContextCreator.pango_context = None
        ContextCreator.cairo.cairo_destroy(ContextCreator.cairo_t)
        ContextCreator.cairo.cairo_surface_destroy(
            ContextCreator.cairo_surface_t
        )
