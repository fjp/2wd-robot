from typing import Optional
from pangocffi import Context
from . import pangocairo
from . import ffi


# Todo: pangocairo.Font needs to be implemented
# def get_cairo_scaled_font_pointer(font: Font) -> ffi.CData:
#     """
#     Gets the cairo_scaled_font_t used by font . The scaled font can be
#     referenced and kept using ``cairo_scaled_font_reference()``.
#
#     :param font:
#         a Pango font from a Pango-Cairo font map
#     :return:
#         the cairo_scaled_font_t used by ``font``
#     """
#     return pangocairo.pango_cairo_font_get_scaled_font(font.get_pointer())


def set_resolution(context: Context, dpi: float) -> None:
    """
    Sets the resolution for the context. This is a scale factor between points
    specified in a PangoFontDescription and Cairo units. The default value is
    96, meaning that a 10 point font will be 13 units high.
    (10 * 96. / 72. = 13.3).

    :param context:
        a Pango context
    :param dpi:
        the resolution in "dots per inch". (Physical inches aren't actually
        involved; the terminology is conventional.) A 0 or negative value
        means to use the resolution from the font map.
    """
    pangocairo.pango_cairo_context_set_resolution(context.get_pointer(), dpi)


def get_resolution(context: Context) -> float:
    """
    Returns the resolution for the Pango context.

    :param context:
        a Pango context
    :return:
        the resolution in "dots per inch". A negative value will be returned
        if no resolution has previously been set.
    """
    return pangocairo.pango_cairo_context_get_resolution(context.get_pointer())


def set_font_options(
        context: Context,
        options: Optional[ffi.CData]
) -> None:
    """
    Sets the font options used when rendering text with this context.
    These options override any options that pango_cairo_update_context()
    derives from the target surface.

    :param context:
        a Pango context
    :param options:
        a cairo_font_options_t, or ``None`` to unset any previously set
        options.
    """
    if options is None:
        options = ffi.NULL
    context_pointer = context.get_pointer()
    pangocairo.pango_cairo_context_set_font_options(context_pointer, options)


def get_font_options(context: Context) -> Optional[ffi.CData]:
    """
        Retrieves any font rendering options previously set with
        pango_cairo_context_set_font_options(). This function does not report
        options that are derived from the target surface by
        pango_cairo_update_context()

        :param context:
            a Pango Context
        :return:
            a cairo_font_options_t pointer previously set on the context,
            otherwise ``None``.
    """
    context_pointer = context.get_pointer()
    font_option_pointer = pangocairo.pango_cairo_context_get_font_options(
        context_pointer
    )
    if font_option_pointer == ffi.NULL:
        return None
    return font_option_pointer
