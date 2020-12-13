import cairocffi
import pangocffi
from . import pangocairo
from . import ffi
from . import _get_cairo_t_from_cairo_ctx


# Todo: Font and GlyphString needs to be implemented
# def show_glyph_string(
#         cairo_context: cairocffi.Context,
#         font: pangocffi.Font,
#         glyphs: pangocffi.GlyphString
# ) -> None:
#     """
#     Draws the glyphs in glyphs in the specified cairo context. The origin of
#     the glyphs (the left edge of the baseline) will be drawn at the current
#     point of the cairo context.
#
#     :param cairo_context:
#         a Cairo context
#     :param font:
#         a Pango font from a PangoCairoFontMap
#     :param glyphs:
#         a Pango glyph string
#     """
#     cairo_context_pointer = _get_cairo_t_from_cairo_ctx(cairo_context)
#     pangocairo.pango_cairo_show_glyph_string(
#         cairo_context_pointer,
#         font.get_pointer(),
#         glyphs.get_pointer()
#     )


def show_glyph_item(
        cairo_context: cairocffi.Context,
        text: str,
        glyph_item: pangocffi.GlyphItem
) -> None:
    """
    Draws the glyphs in ``glyph_item`` in the specified cairo context,
    embedding the text associated with the glyphs in the output if the output
    format supports it (PDF for example), otherwise it acts similar to
    :meth:`show_glyph_string()`.

    The origin of the glyphs (the left edge of the baseline) will be drawn at
    the current point of the cairo context.

    Note that ``text`` is the start of the text for layout, which is then
    indexed by ``glyph_item->item->offset``.

    :param cairo_context:
        a Cairo context
    :param text:
        the UTF-8 text that :param:`glyph_item` refers to
    :param glyph_item:
        a Pango glyph item
    :return:
    """
    cairo_context_pointer = _get_cairo_t_from_cairo_ctx(cairo_context)
    text_pointer = ffi.new('char[]', text.encode('utf8'))
    pangocairo.pango_cairo_show_glyph_item(
        cairo_context_pointer,
        text_pointer,
        glyph_item.get_pointer()
    )


# Todo: LayoutLine needs to be implemented
# def show_layout_line(
#         cairo_context: cairocffi.Context,
#         line: pangocffi.LayoutLine
# ) -> None:
#     """
#     Draws a PangoLayoutLine in the specified cairo context. The origin of the
#     glyphs (the left edge of the line) will be drawn at the current point of
#     the cairo context.
#
#     :param cairo_context:
#         a Cairo context
#     :param line:
#         a Pango layout line
#     """
#     cairo_context_pointer = _get_cairo_t_from_cairo_ctx(cairo_context)
#     pangocairo.pango_cairo_show_layout_line(
#         cairo_context_pointer,
#         line.get_pointer()
#     )


def show_layout(
        cairo_context: cairocffi.Context,
        layout: pangocffi.Layout
) -> None:
    """
    Draws a Pango Layout in the specified cairo context. The top-left corner
    of the PangoLayout will be drawn at the current point of the cairo context.

    :param cairo_context:
        a Cairo context
    :param layout:
        a Pango layout
    """
    cairo_context_pointer = _get_cairo_t_from_cairo_ctx(cairo_context)
    pangocairo.pango_cairo_show_layout(
        cairo_context_pointer,
        layout.get_pointer()
    )


def show_error_underline(
        cairo_context: cairocffi.Context,
        x: float,
        y: float,
        width: float,
        height: float
) -> None:
    """
    Draw a squiggly line in the specified cairo context that approximately
    covers the given rectangle in the style of an underline used to indicate
    a spelling error. (The width of the underline is rounded to an integer
    number of up/down segments and the resulting rectangle is centered in the
    original rectangle)

    :param cairo_context:
        a Cairo context
    :param x:
        The X coordinate of one corner of the rectangle
    :param y:
        The Y coordinate of one corner of the rectangle
    :param width:
        Non-negative width of the rectangle
    :param height:
        Non-negative height of the rectangle
    """
    cairo_context_pointer = _get_cairo_t_from_cairo_ctx(cairo_context)
    pangocairo.pango_cairo_show_error_underline(
        cairo_context_pointer,
        x,
        y,
        width,
        height
    )


# Todo: Font and GlyphString needs to be implemented
# def glyph_string_path(
#         cairo_context: cairocffi.Context,
#         font: pangocffi.Font,
#         glyphs: pangocffi.GlyphString
# ) -> None:
#     """
#     Adds the glyphs in glyphs to the current path in the specified cairo
#     context. The origin of the glyphs (the left edge of the baseline) will
#     be at the current point of the cairo context.
#
#     :param cairo_context:
#         a Cairo context
#     :param font:
#         a Pango font from a PangoCairoFontMap
#     :param glyphs:
#         a Pango glyph string
#     """
#     cairo_context_pointer = _get_cairo_t_from_cairo_ctx(cairo_context)
#     pangocairo.pango_cairo_glyph_string_path(
#         cairo_context_pointer,
#         font.get_pointer(),
#         glyphs.get_pointer()
#     )


# Todo: LayoutLine needs to be implemented
# def layout_line_path(
#     """
#     Adds the text in PangoLayoutLine to the current path in the specified
#     cairo context. The origin of the glyphs (the left edge of the line) will
#     be at the current point of the cairo context.
#
#     :param cairo_context:
#         a Cairo context
#     :param line:
#         a Pango layout line
#     """
#     cairo_context: cairocffi.Context,
#     line: pangocffi.LayoutLine
# ) -> None:
#     cairo_context_pointer = _get_cairo_t_from_cairo_ctx(cairo_context)
#     pangocairo.pango_cairo_layout_line_path(
#         cairo_context_pointer,
#         line.get_pointer()
#     )


def layout_path(
        cairo_context: cairocffi.Context,
        layout: pangocffi.Layout
) -> None:
    """
    Adds the text in a ``Pango.Layout`` to the current path in the specified
    cairo context. The top-left corner of the ``Pango.Layout`` will be at the
    current point of the cairo context.

    :param cairo_context:
        a Cairo context
    :param layout:
        a Pango layout
    """
    cairo_context_pointer = _get_cairo_t_from_cairo_ctx(cairo_context)
    pangocairo.pango_cairo_layout_path(
        cairo_context_pointer,
        layout.get_pointer()
    )


def error_underline_path(
        cairo_context: cairocffi.Context,
        x: float,
        y: float,
        width: float,
        height: float
) -> None:
    """
    Add a squiggly line to the current path in the specified cairo context that
    approximately covers the given rectangle in the style of an underline used
    to indicate a spelling error. (The width of the underline is rounded to an
    integer number of up/down segments and the resulting rectangle is centered
    in the original rectangle)

    :param cairo_context:
        a Cairo context
    :param x:
        The X coordinate of one corner of the rectangle
    :param y:
        The Y coordinate of one corner of the rectangle
    :param width:
        Non-negative width of the rectangle
    :param height:
        Non-negative height of the rectangle
    """
    cairo_context_pointer = _get_cairo_t_from_cairo_ctx(cairo_context)
    pangocairo.pango_cairo_error_underline_path(
        cairo_context_pointer,
        x,
        y,
        width,
        height
    )
