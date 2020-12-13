import cairocffi
import pangocffi
from . import pangocairo
from . import _get_cairo_t_from_cairo_ctx


def create_context(cairo_context: cairocffi.Context) -> pangocffi.Context:
    """
    Creates a context object set up to match the current transformation and
    target surface of the Cairo context. This context can then be used to
    create a layout using ``pangocffi.Layout()``.

    This function is a convenience function that creates a context using the
    default font map, then updates it to ``cairo_context``. If you just need to
    create a layout for use with ``cairo_context`` and do not need to access
    PangoContext directly, you can use :meth:`create_layout()` instead.

    :param cairo_context:
        a Cairo context
    :return:
        a Pango context
    """
    cairo_t_pointer = _get_cairo_t_from_cairo_ctx(cairo_context)
    layout_pointer = pangocairo.pango_cairo_create_context(cairo_t_pointer)
    return pangocffi.Context.from_pointer(layout_pointer)


def update_context(
        cairo_context: cairocffi.Context,
        pango_context: pangocffi.Context
) -> None:
    """
    Updates a PangoContext previously created for use with Cairo to match the
    current transformation and target surface of a Cairo context. If any
    layouts have been created for the context, it's necessary to call
    pango_layout_context_changed() on those layouts.

    :param cairo_context:
        a Cairo context
    :param pango_context:
        a Pango context, from a pango-cairo font map
    """
    cairo_t_pointer = _get_cairo_t_from_cairo_ctx(cairo_context)
    pangocairo.pango_cairo_update_context(
        cairo_t_pointer,
        pango_context.get_pointer()
    )


def create_layout(cairo_context: cairocffi.Context) -> pangocffi.Layout:
    """
    Creates a layout object set up to match the current transformation and
    target surface of the Cairo context. This layout can then be used for text
    measurement with functions like ``get_size()`` or drawing with functions
    like :meth:`show_layout()`. If you change the transformation or target
    surface for ``cairo_context``, you need to call
    :meth:`update_layout()`

    This function is the most convenient way to use Cairo with Pango, however
    it is slightly inefficient since it creates a separate PangoContext object
    for each layout. This might matter in an application that was laying out
    large amounts of text.

    :param cairo_context:
        a Cairo context
    :return:
        a Pango layout
    """
    cairo_t_pointer = _get_cairo_t_from_cairo_ctx(cairo_context)
    layout_pointer = pangocairo.pango_cairo_create_layout(cairo_t_pointer)
    return pangocffi.Layout.from_pointer(layout_pointer)


def update_layout(
        cairo_context: cairocffi.Context,
        layout: pangocffi.Layout
) -> None:
    """
    Updates the private Pango ``Context`` of a Pango ``Layout`` created with
    :meth:`create_layout()` to match the current transformation and target
    surface of a Cairo context.

    :param cairo_context:
        a Cairo context
    :param layout:
        a Pango layout
    """
    cairo_t_pointer = _get_cairo_t_from_cairo_ctx(cairo_context)
    pangocairo.pango_cairo_update_layout(cairo_t_pointer, layout.get_pointer())
