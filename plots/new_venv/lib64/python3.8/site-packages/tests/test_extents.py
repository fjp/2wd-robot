import cairocffi
import pangocffi
import pangocairocffi
from typing import Tuple
import math


def _convert_rectangle_to_pixels(
        rectangle: pangocffi.Rectangle
) -> Tuple[float, float, float, float]:
    x = pangocffi.units_to_double(rectangle.x)
    y = pangocffi.units_to_double(rectangle.y)
    width = pangocffi.units_to_double(rectangle.width)
    height = pangocffi.units_to_double(rectangle.height)
    return x, y, width, height


def _coordinate_path(
        context: cairocffi.Context,
        point: Tuple[int, int],
        radius: float = 1
):
    for i in range(0, 8):
        p_x = pangocffi.units_to_double(point[0]) + \
            math.sin(i/4 * math.pi) * radius
        p_y = pangocffi.units_to_double(point[1]) + \
            math.cos(i/4 * math.pi) * radius
        if i == 0:
            context.move_to(p_x, p_y)
        else:
            context.line_to(p_x, p_y)


def _rectangle_path(
        context: cairocffi.Context,
        rectangle: pangocffi.Rectangle
):
    x, y, width, height = _convert_rectangle_to_pixels(rectangle)
    context.move_to(x, y)
    context.line_to(x + width, y)
    context.line_to(x + width, y + height)
    context.line_to(x, y + height)
    context.line_to(x, y)


def _show_character_extents(
        context: cairocffi.Context,
        layout: pangocffi.Layout
):
    layout_iter = layout.get_iter()
    context.set_line_width(0.5)
    context.set_dash([1, 1])
    alternate = True
    while True:
        alternate = not alternate
        extents = layout_iter.get_char_extents()

        context.set_source_rgba(0, 1 if alternate else 0.5, 0, 0.9)
        _rectangle_path(context, extents)
        context.stroke()
        _coordinate_path(context, (extents.x, extents.y))
        context.fill()

        if not layout_iter.next_char():
            break


def _show_cluster_ink_extents(
        context: cairocffi.Context,
        layout: pangocffi.Layout
):
    layout_iter = layout.get_iter()
    context.set_line_width(0.5)
    context.set_dash([1, 1])
    alternate = True
    while True:
        alternate = not alternate
        extents = layout_iter.get_cluster_extents()

        context.set_source_rgba(1 if alternate else 0.5, 0, 0, 0.9)
        _rectangle_path(context, extents[0])
        context.stroke()
        _coordinate_path(context, (extents[0].x, extents[0].y))
        context.fill()

        if not layout_iter.next_cluster():
            break


def _show_cluster_logical_extents(
        context: cairocffi.Context,
        layout: pangocffi.Layout
):
    layout_iter = layout.get_iter()
    context.set_line_width(0.5)
    context.set_dash([1, 1])
    alternate = True
    while True:
        alternate = not alternate
        extents = layout_iter.get_cluster_extents()

        context.set_source_rgba(1 if alternate else 0.5, 0, 0, 0.9)
        _rectangle_path(context, extents[1])
        context.stroke()
        _coordinate_path(context, (extents[1].x, extents[1].y))
        context.fill()

        if not layout_iter.next_cluster():
            break


def _show_run_ink_extents(
        context: cairocffi.Context,
        layout: pangocffi.Layout
):
    layout_iter = layout.get_iter()
    context.set_line_width(0.5)
    context.set_dash([1, 1])
    alternate = True
    while True:
        alternate = not alternate
        extents = layout_iter.get_run_extents()

        context.set_source_rgba(0, 0, 1 if alternate else 0.5, 0.9)
        _rectangle_path(context, extents[0])
        context.stroke()
        _coordinate_path(context, (extents[0].x, extents[0].y))
        context.fill()

        if not layout_iter.next_run():
            break


def _show_run_logical_extents(
        context: cairocffi.Context,
        layout: pangocffi.Layout
):
    layout_iter = layout.get_iter()
    context.set_line_width(0.5)
    context.set_dash([1, 1])
    alternate = True
    while True:
        alternate = not alternate
        extents = layout_iter.get_run_extents()

        context.set_source_rgba(0, 0, 1 if alternate else 0.5, 0.9)
        _rectangle_path(context, extents[1])
        context.stroke()
        _coordinate_path(context, (extents[1].x, extents[1].y))
        context.fill()

        if not layout_iter.next_run():
            break


def _show_layout_line_ink_extents(
        context: cairocffi.Context,
        layout: pangocffi.Layout
):
    layout_iter = layout.get_iter()
    context.set_line_width(0.5)
    context.set_dash([1, 1])
    alternate = True
    while True:
        alternate = not alternate
        extents = layout_iter.get_line_extents()
        context.set_source_rgba(0, 0, 1 if alternate else 0.5, 0.9)
        _rectangle_path(context, extents[0])
        context.stroke()
        _coordinate_path(context, (extents[0].x, extents[0].y))
        context.fill()

        if not layout_iter.next_run():
            break


def _show_layout_line_logical_extents(
        context: cairocffi.Context,
        layout: pangocffi.Layout
):
    layout_iter = layout.get_iter()
    context.set_line_width(0.5)
    context.set_dash([1, 1])
    alternate = True
    while True:
        alternate = not alternate
        extents = layout_iter.get_line_extents()
        context.set_source_rgba(0, 0, 1 if alternate else 0.5, 0.9)
        _rectangle_path(context, extents[1])
        context.stroke()
        _coordinate_path(context, (extents[1].x, extents[1].y))
        context.fill()

        if not layout_iter.next_run():
            break


def _show_layout_y_ranges(
        context: cairocffi.Context,
        layout: pangocffi.Layout
):
    layout_iter = layout.get_iter()
    context.set_line_width(0.5)
    context.set_dash([1, 1])
    alternate = True
    while True:
        alternate = not alternate
        extents = layout_iter.get_line_extents()
        y_ranges = layout_iter.get_line_yrange()

        context.set_source_rgba(0, 0, 1 if alternate else 0.5, 0.9)
        context.move_to(
            pangocffi.units_to_double(extents[0].x),
            pangocffi.units_to_double(y_ranges[0])
        )
        context.line_to(
            pangocffi.units_to_double(extents[0].x + extents[0].width),
            pangocffi.units_to_double(y_ranges[0])
        )
        context.stroke()

        context.move_to(
            pangocffi.units_to_double(extents[0].x),
            pangocffi.units_to_double(y_ranges[1])
        )
        context.line_to(
            pangocffi.units_to_double(extents[0].x + extents[0].width),
            pangocffi.units_to_double(y_ranges[1])
        )
        context.stroke()

        if not layout_iter.next_run():
            break


def _show_layout_baseline(
        context: cairocffi.Context,
        layout: pangocffi.Layout
):
    layout_iter = layout.get_iter()
    context.set_line_width(0.5)
    context.set_dash([1, 1])
    while True:
        extents = layout_iter.get_line_extents()
        baseline = layout_iter.get_baseline()
        y_ranges = layout_iter.get_line_yrange()

        context.set_source_rgba(1, 0, 0, 0.9)
        context.move_to(
            pangocffi.units_to_double(extents[0].x),
            pangocffi.units_to_double(y_ranges[0])
        )
        context.line_to(
            pangocffi.units_to_double(extents[0].x + extents[0].width),
            pangocffi.units_to_double(y_ranges[0])
        )
        context.stroke()

        context.set_source_rgba(0, 1, 0, 0.9)
        context.stroke()
        context.move_to(
            pangocffi.units_to_double(extents[0].x),
            pangocffi.units_to_double(baseline)
        )
        context.line_to(
            pangocffi.units_to_double(extents[0].x + extents[0].width),
            pangocffi.units_to_double(baseline)
        )
        context.stroke()

        context.set_source_rgba(0, 0, 1, 0.9)
        context.move_to(
            pangocffi.units_to_double(extents[0].x),
            pangocffi.units_to_double(y_ranges[1])
        )
        context.line_to(
            pangocffi.units_to_double(extents[0].x + extents[0].width),
            pangocffi.units_to_double(y_ranges[1])
        )
        context.stroke()

        if not layout_iter.next_run():
            break


def _show_layout_ink_extents(
        context: cairocffi.Context,
        layout: pangocffi.Layout
):
    layout_iter = layout.get_iter()
    context.set_line_width(0.5)
    context.set_dash([1, 1])
    alternate = True
    while True:
        alternate = not alternate
        extents = layout_iter.get_layout_extents()

        context.set_source_rgba(0, 0, 1 if alternate else 0.5, 0.9)
        _rectangle_path(context, extents[0])
        context.stroke()
        _coordinate_path(context, (extents[0].x, extents[0].y))
        context.fill()

        if not layout_iter.next_run():
            break


def _show_layout_logical_extents(
        context: cairocffi.Context,
        layout: pangocffi.Layout
):
    layout_iter = layout.get_iter()
    context.set_line_width(0.5)
    context.set_dash([1, 1])
    alternate = True
    while True:
        alternate = not alternate
        extents = layout_iter.get_layout_extents()

        context.set_source_rgba(0, 0, 1 if alternate else 0.5, 0.9)
        _rectangle_path(context, extents[1])
        context.stroke()
        _coordinate_path(context, (extents[1].x, extents[1].y))
        context.fill()

        if not layout_iter.next_run():
            break


def _show_layout(
        context: cairocffi.Context,
        layout: pangocffi.Layout
):
    context.set_source_rgba(0, 0, 0, 0.9)
    pangocairocffi.show_layout(context, layout)


def _show_label(
        context: cairocffi.Context,
        position: Tuple[float, float],
        width: float,
        text: str
):
    context.translate(position[0], position[1])
    label = pangocairocffi.create_layout(context)
    label.set_width(pangocffi.units_from_double(width))
    label.set_alignment(pangocffi.Alignment.LEFT)
    label.set_markup('<span font-family="sans-serif">%s</span>' % text)
    pangocairocffi.show_layout(context, label)
    context.translate(-position[0], -position[1])


def test_pdf():
    filename = 'tests/output/extents.pdf'
    pt_per_mm = 72 / 25.4
    width, height = 210 * pt_per_mm, 400 * pt_per_mm  # A4 portrait
    surface = cairocffi.PDFSurface(filename, width, height)
    ctx = cairocffi.Context(surface)
    ctx.translate(width / 2, 10)

    layout = pangocairocffi.create_layout(ctx)
    layout.set_width(pangocffi.units_from_double(width / 2))
    layout.set_alignment(pangocffi.Alignment.CENTER)
    layout.set_markup('<span font="italic 30">Hi from Παν語</span>\n'
                      'The text layout engine library for displaying '
                      '<span font-weight="bold">multi-language</span> text!')

    translate_y = 110
    label_pos = (-width / 2 + 30, 30)
    label_width = width / 2.5

    _show_layout_logical_extents(ctx, layout)
    _show_layout(ctx, layout)
    _show_label(ctx, label_pos, label_width, 'Layout (logical extents)')
    ctx.translate(0, translate_y)

    _show_layout_ink_extents(ctx, layout)
    _show_layout(ctx, layout)
    _show_label(ctx, label_pos, label_width, 'Layout (ink extents)')
    ctx.translate(0, translate_y)

    _show_layout_line_logical_extents(ctx, layout)
    _show_layout(ctx, layout)
    _show_label(ctx, label_pos, label_width, 'Layout Line (logical extents)')
    ctx.translate(0, translate_y)

    _show_layout_line_ink_extents(ctx, layout)
    _show_layout(ctx, layout)
    _show_label(ctx, label_pos, label_width, 'Layout Line (ink extents)')
    ctx.translate(0, translate_y)

    _show_layout_baseline(ctx, layout)
    _show_layout(ctx, layout)
    _show_label(
        ctx,
        label_pos,
        label_width,
        'y-range (top), Baseline,\ny-range (bottom)'
    )
    ctx.translate(0, translate_y)

    _show_run_logical_extents(ctx, layout)
    _show_layout(ctx, layout)
    _show_label(ctx, label_pos, label_width, 'Layout Run (logical extents)')
    ctx.translate(0, translate_y)

    _show_run_ink_extents(ctx, layout)
    _show_layout(ctx, layout)
    _show_label(ctx, label_pos, label_width, 'Layout Run (ink extents)')
    ctx.translate(0, translate_y)

    _show_cluster_logical_extents(ctx, layout)
    _show_layout(ctx, layout)
    _show_label(ctx, label_pos, label_width, 'Cluster (logical extents)')
    ctx.translate(0, translate_y)

    _show_cluster_ink_extents(ctx, layout)
    _show_layout(ctx, layout)
    _show_label(ctx, label_pos, label_width, 'Cluster (ink extents)')
    ctx.translate(0, translate_y)

    _show_character_extents(ctx, layout)
    _show_layout(ctx, layout)
    _show_label(ctx, label_pos, label_width, 'Character extents')
    ctx.translate(0, translate_y)

    surface.finish()
