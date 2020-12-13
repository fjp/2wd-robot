import cairocffi
import pangocffi
import pangocairocffi
from typing import Tuple


def _convert_rectangle_to_pixels(
        rectangle: pangocffi.Rectangle
) -> Tuple[float, float, float, float]:
    x = pangocffi.units_to_double(rectangle.x)
    y = pangocffi.units_to_double(rectangle.y)
    width = pangocffi.units_to_double(rectangle.width)
    height = pangocffi.units_to_double(rectangle.height)
    return x, y, width, height


def test_pdf():
    filename = 'tests/output/error_underline.pdf'
    pt_per_mm = 72 / 25.4
    width, height = 45 * pt_per_mm, 10 * pt_per_mm  # A4 portrait
    surface = cairocffi.PDFSurface(filename, width, height)
    ctx = cairocffi.Context(surface)

    layout = pangocairocffi.create_layout(ctx)
    layout.set_width(pangocffi.units_from_double(width))
    layout.set_alignment(pangocffi.Alignment.CENTER)
    layout.set_markup('Hi from Παν語')

    layout_logical_extent = layout.get_extents()[1]
    layout_baseline = pangocffi.units_to_double(layout.get_baseline())
    layout_logical_extent_pixels = _convert_rectangle_to_pixels(
        layout_logical_extent
    )

    pangocairocffi.show_layout(ctx, layout)

    ctx.set_source_rgb(0.8, 0, 0)
    pangocairocffi.show_error_underline(
        ctx,
        layout_logical_extent_pixels[0],
        layout_baseline,
        layout_logical_extent_pixels[2],
        2
    )

    surface.finish()
