import cairocffi
import pangocffi
import pangocairocffi


def test_pdf():
    filename = 'tests/output/test.pdf'
    pt_per_mm = 72 / 25.4
    width, height = 210 * pt_per_mm, 297 * pt_per_mm  # A4 portrait
    surface = cairocffi.PDFSurface(filename, width, height)
    context = cairocffi.Context(surface)
    context.translate(0, height / 2)
    context.rotate(-0.1)

    layout = pangocairocffi.create_layout(context)
    layout.set_width(pangocffi.units_from_double(width))
    layout.set_alignment(pangocffi.Alignment.CENTER)
    layout.set_markup('<span font="italic 30">Hi from Παν語</span>')

    pangocairocffi.show_layout(context, layout)

    surface.finish()
