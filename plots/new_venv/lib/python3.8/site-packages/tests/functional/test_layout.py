from pangocffi import Context, FontDescription, Layout, Alignment, ffi,\
    EllipsizeMode
import unittest


class TestLayout(unittest.TestCase):
    @staticmethod
    def test_layout_returns_identical_context():
        context = Context()
        layout = Layout(context)
        identical_context = layout.get_context()
        assert identical_context.get_pointer() == context.get_pointer()

    @staticmethod
    def test_layout_get_pointer_returns_identical_layout():
        context = Context()
        layout = Layout(context)
        identical_layout = Layout.from_pointer(layout.get_pointer())
        assert identical_layout.get_pointer() == layout.get_pointer()

    def test_layout_not_implemented_equality(self):
        context = Context()
        layout = Layout(context)
        assert ('not an object' != layout)

    def test_layout_returns_null_from_null_pointer(self):
        with self.assertRaises(ValueError):
            Layout.from_pointer(ffi.NULL)

    @staticmethod
    def test_layout_setting_font_description():
        context = Context()
        layout = Layout(context)

        # Assert that the font description is not set
        assert layout.get_font_description() is None

        # Creating the font description
        desc = FontDescription()
        desc.set_family('sans-serif')
        layout.set_font_description(desc)

        # Verifying the font description was set
        same_desc = layout.get_font_description()
        assert same_desc.get_family() == desc.get_family()

        # Changing the font description
        desc.set_family('serif')
        assert same_desc.get_family() != desc.get_family()

        # Resetting the font description
        layout.set_font_description(None)
        assert layout.get_font_description() is None

    @staticmethod
    def test_layout_setting_properties():
        context = Context()
        layout = Layout(context)

        layout.set_width(300)
        assert layout.get_width() == 300

        layout.set_height(400)
        assert layout.get_height() == 400

        assert layout.get_spacing() == 0
        layout.set_spacing(30)
        assert layout.get_spacing() == 30

        layout.set_alignment(Alignment.CENTER)
        assert layout.get_alignment() is Alignment.CENTER

        layout.set_ellipsize(EllipsizeMode.MIDDLE)
        assert layout.get_ellipsize() is EllipsizeMode.MIDDLE

        ink_rect, logical_rect = layout.get_extents()
        assert logical_rect.width == 0
        assert logical_rect.height == 0

        width, height = layout.get_size()
        assert width == 0
        assert height == 0

        baseline = layout.get_baseline()
        assert baseline == 0

        line_count = layout.get_line_count()
        assert line_count == 1

    @staticmethod
    def test_layout_setting_text():
        context = Context()
        layout = Layout(context)

        layout.set_width(300)

        layout.set_text('Hi from Pango')
        layout.set_markup('<span font="italic 30">Hi from Παν語</span>')
