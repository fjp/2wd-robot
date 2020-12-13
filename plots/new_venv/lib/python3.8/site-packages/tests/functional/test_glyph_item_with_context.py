from pangocffi import Layout
from .context_creator import ContextCreator
import unittest


class TestGlyphItemIterWithContext(unittest.TestCase):

    def setUp(self):
        self.pango_context = ContextCreator.get_pango_context()

    def tearDown(self):
        self.pango_context = None
        ContextCreator.free()

    def test_glyph_item_copies(self):
        text = 'Hi from Παν語'
        layout = Layout(self.pango_context)
        layout.set_text(text)
        layout_iter = layout.get_iter()
        layout_run = layout_iter.get_run()

        assert layout_run.copy().get_pointer() != layout_run.get_pointer()

    def test_glyph_item_splits(self):
        text = 'Hi from Παν語'
        layout = Layout(self.pango_context)
        layout.set_text(text)
        layout_iter = layout.get_iter()
        layout_run = layout_iter.get_run()
        layout_run_hi = layout_run.split(text, 3)

        # Substring "Hi "
        assert layout_run_hi.item.num_chars == 3
        assert layout_run_hi.item.offset == 0

        # Substring "from "
        assert layout_run.item.num_chars == 5
        assert layout_run.item.offset == 3

    def test_glyph_item_get_logical_widths(self):
        text = 'Hi from Παν語'
        layout = Layout(self.pango_context)
        layout.set_text(text)
        layout_iter = layout.get_iter()
        layout_run = layout_iter.get_run()
        logical_widths = layout_run.get_logical_widths(text)

        assert len(logical_widths) == 8
        assert isinstance(logical_widths[0], int)
