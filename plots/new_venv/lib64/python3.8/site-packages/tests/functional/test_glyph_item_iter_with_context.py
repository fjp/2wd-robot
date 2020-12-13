from pangocffi import Layout, GlyphItemIter, GlyphItem, ffi
from .context_creator import ContextCreator
import unittest


class TestGlyphItemIter(unittest.TestCase):

    def setUp(self):
        self.pango_context = ContextCreator.get_pango_context()

    def tearDown(self):
        self.pango_context = None
        ContextCreator.free()

    def test_glyph_item_iterator(self):
        text = 'Hi from Παν語'
        layout = Layout(self.pango_context)
        layout.set_text(text)
        layout_iter = layout.get_iter()

        glyph_item_iter = GlyphItemIter()
        assert isinstance(glyph_item_iter.get_pointer(), ffi.CData)

        assert glyph_item_iter.init_start(
            layout_iter.get_run(),
            text
        )

        assert isinstance(glyph_item_iter.glyph_item, GlyphItem)
        assert glyph_item_iter.text == text

        # Verify first cluster
        assert glyph_item_iter.start_glyph == 0
        assert glyph_item_iter.start_index == 0
        assert glyph_item_iter.start_char == 0
        assert glyph_item_iter.end_glyph == 1
        assert glyph_item_iter.end_index == 1
        assert glyph_item_iter.end_char == 1

        assert not glyph_item_iter.prev_cluster()
        assert glyph_item_iter.next_cluster()

        # Verify second cluster
        assert glyph_item_iter.start_glyph == 1
        assert glyph_item_iter.start_index == 1
        assert glyph_item_iter.start_char == 1
        assert glyph_item_iter.end_glyph == 2
        assert glyph_item_iter.end_index == 2
        assert glyph_item_iter.end_char == 2

        assert glyph_item_iter.init_end(
            layout_iter.get_run(),
            text
        )

        # Verify last cluster
        assert glyph_item_iter.start_glyph == 7
        assert glyph_item_iter.start_index == 7
        assert glyph_item_iter.start_char == 7
        assert glyph_item_iter.end_glyph == 8
        assert glyph_item_iter.end_index == 8
        assert glyph_item_iter.end_char == 8

        assert not glyph_item_iter.next_cluster()
        assert glyph_item_iter.prev_cluster()

        # Verify second to last cluster
        assert glyph_item_iter.start_glyph == 6
        assert glyph_item_iter.start_index == 6
        assert glyph_item_iter.start_char == 6
        assert glyph_item_iter.end_glyph == 7
        assert glyph_item_iter.end_index == 7
        assert glyph_item_iter.end_char == 7
