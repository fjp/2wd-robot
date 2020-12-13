from pangocffi import Layout, LayoutRun, Item, ffi
from .context_creator import ContextCreator
import unittest


class TestLayoutRunWithContext(unittest.TestCase):

    def setUp(self):
        self.pango_context = ContextCreator.get_pango_context()

    def tearDown(self):
        self.pango_context = None
        ContextCreator.free()

    def test_layout_run_from_pointer(self):
        layout = Layout(self.pango_context)
        layout.set_text('Hello World')
        layout_iter = layout.get_iter()

        layout_run = layout_iter.get_run()
        same_layout = LayoutRun.from_pointer(layout_run.get_pointer())

        assert isinstance(same_layout, LayoutRun)

    def test_layout_run_from_null_pointer(self):
        with self.assertRaises(ValueError):
            LayoutRun.from_pointer(ffi.NULL)

    def test_layout_run_properties(self):
        layout = Layout(self.pango_context)
        layout.set_text('Hello World')
        layout_iter = layout.get_iter()

        layout_run = layout_iter.get_run()
        layout_run_item = layout_run.item
        assert isinstance(layout_run_item, Item)
