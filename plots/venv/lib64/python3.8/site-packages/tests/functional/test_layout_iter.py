from pangocffi import Context, Layout, LayoutIter, Rectangle, ffi
import unittest


class TestLayoutIter(unittest.TestCase):

    @staticmethod
    def test_layout_iter_pointer():
        context = Context()
        layout = Layout(context)
        layout_iter = layout.get_iter()
        assert isinstance(layout_iter.get_pointer(), ffi.CData)

    def test_layout_iter_returns_null_from_null_pointer(self):
        with self.assertRaises(ValueError):
            LayoutIter.from_pointer(ffi.NULL)

    @staticmethod
    def test_layout_iter_properties():
        context = Context()
        layout = Layout(context)
        layout_iter = layout.get_iter()

        assert layout_iter.next_run() is False
        assert layout_iter.next_char() is False
        assert layout_iter.next_cluster() is False
        assert layout_iter.next_line() is False
        assert layout_iter.at_last_line() is True
        assert layout_iter.get_index() == 0
        assert layout_iter.get_baseline() == 0

        assert isinstance(layout_iter.get_char_extents(), Rectangle)

        cluster_ink, cluster_logical = layout_iter.get_cluster_extents()
        assert isinstance(cluster_ink, Rectangle)
        assert isinstance(cluster_logical, Rectangle)

        run_ink, run_logical = layout_iter.get_run_extents()
        assert isinstance(run_ink, Rectangle)
        assert isinstance(run_logical, Rectangle)

        y0, y1 = layout_iter.get_line_yrange()
        assert y0 == 0
        assert y1 == 0

        line_ink, line_logical = layout_iter.get_line_extents()
        assert isinstance(line_ink, Rectangle)
        assert isinstance(line_logical, Rectangle)

        layout_ink, layout_logical = layout_iter.get_layout_extents()
        assert isinstance(layout_ink, Rectangle)
        assert isinstance(layout_logical, Rectangle)

    @staticmethod
    def test_layout_iterator_run_returns_none():
        context = Context()
        layout = Layout(context)
        layout_iter = layout.get_iter()

        assert layout_iter.get_run() is None
