from pangocffi import Item, ffi
import unittest


class TestItem(unittest.TestCase):

    def test_item_returns_null_from_null_pointer(self):
        with self.assertRaises(ValueError):
            Item.from_pointer(ffi.NULL)
