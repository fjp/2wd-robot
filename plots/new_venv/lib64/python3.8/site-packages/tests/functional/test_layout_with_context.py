from pangocffi import Layout
from .context_creator import ContextCreator
import unittest


class TestLayoutWithContext(unittest.TestCase):
    def setUp(self):
        self.pango_context = ContextCreator.get_pango_context()

    def tearDown(self):
        self.pango_context = None
        ContextCreator.free()

    def test_layout_setting_text(self):
        layout = Layout(self.pango_context)

        layout.set_text('Hi from Παν語')
        assert layout.get_text() == 'Hi from Παν語'

        layout.set_markup('<span font="italic 30">Hello from Παν語</span>')
        assert layout.get_text() == 'Hello from Παν語'
