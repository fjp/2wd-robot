from pangocffi import Rectangle


def test_rectangle_properties():
    rect = Rectangle()
    assert rect.x == 0
    assert rect.y == 0
    assert rect.width == 0
    assert rect.height == 0


def test_rectangle_pointers():
    rect = Rectangle()
    same_rect = rect.from_pointer(rect.get_pointer())
    assert same_rect.x == 0
