import pangocffi


def test_units_to_double():
    assert pangocffi.units_to_double(0) == 0
    assert pangocffi.units_to_double(123) == 0.1201171875


def test_context_properties():
    assert pangocffi.units_from_double(0) == 0
    assert pangocffi.units_from_double(0.1201171875) == 123
