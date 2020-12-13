from . import pango


def units_to_double(i: int) -> float:
    """
    Converts a number in Pango units to floating-point: divides it by
    ``PANGO_SCALE``.

    :param i:
        value in Pango units
    """
    return pango.pango_units_to_double(i)


def units_from_double(d: float) -> int:
    """
    Converts a floating-point number to Pango units: multiplies it by
    ``PANGO_SCALE`` and rounds to nearest integer.

    :param d:
        double floating-point value
    """
    return pango.pango_units_from_double(d)
