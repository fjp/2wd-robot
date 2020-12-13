from typing import Optional
from . import ffi


class Rectangle:
    """
    The :class:`Rectangle` structure represents a rectangle. It is
    frequently used to represent the logical or ink extents of a single glyph
    or section of text.
    """
    def __init__(
            self,
            pointer: Optional[ffi.CData] = None
    ):
        if pointer is None:
            self.pointer = ffi.new("PangoRectangle *")
        else:
            self.pointer = pointer

    def get_pointer(self) -> ffi.CData:
        """
        Returns a pointer to the :class:`Rectangle`.

        :return:
            a pointer to ``PangoRectangle``
        """
        return self.pointer

    @staticmethod
    def from_pointer(pointer: ffi.CData) -> 'Rectangle':
        """
        Returns an instance of a :class:`Rectangle` from a pointer.

        :return:
            the :class:`Rectangle`.
        """
        return Rectangle(
            pointer=pointer
        )

    @property
    def x(self) -> int:
        """
        :return:
            X coordinate of the left side of the rectangle.
        :type: int
        """
        return self.pointer.x

    @property
    def y(self) -> int:
        """
        :return:
            Y coordinate of the the top side of the rectangle.
        :type: int
        """
        return self.pointer.y

    @property
    def width(self) -> int:
        """
        :return:
            width of the rectangle.
        :type: int
        """
        return self.pointer.width

    @property
    def height(self) -> int:
        """
        :return:
            height of the rectangle.
        :type: int
        """
        return self.pointer.height
