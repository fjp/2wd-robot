from . import ffi


class Item:
    """
    An :class:`Item` structure stores information about a segment of text.
    """

    def _init_pointer(self, pointer: ffi.CData):
        self._pointer = pointer

    @classmethod
    def from_pointer(cls, pointer: ffi.CData) -> 'Item':
        """
        Instantiates a :class:`Item` from a pointer.

        :return:
            the item.
        """
        if pointer == ffi.NULL:
            raise ValueError('Null pointer')
        self = object.__new__(cls)
        cls._init_pointer(self, pointer)
        return self

    def get_pointer(self) -> ffi.CData:
        """
        Returns the pointer to this item.

        :return:
            a pointer to the item.
        """
        return self._pointer

    @property
    def offset(self) -> int:
        """
        :return:
            byte offset of the start of this item in text.
        :type: int
        """
        return self._pointer.offset

    @property
    def length(self) -> int:
        """
        :return:
            length of this item in bytes.
        :type: int
        """
        return self._pointer.length

    @property
    def num_chars(self) -> int:
        """
        :return:
            number of Unicode characters in the item.
        :type: int
        """
        return self._pointer.num_chars
