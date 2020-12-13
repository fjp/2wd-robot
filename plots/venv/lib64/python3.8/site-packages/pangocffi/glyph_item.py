from typing import List
from . import pango, ffi, Item


class GlyphItem:
    """
    A :class:`GlyphItem` is a pair of a :class:`Item` and the glyphs resulting
    from shaping the text corresponding to an item. As an example of the usage
    of :class:`GlyphItem`, the results of shaping text with :class:`Layout` is
    a list of :class:`LayoutLine`, each of which contains a list of
    :class:`GlyphItem`.
    """

    def _init_pointer(self, pointer: ffi.CData):
        self._pointer = pointer

    @classmethod
    def from_pointer(cls, pointer: ffi.CData) -> 'GlyphItem':
        """
        Instantiates a :class:`GlyphItem` from a pointer.

        :return:
            the glyph item.
        """
        if pointer == ffi.NULL:
            raise ValueError('Null pointer')
        self = object.__new__(cls)
        cls._init_pointer(self, pointer)
        return self

    def get_pointer(self) -> ffi.CData:
        """
        Returns the pointer to this glyph item.

        :return:
            a pointer to the glyph item.
        """
        return self._pointer

    def copy(self) -> 'GlyphItem':
        """
        Make a deep copy of the :class:`GlyphItem` structure.

        :return:
            a copy of the :class:`GlyphItem`
        """
        glyph_item_pointer = pango.pango_glyph_item_copy(self._pointer)
        glyph_item_pointer = ffi.gc(
            glyph_item_pointer,
            pango.pango_glyph_item_free
        )
        return GlyphItem.from_pointer(glyph_item_pointer)

    def split(self, text: str, split_index: int) -> 'GlyphItem':
        """
        Modifies ``orig`` to cover only the text after ``split_index``, and
        returns a new item that covers the text before ``split_index`` that
        used to be in ``orig``. You can think of ``split_index`` as the length
        of the returned item. ``split_index`` may not be 0, and it may not be
        greater than or equal to the length of orig (that is, there must be at
        least one byte assigned to each item, you can't create a zero-length
        item).

        This function is similar in function to :meth:`Item.split()` (and uses
        it internally.)

        :param text:
            text to which positions in ``orig`` apply
        :param split_index:
            byte index of position to split item, relative to the start of the
            item
        :return:
             a new :class:`GlyphItem` representing text before ``split_index``.
        """
        text_pointer = ffi.new('char[]', text.encode('utf8'))
        glyph_item_pointer = pango.pango_glyph_item_split(
            self._pointer,
            text_pointer,
            split_index
        )
        glyph_item_pointer = ffi.gc(
            glyph_item_pointer,
            pango.pango_glyph_item_free
        )
        return GlyphItem.from_pointer(glyph_item_pointer)

    # Todo: apply_attrs(self, text: str, list: AttrList)

    # Todo:
    # def letter_space(
    #    self,
    #    text: str,
    #    log_attrs: LogAttr,
    #    letter_spacing: int
    # )

    def get_logical_widths(self, text: str) -> List[int]:
        """
        Given a :class:`GlyphItem` and the corresponding text, determine the
        screen width corresponding to each character. When multiple characters
        compose a single cluster, the width of the entire cluster is divided
        equally among the characters.

        See also :meth:`GlyphString.get_logical_widths()`.

        :param text:
            text that ``glyph_item`` corresponds to (
            ``glyph_item->item->offset`` is an offset from the start of
            ``text``)
        :return:
            an array whose length is the number of characters in `glyph_item`
            (equal to ``glyph_item->item->num_chars``) to be filled in with
            the resulting character widths.)
        """
        logical_widths_pointer = ffi.new('int [%d]' % self.item.num_chars)
        text_pointer = ffi.new('char[]', text.encode('utf8'))
        pango.pango_glyph_item_get_logical_widths(
            self._pointer,
            text_pointer,
            logical_widths_pointer
        )

        # type = ffi.typeof(logical_widths_pointer)
        return [logical_widths_pointer[i] for i in range(self.item.num_chars)]

    @property
    def item(self) -> Item:
        """
        :return:
            corresponding :class:`Item`.
        :type: Item
        """
        return Item.from_pointer(self._pointer.item)
