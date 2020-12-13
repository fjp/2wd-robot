from . import pango, ffi
from . import GlyphItem


class GlyphItemIter:
    """
    A :class:`GlyphItemIter` is an iterator over the clusters in a
    :class:`GlyphItem`. The forward direction of the iterator is the logical
    direction of text. That is, with increasing ``start_index`` and
    ``start_char`` values. If ``glyph_item`` is right-to-left (that is, if
    ``glyph_item->item->analysis.level`` is odd), then ``start_glyph``
    decreases as the iterator moves forward. Moreover, in right-to-left cases,
    ``start_glyph`` is greater than ``end_glyph``.

    Note that ``text`` is the start of the text for layout, which is then
    indexed by ``glyph_item->item->offset`` to get to the text of
    ``glyph_item``. The ``start_index`` and ``end_index`` values can directly
    index into ``text``. The ``start_glyph``, ``end_glyph``, ``start_char``,
    and ``end_char`` values however are zero-based for the ``glyph_item``.
    For each cluster, the item pointed at by the start variables is included in
    the cluster while the one pointed at by end variables is not.

    None of the members of a :class:`GlyphItemIter` should be modified
    manually.
    """

    def __init__(self):
        """
        Returns an iterator to iterate over the visual extents of the layout.

        :param layout:
            a Pango :class:`Layout`
        """
        self._pointer = ffi.new("PangoGlyphItemIter *")

    def get_pointer(self) -> ffi.CData:
        """
        Returns the pointer to this iterator.

        :return:
            a pointer to the iterator.
        """
        return self._pointer

    def init_start(self, glyph_item: GlyphItem, text: str) -> bool:
        """
        Initializes the :class:`GlyphItemIter` structure to point to the first
        cluster in a glyph item. See :class:`GlyphItemIter` for details of
        cluster orders.

        :return:
            ``False`` if there are no clusters in the glyph item
        """
        text_pointer = ffi.new('char[]', text.encode('utf8'))
        return bool(pango.pango_glyph_item_iter_init_start(
            self._pointer,
            glyph_item.get_pointer(),
            text_pointer
        ))

    def init_end(self, glyph_item: GlyphItem, text: str) -> bool:
        """
        Initializes the :class:`GlyphItemIter` structure to point to the last
        cluster in a glyph item. See :class:`GlyphItemIter` for details of
        cluster orders.

        :return:
            ``False`` if there are no clusters in the glyph item
        """
        text_pointer = ffi.new('char[]', text.encode('utf8'))
        return bool(pango.pango_glyph_item_iter_init_end(
            self._pointer,
            glyph_item.get_pointer(),
            text_pointer
        ))

    def next_cluster(self) -> bool:
        """
        Moves the iterator forward to the next run in visual order. If the
        iterator was already at the end of the layout, it will
        return ``False``.

        :return:
            ``True`` if the iterator was advanced, ``False`` if we were
            already on the last cluster.
        """
        return bool(pango.pango_glyph_item_iter_next_cluster(self._pointer))

    def prev_cluster(self) -> bool:
        """
        Moves the iterator to the preceding cluster in the glyph item. See
        :class:`GlyphItemIter` for details of cluster orders.

        :return:
            ``True`` if the iterator was moved, ``False`` if we were already
            on the first cluster.
        """
        return bool(pango.pango_glyph_item_iter_prev_cluster(self._pointer))

    @property
    def glyph_item(self) -> GlyphItem:
        """
        :return:
            the glyph item being iterated over
        :type: GlyphItem
        """
        return GlyphItem.from_pointer(self._pointer.glyph_item)

    @property
    def text(self) -> str:
        """
        :return:
            the text for the layout
        :type: str
        """
        return ffi.string(self._pointer.text).decode('utf-8')

    @property
    def start_glyph(self) -> int:
        """
        :type: int
        """
        return self._pointer.start_glyph

    @property
    def start_index(self) -> int:
        """
        :return:
            the cluster start index within text
        :type: int
        """
        return self._pointer.start_index

    @property
    def start_char(self) -> int:
        """
        :type: int
        """
        return self._pointer.start_char

    @property
    def end_glyph(self) -> int:
        """
        :type: int
        """
        return self._pointer.end_glyph

    @property
    def end_index(self) -> int:
        """
        :return:
            the cluster end index within text
        :type: int
        """
        return self._pointer.end_index

    @property
    def end_char(self) -> int:
        """
        :type: int
        """
        return self._pointer.end_char
