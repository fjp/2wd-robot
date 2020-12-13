from . import pangocairo, ffi
from pangocffi import Context
from pangocffi import pango


# Todo: Implement FontMap in pango
# def get_default_font_map() -> pangocffi.FontMap:
#     return FontMap.from_pointer(
#         pangocairo.pango_cairo_font_map_get_default()
#     )
# def set_default_font_map(font_map: pangocffi.FontMap) -> None:
#     return pangocairo.pango_cairo_font_map_set_default(
#         font_map.get_pointer()
#     )


# Todo: extend FontMap from pangocffi
class PangoCairoFontMap:
    """
    PangoCairoFontMap is an interface exported by font maps for use with Cairo.
    The actual type of the font map will depend on the particular font
    technology Cairo was compiled to use.
    """

    def __init__(self):
        """
        Creates a new PangoCairoFontMap object; a ``fontmap`` is used to cache
        information about available fonts, and holds certain global parameters
        such as the resolution. In most cases, you can use
        ``PangoCairoFontMap.get_default()`` instead.

        Note that the type of the returned object will depend on the particular
        font backend Cairo was compiled to use; You generally should only use
        the ``PangoFontMap`` and ``PangoCairoFontMap`` interfaces on the
        returned object.

        You can override the type of backend returned by using an environment
        variable ``PANGOCAIRO_BACKEND``. Supported types, based on your build,
        are fc (fontconfig), win32, and coretext. If requested type is not
        available, NULL is returned. I.E. this is only useful for testing,
        when at least two backends are compiled in.
        """
        self._init_pointer(pangocairo.pango_cairo_font_map_new())

    def _init_pointer(self, pointer: ffi.CData):
        self._pointer = pointer

    def get_pointer(self) -> ffi.CData:
        """
        Returns the pointer to the font map

        :return:
            the pointer to the font map.
        """
        return self._pointer

    @classmethod
    def from_pointer(cls, pointer: ffi.CData) -> 'PangoCairoFontMap':
        """
        Instantiates a :class:`PangoCairoFontMap` from a pointer.

        :return:
            the pango-cairo font map.
        """
        if pointer == ffi.NULL:
            raise ValueError('Null pointer')
        self = object.__new__(cls)
        cls._init_pointer(self, pointer)
        return self

    @classmethod
    def from_cairo_font_type(
            cls,
            cairo_font_type_pointer: ffi.CData
    ) -> 'PangoCairoFontMap':
        """
        Instantiates a :class:`PangoCairoFontMap` from a Cairo context.

        :return:
            the pango-cairo font map.
        """
        font_map_pointer = pangocairo.pango_cairo_font_map_new_for_font_type(
            cairo_font_type_pointer
        )
        return cls.from_pointer(font_map_pointer)

    def __eq__(self, other):
        if isinstance(other, PangoCairoFontMap):
            return self.get_pointer() == other.get_pointer()
        return NotImplemented

    def get_cairo_font_type_pointer(self) -> ffi.CData:
        """
        Returns the pointer to the type of Cairo font backend that ``fontmap``
        uses

        :return:
            the pointer to the ``cairo_font_type_t``.
        """
        return pangocairo.pango_cairo_font_map_get_font_type(
            self.get_pointer()
        )

    def set_resolution(self, dpi: float) -> None:
        """
        Sets the resolution for the ``fontmap``. This is a scale factor between
        points specified in a Pango FontDescription and Cairo units. The
        default value is 96, meaning that a 10 point font will be 13
        units high. (10 * 96. / 72. = 13.3).

        :param dpi:
             the resolution in "dots per inch". (Physical inches aren't
             actually involved; the terminology is conventional.)
        """
        pangocairo.pango_cairo_font_map_set_resolution(self.get_pointer(), dpi)

    def get_resolution(self) -> float:
        """
        Returns the resolution for the ``fontmap``.

        :return:
            the resolution in "dots per inch"
        """
        return pangocairo.pango_cairo_font_map_get_resolution(
            self.get_pointer()
        )

    def create_context(self) -> Context:
        """
        Creates a Pango ``Context`` connected to ``fontmap``. This is
        equivalent to ``pango.Context()`` followed by
        ``context.set_font_map()``.

        :return:
            the newly allocated Pango ``Context``.
        """
        context_pointer = pango.pango_font_map_create_context(
            self.get_pointer()
        )
        return Context.from_pointer(context_pointer)
