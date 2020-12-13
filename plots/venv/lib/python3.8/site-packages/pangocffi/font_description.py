from typing import Optional, Union
from . import pango, ffi
from . import Style, Variant, Weight, Stretch, Gravity


class FontDescription(object):
    """
    The :class:`FontDescription` represents the description of an ideal font.
    These structures are used both to list what fonts are available on the
    system and also for specifying the characteristics of a font to load.
    """

    def __init__(self):
        """
        Creates a new font description structure with all fields unset.

        :param:
            a :class:`FontDescription`
        """
        self._init_pointer(pango.pango_font_description_new())

    def _init_pointer(self, pointer: ffi.CData):
        self._pointer = pointer
        # self._pointer = ffi.gc(pointer, pango.pango_font_description_free)

    def get_pointer(self) -> ffi.CData:
        """
        Returns the pointer to the font description

        :return:
            the pointer to the font description.
        """
        return self._pointer

    @classmethod
    def from_pointer(cls, pointer: ffi.CData) -> 'FontDescription':
        """
        Instantiates a :class:`FontDescription` from a pointer.

        :return:
            the font description.
        """
        if pointer == ffi.NULL:
            raise ValueError('Null pointer')
        self = object.__new__(cls)
        cls._init_pointer(self, pointer)
        return self

    def __eq__(self, other):
        if isinstance(other, FontDescription):
            return self.get_pointer() == other.get_pointer()
        return NotImplemented

    def set_family(self, family: str) -> None:
        """
        Sets the family name field of a font description. The family name
        represents a family of related font styles, and will resolve to a
        particular :class:`FontFamily`. In some uses of
        :class:`FontDescription`, it is also possible to use a comma separated
        list of family names for this field.

        :param family:
            a string representing the family name.
        """
        family_pointer = ffi.new('char[]', family.encode('utf8'))
        pango.pango_font_description_set_family(self._pointer, family_pointer)

    def get_family(self) -> Optional[str]:
        """
        Returns the family name field of a font description.

        :return:
            the family name field for the font description, or ``None`` if not
            previously set.
        """
        family_pointer = pango.pango_font_description_get_family(self._pointer)
        if family_pointer == ffi.NULL:
            return None
        return ffi.string(family_pointer).decode()

    def set_style(self, style: Style) -> None:
        """
        Sets the style field of a :class:`FontDescription`. The :class:`Style`
        enumeration describes whether the font is slanted and the manner in
        which it is slanted.

        Most fonts will either have a italic style or an oblique style, but not
        both, and font matching in Pango will match italic specifications with
        oblique fonts and vice-versa if an exact match is not found.

        :param style:
            the style for the font description
        """
        pango.pango_font_description_set_style(self._pointer, style.value)

    def get_style(self) -> Style:
        """
        Returns the style field of a :class:`FontDescription`.

        :return:
             the style field for the font description.
        """
        return Style(pango.pango_font_description_get_style(self._pointer))

    def set_variant(self, variant: Variant) -> None:
        """
        Sets the variant field of a font description. The :class:`Variant`
        enumeration describes whether the font is normal or small caps.

        :param variant:
            the variant type for the font description.
        """
        pango.pango_font_description_set_variant(self._pointer, variant.value)

    def get_variant(self) -> Variant:
        """
        Returns the variant field of a :class:`FontDescription`.

        :return:
            the variant field for the font description.
        """
        return Variant(pango.pango_font_description_get_variant(self._pointer))

    def set_weight(self, weight: Union[Weight, int]) -> None:
        """
        Sets the weight field of a font description. The weight field specifies
        how bold or light the font should be. In addition to the values of the
        :class:`Weight` enumeration, other intermediate numeric values are
        possible.

        :param weight:

        """
        if isinstance(weight, Weight):
            pango.pango_font_description_set_weight(
                self._pointer,
                weight.value
            )
        else:
            pango.pango_font_description_set_weight(self._pointer, weight)

    def get_weight(self) -> int:
        """
        Returns the weight field of a font description.

        :return:
            the weight field for the font description.
        """
        return pango.pango_font_description_get_weight(self._pointer)

    def set_stretch(self, stretch: Stretch) -> None:
        """
        Sets the stretch field of a font description. The stretch field
        specifies how narrow or wide the font should be.

        :param stretch:
             the stretch for the font description
        """
        pango.pango_font_description_set_stretch(self._pointer, stretch.value)

    def get_stretch(self) -> Stretch:
        """
        Returns the stretch field of a font description.

        :return:
            the stretch field for the font description.
        """
        return Stretch(pango.pango_font_description_get_stretch(self._pointer))

    def set_size(self, size: int) -> None:
        """
        Sets the size field of a font description in fractional points. This is
        mutually exclusive with :meth:`set_absolute_size()`.

        :param size:
             the size of the font in points, scaled by ``PANGO_SCALE``. (That
             is, a size value of 10 * ``PANGO_SCALE`` is a 10 point font. The
             conversion factor between points and device units depends on
             system configuration and the output device. For screen display, a
             logical DPI of 96 is common, in which case a 10 point font
             corresponds to a 10 * (96 / 72) = 13.3 pixel font. Use
             :meth:`set_absolute_size()` if you need a particular size in
             device units.
        """
        pango.pango_font_description_set_size(self._pointer, size)

    def get_size(self) -> int:
        """
        Returns the size field of a font description.

        :return:
            the size field for the font description in points or device units.
            You must call :meth:`get_size_is_absolute()` to find out which is
            the case. Returns 0 if the size field has not previously been set
            or it has been set to 0 explicitly.
        """
        return pango.pango_font_description_get_size(self._pointer)

    def set_absolute_size(self, size: float) -> None:
        """
        Sets the size field of a font description, in device units. This is
        mutually exclusive with :meth:`set_size()` which sets the font size in
        points.

        :param size:
             the new size, in Pango units. There are ``PANGO_SCALE`` Pango
             units in one device unit. For an output backend where a device
             unit is a pixel, a size value of 10 * ``PANGO_SCALE`` gives a 10
             pixel font.
        """
        pango.pango_font_description_set_absolute_size(self._pointer, size)

    def get_size_is_absolute(self) -> bool:
        """
        Determines whether the size of the font is in points (not absolute) or
        device units (absolute). See :meth:`set_size()` and
        :meth:`set_absolute_size()`.

        :return:
            whether the size for the font description is in points or device
            units.
        """
        return pango.pango_font_description_get_size_is_absolute(self._pointer)

    def set_gravity(self, gravity: Gravity) -> None:
        """
        Sets the gravity field of a font description. The gravity field
        specifies how the glyphs should be rotated. If gravity is
        ``Gravity.AUTO``, this actually unsets the gravity mask on the font
        description.

        :param gravity:
             the gravity for the font description.
        """
        pango.pango_font_description_set_gravity(self._pointer, gravity.value)

    def get_gravity(self) -> Gravity:
        """
        Returns the gravity field of a font description.

        :return:
            the gravity field for the font description.
        """
        return Gravity(pango.pango_font_description_get_gravity(self._pointer))
