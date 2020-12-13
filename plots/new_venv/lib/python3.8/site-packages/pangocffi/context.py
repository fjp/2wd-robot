from . import pango, gobject, ffi, FontDescription, Gravity, GravityHint


class Context(object):
    """
    The :class:`Context` structure stores global information used to control
    the itemization process.
    """

    def __init__(self):
        """
        Creates a new PangoContext initialized to default values.

        This function is not particularly useful as it should always be
        followed by a :meth:`set_font_map()` call, and the function
        ``FontMap.create_context()`` does these two steps together and hence
        users are recommended to use that.

        If you are using Pango as part of a higher-level system, that system
        may have it's own way of create a :class:`Context`. For instance, the
        PangoCairo toolkit has pango_cairo_create_context(). Use those instead.
        """
        self._init_pointer(pango.pango_context_new())

    def _init_pointer(self, pointer: ffi.CData):
        self._pointer = ffi.gc(pointer, gobject.g_object_unref)

    def get_pointer(self) -> ffi.CData:
        """
        Returns the pointer to the context

        :return:
            the pointer to the context.
        """
        return self._pointer

    @classmethod
    def from_pointer(cls, pointer: ffi.CData) -> 'Context':
        """
        Instantiates a :class:`Context` from a pointer.

        :return:
            the context.
        """
        if pointer == ffi.NULL:
            raise ValueError('Null pointer')
        self = object.__new__(cls)
        cls._init_pointer(self, pointer)
        return self

    def __eq__(self, other):
        if isinstance(other, Context):
            return self.get_pointer() == other.get_pointer()
        return NotImplemented

    def set_font_description(self, desc: FontDescription) -> None:
        """
        Set the default font description for the context.

        :param desc:
            the new pango font description.
        """
        pango.pango_context_set_font_description(
            self._pointer, desc.get_pointer()
        )

    def get_font_description(self) -> FontDescription:
        """
        Retrieve the default font description for the context.

        :return:
            the context's default font description.
        """
        return FontDescription.from_pointer(
            pango.pango_context_get_font_description(self._pointer)
        )

    def get_base_gravity(self) -> Gravity:
        """
        Retrieves the base gravity for the context.
        See :meth:`set_base_gravity()`.

        :return:
            the base gravity for the context.
        """
        return Gravity(
            pango.pango_context_get_base_gravity(self._pointer)
        )

    def set_base_gravity(self, gravity: Gravity):
        """
        Sets the base gravity for the context.

        The base gravity is used in laying vertical text out.

        :param gravity:
            the new base gravity
        """
        pango.pango_context_set_base_gravity(self._pointer, gravity.value)

    def get_gravity(self) -> Gravity:
        """
        Retrieves the gravity for the context. This is similar to
        :meth:`get_base_gravity()`, except for when the base gravity is
        :meth:`Gravity.AUTO` for which :meth:`get_for_matrix()` is used to
        return the gravity from the current context matrix.

        :return:
            the resolved gravity for the context.
        """
        return Gravity(
            pango.pango_context_get_gravity(self._pointer)
        )

    def get_gravity_hint(self) -> GravityHint:
        """
        Retrieves the gravity hint for the context.
        See :meth:`set_gravity_hint()` for details.

        :return:
            the gravity hint for the context.
        """
        return GravityHint(
            pango.pango_context_get_base_gravity(self._pointer)
        )

    def set_gravity_hint(self, hint: GravityHint):
        """
        Sets the gravity hint for the context.

        The gravity hint is used in laying vertical text out, and is only
        relevant if gravity of the context as returned by
        :meth:`get_gravity()` is set :meth:`Gravity.EAST` or
        :meth:`Gravity.WEST`.

        :param hint:
            the new gravity hint
        """
        pango.pango_context_set_gravity_hint(self._pointer, hint.value)
