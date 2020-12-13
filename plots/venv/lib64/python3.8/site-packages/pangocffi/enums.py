from pangocffi import pango
from enum import Enum


class Style(Enum):
    """
    An enumeration specifying the various slant styles possible for a font.
    """
    NORMAL = pango.PANGO_STYLE_NORMAL
    """the font is upright."""
    OBLIQUE = pango.PANGO_STYLE_OBLIQUE
    """the font is slanted, but in a roman style."""
    ITALIC = pango.PANGO_STYLE_ITALIC
    """the font is slanted in an italic style."""


class Weight(Enum):
    """
    An enumeration specifying the weight (boldness) of a font. This is a
    numerical value ranging from 100 to 1000, but there are some predefined
    values.
    """
    THIN = pango.PANGO_WEIGHT_THIN
    """the thin weight( = 100; Since: 1.24)"""
    ULTRALIGHT = pango.PANGO_WEIGHT_ULTRALIGHT
    """the ultralight weight( = 200)"""
    LIGHT = pango.PANGO_WEIGHT_LIGHT
    """the light weight( = 300)"""
    SEMILIGHT = pango.PANGO_WEIGHT_SEMILIGHT
    """the semilight weight( = 350; Since: 1.36.7)"""
    BOOK = pango.PANGO_WEIGHT_BOOK
    """the book weight( = 380; Since: 1.24)"""
    NORMAL = pango.PANGO_WEIGHT_NORMAL
    """the default weight( = 400)"""
    MEDIUM = pango.PANGO_WEIGHT_MEDIUM
    """the normal weight( = 500; Since: 1.24)"""
    SEMIBOLD = pango.PANGO_WEIGHT_SEMIBOLD
    """the semibold weight( = 600)"""
    BOLD = pango.PANGO_WEIGHT_BOLD
    """the bold weight( = 700)"""
    ULTRABOLD = pango.PANGO_WEIGHT_ULTRABOLD
    """the ultrabold weight( = 800)"""
    HEAVY = pango.PANGO_WEIGHT_HEAVY
    """the heavy weight( = 900)"""
    ULTRAHEAVY = pango.PANGO_WEIGHT_ULTRAHEAVY
    """the ultraheavy weight( = 1000; Since: 1.24)"""


class Variant(Enum):
    """
    An enumeration specifying capitalization variant of the font.
    """
    NORMAL = pango.PANGO_VARIANT_NORMAL
    """A normal font."""
    SMALL_CAPS = pango.PANGO_VARIANT_SMALL_CAPS
    """
    A font with the lower case characters replaced by smaller variants of
    the capital characters.
    """


class Stretch(Enum):
    """
    An enumeration specifying the width of the font relative to other designs
    within a family.
    """
    ULTRA_CONDENSED = pango.PANGO_STRETCH_ULTRA_CONDENSED
    """Ultra Condensed width"""
    EXTRA_CONDENSED = pango.PANGO_STRETCH_EXTRA_CONDENSED
    """Extra Condensed width"""
    CONDENSED = pango.PANGO_STRETCH_CONDENSED
    """Condensed width"""
    SEMI_CONDENSED = pango.PANGO_STRETCH_SEMI_CONDENSED
    """Semi condensed width"""
    NORMAL = pango.PANGO_STRETCH_NORMAL
    """The normal width"""
    SEMI_EXPANDED = pango.PANGO_STRETCH_SEMI_EXPANDED
    """Semi expanded width"""
    EXPANDED = pango.PANGO_STRETCH_EXPANDED
    """Expanded width"""
    EXTRA_EXPANDED = pango.PANGO_STRETCH_EXTRA_EXPANDED
    """Extra Expanded width"""
    ULTRA_EXPANDED = pango.PANGO_STRETCH_ULTRA_EXPANDED
    """Ultra Expanded width"""


class FontMask(Enum):
    """
    The bits in a :class:`FontMask` correspond to fields in a
    :class:`FontDescription` that have been set.
    """
    FAMILY = pango.PANGO_FONT_MASK_FAMILY
    """the font family is specified."""
    STYLE = pango.PANGO_FONT_MASK_STYLE
    """the font style is specified."""
    VARIANT = pango.PANGO_FONT_MASK_VARIANT
    """the font variant is specified."""
    WEIGHT = pango.PANGO_FONT_MASK_WEIGHT
    """the font weight is specified."""
    STRETCH = pango.PANGO_FONT_MASK_STRETCH
    """the font stretch is specified."""
    SIZE = pango.PANGO_FONT_MASK_SIZE
    """the font size is specified."""
    GRAVITY = pango.PANGO_FONT_MASK_GRAVITY
    """the font gravity is specified(Since: 1.16.)"""


class Alignment(Enum):
    """
    :class:`Alignment` describes how to align the lines of a :class:`Layout`
    within the available space. If the :class:`Layout` is set to justify using
    :meth:`set_justify()`, this only has effect for partial lines.
    """
    LEFT = pango.PANGO_ALIGN_LEFT
    """Put all available space on the right"""
    CENTER = pango.PANGO_ALIGN_CENTER
    """Center the line within the available space"""
    RIGHT = pango.PANGO_ALIGN_RIGHT
    """Put all available space on the left"""


class EllipsizeMode(Enum):
    """
    :class:`EllipsizeMode` describes what sort of (if any) ellipsization
    should be applied to a line of text. In the ellipsization process
    characters are removed from the text in order to make it fit to a given
    width and replaced with an ellipsis.
    """
    NONE = pango.PANGO_ELLIPSIZE_NONE
    """No ellipsization"""
    START = pango.PANGO_ELLIPSIZE_START
    """Omit characters at the start of the text"""
    MIDDLE = pango.PANGO_ELLIPSIZE_MIDDLE
    """Omit characters in the middle of the text"""
    END = pango.PANGO_ELLIPSIZE_END
    """Omit characters at the end of the text"""


class Gravity(Enum):
    """
    :class:`Gravity` represents the orientation of glyphs in a segment of text.
    This is useful when rendering vertical text layouts. In those situations,
    the layout is rotated using a non-identity :class:`Matrix`, and then glyph
    orientation is controlled using :class:`Gravity`. Not every value in this
    enumeration makes sense for every usage of :class:`Gravity`; for example,
    ``Gravity.AUTO`` only can be passed to ``Context.set_base_gravity()`` and
    can only be returned by ``Context.get_base_gravity()``.
    """
    SOUTH = pango.PANGO_GRAVITY_SOUTH
    """Glyphs stand upright (default)"""
    EAST = pango.PANGO_GRAVITY_EAST
    """Glyphs are rotated 90 degrees clockwise"""
    NORTH = pango.PANGO_GRAVITY_NORTH
    """Glyphs are upside-down"""
    WEST = pango.PANGO_GRAVITY_WEST
    """Glyphs are rotated 90 degrees counter-clockwise"""
    AUTO = pango.PANGO_GRAVITY_AUTO
    """Gravity is resolved from the context matrix"""


class GravityHint(Enum):
    """
    :class:`GravityHint` defines how horizontal scripts should behave in a
    vertical context. That is, English excerpt in a vertical paragraph for
    example.

    See :class:`Gravity`.
    """
    NATURAL = pango.PANGO_GRAVITY_HINT_NATURAL
    """
    scripts will take their natural gravity based on the base gravity and the
    script. This is the default.
    """
    STRONG = pango.PANGO_GRAVITY_HINT_STRONG
    """always use the base gravity set, regardless of the script."""
    LINE = pango.PANGO_GRAVITY_HINT_LINE
    """
    for scripts not in their natural direction (eg. Latin in East gravity),
    choose per-script gravity such that every script respects the line
    progression. This means, Latin and Arabic will take opposite gravities and
    both flow top-to-bottom for example.
    """
