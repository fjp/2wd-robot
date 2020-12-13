from typing import List, Optional

import cairocffi
import pangocffi
import pangocairocffi


def render_run_glyph_items(
        ctx: cairocffi.Context,
        layout: pangocffi.Layout
) -> None:
    """
    Renders each layout run within a layout with a unique rotation and color.

    :param ctx:
        a Cairo context
    :param layout:
        a Pango layout
    """
    layout_iter = layout.get_iter()
    layout_text = layout.get_text()

    alternate = False
    while True:
        layout_run = layout_iter.get_run()
        layout_run_extents = layout_iter.get_run_extents()[1]
        layout_line_baseline = layout_iter.get_baseline()

        if layout_run is None:
            if not layout_iter.next_run():
                break
            continue

        alternate = not alternate
        ctx.set_source_rgba(0, 0.5, 1 if alternate else 0.5, 0.9)
        ctx.save()
        ctx.translate(
            pangocffi.units_to_double(layout_run_extents.x),
            pangocffi.units_to_double(layout_line_baseline)
        )
        ctx.rotate(0.05 if alternate else -0.05)
        pangocairocffi.show_glyph_item(ctx, layout_text, layout_run)
        ctx.restore()

        if not layout_iter.next_run():
            break


def split_glyph_item_at_first_cluster(
        glyph_item: pangocffi.GlyphItem,
        text: str
) -> Optional[pangocffi.GlyphItem]:
    """

    :param glyph_item:
        the glyph item to split
    :param text:
        the full layout's text
    :return:
        the first cluster of the :param:`glyph_item`. The input parameter will
        also be split from the end of the first cluster.
    """
    i = 1
    while True:
        if i >= glyph_item.item.length:
            break
        try:
            return glyph_item.split(text, i)
        except ValueError:
            i += 1
            pass
    return None


def get_clusters_from_glyph_item(
        glyph_item: pangocffi.GlyphItem,
        text: str
) -> List[pangocffi.GlyphItem]:
    """
    Splits a glyph item (which is composed of multiple clusters) into
    an array of individual glyph items for each cluster.

    Warning: Does not support bidirectional text.

    :param glyph_item:
        the glyph item, or layout run, to split into individual glyphs
    :param text:
        the full layout's text
    :return:
        an array og individual glyph items
    """
    cluster_glyph_items = []
    glyph_item_copy = glyph_item.copy()
    while True:
        first_cluster = split_glyph_item_at_first_cluster(
            glyph_item_copy,
            text
        )
        if first_cluster is None:
            break
        cluster_glyph_items.append(first_cluster)
    cluster_glyph_items.append(glyph_item_copy)
    return cluster_glyph_items


def render_cluster_glyph_items(
        ctx: cairocffi.Context,
        layout: pangocffi.Layout
):
    """
    Renders each cluster within a layout with a unique rotation and color.

    Warning: Does not support bidirectional text.

    :param ctx:
        a Cairo context
    :param layout:
        a Pango layout
    """
    layout_run_iter = layout.get_iter()
    layout_cluster_iter = layout.get_iter()
    layout_text = layout.get_text()

    alternate = False
    while True:

        layout_run = layout_run_iter.get_run()
        layout_line_baseline = layout_run_iter.get_baseline()

        if layout_run is None:
            if not layout_run_iter.next_run():
                break
            continue

        clusters = get_clusters_from_glyph_item(layout_run, layout_text)
        for cluster in clusters:
            cluster_extents = layout_cluster_iter.get_cluster_extents()[1]
            layout_cluster_iter.next_cluster()
            alternate = not alternate
            ctx.set_source_rgba(0.5, 0, 1 if alternate else 0.5, 0.9)
            ctx.save()
            ctx.translate(
                pangocffi.units_to_double(cluster_extents.x),
                pangocffi.units_to_double(layout_line_baseline)
            )
            ctx.rotate(-0.05 if alternate else 0.05)
            pangocairocffi.show_glyph_item(
                ctx,
                layout_text,
                cluster
            )
            ctx.restore()

        if not layout_run_iter.next_run():
            break


def test_pdf():
    filename = 'tests/output/glyph_item.pdf'
    pt_per_mm = 72 / 25.4
    width, height = 210 * pt_per_mm, 297 * pt_per_mm  # A4 portrait
    surface = cairocffi.PDFSurface(filename, width, height)
    ctx = cairocffi.Context(surface)
    ctx.translate(width / 4, 100)

    layout = pangocairocffi.create_layout(ctx)
    layout.set_width(pangocffi.units_from_double(width / 2))
    layout.set_alignment(pangocffi.Alignment.CENTER)
    layout.set_markup(
        '<span font="italic 30">Hi from Παν語</span>\n'
        '<span font="sans-serif">The text layout engine library for '
        'displaying <span font-weight="bold">multi-language</span> text!\n'
        'Arabic: السَّلام عليكُم\n'
        'Hebrew: שלום\n'
        'Hindi नमस्ते, नमस्कार।\n'
        'Russian: Здравствуйте!\n'
        'Japanese: こんにちは, ｺﾝﾆﾁﾊ</span>'
    )

    y_diff = 200
    pangocairocffi.show_layout(ctx, layout)
    ctx.translate(0, y_diff)
    render_run_glyph_items(ctx, layout)
    ctx.translate(0, y_diff)
    render_cluster_glyph_items(ctx, layout)

    surface.finish()
