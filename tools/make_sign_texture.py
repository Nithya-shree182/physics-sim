#!/usr/bin/env python3
"""Render a line of sign text to a transparent PNG for the cell's wall sign.

USD has no text primitive, and spelling words out of little boxes looks like what it is. The
NTT DATA mark on the fence is already a textured quad reading its PNG straight from the
dashboard's asset, so the subtitle under it is made the same way: white letters on transparency,
which the scene then lights as an illuminated sign.

    tools/make_sign_texture.py "India Innovation Centre" --out assets/brand/sign_india.png

Regenerate rather than edit: the point is that the asset is reproducible from the repository.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
FONT_CANDIDATES = (
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    "/usr/share/fonts/truetype/lato/Lato-Bold.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
)


def pick_font(size: int):
    from PIL import ImageFont
    for path in FONT_CANDIDATES:
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    print("[sign] no TrueType font found; falling back to PIL's bitmap default", file=sys.stderr)
    return ImageFont.load_default()


def render(text: str, out: Path, width=1400, height=210, tracking=10, pad=28):
    """White text, transparent elsewhere, centred, with a little letter spacing.

    The background is transparent BLACK, not transparent white. The sign shader drives emission
    from the texture's RGB while opacity comes from its alpha, and emission is not masked by
    opacity — so a transparent-white background makes the whole quad glow as a solid white slab
    (seen 2026-11-08). Black there emits nothing, which is the convention the NTT DATA logo PNG
    already follows."""
    from PIL import Image, ImageDraw
    img = Image.new("RGBA", (width, height), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)

    size = height - 2 * pad
    while size > 8:
        font = pick_font(size)
        widths = [d.textlength(ch, font=font) for ch in text]
        total = sum(widths) + tracking * max(len(text) - 1, 0)
        if total <= width - 2 * pad:
            break
        size -= 2
    else:
        font = pick_font(10)
        widths = [d.textlength(ch, font=font) for ch in text]
        total = sum(widths) + tracking * max(len(text) - 1, 0)

    box = d.textbbox((0, 0), text, font=font)
    x = (width - total) / 2.0
    y = (height - (box[3] - box[1])) / 2.0 - box[1]
    for ch, w in zip(text, widths):
        d.text((x, y), ch, font=font, fill=(255, 255, 255, 255))
        x += w + tracking

    out.parent.mkdir(parents=True, exist_ok=True)
    img.save(out)
    return img.size


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("text", nargs="?", default="India Innovation Centre")
    p.add_argument("--out", type=Path, default=REPO / "assets" / "brand" / "sign_india.png")
    p.add_argument("--width", type=int, default=1400)
    p.add_argument("--height", type=int, default=210)
    a = p.parse_args(argv)
    size = render(a.text, a.out, a.width, a.height)
    print(f"wrote {a.out}  {size[0]}x{size[1]}  text={a.text!r}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
