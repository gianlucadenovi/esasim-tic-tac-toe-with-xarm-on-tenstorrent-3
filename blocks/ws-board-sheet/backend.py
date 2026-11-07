# ── EsaSim block ──────────────────────────────────────────────────────
# Board sheet (ws-board-sheet) · version 2 · machine block, Python: def run(ctx)
# The printable tic-tac-toe board: four ArUco markers around a square, so a camera finds and straightens it by itself.
# Workspace: Gianluca De Novi's workspace · Author: Gianluca De Novi
# Saved: 2026-11-07
#
# Inputs   ctx.inputs["id"] is a list of files: .path .name .size .read_text() .read_json()
#   (none)
# Outputs  ctx.output("id", "name.ext") is the path to write; ctx.write_json / ctx.write_text
#   sheet       Files (pdf, png, json)    Sheet · board-sheet.pdf to print, the same as PNG, and its measures
#   telemetry   Live telemetry            ctx.telemetry("channel", data) while it runs
# Settings ctx.params["key"]
#   board_mm    number    Board size (mm) = 140
#   marker_mm   number    Marker size (mm) = 25
#   dpi         number    Resolution (dpi) = 300
#   title       text      Title on the sheet = "EsaSim · tic-tac-toe with the xArm"
# Runs on  Any machine. Return a dict to fill the "result" output.
# Packages opencv-python-headless>=4.7, pillow, numpy (installed by the worker with pip when missing)
# EsaSim rewrites these lines from the block's definition each time it is saved: change the definition, not them.
# ── end of EsaSim block header ────────────────────────────────────────

"""Board sheet: the printable board for the tic-tac-toe demo. Four ArUco markers (4x4, ids 0-3) sit outside the
board's corners, each touching it with one corner, so a camera finds and straightens the board by itself. The robot
draws the grid; the sheet only has the markers and a light outline of the board."""
import math


def run(ctx):
    import cv2
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont

    p = ctx.params
    dpi = int(p.get("dpi", 300))
    board_mm = float(p.get("board_mm", 140))
    marker_mm = float(p.get("marker_mm", 25))
    page_w_mm, page_h_mm = 210.0, 297.0  # A4 portrait
    need = board_mm + 2 * marker_mm
    if need > page_w_mm - 8:
        raise ValueError("Board and markers are %.0f mm wide: on A4 (210 mm) keep board + 2 x marker under 202 mm." % need)
    px = lambda mm: int(round(mm * dpi / 25.4))
    W, H = px(page_w_mm), px(page_h_mm)
    img = Image.new("L", (W, H), 255)
    d = ImageDraw.Draw(img)

    # board square, centred horizontally, its top 30 mm below the page's top edge plus a marker
    bx0 = (page_w_mm - board_mm) / 2
    by0 = 30 + marker_mm
    x0, y0, x1, y1 = px(bx0), px(by0), px(bx0 + board_mm), px(by0 + board_mm)
    d.rectangle([x0, y0, x1, y1], outline=200, width=max(1, px(0.3)))

    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    m = px(marker_mm)
    # id 0 top-left, 1 top-right, 2 bottom-right, 3 bottom-left; each touches the board with its inner corner
    places = {0: (x0 - m, y0 - m), 1: (x1, y0 - m), 2: (x1, y1), 3: (x0 - m, y1)}
    for mid, (mx, my) in places.items():
        mk = cv2.aruco.generateImageMarker(dic, mid, m, borderBits=1)
        img.paste(Image.fromarray(mk), (mx, my))

    def font_of(size):
        for name in ("DejaVuSans.ttf", "arial.ttf", "Arial.ttf", "Helvetica.ttc", "LiberationSans-Regular.ttf"):
            try:
                return ImageFont.truetype(name, size)
            except OSError:
                pass
        try:
            return ImageFont.load_default(size=size)  # Pillow 10.1 and later
        except TypeError:
            return ImageFont.load_default()
    font, small = font_of(px(4.5)), font_of(px(3))
    ty = y1 + m + px(12)
    d.text((px(20), ty), str(p.get("title") or "EsaSim · tic-tac-toe with the xArm"), fill=0, font=font)
    lines = [
        "Print at 100%% (no scaling). The light square is the board: %.0f mm. Check it with a ruler." % board_mm,
        "Tape the sheet flat under the camera. Keep the four black markers visible and unmarked.",
        "The xArm draws the grid in the square; the visitor plays with a marker in the cells.",
        "Markers: ArUco 4x4, ids 0 (top left), 1 (top right), 2 (bottom right), 3 (bottom left), %.0f mm." % marker_mm,
    ]
    for i, t in enumerate(lines):
        d.text((px(20), ty + px(9) + i * px(5.5)), t, fill=60, font=small)

    png = ctx.output("sheet", "board-sheet.png")
    img.save(png, dpi=(dpi, dpi))
    pdf = ctx.output("sheet", "board-sheet.pdf")
    img.convert("RGB").save(pdf, "PDF", resolution=dpi)
    ctx.picture("sheet", img.convert("RGB"), width=420)
    info = {"board_mm": board_mm, "marker_mm": marker_mm, "dictionary": "DICT_4X4_50", "ids": {"top_left": 0, "top_right": 1, "bottom_right": 2, "bottom_left": 3},
            "dpi": dpi, "page": "A4 portrait"}
    ctx.write_json("sheet", "board-sheet.json", info)
    ctx.log("Board sheet: %.0f mm board, %.0f mm markers, %d dpi. Print board-sheet.pdf at 100%%." % (board_mm, marker_mm, dpi))
    return info
