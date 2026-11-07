# ── EsaSim block ──────────────────────────────────────────────────────
# Render cells (ws-render-cells) · version 1 · machine block, Python: def run(ctx)
# Synthetic pictures of tic-tac-toe cells (empty, X, O) as a camera sees them on paper, to train the board reader.
# Workspace: Gianluca De Novi's workspace · Author: Gianluca De Novi
# Saved: 2026-11-07
#
# Inputs   ctx.inputs["id"] is a list of files: .path .name .size .read_text() .read_json()
#   (none)
# Outputs  ctx.output("id", "name.ext") is the path to write; ctx.write_json / ctx.write_text
#   cells       Files (npz)               Cells · cells_01.npz: x (N x 32 x 32 uint8), y (0 empty, 1 X, 2 O)
#   samples     Files (png)               Examples · A mosaic of the first 192 cells
#   telemetry   Live telemetry            ctx.telemetry("channel", data) while it runs
# Settings ctx.params["key"]
#   count       number    Cells = 30000
#   size        number    Size (pixels) = 32
#   seed        number    Seed = 1
# Runs on  Any machine. Return a dict to fill the "result" output.
# Packages numpy, pillow (installed by the worker with pip when missing)
# EsaSim rewrites these lines from the block's definition each time it is saved: change the definition, not them.
# ── end of EsaSim block header ────────────────────────────────────────

"""Render cells: synthetic pictures of tic-tac-toe cells (empty, X, O) as a camera sees them on paper, for training
the board reader. Strokes of many hands and pens, paper tones, uneven light, small perspective, blur and noise, and
bits of the grid lines at the edges, like the crops the Game block cuts from the camera. One compressed file per
task (cells_<task>.npz: x = N x S x S uint8, y = 0 empty, 1 X, 2 O) and a mosaic of examples."""
import math
import time

CLASSES = ["empty", "X", "O"]


def run(ctx):
    import numpy as np
    from PIL import Image, ImageDraw, ImageFilter

    p = ctx.params
    n = int(p.get("count", 20000))
    S = int(p.get("size", 32))
    task = ctx.task.index if ctx.task else 0
    rng = np.random.default_rng(int(p.get("seed", 1)) * 1000 + task)
    K = 4  # drawn K times larger, then reduced: smooth strokes
    big = S * K

    def jitter(a):
        return a * (1 + rng.normal(0, 0.06))

    def stroke(d, pts, width, ink):
        # a slightly wobbly hand-drawn line through pts
        out = []
        for (ax, ay), (bx, by) in zip(pts[:-1], pts[1:]):
            steps = max(2, int(math.hypot(bx - ax, by - ay) / (big / 24)))
            bend = rng.normal(0, big * 0.02)
            for i in range(steps + 1):
                t = i / steps
                nx, ny = -(by - ay), (bx - ax)
                ln = math.hypot(nx, ny) or 1
                off = bend * math.sin(math.pi * t) + rng.normal(0, big * 0.004)
                out.append((ax + (bx - ax) * t + nx / ln * off, ay + (by - ay) * t + ny / ln * off))
        d.line(out, fill=ink, width=width, joint="curve")
        r = width / 2
        for x, y in (out[0], out[-1]):
            d.ellipse([x - r, y - r, x + r, y + r], fill=ink)

    def cell(label):
        paper = int(rng.integers(150, 250))
        img = Image.new("L", (big, big), paper)
        d = ImageDraw.Draw(img)
        ink = int(max(0, paper - rng.integers(90, 200)))
        width = max(2, int(big * rng.uniform(0.035, 0.11)))
        # bits of the robot's grid lines at the edges (crops are not perfect)
        gw = max(2, int(big * rng.uniform(0.02, 0.06)))
        for side in range(4):
            if rng.random() < 0.35:
                off = rng.uniform(-0.04, 0.06) * big
                gi = int(max(0, paper - rng.integers(60, 180)))
                if side == 0:
                    d.line([(off, 0), (off + rng.normal(0, 2), big)], fill=gi, width=gw)
                elif side == 1:
                    d.line([(big - off, 0), (big - off + rng.normal(0, 2), big)], fill=gi, width=gw)
                elif side == 2:
                    d.line([(0, off), (big, off + rng.normal(0, 2))], fill=gi, width=gw)
                else:
                    d.line([(0, big - off), (big, big - off + rng.normal(0, 2))], fill=gi, width=gw)
        c = big / 2 + rng.normal(0, big * 0.05, 2)
        size = big * rng.uniform(0.22, 0.38)
        if label == 1:  # X: two strokes, of different lengths and angles
            a = rng.normal(math.pi / 4, 0.18)
            for k in range(2):
                ang = a + k * (math.pi / 2 + rng.normal(0, 0.15))
                r1, r2 = jitter(size), jitter(size)
                stroke(d, [(c[0] - r1 * math.cos(ang), c[1] - r1 * math.sin(ang)), (c[0] + r2 * math.cos(ang), c[1] + r2 * math.sin(ang))], width, ink)
        elif label == 2:  # O: a closed or almost closed loop, a bit oval and tilted
            rx, ry = jitter(size), jitter(size) * rng.uniform(0.75, 1.15)
            tilt, start = rng.uniform(0, math.pi), rng.uniform(0, 2 * math.pi)
            span = 2 * math.pi * rng.uniform(0.9, 1.08)
            pts = []
            for i in range(41):
                t = start + span * i / 40
                wob = 1 + rng.normal(0, 0.02)
                x, y = rx * wob * math.cos(t), ry * wob * math.sin(t)
                pts.append((c[0] + x * math.cos(tilt) - y * math.sin(tilt), c[1] + x * math.sin(tilt) + y * math.cos(tilt)))
            d.line(pts, fill=ink, width=width, joint="curve")
        else:  # empty: sometimes a smudge or a dot
            if rng.random() < 0.25:
                x, y = rng.uniform(0.2, 0.8, 2) * big
                r = big * rng.uniform(0.01, 0.04)
                d.ellipse([x - r, y - r, x + r, y + r], fill=int(paper - rng.integers(20, 80)))
        # perspective: a small random affine
        if rng.random() < 0.9:
            ang = rng.normal(0, 0.06)
            sx, sy, sh = 1 + rng.normal(0, 0.05), 1 + rng.normal(0, 0.05), rng.normal(0, 0.05)
            ca, sa = math.cos(ang), math.sin(ang)
            a, b, cc, dd = ca * sx, -sa + sh, sa, ca * sy
            det = a * dd - b * cc
            ia, ib, ic, idd = dd / det, -b / det, -cc / det, a / det
            cx = cy = big / 2
            tx = cx - (ia * cx + ib * cy) + rng.normal(0, big * 0.02)
            ty = cy - (ic * cx + idd * cy) + rng.normal(0, big * 0.02)
            img = img.transform((big, big), Image.AFFINE, (ia, ib, tx, ic, idd, ty), resample=Image.BILINEAR, fillcolor=paper)
        img = img.filter(ImageFilter.GaussianBlur(rng.uniform(0, 1.6) * K))
        arr = np.asarray(img.resize((S, S), Image.BILINEAR), dtype=np.float32)
        # light: a gradient and a contrast change, then sensor noise
        gx, gy = np.meshgrid(np.linspace(-1, 1, S), np.linspace(-1, 1, S))
        arr = arr * (1 + 0.18 * rng.normal() * gx + 0.18 * rng.normal() * gy) * rng.uniform(0.7, 1.15) + rng.normal(0, 8)
        arr += rng.normal(0, rng.uniform(1, 9), (S, S))
        return np.clip(arr, 0, 255).astype(np.uint8)

    xs = np.zeros((n, S, S), np.uint8)
    ys = rng.integers(0, 3, n).astype(np.uint8)
    t0, last = time.time(), 0.0
    for i in range(n):
        xs[i] = cell(int(ys[i]))
        now = time.time()
        if now - last > 0.5 or i == n - 1:
            last = now
            ctx.progress(100 * (i + 1) / n, "%d of %d cells" % (i + 1, n))
            ctx.telemetry("render", {"cells": i + 1, "of": n, "cells_s": round((i + 1) / max(1e-6, now - t0), 1), "task": task + 1})
        if (i + 1) % 2000 == 0 or i == n - 1:
            k = min(i + 1, 96)
            ctx.picture("cells", mosaic(xs[i + 1 - k:i + 1], 12), width=480, done=i + 1)

    out = ctx.output("cells", "cells_%02d.npz" % (task + 1))
    np.savez_compressed(out, x=xs, y=ys, size=S, classes=np.array(CLASSES))
    Image.fromarray(mosaic(xs[:192], 16)).save(ctx.output("samples", "samples_%02d.png" % (task + 1)))
    counts = {c: int((ys == k).sum()) for k, c in enumerate(CLASSES)}
    ctx.log("Rendered %d cells of %dx%d in %.1f s: %s" % (n, S, S, time.time() - t0, counts))
    return {"cells": n, "size": S, "seconds": round(time.time() - t0, 1), **counts}


def mosaic(xs, cols):
    import numpy as np
    k, S = len(xs), xs.shape[1]
    rows = max(1, math.ceil(k / cols))
    out = np.full((rows * (S + 2), cols * (S + 2)), 255, np.uint8)
    for i in range(k):
        r, c = divmod(i, cols)
        out[r * (S + 2) + 1:r * (S + 2) + 1 + S, c * (S + 2) + 1:c * (S + 2) + 1 + S] = xs[i]
    return out
