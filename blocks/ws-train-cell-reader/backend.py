# ── EsaSim block ──────────────────────────────────────────────────────
# Train cell reader (ws-train-cell-reader) · version 2 · machine block, Python: def run(ctx)
# Trains the board reader (PyTorch CNN: empty, X or O in a 32x32 cell) on the accelerator you choose: NVIDIA GPU, Tenstorrent or CPU.
# Workspace: Gianluca De Novi's workspace · Author: Gianluca De Novi
# Saved: 2026-11-07
#
# Inputs   ctx.inputs["id"] is a list of files: .path .name .size .read_text() .read_json()
#   cells       Files (npz)               Cells · several · *.npz from Simulate games or Render cells
#   real        Files (npz)               Real cells · several · optional · Optional: cells from real pictures (Read board, games), same format; they count more
# Outputs  ctx.output("id", "name.ext") is the path to write; ctx.write_json / ctx.write_text
#   model       Files (pt, json)          Board reader · reader.pt (weights and facts) and reader.json
#   metrics     Metrics (json)            Metrics
#   telemetry   Live telemetry            ctx.telemetry("channel", data) while it runs
# Settings ctx.params["key"]
#   accelerator select    Accelerator = "NVIDIA GPU" of NVIDIA GPU | Tenstorrent | CPU
#   epochs      number    Epochs = 6
#   batch       number    Batch = 256
#   lr          number    Learning rate = 0.002
#   validation  number    Share for validation = 0.1
#   seed        number    Seed = 1
# Runs on  Any machine, NVIDIA GPU, Tenstorrent. Return a dict to fill the "result" output.
# Packages torch[cuda], numpy (installed by the worker with pip when missing)
# EsaSim rewrites these lines from the block's definition each time it is saved: change the definition, not them.
# ── end of EsaSim block header ────────────────────────────────────────

"""Train cell reader: a small convolutional network (PyTorch) that tells empty, X and O apart in 32x32 pictures of
board cells, trained on the cells of Simulate games and Render cells (and real cells, same format), on the
accelerator chosen in its settings: an NVIDIA GPU, a Tenstorrent card (PyTorch/XLA from TT-Forge) or the CPU.
Live: loss, accuracy, pictures per second, and the confusion matrix after each epoch."""
import math
import os
import time

CLASSES = ["empty", "X", "O"]


def make_reader(nn):
    return nn.Sequential(
        nn.Conv2d(1, 16, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),
        nn.Conv2d(16, 32, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),
        nn.Conv2d(32, 64, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),
        nn.Flatten(), nn.Linear(64 * 4 * 4, 64), nn.ReLU(), nn.Linear(64, 3))


def normalize(x):
    """uint8 pictures N x 32 x 32 -> float, each picture to mean 0 and unit spread (light does not matter)."""
    import numpy as np
    x = x.astype(np.float32)
    m = x.mean(axis=(1, 2), keepdims=True)
    s = x.std(axis=(1, 2), keepdims=True)
    return (x - m) / (s + 8.0)


def pick_device(ctx, want):
    """(device, label, xla module or None) for the Accelerator setting: NVIDIA GPU (CUDA), Tenstorrent (PyTorch/XLA
    through TT-Forge's tt-xla) or CPU. The one asked for is used, or the block stops and says why: a run never
    silently moves to another accelerator."""
    import torch
    w = str(want or "NVIDIA GPU").strip().lower()
    if w.startswith("tenstorrent") or w == "tt":
        try:
            import torch_xla.core.xla_model as xm
        except Exception as e:
            raise RuntimeError("Tenstorrent asked, but this machine's Python has no PyTorch/XLA (%s). On the Linux machine with the cards, "
                               "install TT-Forge's PyTorch support (tt-xla) for the worker's Python first, then run again." % e)
        try:
            import torch_xla.runtime as xr
            xr.set_device_type("TT")
        except Exception:
            pass
        dev = xm.xla_device()
        hw = ""
        try:
            hw = str(xm.xla_device_hw(dev))
        except Exception:
            pass
        if hw.upper() == "CPU":
            raise RuntimeError("Tenstorrent asked, but PyTorch/XLA found only the CPU: check that tt-smi sees the cards and that tt-xla is installed.")
        return dev, "Tenstorrent (PyTorch/XLA%s)" % (", " + hw if hw else ""), xm
    if w.startswith("nvidia") or w in ("cuda", "gpu", "auto"):
        if torch.cuda.is_available():
            return torch.device("cuda"), "NVIDIA " + torch.cuda.get_device_name(0), None
        why = ("this PyTorch was built without CUDA (%s)" % torch.__version__) if not torch.version.cuda else "no NVIDIA GPU or driver usable from the worker"
        raise RuntimeError("NVIDIA GPU asked, but " + why + ". On Windows EsaSim 0.12.13 or later installs PyTorch with CUDA for blocks that "
                           "list torch[cuda]; check that nvidia-smi works on the machine. Or set Accelerator to CPU.")
    return torch.device("cpu"), "CPU (%d threads)" % torch.get_num_threads(), None


def load_cells(ctx, port):
    import numpy as np
    xs, ys, names = [], [], []
    for f in ctx.inputs.get(port, []):
        if not f.name.lower().endswith(".npz"):
            continue
        d = np.load(f.path)
        if d["x"].shape[1:] != (32, 32):
            raise ValueError("%s has cells of %s pixels: this reader takes 32x32 (Render cells, size 32)." % (f.name, d["x"].shape[1:]))
        xs.append(d["x"]); ys.append(d["y"]); names.append(f.rel or f.name)
    if not xs:
        return None, None, []
    return np.concatenate(xs), np.concatenate(ys).astype(np.int64), names


def run(ctx):
    import numpy as np
    import torch
    import torch.nn as nn

    p = ctx.params
    torch.manual_seed(int(p.get("seed", 1)))
    x, y, names = load_cells(ctx, "cells")
    if x is None:
        raise ValueError("No cells to train on: connect the cells of Simulate games or Render cells (or the cells-synthetic dataset).")
    rx, ry, rnames = load_cells(ctx, "real")
    if rx is not None:
        # real cells count more: repeated so they are about a fifth of what the reader sees
        rep = max(1, int(0.2 * len(x) / max(1, len(rx))))
        x, y = np.concatenate([x] + [rx] * rep), np.concatenate([y] + [ry] * rep)
        ctx.log("Real cells: %d (from %s), used %d times each." % (len(rx), ", ".join(rnames), rep))
    n = len(x)
    order = np.random.default_rng(int(p.get("seed", 1))).permutation(n)
    nval = max(200, int(n * float(p.get("validation", 0.1))))
    vi, ti = order[:nval], order[nval:]
    X = torch.from_numpy(normalize(x)).unsqueeze(1)
    Y = torch.from_numpy(y)
    dev, label, xm = pick_device(ctx, p.get("accelerator", "NVIDIA GPU"))
    ctx.log("Training on %s: %d cells (%d for validation) from %s" % (label, n, nval, ", ".join(names)))
    ctx.telemetry("device", {"device": label, "cells": n})

    model = make_reader(nn).to(dev)
    opt = torch.optim.Adam(model.parameters(), lr=float(p.get("lr", 0.002)))
    lossf = nn.CrossEntropyLoss()
    epochs, bs = int(p.get("epochs", 6)), int(p.get("batch", 256))
    steps_per = math.ceil(len(ti) / bs)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=float(p.get("lr", 0.002)), total_steps=epochs * steps_per)
    t0, last, seen, step = time.time(), 0.0, 0, 0
    best = {"val_acc": 0.0}

    def evaluate():
        model.eval()
        conf = np.zeros((3, 3), np.int64)
        tot_loss = 0.0
        with torch.no_grad():
            for i in range(0, nval, 1024):
                idx = vi[i:i + 1024]
                out = model(X[idx].to(dev))
                tot_loss += float(lossf(out, Y[idx].to(dev))) * len(idx)
                pred = out.argmax(1).cpu().numpy()
                for a, b in zip(y[idx], pred):
                    conf[a, b] += 1
        model.train()
        return conf, tot_loss / nval

    for ep in range(epochs):
        perm = torch.from_numpy(np.random.permutation(ti))
        for b in range(steps_per):
            idx = perm[b * bs:(b + 1) * bs]
            xb = X[idx]
            # flips and quarter turns do not change empty, X or O
            k = int(torch.randint(0, 4, (1,)))
            xb = torch.rot90(xb, k, (2, 3))
            if torch.rand(1).item() < 0.5:
                xb = torch.flip(xb, (3,))
            xb, yb = xb.to(dev), Y[idx].to(dev)
            out = model(xb)
            loss = lossf(out, yb)
            opt.zero_grad()
            loss.backward()
            opt.step()
            sched.step()
            if xm is not None:
                xm.mark_step()
            step += 1
            seen += len(idx)
            now = time.time()
            if now - last > 0.5 or b == steps_per - 1:
                last = now
                l = float(loss.detach().cpu())
                acc = float((out.argmax(1) == yb).float().mean().cpu())
                ctx.telemetry("train", {"epoch": ep + 1, "step": step, "loss": round(l, 4), "acc": round(acc, 4),
                                        "img_s": round(seen / max(1e-6, now - t0)), "lr": round(sched.get_last_lr()[0], 6)})
                ctx.progress(100 * step / (epochs * steps_per), "epoch %d of %d" % (ep + 1, epochs))
        conf, vloss = evaluate()
        vacc = float(np.trace(conf) / max(1, conf.sum()))
        ctx.telemetry("val", {"epoch": ep + 1, "val_acc": round(vacc, 4), "val_loss": round(vloss, 4)})
        ctx.telemetry("confusion", {"matrix": conf.tolist(), "rows": "true", "cols": "read", "classes": CLASSES})
        ctx.log("Epoch %d: validation accuracy %.2f%%, loss %.4f" % (ep + 1, 100 * vacc, vloss))
        if vacc >= best["val_acc"]:
            best = {"val_acc": vacc, "epoch": ep + 1, "confusion": conf.tolist()}
            state = {k: v.detach().cpu() for k, v in model.state_dict().items()}

    secs = time.time() - t0
    meta = {"kind": "esasim.ttt.reader/1", "arch": "cnn32", "classes": CLASSES, "size": 32, "normalize": "per-picture mean/std+8",
            "val_acc": round(best["val_acc"], 4), "epoch": best["epoch"], "cells": n, "trained_on": names + rnames, "device": label,
            "seconds": round(secs, 1), "img_s": round(seen / max(1e-6, secs))}
    torch.save({"meta": meta, "state": state}, ctx.output("model", "reader.pt"))
    ctx.write_json("model", "reader.json", meta)
    ctx.write_json("metrics", "reader-metrics.json", {"val_acc": meta["val_acc"], "epochs": epochs, "cells": n, "seconds": meta["seconds"], "img_s": meta["img_s"], "device": label})
    ctx.log("Board reader: %.2f%% on validation, %.0f s on %s." % (100 * best["val_acc"], secs, label))
    return {"val_acc": meta["val_acc"], "device": label, "seconds": meta["seconds"], "cells": n}
