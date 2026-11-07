# ── EsaSim block ──────────────────────────────────────────────────────
# Train player by self-play (ws-train-player-by-self-play) · version 2 · machine block, Python: def run(ctx)
# A small value network (PyTorch) learns tic-tac-toe by playing thousands of games against itself, on the accelerator you choose.
# Workspace: Gianluca De Novi's workspace · Author: Gianluca De Novi
# Saved: 2026-11-07
#
# Inputs   ctx.inputs["id"] is a list of files: .path .name .size .read_text() .read_json()
#   (none)
# Outputs  ctx.output("id", "name.ext") is the path to write; ctx.write_json / ctx.write_text
#   model       Files (pt, json)          Player · player.pt (weights and facts) and player.json
#   metrics     Metrics (json)            Metrics
#   telemetry   Live telemetry            ctx.telemetry("channel", data) while it runs
# Settings ctx.params["key"]
#   accelerator select    Accelerator = "NVIDIA GPU" of NVIDIA GPU | Tenstorrent | CPU
#   games       number    Games = 60000
#   parallel    number    Games played side by side = 256
#   eval_every  number    Test every (games) = 4000
#   explore_start number    Random moves at the start = 0.5
#   explore_end number    Random moves at the end = 0.05
#   lookahead   number    Weight of the look ahead = 0.8
#   updates     number    Training steps per batch of games = 8
#   lr          number    Learning rate = 0.002
#   seed        number    Seed = 1
# Runs on  Any machine, NVIDIA GPU, Tenstorrent. Return a dict to fill the "result" output.
# Packages torch[cuda], numpy (installed by the worker with pip when missing)
# EsaSim rewrites these lines from the block's definition each time it is saved: change the definition, not them.
# ── end of EsaSim block header ────────────────────────────────────────

"""Train player by self-play: a small value network (PyTorch) learns tic-tac-toe by playing thousands of games
against itself, in batches of games played side by side. After each move the network scores the position for the
player who just moved; it plays the move with the best score (sometimes a random one, to explore) and learns from
how the games ended. Tested as it learns against a random player and the perfect player, and on every position of the
game (the share of moves that are optimal). On the accelerator chosen in its settings: NVIDIA GPU, Tenstorrent or CPU."""
import math
import random
import time

LINES = [(0, 1, 2), (3, 4, 5), (6, 7, 8), (0, 3, 6), (1, 4, 7), (2, 5, 8), (0, 4, 8), (2, 4, 6)]


def winner(b):
    for a, c, d in LINES:
        if b[a] != 0 and b[a] == b[c] == b[d]:
            return b[a]
    return 0


def make_player(nn):
    return nn.Sequential(nn.Linear(18, 96), nn.ReLU(), nn.Linear(96, 96), nn.ReLU(), nn.Linear(96, 1), nn.Tanh())


def features(boards, mover):
    """boards: N x 9 (1 = X, -1 = O, 0), mover: N (1 or -1) -> N x 18 (the mover's marks, then the other's)."""
    import numpy as np
    rel = boards * mover[:, None]
    return np.concatenate([(rel == 1), (rel == -1)], axis=1).astype(np.float32)


class Perfect:
    """Exact play (negamax over all positions, remembered): scores for the player to move, +1 win, 0 draw, -1 loss."""

    def __init__(self):
        self.memo = {}

    def value(self, b, turn):
        key = (b, turn)
        if key in self.memo:
            return self.memo[key]
        w = winner(b)
        if w:
            v = 1 if w == turn else -1
        elif 0 not in b:
            v = 0
        else:
            v = max(-self.value(b[:i] + (turn,) + b[i + 1:], -turn) for i in range(9) if b[i] == 0)
        self.memo[key] = v
        return v

    def best_moves(self, b, turn):
        scores = {i: -self.value(b[:i] + (turn,) + b[i + 1:], -turn) for i in range(9) if b[i] == 0}
        top = max(scores.values())
        # among equally good moves, prefer winning now
        now = [i for i, s in scores.items() if s == top and winner(b[:i] + (turn,) + b[i + 1:]) == turn]
        return now or [i for i, s in scores.items() if s == top]


def all_positions():
    """Every position reachable in a game that is not over, with the player to move."""
    seen, out, stack = set(), [], [((0,) * 9, 1)]
    while stack:
        b, t = stack.pop()
        if (b, t) in seen:
            continue
        seen.add((b, t))
        if winner(b) or 0 not in b:
            continue
        out.append((b, t))
        for i in range(9):
            if b[i] == 0:
                stack.append((b[:i] + (t,) + b[i + 1:], -t))
    return out


def run(ctx):
    import numpy as np
    import torch
    import torch.nn as nn

    p = ctx.params
    seed = int(p.get("seed", 1))
    rng = np.random.default_rng(seed)
    random.seed(seed)
    torch.manual_seed(seed)
    total = int(p.get("games", 60000))
    B = int(p.get("parallel", 256))
    eps0, eps1 = float(p.get("explore_start", 0.5)), float(p.get("explore_end", 0.05))
    dev, label, xm = pick_device(ctx, p.get("accelerator", "NVIDIA GPU"))
    ctx.log("Self-play on %s: %d games, %d at a time." % (label, total, B))
    ctx.telemetry("device", {"device": label})
    net = make_player(nn).to(dev)
    opt = torch.optim.Adam(net.parameters(), lr=float(p.get("lr", 0.002)))
    perfect = Perfect()
    positions = all_positions()

    def scores(boards, mover):
        with torch.no_grad():
            v = net(torch.from_numpy(features(boards, mover)).to(dev)).squeeze(1)
            return v.cpu().numpy()

    def choose(b, turn, eps=0.0):
        """The network's move for one position (tuple), and the scores of all legal moves."""
        legal = [i for i in range(9) if b[i] == 0]
        nb = np.array([b[:i] + (turn,) + b[i + 1:] for i in legal], np.float32)
        s = scores(nb, np.full(len(legal), turn, np.float32))
        if eps and random.random() < eps:
            return random.choice(legal), dict(zip(legal, s))
        return legal[int(np.argmax(s))], dict(zip(legal, s))

    def play(x_player, o_player):
        b, turn = (0,) * 9, 1
        while True:
            w = winner(b)
            if w or 0 not in b:
                return w
            i = (x_player if turn == 1 else o_player)(b, turn)
            b = b[:i] + (turn,) + b[i + 1:]
            turn = -turn

    netp = lambda b, t: choose(b, t)[0]
    rand = lambda b, t: random.choice([i for i in range(9) if b[i] == 0])
    perf = lambda b, t: random.choice(perfect.best_moves(b, t))

    def evaluate(n_rand=200, n_perf=60):
        net.eval()
        r = {}
        for name, opp, k in (("random", rand, n_rand), ("perfect", perf, n_perf)):
            w = d = l = 0
            for g in range(k):
                as_x = g % 2 == 0
                res = play(netp, opp) if as_x else play(opp, netp)
                me = 1 if as_x else -1
                if res == me:
                    w += 1
                elif res == 0:
                    d += 1
                else:
                    l += 1
            r[name] = (w / k, d / k, l / k)
        # every position: is the network's move one of the best?
        ok = 0
        for b, t in positions:
            if choose(b, t)[0] in perfect.best_moves(b, t):
                ok += 1
        net.train()
        return {"vs_random_win": round(r["random"][0], 3), "vs_random_draw": round(r["random"][1], 3), "vs_random_loss": round(r["random"][2], 3),
                "vs_perfect_draw": round(r["perfect"][1], 3), "vs_perfect_loss": round(r["perfect"][2], 3), "optimal_moves": round(ok / len(positions), 4)}

    def first_move_map():
        nb = np.zeros((9, 9), np.float32)
        for i in range(9):
            nb[i, i] = 1
        return [[round(float(v), 3) for v in row] for row in scores(nb, np.ones(9, np.float32)).reshape(3, 3)]

    td = float(p.get("lookahead", 0.8))

    def lookahead(boards, mover):
        """Value of each position for the player who just moved: the end of the game if it is over, else minus the
        best the opponent can reach in one move, as the network sees it now."""
        out = np.zeros(len(boards), np.float32)
        kids, parent = [], []
        for k, b in enumerate(boards):
            w = winner(tuple(int(v) for v in b))
            if w:
                out[k] = w * mover[k]
            elif (b == 0).any():
                for i in np.nonzero(b == 0)[0]:
                    nb = b.copy()
                    nb[i] = -mover[k]
                    kids.append(nb)
                    parent.append(k)
        if kids:
            parent = np.array(parent)
            v = scores(np.array(kids, np.float32), -mover[parent])
            best = np.full(len(boards), -np.inf, np.float32)
            np.maximum.at(best, parent, v)
            has = best > -np.inf
            out[has] = -best[has]
        return out

    buf_b, buf_m, buf_z = [], [], []
    played, it, t0, last = 0, 0, time.time(), 0.0
    eval_every = max(1, int(p.get("eval_every", 4000)))
    next_eval = eval_every
    hist = []
    while played < total:
        n = min(B, total - played)
        eps = eps1 + (eps0 - eps1) * max(0.0, 1 - played / (0.8 * total))
        boards = np.zeros((n, 9), np.float32)
        turn = np.ones(n, np.float32)
        alive = np.ones(n, bool)
        recs = []  # (game, position after the move, mover)
        while alive.any():
            act = np.nonzero(alive)[0]
            cand_b, cand_g = [], []
            for g in act:
                for i in np.nonzero(boards[g] == 0)[0]:
                    nb = boards[g].copy()
                    nb[i] = turn[g]
                    cand_b.append(nb); cand_g.append(g)
            cand_b = np.array(cand_b, np.float32)
            cand_g = np.array(cand_g)
            s = scores(cand_b, turn[cand_g])
            for g in act:
                m = np.nonzero(cand_g == g)[0]
                k = m[int(np.argmax(s[m]))] if rng.random() >= eps else rng.choice(m)
                boards[g] = cand_b[k]
                recs.append((g, cand_b[k].copy(), turn[g]))
                w = winner(tuple(int(v) for v in boards[g]))
                if w or not (boards[g] == 0).any():
                    alive[g] = False
                turn[g] = -turn[g]
        res = np.array([winner(tuple(int(v) for v in b)) for b in boards], np.float32)
        fb = np.array([r[1] for r in recs], np.float32)
        mv = np.array([r[2] for r in recs], np.float32)
        buf_b.append(fb); buf_m.append(mv); buf_z.append(res[[r[0] for r in recs]] * mv)
        buf_b, buf_m, buf_z = buf_b[-40:], buf_m[-40:], buf_z[-40:]
        played += n
        it += 1
        # learn from the recent games: each position's target mixes how the game ended with the network's own look
        # one move ahead (the opponent's best reply), which spreads what it knows about the ends of games backwards
        Bb, Bm, Bz = np.concatenate(buf_b), np.concatenate(buf_m), np.concatenate(buf_z)
        for _ in range(int(p.get("updates", 8))):
            idx = rng.integers(0, len(Bb), min(len(Bb), 1024))
            tgt = td * lookahead(Bb[idx], Bm[idx]) + (1 - td) * Bz[idx]
            xb = torch.from_numpy(features(Bb[idx], Bm[idx])).to(dev)
            loss = ((net(xb).squeeze(1) - torch.from_numpy(tgt.astype(np.float32)).to(dev)) ** 2).mean()
            opt.zero_grad()
            loss.backward()
            opt.step()
            if xm is not None:
                xm.mark_step()
        now = time.time()
        if now - last > 0.5:
            last = now
            ctx.telemetry("train", {"games": played, "loss": round(float(loss.detach().cpu()), 4), "explore": round(eps, 3),
                                    "games_s": round(played / max(1e-6, now - t0)), "x_wins": round(float((res == 1).mean()), 3),
                                    "o_wins": round(float((res == -1).mean()), 3), "draws": round(float((res == 0).mean()), 3)})
            ctx.progress(100 * played / total, "%d of %d games" % (played, total))
        if played >= next_eval or played >= total:
            next_eval += eval_every
            e = evaluate()
            e["games"] = played
            hist.append(e)
            ctx.telemetry("eval", e)
            ctx.telemetry("first_move", {"matrix": first_move_map(), "note": "score of each first move for X"})
            ctx.log("After %d games: optimal moves %.1f%%, against random %.0f%% wins %.0f%% losses, against perfect %.0f%% draws"
                    % (played, 100 * e["optimal_moves"], 100 * e["vs_random_win"], 100 * e["vs_random_loss"], 100 * e["vs_perfect_draw"]))

    final = hist[-1] if hist else evaluate()
    secs = time.time() - t0
    meta = {"kind": "esasim.ttt.player/1", "arch": "mlp18", "games": played, "device": label, "seconds": round(secs, 1),
            "games_s": round(played / max(1e-6, secs)), **final}
    torch.save({"meta": meta, "state": {k: v.detach().cpu() for k, v in net.state_dict().items()}}, ctx.output("model", "player.pt"))
    ctx.write_json("model", "player.json", meta)
    ctx.write_json("metrics", "player-metrics.json", meta)
    ctx.log("Player: %d games in %.0f s on %s; %.1f%% optimal moves." % (played, secs, label, 100 * final["optimal_moves"]))
    return {"games": played, "device": label, "seconds": meta["seconds"], "optimal_moves": final["optimal_moves"],
            "vs_random_win": final["vs_random_win"], "vs_perfect_draw": final["vs_perfect_draw"]}


# ---- shared with the other blocks of the tic-tac-toe demo ----

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
