# ── EsaSim block ──────────────────────────────────────────────────────
# Tic-tac-toe game (ws-tic-tac-toe-game) · version 7 · machine block, Python: def run(ctx)
# The xArm plays tic-tac-toe on paper against a visitor, in simulation (the digital twin) or with the real arm and camera.
# Workspace: Gianluca De Novi's workspace · Author: Gianluca De Novi
# Saved: 2026-11-09
#
# Inputs   ctx.inputs["id"] is a list of files: .path .name .size .read_text() .read_json()
#   models      Files (pt)                Models · several · optional · reader.pt and player.pt (the models dataset, or the training blocks)
#   robot       Files (gz, json)          Robot model · optional · xarm6.robot.json.gz from xArm model
# Outputs  ctx.output("id", "name.ext") is the path to write; ctx.write_json / ctx.write_text
#   games       Files (json, png)         Games · game_001.json: who started, moves, times, result; the simulated paper
#   cells       Files (npz)               Real cells · The cells seen in the games, labelled by the game
#   log         Table (csv)               Poses · poses.csv: every pose (and, in simulation, the joints)
#   telemetry   Live telemetry            ctx.telemetry("channel", data) while it runs
# Settings ctx.params["key"]
#   robot       select    Robot = "simulation" of simulation | real xArm
#   camera      text      Camera = "simulated"
#   visitor     select    Visitor (simulation) = "me (Controls)" of me (Controls) | random | perfect
#   auto_games  number    Games with a simulated visitor = 5
#   sim_speed   number    Simulation speed = 1
#   first       select    Who starts = "take turns" of take turns | the robot | the visitor
#   robot_plays select    The robot plays = "X" of X | O
#   player      select    Player = "learned, checked" of learned, checked | learned | perfect
#   accelerator select    Accelerator for the models = "CPU" of CPU | NVIDIA GPU | Tenstorrent
#   camera_mount select    Camera mount = "table" of table | wrist
#   camera_pos  text      Camera position (mm) = ""
#   camera_look text      Camera looks at (mm) = ""
#   camera_hfov number    Camera field of view (degrees) = 70
#   wrist_camera text      Wrist camera offset (mm) = "70, 0, -40"
#   wrist_height number    Flange height to read (wrist camera, mm) = 330
#   park        text      Rest position of the pen (mm) = "150, -250, 250"
#   draw_grid   bool      Draw the grid at the start of each game = true
#   keep_playing bool      Offer a new game after each one = true
#   watch       bool      See the visitor's move without the button (real camera) = false
#   sure        number    Least confidence to accept a mark = 0.7
#   minutes     number    Longest session (minutes) = 30
#   channel     text      Controls channel = "control"
#   ip          text      Arm IP address (real xArm) = "192.168.1.200"
#   board_x     number    Board X (mm) = 390
#   board_y     number    Board Y (mm) = -70
#   board_rotation number    Board rotation (degrees) = 90
#   board_size  number    Board size (mm) = 140
#   table_z     number    Table height (mm) = 0
#   pen_length  number    Pen length below the flange (mm) = 100
#   tcp         select    The arm's TCP is = "flange" of flange | pen tip
#   z_draw      number    Draw height (mm) = 100
#   z_up        number    Travel height (mm) = 130
#   roll        number    Tool roll (degrees) = 180
#   pitch       number    Tool pitch (degrees) = 0
#   yaw         number    Tool yaw (degrees) = 0
#   speed       number    Pen speed (mm/s) = 80
#   accel       number    Pen acceleration (mm/s²) = 400
#   joint_speed number    Joint speed (degrees/s) = 30
#   joint_accel number    Joint acceleration (degrees/s²) = 80
#   safe_box    text      Safe box (mm) = "100,600, -350,350, 20,450"
#   pen_width   number    Pen width (mm, simulation) = 1.2
#   width       number    Camera width (pixels) = 1280
#   height      number    Camera height (pixels) = 720
# Runs on  Any machine. Return a dict to fill the "result" output.
# Packages opencv-python-headless>=4.7, numpy, torch[cuda], xarm-python-sdk (installed by the worker with pip when missing)
# EsaSim rewrites these lines from the block's definition each time it is saved: change the definition, not them.
# ── end of EsaSim block header ────────────────────────────────────────

"""Tic-tac-toe game: the xArm plays a visitor on paper, in simulation (the digital twin: the arm, its pen, the paper
and the camera simulated) or with the real arm and camera. One long-running block holds the camera, the models and
the arm for the whole session: it draws the grid, plays its moves with the pen, and reads the visitor's moves with the
camera and the board reader. In simulation the visitor chooses a cell in the Controls view (or a simulated visitor
plays) and a simulated hand draws it, so the whole chain runs: camera, reader, player, arm. With the real arm, its
twin follows every move, and how far the real arm is from its twin is reported live.

In simulation, the switch Real arm in the controls makes the real xArm (at Arm IP address) follow the digital twin:
it first comes gently to the twin's pose, then receives the twin's joint angles as they are computed (servo mode,
every 20 ms). Every motion of the twin is planned and checked before it moves, so the real arm only follows checked
motions; while it follows, the simulation runs no faster than real time, and a jump faster than 60°/s becomes a
gentle catch-up move. Switched off (or on any error of the arm), the real arm holds where it is."""
import csv
import json
import math
import os
import random
import signal
import sys
import time

SYM = {1: "X", -1: "O"}
NAMES = ["top left", "top middle", "top right", "middle left", "centre", "middle right", "bottom left", "bottom middle", "bottom right"]


def run(ctx):
    import numpy as np
    import torch

    p = ctx.params
    sim = not str(p.get("robot", "simulation")).lower().startswith("real")
    robot_sym = "O" if str(p.get("robot_plays", "X")).upper() == "O" else "X"
    me, you = (1, -1) if robot_sym == "X" else (-1, 1)  # values on the board: X = 1, O = -1
    mode = str(p.get("player", "learned, checked"))
    visitor = str(p.get("visitor", "me (Controls)"))
    auto = not visitor.startswith("me")
    who = str(p.get("first", "take turns")).lower()

    def starter(n):
        """Who starts game n (0, 1 …): "robot" or "visitor". Taking turns, the robot starts the first game."""
        if who.startswith("robot") or who.startswith("the robot"):
            return "robot"
        if who.startswith("visitor") or who.startswith("the visitor"):
            return "visitor"
        return "robot" if n % 2 == 0 else "visitor"
    channel = str(p.get("channel") or "control")
    deadline = time.time() + float(p.get("minutes", 30)) * 60
    board_mm = float(p.get("board_size", 140))
    rng = np.random.default_rng(int(time.time()))
    rpy = [float(p.get("roll", 180)), float(p.get("pitch", 0)), float(p.get("yaw", 0))]

    def interrupted(*_):
        raise KeyboardInterrupt()
    try:
        signal.signal(signal.SIGTERM, interrupted)
    except (ValueError, OSError):
        pass

    def say(text, **extra):
        ctx.log(text)
        ctx.telemetry("say", dict(text=text, **extra))

    dev, label, _ = pick_device(ctx, p.get("accelerator", "CPU"))
    models = load_models(ctx, "models", dev)
    reader, player = models.get("reader"), models.get("player")
    if player:
        ctx.log("Player: %s (%s games, %.1f%% optimal moves)" % (player[1].get("file"), player[1].get("games"), 100 * float(player[1].get("optimal_moves", 0))))
    elif mode != "perfect":
        ctx.log("No player.pt among the models: the robot plays perfectly instead.")
        mode = "perfect"
    if reader:
        ctx.log("Board reader: %s (%.1f%% on validation)" % (reader[1].get("file"), 100 * float(reader[1].get("val_acc", 0))))
    ctx.telemetry("models", {"device": label, "reader": (reader or [None, {}])[1].get("file"), "player": (player or [None, {}])[1].get("file"), "mode": mode})

    # the world: in simulation the paper, the arm and the camera are simulated
    place = Place(p)
    sheet = Sheet(board_mm, float(p.get("marker_size", 25)), 4.0, 246, rng) if sim else None
    arm_model = load_model(ctx.inputs.get("robot", []), cell=10) if sim else None
    cam_spec = str(p.get("camera", "simulated")).strip()
    sim_cam = sim and cam_spec.lower() in ("", "simulated", "sim")
    cam = None
    if sim_cam:
        cam = SimCamera(p, board_mm)
    elif cam_spec.lower() not in ("none", "off", "simulated", "sim") and reader:
        cam = Camera(ctx, cam_spec, p.get("width", 1280), p.get("height", 720))
    if cam is not None and not reader:
        ctx.log("No reader.pt among the models: the camera is not used, moves come from the controls.")
        cam = None
    mirror = None
    if sim:
        arm = TwinArm(ctx, p, sheet, place, realtime=float(p.get("sim_speed", 1)))
        ctx.log("Simulation: the digital twin of the xArm 6 (%s), pen %.0f mm, paper at z = %.0f mm, camera %s." % (
            "UFACTORY meshes" if arm_model else "no model connected", arm.pen_length, place.z, (cam.mount if sim_cam else cam_spec)))
        # the switch Real arm: the real xArm follows the twin. The switch is read whenever the game reads its
        # controls, and every 0.2 s while the twin moves; the game's other messages are kept for it.
        mirror = Mirror(ctx, p, say, arm)
        arm.mirror = mirror
        _messages, pending = ctx.messages, []

        def poll():
            for m in _messages(channel):
                if not mirror.take(m):
                    pending.append(m)

        def messages(ch=None, wait=0.0):
            out = list(pending)
            del pending[:]
            if not out:
                out = [m for m in _messages(ch, wait=wait) if not mirror.take(m)]
            return out
        ctx.messages = messages
        arm.poll = poll
    else:
        arm = TwinChecked(ctx, p)
    board_geo = Board(dict(p, board_size=board_mm))
    def send_scene():
        """What stands around the arm, for the 3D view: the table, the sheet (with the live picture of the paper in
        simulation), the camera on its stand and the pen."""
        ctx.telemetry("scene", scene_objects(place, sheet, board_mm, p, cam if sim_cam else SimCamera(p, board_mm), arm.q, sim))
    send_scene()

    perfect = Perfect()
    games_played, results = 0, []
    reads = {"reads": 0, "right": 0}

    def paper():
        if sheet is not None and sheet.changed:
            sheet.changed = False
            ctx.picture("paper", cv2_gray(sheet.img), width=360, quality=75)

    def scores_for(state):
        legal = [i for i in range(9) if state[i] == 0]
        if not player:
            return {}
        nb = np.array([tuple(state[:i]) + (me,) + tuple(state[i + 1:]) for i in legal], np.float32)
        with torch.no_grad():
            v = player[0](torch.from_numpy(features(nb, np.full(len(legal), me, np.float32))).to(dev)).squeeze(1).cpu().numpy()
        return dict(zip(legal, [float(x) for x in v]))

    def robot_move(state):
        best = perfect.best_moves(tuple(state), me)
        sc = scores_for(state)
        if mode == "perfect" or not sc:
            move, why = random.choice(best), "perfect play"
        else:
            move = max(sc, key=sc.get)
            why = "the network's choice (score %.2f)" % sc[move]
            if mode == "learned, checked" and move not in best:
                alt = max(best, key=lambda i: sc.get(i, -9))
                why = "the network preferred the %s, but the check found a better move" % NAMES[move]
                ctx.telemetry("check", {"network": move + 1, "played": alt + 1, "network_cell": NAMES[move], "played_cell": NAMES[alt]})
                move = alt
        mat = [[(round(sc[r * 3 + c], 3) if (r * 3 + c) in sc else "–") for c in range(3)] for r in range(3)]
        ctx.telemetry("thinking", {"matrix": mat, "move": move + 1, "cell": NAMES[move], "why": why})
        return move, why

    def show(state, note=""):
        rows = ["".join(SYM.get(state[r * 3 + c], "·") for c in range(3)) for r in range(3)]
        ctx.telemetry("board", {"board": rows, "matrix": [[state[r * 3 + c] for c in range(3)] for r in range(3)], "note": note,
                                "robot": robot_sym, "games": games_played})

    rest_at = tuple(_vec(p.get("park"), 3, (150.0, -250.0, 250.0)))

    def rest():
        """The rest position: the pen up and out of the way (and of a stand camera's view), reached through the
        joints. The arm starts there, waits there between moves and goes back there when it is done."""
        arm.travel(*rest_at)

    def look():
        """The arm at rest (camera on a stand) or holding the wrist camera over the board."""
        if str(p.get("camera_mount", "table")).lower().startswith("wrist"):
            arm.travel(*look_pose(p, board_mm, rpy))
        else:
            rest()

    def show_board():
        """A picture of the board after the robot's own move, so the camera view always shows the latest board."""
        if cam is not None and reader:
            try:
                read_once()
            except Exception as e:
                ctx.log("No picture of the board: %s" % e)

    def read_once():
        """(labels of the 9 cells as 1/-1/0 or None, probabilities, cells, reason) from one picture."""
        frame = cam.render(sheet, place, arm.q, arm_model, arm.pen_length, rng) if sim_cam else cam.grab()
        board, H, found = find_board(frame)
        if board is None:
            ctx.picture("camera", frame, bgr=True, note="board not found: " + found)
            return None, None, None, found
        cells = cut_cells(board)
        probs = read_cells(reader[0], cells, dev)
        lab = [[0, 1, -1][int(i)] for i in probs.argmax(1)]
        ctx.picture("camera", overlay(board, [SYM.get(v, "") for v in lab], probs), bgr=True)
        ctx.telemetry("read", {"matrix": [[round(float(probs[r * 3 + c].max()), 3) for c in range(3)] for r in range(3)]})
        return lab, probs, cells, ""

    real_cells, real_labels = [], []

    def keep_cells(cells, state):
        real_cells.append(cells)
        real_labels.append(np.array([{0: 0, 1: 1, -1: 2}[x] for x in state], np.uint8))

    def read_move(state, expect=None):
        """Reads the visitor's new mark: its cell, or None (and says why). In simulation `expect` is the cell the
        simulated hand drew: the reading is checked against it."""
        look()
        seen_board, reason = False, ""
        for attempt in range(4):
            lab, probs, cells, reason = read_once()
            if lab is None:
                time.sleep(0.3)
                continue
            seen_board = True
            new = [i for i in range(9) if state[i] == 0 and lab[i] == you and probs[i].max() >= float(p.get("sure", 0.7))]
            truth = list(state)
            if expect is not None:
                truth[expect] = you
                reads["reads"] += 1
                ok = new == [expect] and all(lab[i] == truth[i] for i in range(9))
                reads["right"] += int(ok)
                ctx.telemetry("reading", {"reads": reads["reads"], "right": reads["right"], "accuracy": round(reads["right"] / reads["reads"], 3)})
                if not ok:
                    wrong = [NAMES[i] for i in range(9) if lab[i] != truth[i]]
                    say("My reader made a mistake (%s): I go on with the cell you chose." % ", ".join(wrong))
                keep_cells(cells, truth)
                return expect
            if len(new) == 1:
                truth[new[0]] = you
                keep_cells(cells, truth)
                return new[0]
            if len(new) > 1:
                say("I see %d new marks: %s. Make one move, then press Your move again." % (len(new), ", ".join(NAMES[i] for i in new)))
                return None
            time.sleep(0.3)
        if seen_board:
            say("I do not see your %s yet. Draw it clearly inside a free cell, move your hand away, and press Your move." % SYM[you])
        else:
            say("I cannot find the board: keep the four corner markers visible (%s)." % reason)
        return None

    def hand(k):
        """The simulated visitor draws their mark in cell k (0-8)."""
        strokes, w, ink = hand_mark(SYM[you], k + 1, board_mm, rng)
        for st in strokes:
            sheet.stroke(st, w, ink)
        paper()

    def visitor_move(state):
        """The visitor's cell (0-8), or "new" / "stop"."""
        if auto and sim:
            time.sleep(min(1.5, 1.5 / max(0.1, float(p.get("sim_speed", 1)))))
            for m in ctx.messages(channel):
                c = str((m.get("data") or {}).get("control", "") if isinstance(m.get("data"), dict) else m.get("data")).strip().lower()
                if c in ("stop", "end"):
                    return "stop"
            free = [i for i in range(9) if state[i] == 0]
            k = random.choice(perfect.best_moves(tuple(state), you)) if visitor.startswith("perfect") else random.choice(free)
            hand(k)
            say("The simulated visitor played the %s." % NAMES[k])
            return read_move(state, k) if cam else k
        if sim:
            say("Your turn: choose the cell of your %s in the controls." % SYM[you])
        else:
            say("Your turn: draw an %s, then press Your move." % SYM[you] if cam else "Your turn: choose the cell of your %s." % SYM[you])
        last_seen, stable = None, 0
        while time.time() < deadline:
            for m in ctx.messages(channel, wait=0.5 if not p.get("watch") else 1.2):
                d = m.get("data") if isinstance(m.get("data"), dict) else {"control": str(m.get("data")), "value": True}
                c, v = str(d.get("control", "")).strip().lower(), d.get("value")
                if c in ("stop", "end"):
                    return "stop"
                if c in ("new game", "restart"):
                    return "new"
                if c in ("cell", "move") and str(v).strip().isdigit():
                    k = int(str(v).strip()) - 1
                    if not (0 <= k < 9) or state[k] != 0:
                        say("Cell %s is not free." % v)
                        continue
                    if sim:
                        hand(k)
                        if cam:
                            got = read_move(state, k)
                            if got is not None:
                                return got
                            continue
                    return k
                if c in ("your move", "done", "read") and cam and not sim:
                    got = read_move(state)
                    if got is not None:
                        return got
            if p.get("watch") and cam and not sim:
                lab, probs, cells, _ = read_once()
                if lab is not None:
                    new = tuple(i for i in range(9) if state[i] == 0 and lab[i] == you and probs[i].max() >= 0.9)
                    stable = stable + 1 if new and new == last_seen else 0
                    last_seen = new
                    if len(new) == 1 and stable >= 2:
                        truth = list(state); truth[new[0]] = you
                        keep_cells(cells, truth)
                        return new[0]
        return "stop"

    log_rows = []
    stopped = False
    try:
        say("Hello. I play %s%s." % (robot_sym, " (simulation)" if sim else ""))
        paper()
        rest()
        while time.time() < deadline:
            state = [0] * 9
            if sim and games_played:
                sheet.__init__(board_mm, float(p.get("marker_size", 25)), 4.0, 246, rng)  # a new sheet
                paper()
            show(state, "new game")
            send_scene()
            if p.get("draw_grid", True):
                draw_safe(arm, board_geo, figure(board_geo, "tic-tac-toe grid", 5))
                paper()
            look()
            show_board()
            first = starter(len(log_rows))
            turn = me if first == "robot" else you
            if not (auto and sim):
                say("I start this game." if first == "robot" else "You start this game.")
            moves, start, outcome = [], time.time(), None
            while outcome is None:
                if turn == me:
                    k, why = robot_move(state)
                    say("I play the %s." % NAMES[k], why=why)
                    ctx.telemetry("arm", {"drawing": robot_sym, "cell": k + 1})
                    draw_safe(arm, board_geo, figure(board_geo, robot_sym, k + 1))
                    paper()
                    look()
                    show_board()
                    send_scene()
                else:
                    k = visitor_move(state)
                    if k in ("stop", "new"):
                        outcome = k
                        break
                    if not (auto and sim):
                        say("You played the %s." % NAMES[k])
                state[k] = turn
                moves.append({"player": "robot" if turn == me else "visitor", "cell": k + 1, "t": round(time.time() - start, 1)})
                show(state)
                w = winner(tuple(state))
                if w:
                    outcome = "robot" if w == me else "visitor"
                elif 0 not in state:
                    outcome = "draw"
                turn = -turn
            if outcome == "stop":
                break
            if outcome != "new":
                games_played += 1
                results.append(outcome)
                say({"robot": "I win. Thank you for the game!", "visitor": "You win. Well played!", "draw": "A draw. Well played!"}[outcome])
                ctx.telemetry("result", {"games": games_played, "robot_wins": results.count("robot"), "draws": results.count("draw"), "visitor_wins": results.count("visitor")})
            log_rows.append({"game": games_played, "outcome": outcome, "first": first, "moves": moves, "robot": robot_sym, "player": mode, "simulation": sim, "seconds": round(time.time() - start, 1)})
            ctx.write_json("games", "game_%03d.json" % len(log_rows), log_rows[-1])
            if outcome == "new":
                continue
            if auto and sim:
                if games_played >= int(p.get("auto_games", 5)):
                    break
                continue
            if not p.get("keep_playing", True):
                break
            say("New sheet, please, then press New game. Press Stop to finish." if not sim else "Press New game for another one, or Stop.")
            got = None
            while got is None and time.time() < deadline:
                for m in ctx.messages(channel, wait=1):
                    d = m.get("data") if isinstance(m.get("data"), dict) else {"control": str(m.get("data"))}
                    c = str(d.get("control", "")).strip().lower()
                    if c in ("new game", "restart", "stop", "end"):
                        got = c
            if got in (None, "stop", "end"):
                break
    except KeyboardInterrupt:
        arm.stop()
        if mirror is not None:
            mirror.halt()
        stopped = True
        raise
    finally:
        if not stopped:
            try:
                rest()
                say("Done: the arm is back at rest.")
            except Exception as e:
                ctx.log("Could not go back to rest: %s" % e)
        if real_cells:
            np.savez_compressed(ctx.output("cells", "%s_cells_%d.npz" % ("sim" if sim else "game", int(time.time()))), x=np.concatenate(real_cells), y=np.concatenate(real_labels),
                                size=32, classes=np.array(["empty", "X", "O"]))
        if sheet is not None:
            import cv2
            cv2.imwrite(ctx.output("games", "paper_%d.png" % int(time.time())), sheet.img)
        if cam is not None and not sim_cam:
            cam.close()
        try:
            arm.report(force=True)
        finally:
            arm.close()
            if mirror is not None:
                mirror.close()
    say("Goodbye.")
    res = {"games": games_played, "robot_wins": results.count("robot"), "draws": results.count("draw"), "visitor_wins": results.count("visitor"),
           "robot": robot_sym, "player": mode, "simulation": sim}
    if reads["reads"]:
        res["reading_accuracy"] = round(reads["right"] / reads["reads"], 3)
    return res


def cv2_gray(img):
    import numpy as np
    return np.stack([img, img, img], -1)


class TwinChecked:
    """The real arm, every motion planned and checked on its digital twin first (the same rules as in simulation: the
    allowed space, no sudden swing of a joint, the speed and acceleration limits) and only then sent to the arm. The
    arm's controller is told the limits too (reduced mode: top speeds, and the safe box as a fence), so it enforces
    them by itself. After each move the real joints are compared with the twin's (live: twin_check)."""

    def __init__(self, ctx, p):
        self.real = Arm(ctx, False)
        self.twin = TwinArm(ctx, p, None, Place(p), realtime=0, log_name=None, report=False)
        self.ctx = ctx
        self.pen_length = self.twin.pen_length
        self.prev_reduced = None
        self._sync()
        self._limits()

    def __getattr__(self, name):
        return getattr(self.real, name)

    @property
    def q(self):
        return self.twin.q

    @property
    def pen(self):
        return self.real.pen

    @pen.setter
    def pen(self, down):
        self.real.pen = self.twin.pen = down

    def _sync(self):
        api = self.real.api
        code, angles = api.get_servo_angle()
        if code == 0 and angles:
            self.twin.q = [float(a) for a in angles[:6]]
            self.twin.pose[:3] = [float(v) for v in tool(self.twin.q, self.twin.tcp_z)[:3, 3]]

    def _limits(self):
        """The limits set in the arm's controller as well: top speeds of the tool point and of the joints, the safe box
        as a fence the tool point cannot cross, top accelerations, self-collision detection."""
        api, tw = self.real.api, self.twin
        b = tw.box
        done, failed = [], []

        def setting(name, fn):
            try:
                code = fn()
                (done if code in (0, None) else failed).append(name)
            except Exception:
                failed.append(name)
        try:
            code, st = api.get_reduced_states()
            if code == 0 and st:
                self.prev_reduced = st
        except Exception:
            pass
        setting("tool point at most %.0f mm/s" % tw.speed, lambda: api.set_reduced_max_tcp_speed(tw.speed))
        setting("joints at most %.0f°/s" % tw.joint_speed, lambda: api.set_reduced_max_joint_speed(tw.joint_speed))
        setting("the safe box as a fence", lambda: api.set_reduced_tcp_boundary([b[1], b[0], b[3], b[2], b[5], b[4]]))
        setting("fence on", lambda: api.set_fence_mode(True))
        setting("reduced mode on", lambda: api.set_reduced_mode(True))
        setting("tool point acceleration at most %.0f mm/s²" % tw.accel, lambda: api.set_tcp_maxacc(tw.accel))
        setting("joint acceleration at most %.0f°/s²" % tw.joint_accel, lambda: api.set_joint_maxacc(tw.joint_accel))
        setting("self-collision detection", lambda: api.set_self_collision_detection(True))
        self.ctx.log("Limits set in the arm's controller: " + (", ".join(done) or "none") + ".")
        if failed:
            self.ctx.log("Not accepted by this controller (the twin still checks every move): " + ", ".join(failed) + ".")
        self.ctx.telemetry("limits", {"set": done, "not_set": failed})
        try:
            if api.warn_code:
                api.clean_warn()  # older firmware refuses some settings with a warning, which would hold up the next moves
        except Exception:
            pass

    def _check_arm(self):
        api = self.real.api
        if api.error_code:
            raise Stop("The arm reports error %s: it stopped. Check it and clear the error in UFACTORY's app." % api.error_code)

    def _compare(self, target):
        import numpy as np
        code, angles = self.real.api.get_servo_angle()
        if code != 0 or not angles:
            return
        real_q = np.array(angles[:6], float)
        tq = np.array(self.twin.q)
        fk_tcp = tool(real_q, self.twin.tcp_z)[:3, 3]
        self.ctx.telemetry("twin", {"j%d" % (i + 1): round(float(a), 2) for i, a in enumerate(tq)})
        self.ctx.telemetry("twin_check", {"joints_deg": round(float(np.max(np.abs(real_q - tq))), 3),
                                          "model_mm": round(float(np.linalg.norm(fk_tcp - np.array(self.real.pose[:3]))), 3),
                                          "target_mm": round(float(np.linalg.norm(np.array(self.real.pose[:3]) - np.array(target))), 3)})

    def stroke(self, points, speed=None):
        """Straight lines of the tool point: planned and checked on the twin, then sent as one blended motion."""
        self.twin.stroke(points, speed)
        real_stroke(self.real, points, min(speed or self.twin.speed, self.twin.speed), self.twin.accel)
        self._check_arm()
        self._compare(points[-1])

    def move(self, x, y, z, wait=True):
        self.stroke([(x, y, z)])

    def travel(self, x, y, z):
        """A joint move: the twin plans and checks it, and the arm gets the twin's joint angles, after its own
        kinematics confirms they put the tool point where it should be."""
        self.twin.travel(x, y, z)
        q = self.twin.planned
        api = self.real.api
        try:
            code, pose = api.get_forward_kinematics(q, input_is_radian=False, return_is_radian=False)
        except Exception:
            code, pose = -1, None
        if code == 0 and pose:
            off = math.dist(pose[:3], (x, y, z))
            if off > 3.0:
                raise Stop("The arm's controller puts these joint angles %.1f mm away from %.0f, %.0f, %.0f mm (check the TCP setting and the pen length): nothing was sent." % (off, x, y, z))
        self.real.ctx.telemetry("target", {"x": round(x, 2), "y": round(y, 2), "z": round(z, 2), "joint_move": True})
        code = api.set_servo_angle(angle=[float(a) for a in q], speed=max(1.0, self.twin.last_joint_speed), mvacc=max(5.0, self.twin.last_joint_accel),
                                   is_radian=False, wait=True)
        if code != 0:
            raise RuntimeError("The xArm refused the joint move (code %s, error %s, warning %s)." % (code, api.error_code, api.warn_code))
        c, pos = api.get_position()
        if c == 0 and pos:
            self.real.pose = [float(v) for v in pos[:6]]
        self.real.report(force=True)
        self._check_arm()
        self._compare((x, y, z))

    def home(self):
        self.travel(*self.twin.pose[:3])

    def close(self):
        api = self.real.api
        if self.prev_reduced is not None and api is not None:
            try:  # the controller as it was before this session
                api.set_reduced_mode(bool(self.prev_reduced[0]))
                if len(self.prev_reduced) > 5:
                    api.set_fence_mode(bool(self.prev_reduced[5]))
            except Exception:
                pass
        self.real.close()


def real_stroke(real, points, speed, accel):
    """The real arm through `points` in straight lines, blended at the corners (radius 1 mm) so it does not stop at
    each one; the last point waits for the arm."""
    api = real.api
    for x, y, z in points:
        real.check(x, y, z)
    for k, (x, y, z) in enumerate(points):
        last = k == len(points) - 1
        code = api.set_position(x=x, y=y, z=z, roll=real.rpy[0], pitch=real.rpy[1], yaw=real.rpy[2], speed=speed, mvacc=accel,
                                radius=-1 if last else 1.0, wait=last)
        if code != 0:
            raise RuntimeError("The xArm refused the move (code %s, error %s, warning %s)." % (code, api.error_code, api.warn_code))
    c, pos = api.get_position()
    if c == 0 and pos:
        real.pose = [float(v) for v in pos[:6]]
    real.log.writerow([round(time.time(), 3)] + [round(v, 2) for v in real.pose] + [int(real.pen), 0])
    real.report(force=True)


def draw_safe(arm, board, strokes):
    """The strokes of a figure with the pen: to above the first stroke through the joints (from rest), the pen down
    in a straight line, each stroke one smooth motion, the pen up, and straight to the next stroke above the paper.
    Every motion is planned and checked before it starts."""
    for k, stroke in enumerate(strokes):
        pts = [board.to_arm(float(a), float(b)) for a, b in stroke]
        if not pts:
            continue
        x, y = pts[0]
        if k == 0:
            arm.travel(x, y, board.z_up)
        else:
            arm.move(x, y, board.z_up)
        arm.pen = True
        arm.move(x, y, board.z_draw)
        arm.stroke([(px, py, board.z_draw) for px, py in pts[1:]])
        arm.pen = False
        x, y = pts[-1]
        arm.move(x, y, board.z_up)
        arm.ctx.progress(100 * (k + 1) / len(strokes), "stroke %d of %d" % (k + 1, len(strokes)))


def scene_objects(place, sheet, board_mm, p, cam=None, q=None, sim=True):
    """The objects around the arm for the 3D view (its scene channel), in mm, z up: the table it stands on, the sheet
    (showing the live picture of the paper in simulation), the camera on its stand and the pen."""
    z = place.z
    objs = [{"id": "table", "kind": "box", "center": [300, 0, z - 15], "size": [1200, 900, 30], "color": "#b79b7a", "fit": False}]
    for lx in (-280, 880):
        for ly in (-430, 430):
            objs.append({"id": "leg", "kind": "box", "center": [lx, ly, z - 30 - 355], "size": [40, 40, 710], "color": "#6b5a48", "fit": False})
    if sheet is not None:
        objs.append({"id": "paper", "kind": "quad", "corners": [list(place.world(bx, by)) for bx, by in sheet.page_corners_board()], "image_channel": "paper"})
    else:
        s = board_mm
        objs.append({"id": "board", "kind": "quad", "corners": [list(place.world(bx, by)) for bx, by in ((0, s), (s, s), (s, 0), (0, 0))], "color": "#f4f4f2"})
    if cam is not None and getattr(cam, "mount", "table") == "table":
        c = cam.info(q)
        reach = 0.8 * (math.dist(c["pos"], getattr(cam, "look", c["pos"])) or 300.0)  # the view drawn most of the way to what it looks at
        objs.append({"id": "camera", "kind": "camera", "pos": c["pos"], "forward": c["forward"], "up": c["up"], "hfov": c["hfov"],
                     "aspect": c["aspect"], "stand_to": z, "frustum": round(reach)})
    objs.append({"id": "pen", "kind": "tool", "length": float(p.get("pen_length", 100)), "radius": 6})
    return {"units": "mm", "up": "z", "floor": z - 740, "simulation": bool(sim), "objects": objs}


# ---- shared with the other blocks of the tic-tac-toe demo ----

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


CELL_PX = 96   # each cell of the straightened board, in pixels


MARGIN = 0.10  # share of a cell cut away at its edges (the grid lines)


def find_board(img):
    """The board in a picture (BGR or gray): (straightened gray board 3*CELL_PX square, homography, found ids) or
    (None, None, reason). Markers: ids 0 top left, 1 top right, 2 bottom right, 3 bottom left, each touching the board
    with its inner corner."""
    import cv2
    import numpy as np
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, "ArucoDetector"):
        corners, ids, _ = cv2.aruco.ArucoDetector(dic, cv2.aruco.DetectorParameters()).detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dic)
    W = 3 * CELL_PX
    inner = {0: 2, 1: 3, 2: 0, 3: 1}  # which corner of each marker touches the board (TL, TR, BR, BL order)
    dest = {0: (0, 0), 1: (W, 0), 2: (W, W), 3: (0, W)}
    src, dst, found = [], [], []
    for c, i in zip(corners if ids is not None else [], ids.flatten() if ids is not None else []):
        i = int(i)
        if i in inner and i not in found:
            src.append(c[0][inner[i]]); dst.append(dest[i]); found.append(i)
    if len(found) == 4:
        H = cv2.getPerspectiveTransform(np.float32(src), np.float32(dst))
    elif len(found) == 3:  # one marker covered: a flat board seen from above is close to an affine picture
        A = cv2.getAffineTransform(np.float32(src), np.float32(dst))
        H = np.vstack([A, [0, 0, 1]])
    else:
        return None, None, "found %d of the 4 board markers%s" % (len(found), (" (" + ", ".join(map(str, sorted(found))) + ")") if found else "")
    board = cv2.warpPerspective(gray, H, (W, W), flags=cv2.INTER_AREA)
    return board, H, sorted(found)


def cut_cells(board):
    """Nine 32x32 crops (uint8), row by row from the top left, without the grid lines at their edges."""
    import cv2
    import numpy as np
    out = np.zeros((9, 32, 32), np.uint8)
    m = int(CELL_PX * MARGIN)
    for k in range(9):
        r, c = divmod(k, 3)
        crop = board[r * CELL_PX + m:(r + 1) * CELL_PX - m, c * CELL_PX + m:(c + 1) * CELL_PX - m]
        out[k] = cv2.resize(crop, (32, 32), interpolation=cv2.INTER_AREA)
    return out


def read_cells(reader, cells, device="cpu"):
    """Probabilities 9 x 3 (empty, X, O) for the nine crops."""
    import torch
    x = torch.from_numpy(normalize(cells)).unsqueeze(1).to(device)
    with torch.no_grad():
        return torch.softmax(reader(x), 1).cpu().numpy()


def load_models(ctx, port, device="cpu"):
    """The newest reader.pt and player.pt among the files of `port`: {"reader": (model, meta), "player": (model, meta)}."""
    import torch
    import torch.nn as nn
    got = {}
    files = sorted(ctx.inputs.get(port, []), key=lambda f: os.path.getmtime(f.path))
    for f in files:
        name = f.name.lower()
        if name.endswith("reader.pt") or name.endswith("player.pt"):
            d = torch.load(f.path, map_location="cpu", weights_only=False)
            kind = "reader" if name.endswith("reader.pt") else "player"
            m = make_reader(nn) if kind == "reader" else make_player(nn)
            m.load_state_dict(d["state"])
            m.eval().to(device)
            got[kind] = (m, dict(d.get("meta") or {}, file=f.rel or f.name))
    return got


def overlay(board, labels, probs=None, title=""):
    """The straightened board in colour, with what was read in each cell."""
    import cv2
    img = cv2.cvtColor(board, cv2.COLOR_GRAY2BGR)
    for k in range(9):
        r, c = divmod(k, 3)
        x, y = c * CELL_PX, r * CELL_PX
        cv2.rectangle(img, (x + 2, y + 2), (x + CELL_PX - 3, y + CELL_PX - 3), (200, 200, 200), 1)
        lab = labels[k]
        if lab:
            col = (40, 40, 220) if lab == "X" else (220, 120, 30)
            cv2.putText(img, lab, (x + 8, y + 26), cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2, cv2.LINE_AA)
        if probs is not None:
            cv2.putText(img, "%d%%" % round(100 * float(probs[k].max())), (x + 8, y + CELL_PX - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (90, 160, 90), 1, cv2.LINE_AA)
    if title:
        cv2.putText(img, title, (6, 3 * CELL_PX - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
    return img


class Camera:
    """A camera of the machine (OpenCV), kept open; grab() gives a fresh frame."""

    def __init__(self, ctx, spec, width=1280, height=720):
        import cv2
        spec = str(spec if spec is not None else "0").strip()
        if spec.lstrip("-").isdigit():
            idx = int(spec)
            self.cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW) if sys.platform == "win32" else cv2.VideoCapture(idx)
        else:
            self.cap = cv2.VideoCapture(spec)
        if not self.cap or not self.cap.isOpened():
            raise RuntimeError("Cannot open the camera %r on this machine: check that it is plugged in and not used by another program%s."
                               % (spec, "; a worker running as a Windows service may not be allowed to use cameras (run it with esasim-worker run)" if sys.platform == "win32" else ""))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
        self.file = not spec.lstrip("-").isdigit() and not spec.startswith(("rtsp:", "http:", "https:"))
        for _ in range(10):  # exposure settles over the first frames
            self.cap.read()

    def grab(self):
        if not self.file:
            for _ in range(3):  # frames waiting in the driver's buffer are old
                self.cap.grab()
        ok, frame = self.cap.read()
        if (not ok or frame is None) and self.file:
            import cv2
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
        if not ok or frame is None:
            raise RuntimeError("The camera gives no picture.")
        return frame

    def close(self):
        self.cap.release()


XARM6_JOINTS = [((0.0, 0.0, 267.0), (0.0, 0.0, 0.0)), ((0.0, 0.0, 0.0), (-1.5708, 0.0, 0.0)), ((53.5, -284.5, 0.0), (0.0, 0.0, 0.0)),
                ((77.5, 342.5, 0.0), (-1.5708, 0.0, 0.0)), ((0.0, 0.0, 0.0), (1.5708, 0.0, 0.0)), ((76.0, 97.0, 0.0), (-1.5708, 0.0, 0.0))]


XARM6_LIMITS = [(-178.2, 178.2), (-118.0, 120.0), (-178.2, 11.0), (-178.2, 178.2), (-97.0, 178.2), (-178.2, 178.2)]


def _rpy(r, p, y):
    import numpy as np
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def rpy_matrix(roll, pitch, yaw):
    """xArm's orientation (degrees): R = Rz(yaw) Ry(pitch) Rx(roll)."""
    return _rpy(math.radians(roll), math.radians(pitch), math.radians(yaw))


def fk(q_deg):
    """Frames (4x4, mm) of the base and links 1-6 for joint angles in degrees, and the joint frames (axis = their z)."""
    import numpy as np
    T = np.eye(4)
    links, joints = [T.copy()], []
    for (xyz, rpy), qi in zip(XARM6_JOINTS, q_deg):
        O = np.eye(4)
        O[:3, :3] = _rpy(*rpy)
        O[:3, 3] = xyz
        T = T @ O
        joints.append(T.copy())
        c, s = math.cos(math.radians(qi)), math.sin(math.radians(qi))
        Rz = np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])
        T = T @ Rz
        links.append(T.copy())
    return links, joints


def tool(q_deg, tool_z=0.0):
    import numpy as np
    T = fk(q_deg)[0][6]
    return T @ np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, tool_z], [0, 0, 0, 1.0]])


def ik(pos, rpy_deg, q0, tool_z=0.0, iters=80):
    """Joint angles (degrees) putting the tool point at pos (mm) with orientation rpy (degrees), starting from q0;
    damped least squares within the joint limits. Returns (q, position error mm, orientation error degrees)."""
    import numpy as np
    target = np.asarray(pos, float)
    Rt = rpy_matrix(*rpy_deg)
    q = np.radians(np.asarray(q0, float))
    lo, hi = np.radians([l for l, _ in XARM6_LIMITS]), np.radians([h for _, h in XARM6_LIMITS])
    W = 200.0  # mm per radian: how orientation errors weigh against position errors
    ep, ew = 1e9, 1e9
    for _ in range(iters):
        links, joints = fk(np.degrees(q))
        T = links[6] @ np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, tool_z], [0, 0, 0, 1.0]])
        p = T[:3, 3]
        e_p = target - p
        Re = Rt @ T[:3, :3].T
        ang = math.acos(max(-1.0, min(1.0, (np.trace(Re) - 1) / 2)))
        if ang < 1e-9:
            e_w = np.zeros(3)
        else:
            axis = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]]) / (2 * math.sin(ang)) if ang < math.pi - 1e-6 else _axis_pi(Re)
            e_w = axis * ang
        ep, ew = float(np.linalg.norm(e_p)), math.degrees(float(np.linalg.norm(e_w)))
        if ep < 0.02 and ew < 0.01:
            break
        J = np.zeros((6, 6))
        for i, Jf in enumerate(joints):
            z, o = Jf[:3, 2], Jf[:3, 3]
            J[:3, i] = np.cross(z, p - o)
            J[3:, i] = z * W
        e = np.concatenate([e_p, e_w * W])
        lam = 2.0 + min(50.0, ep * 0.05)
        dq = J.T @ np.linalg.solve(J @ J.T + lam * lam * np.eye(6), e)
        # pointing down, joints 4 and 6 turn the same way (a wrist singularity): keep joint 4 near 0, like the arm does
        N = np.eye(6) - np.linalg.pinv(J, rcond=1e-3) @ J
        dq += N @ np.array([0, 0, 0, -0.5 * q[3], 0, 0])
        m = float(np.max(np.abs(dq)))
        if m > 0.25:
            dq *= 0.25 / m
        q = np.clip(q + dq, lo, hi)
    return np.degrees(q), ep, ew


def _axis_pi(R):
    import numpy as np
    k = int(np.argmax(np.diag(R)))
    v = R[:, k] + (np.eye(3)[:, k])
    return v / (np.linalg.norm(v) or 1)


def decimate(tris, cell):
    """Vertex clustering: vertices closer than `cell` merge, triangles that collapse go. Returns (vertices, faces)."""
    import numpy as np
    v = tris.reshape(-1, 3).astype(np.float64)
    keys = np.floor(v / cell).astype(np.int64)
    uniq, inv = np.unique(keys, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    sums = np.zeros((len(uniq), 3)); np.add.at(sums, inv, v)
    cnt = np.bincount(inv, minlength=len(uniq))[:, None]
    verts = sums / cnt
    f = inv.reshape(-1, 3)
    f = f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])]
    if len(f):
        _, idx = np.unique(np.sort(f, axis=1), axis=0, return_index=True)
        f = f[np.sort(idx)]
    used = np.unique(f)
    remap = -np.ones(len(verts), np.int64); remap[used] = np.arange(len(used))
    return verts[used], remap[f]


def load_model(files, cell=None):
    """The model (esasim.robot/1) among input files (.json or .json.gz), as numpy arrays per link (merged again into
    `cell` mm when given: a lighter model to draw in the simulated camera); None if absent."""
    import gzip
    import json
    import numpy as np
    for f in files or []:
        n = f.name.lower()
        if n.endswith(".robot.json") or n.endswith(".robot.json.gz"):
            raw = open(f.path, "rb").read()
            d = json.loads(gzip.decompress(raw) if n.endswith(".gz") else raw)
            s = float(d.get("scale", 1))
            out = []
            for l in d["links"]:
                v, fc = np.array(l["v"], np.float64).reshape(-1, 3) * s, np.array(l["f"], np.int64).reshape(-1, 3)
                out.append(decimate(v[fc], cell) if cell else (v, fc))
            return out
    return None


class Sheet:
    """The A4 sheet of Board sheet (markers ids 0-3 touching the board's corners), in pixels (res per mm), with what
    has been drawn on it. Board coordinates: mm from the board's lower-left corner, y towards the top of the page."""
    W_MM, H_MM = 210.0, 297.0

    def __init__(self, board_mm=140.0, marker_mm=25.0, res=4.0, paper=246, rng=None):
        import cv2
        import numpy as np
        self.board, self.marker, self.res = float(board_mm), float(marker_mm), float(res)
        self.bx0 = (self.W_MM - self.board) / 2
        self.by_top = 30 + self.marker
        self.by_bottom = self.by_top + self.board
        H, W = int(round(self.H_MM * res)), int(round(self.W_MM * res))
        self.img = np.full((H, W), paper, np.uint8)
        if rng is not None:  # the grain of the paper
            self.img = np.clip(self.img.astype(np.int16) + rng.normal(0, 2.0, (H, W)).astype(np.int16), 0, 255).astype(np.uint8)
        px = lambda mm: int(round(mm * res))
        cv2.rectangle(self.img, (px(self.bx0), px(self.by_top)), (px(self.bx0 + self.board), px(self.by_bottom)), int(paper * 0.82), max(1, px(0.3)))
        dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        m = px(self.marker)
        x0, y0, x1, y1 = px(self.bx0), px(self.by_top), px(self.bx0 + self.board), px(self.by_bottom)
        for mid, (mx, my) in {0: (x0 - m, y0 - m), 1: (x1, y0 - m), 2: (x1, y1), 3: (x0 - m, y1)}.items():
            mk = cv2.aruco.generateImageMarker(dic, mid, m, borderBits=1)
            self.img[my:my + m, mx:mx + m] = np.where(mk > 127, paper, 18).astype(np.uint8)
        self.changed = True

    def to_px(self, bx, by):
        return ((self.bx0 + bx) * self.res, (self.by_bottom - by) * self.res)

    def page_corners_board(self):
        """The page's corners (top left, top right, bottom right, bottom left) in board coordinates."""
        return [(-self.bx0, self.by_bottom), (self.W_MM - self.bx0, self.by_bottom), (self.W_MM - self.bx0, self.by_bottom - self.H_MM), (-self.bx0, self.by_bottom - self.H_MM)]

    def stroke(self, pts, width_mm=1.2, ink=25):
        import cv2
        import numpy as np
        if len(pts) < 2:
            return
        p = np.array([self.to_px(x, y) for x, y in pts]) * 16
        cv2.polylines(self.img, [p.astype(np.int32)], False, int(ink), max(1, int(round(width_mm * self.res))), cv2.LINE_AA, shift=4)
        self.changed = True


def hand_mark(sym, cell, board_mm, rng):
    """A visitor's X or O in a cell (1-9 from the top left), as a hand draws it: strokes in board mm, width, ink."""
    w = board_mm / 3
    n = int(cell) - 1
    col, row = n % 3, n // 3
    cx = col * w + w / 2 + rng.normal(0, w * 0.05)
    cy = board_mm - (row * w + w / 2) + rng.normal(0, w * 0.05)
    size = w * rng.uniform(0.22, 0.36)
    strokes = []
    if sym == "X":
        a = rng.normal(math.pi / 4, 0.18)
        for k in range(2):
            ang = a + k * (math.pi / 2 + rng.normal(0, 0.15))
            r1, r2 = size * (1 + rng.normal(0, 0.08)), size * (1 + rng.normal(0, 0.08))
            bend = rng.normal(0, size * 0.06)
            pts = []
            for i in range(13):
                t = i / 12
                x = cx - r1 * math.cos(ang) + (r1 + r2) * math.cos(ang) * t
                y = cy - r1 * math.sin(ang) + (r1 + r2) * math.sin(ang) * t
                off = bend * math.sin(math.pi * t)
                pts.append((x - math.sin(ang) * off, y + math.cos(ang) * off))
            strokes.append(pts)
    else:
        rx, ry = size * (1 + rng.normal(0, 0.06)), size * rng.uniform(0.8, 1.15)
        tilt, start, span = rng.uniform(0, math.pi), rng.uniform(0, 2 * math.pi), 2 * math.pi * rng.uniform(0.92, 1.08)
        pts = []
        for i in range(41):
            t = start + span * i / 40
            wob = 1 + rng.normal(0, 0.02)
            x, y = rx * wob * math.cos(t), ry * wob * math.sin(t)
            pts.append((cx + x * math.cos(tilt) - y * math.sin(tilt), cy + x * math.sin(tilt) + y * math.cos(tilt)))
        strokes.append(pts)
    return strokes, rng.uniform(0.9, 2.2), int(rng.integers(10, 80))


class Place:
    """The board on the table, in the arm's base frame: lower-left corner (marker 3 side) at board_x, board_y,
    turned by board_rotation, on the table at table_z."""

    def __init__(self, p):
        self.x0, self.y0 = float(p.get("board_x", 390)), float(p.get("board_y", -70))
        self.rot = math.radians(float(p.get("board_rotation", 90)))
        self.z = float(p.get("table_z", 0))

    def world(self, bx, by):
        c, s = math.cos(self.rot), math.sin(self.rot)
        return (self.x0 + bx * c - by * s, self.y0 + bx * s + by * c, self.z)

    def board_of(self, x, y):
        c, s = math.cos(self.rot), math.sin(self.rot)
        dx, dy = x - self.x0, y - self.y0
        return (dx * c + dy * s, -dx * s + dy * c)


def _vec(text, n, default):
    try:
        v = [float(x) for x in str(text).replace(";", ",").split(",") if x.strip() != ""]
        return v if len(v) == n else list(default)
    except ValueError:
        return list(default)


class SimCamera:
    """A pinhole camera on a stand above the table ("table") or on the arm's wrist ("wrist"), with the lens, light and
    noise of a webcam. render() draws the table, the sheet and the arm as the camera sees them."""

    def __init__(self, p, board_mm=140.0, rng=None):
        self.mount = "wrist" if str(p.get("camera_mount", "table")).lower().startswith("wrist") else "table"
        self.W, self.H = int(p.get("sim_width", 1280)), int(p.get("sim_height", 720))
        self.hfov = float(p.get("camera_hfov", 70))
        place = Place(p)
        centre = place.world(board_mm / 2, board_mm / 2)
        self.pos = _vec(p.get("camera_pos"), 3, (centre[0] + 200, centre[1], place.z + 450))
        self.look = _vec(p.get("camera_look"), 3, centre)
        self.wrist = _vec(p.get("wrist_camera"), 3, (70.0, 0.0, -40.0))  # mm in the flange frame (x, y, z)
        self.tilt = float(p.get("wrist_tilt", 0))
        if rng is not None:  # another stand, another lens: each simulated picture differs a little
            self.pos = [v + rng.normal(0, 25) for v in self.pos]
            self.look = [v + rng.normal(0, 12) for v in self.look]
            self.hfov += rng.normal(0, 4)
        self.f = (self.W / 2) / math.tan(math.radians(self.hfov) / 2)

    def frame(self, q=None):
        """Rotation world->camera (rows: x right, y down, z forward) and the camera's position."""
        import numpy as np
        if self.mount == "wrist" and q is not None:
            T6 = fk(q)[0][6]
            c = T6[:3, :3] @ np.array(self.wrist) + T6[:3, 3]
            t = math.radians(self.tilt)  # looking along the flange's z (down), tilted towards the pen
            fwd = T6[:3, :3] @ np.array([-math.sin(t), 0, math.cos(t)])
            up_hint = T6[:3, :3] @ np.array([1.0, 0, 0])
        else:
            c = np.array(self.pos, float)
            fwd = np.array(self.look, float) - c
            up_hint = np.array([0, 0, 1.0])
        z = fwd / np.linalg.norm(fwd)
        x = np.cross(z, up_hint)
        if np.linalg.norm(x) < 1e-6:
            x = np.cross(z, [1.0, 0, 0])
        x /= np.linalg.norm(x)
        y = np.cross(z, x)
        return np.stack([x, y, z]), c

    def project(self, P, R, c):
        import numpy as np
        X = (np.asarray(P, float) - c) @ R.T
        d = X[:, 2]
        uv = np.stack([self.f * X[:, 0] / np.maximum(d, 1e-6) + self.W / 2, self.f * X[:, 1] / np.maximum(d, 1e-6) + self.H / 2], 1)
        return uv, d

    def info(self, q=None):
        R, c = self.frame(q)
        return {"mount": self.mount, "pos": [round(float(v), 1) for v in c], "forward": [round(float(v), 4) for v in R[2]],
                "up": [round(float(v), 4) for v in -R[1]], "hfov": round(self.hfov, 1), "aspect": round(self.W / self.H, 4)}

    def render(self, sheet, place, q=None, model=None, pen_length=0.0, rng=None):
        import cv2
        import numpy as np
        rng = rng or np.random.default_rng()
        R, c = self.frame(q)
        W, H = self.W, self.H
        # the table: a plain top with a little texture
        base = np.array([rng.uniform(95, 140), rng.uniform(110, 150), rng.uniform(120, 165)])
        img = np.empty((H, W, 3), np.float32)
        img[:] = base
        img += rng.standard_normal((H, W, 1), dtype=np.float32) * 3
        # the sheet: its corners projected, the picture of the paper warped there
        corners = [place.world(bx, by) for bx, by in sheet.page_corners_board()]
        uv, d = self.project(corners, R, c)
        if (d > 1).all():
            h, w = sheet.img.shape
            Hm = cv2.getPerspectiveTransform(np.float32([[0, 0], [w, 0], [w, h], [0, h]]), np.float32(uv))
            paper = cv2.warpPerspective(sheet.img, Hm, (W, H), flags=cv2.INTER_AREA, borderValue=0).astype(np.float32)
            mask = cv2.warpPerspective(np.full((h, w), 255, np.uint8), Hm, (W, H), flags=cv2.INTER_LINEAR, borderValue=0).astype(np.float32)[..., None] / 255
            tint = (np.array([0.97, 1.0, 1.02]) * rng.uniform(0.92, 1.05)).astype(np.float32)
            img = img * (1 - mask) + paper[..., None] * tint * mask
        # the arm, nearest parts drawn last
        if q is not None:
            img = self._draw_arm(img, q, model, R, c, pen_length, rng)
        # light: brighter on one side, darker in the corners
        if getattr(self, "_grid", None) is None:
            yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
            self._grid = ((xx / W - 0.5) * 2, (yy / H - 0.5) * 2)
        gx, gy = self._grid
        light = ((1 + np.float32(rng.normal(0, 0.12)) * gx + np.float32(rng.normal(0, 0.12)) * gy) * np.float32(rng.uniform(0.75, 1.1))
                 * (1 - np.float32(0.18 * rng.uniform(0.3, 1)) * (gx * gx + gy * gy)))
        img *= light[..., None]
        k = rng.uniform(0, 1.3)
        if k > 0.3:
            img = cv2.GaussianBlur(img, (0, 0), k)
        img += rng.standard_normal(img.shape, dtype=np.float32) * np.float32(rng.uniform(1.5, 6))
        out = np.clip(img, 0, 255).astype(np.uint8)
        ok, jpg = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, int(rng.uniform(70, 95))])
        return cv2.imdecode(jpg, cv2.IMREAD_COLOR) if ok else out

    def _draw_arm(self, img, q, model, R, c, pen_length, rng):
        import cv2
        import numpy as np
        links = fk(q)[0]
        Lw = np.array([0.3, -0.4, 0.87]); Lw /= np.linalg.norm(Lw)
        polys = []
        if model:
            for i, (v, f) in enumerate(model):
                T = links[i]
                vw = v @ T[:3, :3].T + T[:3, 3]
                uv, d = self.project(vw, R, c)
                tri = vw[f]
                n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
                n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-9
                shade = 0.35 + 0.65 * np.abs(n @ Lw)
                dd = d[f]
                u = uv[f]
                # only triangles in front of the camera, facing it, and inside the picture
                facing = np.einsum("ij,ij->i", n, tri.mean(1) - c) < 0
                inside = (u[:, :, 0].max(1) > 0) & (u[:, :, 0].min(1) < self.W) & (u[:, :, 1].max(1) > 0) & (u[:, :, 1].min(1) < self.H)
                keep = (dd > 20).all(1) & facing & inside
                col = 205.0 if i else 150.0
                for k in np.nonzero(keep)[0]:
                    polys.append((float(dd[k].mean()), uv[f[k]], col * shade[k]))
        else:  # no model: the links as rounded bars between the joints
            pts = [T[:3, 3] for T in links]
            for a, b in zip(pts[:-1], pts[1:]):
                for t0 in np.linspace(0, 1, 6)[:-1]:
                    p0, p1 = a + (b - a) * t0, a + (b - a) * (t0 + 0.2)
                    uv, d = self.project([p0, p1], R, c)
                    if (d > 20).all():
                        polys.append((float(d.mean()), ("bar", uv, 45 * self.f / d.mean()), 200.0))
        # the pen
        T6 = links[6]
        tip = T6[:3, :3] @ np.array([0, 0, pen_length]) + T6[:3, 3]
        uv, d = self.project([T6[:3, 3], tip], R, c)
        if (d > 20).all() and pen_length > 0:
            polys.append((float(d.mean()) - 1, ("bar", uv, 12 * self.f / d.mean()), 40.0))
        polys.sort(key=lambda x: -x[0])
        canvas = img
        for _, poly, col in polys:
            if isinstance(poly, tuple):
                _, uv, w = poly
                cv2.line(canvas, tuple(int(v) for v in uv[0]), tuple(int(v) for v in uv[1]), (col, col, col), max(1, int(w)), cv2.LINE_AA)
            else:
                cv2.fillConvexPoly(canvas, np.round(poly * 4).astype(np.int32), (col, col, col * 1.02), cv2.LINE_AA, shift=2)
        return canvas


MAX_TCP_SPEED = 150.0


MAX_TCP_ACC = 800.0


MAX_JOINT_SPEED = 60.0


MAX_JOINT_ACC = 120.0


JOINT_MARGIN = 3.0        # degrees kept from each joint's end stop


STEP_TURN = 2.5           # most any joint may turn for 1 mm of the pen's path: more means a singular pose (a sudden swing)


MIN_RADIUS = 150.0        # the tool point stays this far from the base's axis


def trapezoid(dist, vmax, amax):
    """Time and position along a move of `dist` that starts and ends at rest: accelerating at amax, cruising at
    vmax, slowing at amax (a triangle when the move is short). Returns (duration, s(t))."""
    dist, vmax, amax = max(0.0, float(dist)), max(1e-6, float(vmax)), max(1e-6, float(amax))
    ta = vmax / amax
    if amax * ta * ta >= dist:  # never reaches vmax
        ta = math.sqrt(dist / amax) if dist > 0 else 0.0
        vp, tc = amax * ta, 0.0
    else:
        vp, tc = vmax, (dist - amax * ta * ta) / vmax
    T = 2 * ta + tc

    def s(t):
        t = min(max(t, 0.0), T)
        if t < ta:
            return 0.5 * amax * t * t
        if t < ta + tc:
            return 0.5 * amax * ta * ta + vp * (t - ta)
        d = T - t
        return dist - 0.5 * amax * d * d
    return T, s


def _minjerk(u):
    u = min(1.0, max(0.0, u))
    return u * u * u * (10.0 - 15.0 * u + 6.0 * u * u)


class Mirror:
    """The real xArm following the digital twin, switched on and off with the switch Real arm. On: it connects to
    the arm at Arm IP address, comes gently to the twin's pose (minimum jerk, at most 20°/s), then gets every pose
    of the twin (servo mode). A jump faster than 60°/s becomes a gentle catch-up move instead. Off, or on any error
    of the arm: it holds where it is."""
    MAX_SPEED = 60.0
    MOVE_SPEED = 20.0

    def __init__(self, ctx, p, say, arm):
        self.ctx, self.p, self.say, self.arm = ctx, p, say, arm
        self.api, self.active, self.last, self.t_last = None, False, None, 0.0

    def take(self, m):
        """True if the message is the switch Real arm (and acts on it)."""
        d = m.get("data") if isinstance(m.get("data"), dict) else None
        if not d or str(d.get("control", "")).strip().lower() not in ("real arm", "mirror"):
            return False
        on = d.get("value") in (True, 1, "1", "true", "on", "yes")
        if on:
            self.start()
        else:
            self.stop("Real arm off: only the simulation moves now; the real arm holds where it is.")
        return True

    def start(self):
        if self.active:
            return
        ip = str(self.p.get("ip") or "").strip()
        try:
            if self.api is None:
                from xarm.wrapper import XArmAPI
                self.say("Connecting to the real arm at %s…" % ip)
                self.api = XArmAPI(ip, is_radian=False)
            if not self.api.connected:
                self.api.connect()
            if not self.api.connected:
                raise RuntimeError("it does not answer at %s" % ip)
            if self.api.warn_code:
                self.api.clean_warn()
            if self.api.error_code:
                raise RuntimeError("it reports error C%s: clear it first (UFACTORY Studio or flow 0b)" % self.api.error_code)
            self.api.motion_enable(enable=True)
            self.api.set_mode(1)
            self.api.set_state(state=0)
            time.sleep(0.3)
            code, a = self.api.get_servo_angle(is_radian=False)
            if code != 0 or not a:
                raise RuntimeError("it does not give its joint angles (code %s)" % code)
            self.last, self.t_last = [float(v) for v in a[:6]], time.time()
            self.active = True
            self.say("Real arm on: it comes gently to the simulated arm's pose, then follows it.")
            self.ctx.telemetry("mirror", {"on": True})
            self.goto(self.arm.q)
        except Exception as e:
            self.active = False
            self.say("The real arm cannot follow: %s." % e)
            self.ctx.telemetry("mirror", {"on": False, "why": str(e)})

    def send(self, q):
        code = self.api.set_servo_angle_j(q, is_radian=False)
        self.last, self.t_last = list(q), time.time()
        if code not in (0, 2) and self.api.error_code:
            self.stop("The real arm stopped with error C%s: it no longer follows. Clear the error, then switch Real arm on again." % self.api.error_code)
            return False
        return True

    def goto(self, q):
        """A gentle move of the real arm from where it is to q; returns the seconds it took."""
        q = [float(v) for v in q[:6]]
        far = max(abs(a - b) for a, b in zip(q, self.last))
        if far < 0.05:
            return 0.0
        T = max(1.0, 1.875 * far / self.MOVE_SPEED)
        a0, t0 = list(self.last), time.time()
        while self.active:
            u = min(1.0, (time.time() - t0) / T)
            if not self.send([x + (y - x) * _minjerk(u) for x, y in zip(a0, q)]) or u >= 1.0:
                break
            time.sleep(0.01)
        return time.time() - t0

    def follow(self, q):
        """The twin's new pose for the real arm; returns the seconds the twin must wait (a catch-up move)."""
        if not self.active:
            return 0.0
        q = [float(v) for v in q[:6]]
        dt = max(0.02, time.time() - self.t_last)
        if max(abs(a - b) for a, b in zip(q, self.last)) > self.MAX_SPEED * dt + 0.5:
            return self.goto(q)
        self.send(q)
        return 0.0

    def stop(self, why=None):
        if self.active and self.api is not None:
            try:
                self.api.set_mode(0)
                self.api.set_state(state=0)
            except Exception:
                pass
        was, self.active = self.active, False
        if why and (was or self.api is not None):
            self.say(why)
        self.ctx.telemetry("mirror", {"on": False})

    def halt(self):
        """The run is stopped: the real arm stops at once."""
        if self.api is not None:
            try:
                self.api.set_state(4)
            except Exception:
                pass
        self.active = False

    def close(self):
        if self.active:
            self.stop()
        if self.api is not None:
            try:
                self.api.disconnect()
            except Exception:
                pass


class TwinArm:
    """The arm in simulation, with the calls of the real one (move, stroke, travel, home, report, stop, close). Every
    motion is planned whole before it starts and refused if any point of it leaves the allowed space: the tool point
    inside the safe box and away from the base, the pen not into the table, the elbow and wrist above it, every joint
    within its limits, and no sudden swing of a joint (a singular pose). It then runs smoothly: it speeds up and slows
    down within the acceleration limits and never goes past the speed limits (MAX_*), whatever the settings say. The
    pen draws on the sheet where it touches it. realtime: moves take the time they would take on the arm (1), or none
    (0, to plan and check only)."""

    def __init__(self, ctx, p, sheet=None, place=None, realtime=1.0, log_name="poses.csv", report=True):
        import csv
        self.ctx, self.p = ctx, p
        self.dry = False
        self.speed = max(1.0, min(MAX_TCP_SPEED, float(p.get("speed", 80))))
        self.accel = max(10.0, min(MAX_TCP_ACC, float(p.get("accel", 400))))
        self.joint_speed = max(5.0, min(MAX_JOINT_SPEED, float(p.get("joint_speed", 30))))
        self.joint_accel = max(10.0, min(MAX_JOINT_ACC, float(p.get("joint_accel", 80))))
        self.last_joint_speed, self.last_joint_accel = self.joint_speed, self.joint_accel
        self.rpy = [float(p.get("roll", 180)), float(p.get("pitch", 0)), float(p.get("yaw", 0))]
        box = [float(x) for x in str(p.get("safe_box") or "100,600,-350,350,20,450").replace(" ", "").split(",") if x != ""]
        if len(box) != 6 or box[0] >= box[1] or box[2] >= box[3] or box[4] >= box[5]:
            raise ValueError("Safe box: six numbers, x min, x max, y min, y max, z min, z max")
        self.box = box
        self.pen_length = float(p.get("pen_length", 100))
        self.tcp_z = self.pen_length if str(p.get("tcp", "flange")).startswith("pen") else 0.0
        self.table = float(p.get("table_z", 0))
        self.sheet, self.place = sheet, place
        self.realtime = float(realtime)
        self.q = [0.0] * 6
        self.pose = [float(v) for v in tool(self.q, self.tcp_z)[:3, 3]] + self.rpy
        self.pen = False
        self.last_sent = 0.0
        self.reporting = report
        self.log_file = open(ctx.output("log", log_name), "w", newline="") if log_name else None
        self.log = csv.writer(self.log_file) if self.log_file else None
        if self.log:
            self.log.writerow(["t", "x", "y", "z", "j1", "j2", "j3", "j4", "j5", "j6", "pen_down", "twin"])
        self.drawn = 0.0
        self._tip_prev = None
        self.planned = None
        self.mirror, self.poll, self._polled = None, None, 0.0
        self.report(force=True)

    def tip(self):
        T = tool(self.q, self.pen_length)
        return T[:3, 3]

    def report(self, force=False):
        if not self.reporting:
            return
        now = time.time()
        if not force and now - self.last_sent < 0.1:
            return
        self.last_sent = now
        x, y, z = [round(float(v), 2) for v in self.pose[:3]]
        self.ctx.telemetry("pose", {"x": x, "y": y, "z": z, "roll": self.rpy[0], "pitch": self.rpy[1], "yaw": self.rpy[2], "pen": self.pen, "twin": True})
        self.ctx.telemetry("joints", {"j%d" % (i + 1): round(float(a), 2) for i, a in enumerate(self.q)})

    def check(self, x, y, z):
        b = self.box
        if not (b[0] <= x <= b[1] and b[2] <= y <= b[3] and b[4] <= z <= b[5]):
            raise Stop("Target %.1f, %.1f, %.1f mm is outside the safe box %s: nothing was sent." % (x, y, z, b))

    def allowed(self, q):
        """Why the arm may not be at joint angles q (None when it may)."""
        for i, (a, (lo, hi)) in enumerate(zip(q, XARM6_LIMITS)):
            if not (lo + JOINT_MARGIN <= a <= hi - JOINT_MARGIN):
                return "joint %d would reach %.0f° (limits %.0f to %.0f°)" % (i + 1, a, lo, hi)
        links = fk(q)[0]
        T = links[6]
        tcp = (T @ [0, 0, self.tcp_z, 1.0])[:3]
        b = self.box
        if not (b[0] - 0.5 <= tcp[0] <= b[1] + 0.5 and b[2] - 0.5 <= tcp[1] <= b[3] + 0.5 and b[4] - 0.5 <= tcp[2] <= b[5] + 0.5):
            return "the tool point would leave the safe box (at %.0f, %.0f, %.0f mm)" % tuple(tcp)
        if math.hypot(tcp[0], tcp[1]) < MIN_RADIUS:
            return "the tool point would come within %.0f mm of the arm's base" % MIN_RADIUS
        tip = (T @ [0, 0, self.pen_length, 1.0])[:3]
        if tip[2] < self.table - 2.0:
            return "the pen would go %.0f mm into the table" % (self.table - tip[2])
        for k, name in ((3, "elbow"), (5, "wrist"), (6, "flange")):
            if links[k][2, 3] < self.table + 40:
                return "the %s would come within 40 mm of the table" % name
        return None

    def _set(self, q):
        self.q = [float(v) for v in q]
        self.pose[:3] = [float(v) for v in tool(self.q, self.tcp_z)[:3, 3]]
        tip = self.tip()
        down = self.place is not None and tip[2] <= self.place.z + 1.0
        if self.sheet is not None and down and self._tip_prev is not None and self._tip_prev[2] <= self.place.z + 1.0:
            a, b = self.place.board_of(*self._tip_prev[:2]), self.place.board_of(*tip[:2])
            if math.dist(a, b) > 0.05:
                self.sheet.stroke([a, b], float(self.p.get("pen_width", 1.2)), 20)
                self.drawn += math.dist(a, b)
        self._tip_prev = tip
        if self.log:
            self.log.writerow([round(time.time(), 3)] + [round(v, 2) for v in self.pose[:3]] + [round(v, 2) for v in self.q] + [int(bool(down)), 1])
        self.report()
        blocked = 0.0
        if self.mirror is not None:
            if self.poll is not None and time.time() - self._polled > 0.2:
                self._polled = time.time()
                self.poll()
                blocked = time.time() - self._polled  # switching Real arm on waits for the real arm to come
            blocked += self.mirror.follow(self.q)
        return blocked

    # -- planning ---------------------------------------------------------------
    def plan_line(self, points):
        """Joint angles every millimetre along the straight segments through `points` (the tool point, mm), all
        checked. Returns (arc lengths, joint angles) or raises Stop: nothing has moved yet."""
        import numpy as np
        pts = [np.array(self.pose[:3], float)] + [np.array(p, float) for p in points]
        for p in points:
            self.check(*p)
        s_list, q_list, q, s = [0.0], [np.array(self.q, float)], np.array(self.q, float), 0.0
        for a, b in zip(pts[:-1], pts[1:]):
            seg = float(np.linalg.norm(b - a))
            n = max(1, int(math.ceil(seg / 1.0)))
            for i in range(1, n + 1):
                target = a + (b - a) * i / n
                q2, ep, ew = ik(target, self.rpy, q, self.tcp_z)
                if ep > 0.5 or ew > 0.3:
                    raise Stop("The arm cannot keep the pen pointing down at %.0f, %.0f, %.0f mm (off by %.1f mm): nothing was sent." % (target[0], target[1], target[2], ep))
                turn = float(np.max(np.abs(q2 - q)))
                if turn > STEP_TURN * max(1.0, seg / n):
                    raise Stop("Near %.0f, %.0f, %.0f mm a joint would swing %.0f° for 1 mm of the pen (a singular pose): nothing was sent." % (target[0], target[1], target[2], turn))
                why = self.allowed(q2)
                if why:
                    raise Stop("Not allowed: " + why + ". Nothing was sent.")
                s += seg / n
                s_list.append(s); q_list.append(q2); q = q2
        return np.array(s_list), np.array(q_list)

    def plan_joints(self, target_q):
        """A joint move to target_q, checked at every degree. Returns (largest joint turn in degrees, how fast the tool
        point moves for the whole move done at once: mm per unit of the move at its fastest place), or raises Stop."""
        import numpy as np
        a, b = np.array(self.q, float), np.array(target_q, float)
        here = self.allowed(a)
        if here:
            raise Stop("The arm is where it may not move from (" + here + "): bring it back to its home position with UFACTORY's app, then start again.")
        turn = float(np.max(np.abs(b - a)))
        n = max(2, int(math.ceil(turn)))
        prev = tool(a, self.tcp_z)[:3, 3]
        dpdf = 0.0
        for i in range(1, n + 1):
            q = a + (b - a) * i / n
            why = self.allowed(q)
            if why:
                raise Stop("Not allowed on the way: " + why + ". Nothing was sent.")
            cur = tool(q, self.tcp_z)[:3, 3]
            dpdf = max(dpdf, float(np.linalg.norm(cur - prev)) * n)
            prev = cur
        return turn, dpdf

    # -- running a plan smoothly --------------------------------------------------
    def _run(self, T, at):
        """Runs a planned move of duration T: at(fraction 0..1 of the path) gives the joint angles."""
        dt = 0.02
        steps = max(1, int(math.ceil(T / dt)))
        t0 = time.time()
        for i in range(1, steps + 1):
            t0 += self._set(at(i / steps)) or 0.0  # time the real arm took to catch up, if it had to
            rt = self.realtime
            if self.mirror is not None and self.mirror.active and not (0 < rt <= 1):
                rt = 1.0  # the real arm follows: no faster than real time
            if rt > 0:
                wait_s = t0 + (T * i / steps) / rt - time.time()
                if wait_s > 0:
                    time.sleep(wait_s)
        self.report(force=True)

    def stroke(self, points, speed=None):
        """The tool point through `points` in straight lines (a stroke of the pen), one smooth motion."""
        import numpy as np
        s_list, q_list = self.plan_line(points)
        total = float(s_list[-1])
        if total < 1e-6:
            return
        if self.reporting:
            x, y, z = points[-1]
            self.ctx.telemetry("target", {"x": round(x, 2), "y": round(y, 2), "z": round(z, 2)})
        T, s_of = trapezoid(total, min(self.speed, speed or self.speed), self.accel)

        def at(f):
            s = s_of(f * T)
            k = int(np.searchsorted(s_list, s))
            if k <= 0:
                return q_list[0]
            if k >= len(s_list):
                return q_list[-1]
            w = (s - s_list[k - 1]) / max(1e-9, s_list[k] - s_list[k - 1])
            return q_list[k - 1] + (q_list[k] - q_list[k - 1]) * w
        self._run(T, at)

    def move(self, x, y, z, wait=True):
        """A straight line of the tool point to x, y, z."""
        self.stroke([(x, y, z)])

    def travel(self, x, y, z):
        """To a point through the joints, as the arm's joint moves do: every joint turns at once from the current
        angles to the target's, speeding up and slowing down smoothly, instead of a straight line of the pen (which
        can swing the wrist on a long move). For moves between places; strokes use stroke()."""
        import numpy as np
        self.check(x, y, z)
        q, ep, ew = ik((x, y, z), self.rpy, self.q, self.tcp_z, iters=300)
        if ep > 1.0 or ew > 0.5:  # from a standard pose instead of the current one
            q, ep, ew = ik((x, y, z), self.rpy, [0.0, -20.0, -30.0, 0.0, 50.0, 0.0], self.tcp_z, iters=300)
        if ep > 1.0 or ew > 0.5:
            raise Stop("The arm cannot reach %.0f, %.0f, %.0f mm with the pen pointing down (off by %.1f mm): check the positions." % (x, y, z, ep))
        self.joints_move(q, (x, y, z))

    def joints_move(self, target_q, target_xyz=None):
        """Every joint from its angle to target_q at once, smoothly: no joint faster than joint_speed or speeding up
        faster than joint_accel, and the tool point never faster than MAX_TCP_SPEED."""
        import numpy as np
        turn, dpdf = self.plan_joints(target_q)
        self.planned = list(map(float, target_q))
        if turn < 1e-6:
            return
        if self.reporting and target_xyz is not None:
            self.ctx.telemetry("target", {"x": round(target_xyz[0], 2), "y": round(target_xyz[1], 2), "z": round(target_xyz[2], 2), "joint_move": True})
        a, b = np.array(self.q, float), np.array(target_q, float)
        # slower when the tool point would go faster than the pen speed: at the joints' top speed it moves
        # dpdf * joint_speed / turn mm/s
        k = min(1.0, self.speed * turn / max(1e-9, dpdf * self.joint_speed))
        self.last_joint_speed, self.last_joint_accel = self.joint_speed * k, self.joint_accel * k * k
        T, s_of = trapezoid(turn, self.last_joint_speed, self.last_joint_accel)

        def at(f):
            return a + (b - a) * (s_of(f * T) / turn)
        self._run(max(T, 0.2), at)

    def joints_to(self, target, seconds=2.0):
        self.joints_move(target)

    def home(self):
        self.joints_move([0.0] * 6)

    def stop(self):
        pass

    def close(self):
        if self.log_file:
            self.log_file.close()


def look_pose(p, board_mm, rpy, height=None):
    """Flange position (mm) for reading the board: for a wrist camera, the camera right above the board's centre at
    `wrist_height`; for a camera on a stand, the parking position (out of its view)."""
    import numpy as np
    if str(p.get("camera_mount", "table")).lower().startswith("wrist"):
        place = Place(p)
        cx, cy, _ = place.world(board_mm / 2, board_mm / 2)
        off = rpy_matrix(*rpy) @ np.array(_vec(p.get("wrist_camera"), 3, (70.0, 0.0, -40.0)))
        z = float(height if height is not None else p.get("wrist_height", 330))
        return (cx - off[0], cy - off[1], z)
    return tuple(_vec(p.get("park"), 3, (150.0, -250.0, 250.0)))


class Stop(Exception):
    pass


class Arm:
    """The real arm, or a simulation with the same few calls (dry run)."""

    def __init__(self, ctx, dry):
        p = ctx.params
        self.ctx, self.dry = ctx, dry
        self.speed = max(1.0, min(150.0, float(p.get("speed", 80))))     # never faster than 150 mm/s
        self.accel = max(10.0, min(800.0, float(p.get("accel", 400))))   # never more than 800 mm/s²
        self.rpy = [float(p.get("roll", 180)), float(p.get("pitch", 0)), float(p.get("yaw", 0))]
        box = [float(x) for x in str(p.get("safe_box") or "100,600,-350,350,20,450").replace(" ", "").split(",") if x != ""]
        if len(box) != 6 or box[0] >= box[1] or box[2] >= box[3] or box[4] >= box[5]:
            raise ValueError("Safe box: six numbers, x min, x max, y min, y max, z min, z max")
        self.box = box
        self.pose = [300.0, 0.0, 200.0] + self.rpy
        self.log_file = open(ctx.output("log", "poses.csv"), "w", newline="")
        self.log = csv.writer(self.log_file)
        self.log.writerow(["t", "x", "y", "z", "roll", "pitch", "yaw", "pen", "dry_run"])
        self.pen = False
        self.api = None
        self.last_sent = 0.0
        if not dry:
            from xarm.wrapper import XArmAPI
            ip = str(p.get("ip") or "").strip()
            ctx.log("Connecting to the xArm at %s…" % ip)
            self.api = XArmAPI(ip, is_radian=False)
            if not self.api.connected:
                raise RuntimeError("Cannot reach the xArm at %s: check the address and that this machine is on the arm's network." % ip)
            self.api.clean_warn()
            self.api.clean_error()
            self.api.motion_enable(enable=True)
            self.api.set_mode(0)
            self.api.set_state(state=0)
            time.sleep(0.5)
            code, pos = self.api.get_position()
            if code == 0 and pos:
                self.pose = [float(v) for v in pos[:6]]
            ctx.log("Connected: xArm with %s axes, at %s" % (self.api.axis, [round(v, 1) for v in self.pose]))
            self.limits()
        self.report(force=True)

    def limits(self):
        """The limits set in the arm's controller too (reduced mode), so it enforces them by itself: the top speed of
        the tool point, the safe box as a fence it cannot cross, the top acceleration, self-collision detection."""
        api, b = self.api, self.box
        self.prev_reduced, done, failed = None, [], []
        try:
            code, st = api.get_reduced_states()
            if code == 0 and st:
                self.prev_reduced = st
        except Exception:
            pass
        for name, fn in (("tool point at most %.0f mm/s" % self.speed, lambda: api.set_reduced_max_tcp_speed(self.speed)),
                         ("joints at most 60°/s", lambda: api.set_reduced_max_joint_speed(60)),
                         ("the safe box as a fence", lambda: api.set_reduced_tcp_boundary([b[1], b[0], b[3], b[2], b[5], b[4]])),
                         ("fence on", lambda: api.set_fence_mode(True)), ("reduced mode on", lambda: api.set_reduced_mode(True)),
                         ("acceleration at most %.0f mm/s²" % self.accel, lambda: api.set_tcp_maxacc(self.accel)),
                         ("self-collision detection", lambda: api.set_self_collision_detection(True))):
            try:
                (done if fn() in (0, None) else failed).append(name)
            except Exception:
                failed.append(name)
        self.ctx.log("Limits set in the arm's controller: " + (", ".join(done) or "none") + "." + (" Not accepted: " + ", ".join(failed) + "." if failed else ""))
        try:
            if api.warn_code:
                api.clean_warn()  # older firmware refuses some settings with a warning, which would hold up the next moves
        except Exception:
            pass

    # -- reporting -------------------------------------------------------------
    def report(self, force=False):
        now = time.time()
        if not force and now - self.last_sent < 0.1:
            return
        self.last_sent = now
        x, y, z, r, pi, ya = [round(v, 2) for v in self.pose]
        self.ctx.telemetry("pose", {"x": x, "y": y, "z": z, "roll": r, "pitch": pi, "yaw": ya, "pen": self.pen, "dry_run": self.dry})
        if self.api is not None:
            code, angles = self.api.get_servo_angle()
            if code == 0 and angles:
                self.ctx.telemetry("joints", {"j%d" % (i + 1): round(a, 2) for i, a in enumerate(angles[: self.api.axis])})
            self.ctx.telemetry("state", {"state": self.api.state, "error": self.api.error_code, "warn": self.api.warn_code})

    def check(self, x, y, z):
        b = self.box
        if not (b[0] <= x <= b[1] and b[2] <= y <= b[3] and b[4] <= z <= b[5]):
            raise Stop("Target %.1f, %.1f, %.1f mm is outside the safe box %s: nothing was sent." % (x, y, z, b))

    # -- motion ----------------------------------------------------------------
    def move(self, x, y, z, wait=True):
        self.check(x, y, z)
        self.ctx.telemetry("target", {"x": round(x, 2), "y": round(y, 2), "z": round(z, 2)})
        if self.dry:
            start = list(self.pose[:3])
            dist = math.dist(start, (x, y, z))
            steps = max(1, int(dist / self.speed / 0.05))
            for i in range(1, steps + 1):
                f = i / steps
                self.pose[:3] = [start[k] + ((x, y, z)[k] - start[k]) * f for k in range(3)]
                self.log.writerow([round(time.time(), 3)] + [round(v, 2) for v in self.pose] + [int(self.pen), 1])
                self.report()
                time.sleep(dist / self.speed / steps)
            self.report(force=True)
            return
        code = self.api.set_position(x=x, y=y, z=z, roll=self.rpy[0], pitch=self.rpy[1], yaw=self.rpy[2],
                                     speed=self.speed, mvacc=self.accel, wait=wait)
        if code != 0:
            raise RuntimeError("The xArm refused the move (code %s, error %s, warning %s)." % (code, self.api.error_code, self.api.warn_code))
        c, pos = self.api.get_position()
        if c == 0 and pos:
            self.pose = [float(v) for v in pos[:6]]
        self.log.writerow([round(time.time(), 3)] + [round(v, 2) for v in self.pose] + [int(self.pen), 0])
        self.report(force=True)

    def home(self):
        if self.dry:
            self.move(300.0, 0.0, 250.0)
            return
        code = self.api.move_gohome(speed=30, mvacc=200, wait=True)
        if code != 0:
            raise RuntimeError("The xArm could not go home (code %s, error %s)." % (code, self.api.error_code))
        c, pos = self.api.get_position()
        if c == 0 and pos:
            self.pose = [float(v) for v in pos[:6]]
        self.report(force=True)

    def stop(self):
        if self.api is not None:
            try:
                self.api.set_state(4)  # stop the motion now
            except Exception:
                pass

    def close(self):
        self.log_file.close()
        if self.api is not None and getattr(self, "prev_reduced", None) is not None:
            try:  # the controller as it was before
                self.api.set_reduced_mode(bool(self.prev_reduced[0]))
                if len(self.prev_reduced) > 5:
                    self.api.set_fence_mode(bool(self.prev_reduced[5]))
            except Exception:
                pass
        if self.api is not None:
            try:
                self.api.disconnect()
            except Exception:
                pass


class Board:
    """Board coordinates (mm from its lower-left corner) to the arm's base coordinates."""

    def __init__(self, p):
        self.x0, self.y0 = float(p.get("board_x", 390)), float(p.get("board_y", -70))
        self.rot = math.radians(float(p.get("board_rotation", 90)))
        self.size = float(p.get("board_size", 140))
        self.z_draw, self.z_up = float(p.get("z_draw", 100)), float(p.get("z_up", 130))
        if self.z_up <= self.z_draw:
            raise ValueError("Travel height must be above Draw height.")

    def to_arm(self, bx, by):
        c, s = math.cos(self.rot), math.sin(self.rot)
        return self.x0 + bx * c - by * s, self.y0 + bx * s + by * c

    def cell_box(self, cell):
        n = int(cell) - 1
        if not 0 <= n <= 8:
            raise ValueError("Cells are numbered 1 to 9, from the top left.")
        w = self.size / 3
        col, row = n % 3, n // 3
        return col * w, self.size - (row + 1) * w, w  # lower-left corner and width of the cell


def figure(board, name, cell):
    s, w = board.size, board.size / 3
    if name == "tic-tac-toe grid":
        return [[(w, 0), (w, s)], [(2 * w, s), (2 * w, 0)], [(0, w), (s, w)], [(s, 2 * w), (0, 2 * w)]]
    if name == "square":
        return [[(0, 0), (s, 0), (s, s), (0, s), (0, 0)]]
    x0, y0, cw = board.cell_box(cell)
    m = cw * 0.2
    if name == "X":
        return [[(x0 + m, y0 + m), (x0 + cw - m, y0 + cw - m)], [(x0 + m, y0 + cw - m), (x0 + cw - m, y0 + m)]]
    if name == "O":
        cx, cy, r = x0 + cw / 2, y0 + cw / 2, cw * 0.3
        return [[(cx + r * math.cos(a * math.pi / 18), cy + r * math.sin(a * math.pi / 18)) for a in range(37)]]
    raise ValueError("Unknown figure " + name)
