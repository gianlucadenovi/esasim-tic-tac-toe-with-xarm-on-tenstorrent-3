# ── EsaSim block ──────────────────────────────────────────────────────
# Simulate games (ws-simulate-games) · version 1 · machine block, Python: def run(ctx)
# Training data from the digital twin: simulated games on paper, seen by the simulated camera, cut into labelled cells.
# Workspace: Gianluca De Novi's workspace · Author: Gianluca De Novi
# Saved: 2026-11-07
#
# Inputs   ctx.inputs["id"] is a list of files: .path .name .size .read_text() .read_json()
#   robot       Files (gz, json)          Robot model · optional · xarm6.robot.json.gz from xArm 6 model
# Outputs  ctx.output("id", "name.ext") is the path to write; ctx.write_json / ctx.write_text
#   cells       Files (npz)               Cells · sim_cells_01.npz: x (N x 32 x 32), y (0 empty, 1 X, 2 O)
#   samples     Files (png)               Examples
#   frames      Files (jpg)               Simulated pictures · A few of the camera's pictures
#   metrics     Metrics (json)            Metrics
#   telemetry   Live telemetry            ctx.telemetry("channel", data) while it runs
# Settings ctx.params["key"]
#   games       number    Games = 200
#   variety     select    Variety = "some" of none | some | a lot
#   camera_mount select    Camera = "table" of table | wrist
#   camera_pos  text      Camera position (mm) = ""
#   camera_look text      Camera looks at (mm) = ""
#   camera_hfov number    Camera field of view (degrees) = 70
#   wrist_camera text      Wrist camera offset (mm) = "70, 0, -40"
#   wrist_height number    Flange height to read (wrist camera, mm) = 330
#   park        text      Parked pen (mm) = "150, -250, 250"
#   board_x     number    Board X (mm) = 250
#   board_y     number    Board Y (mm) = -90
#   board_rotation number    Board rotation (degrees) = 0
#   board_size  number    Board size (mm) = 140
#   table_z     number    Table height (mm) = 0
#   pen_length  number    Pen length below the flange (mm) = 100
#   sim_width   number    Picture width (pixels) = 1280
#   sim_height  number    Picture height (pixels) = 720
#   save_frames number    Pictures to keep = 24
#   seed        number    Seed = 1
# Runs on  Any machine. Return a dict to fill the "result" output.
# Packages opencv-python-headless>=4.7, numpy (installed by the worker with pip when missing)
# EsaSim rewrites these lines from the block's definition each time it is saved: change the definition, not them.
# ── end of EsaSim block header ────────────────────────────────────────

"""Simulate games: training data from the digital twin. In each simulated game the arm draws the grid and its marks
with the pen (the strokes the real arm draws), a simulated visitor draws theirs by hand, and the simulated camera
(on a stand above the table or on the wrist, like the real one) takes a picture after every move. Each picture goes
through the same steps as in a real game (markers found, board straightened, nine cells cut), and the game knows what
is in every cell: labelled cells for Train cell reader, in the same format as Render cells. Paper, ink, pen, light,
camera position and lens change from game to game, so the reader learns what does not matter."""
import json
import math
import random
import time

SYMS = {1: "X", -1: "O"}


def run(ctx):
    import cv2
    import numpy as np

    p = ctx.params
    games = int(p.get("games", 200))
    board_mm = float(p.get("board_size", 140))
    seed = int(p.get("seed", 1)) * 1000 + (ctx.task.index if ctx.task else 0)
    rng = np.random.default_rng(seed)
    random.seed(seed)
    vary = {"none": 0.0, "some": 0.6, "a lot": 1.4}.get(str(p.get("variety", "some")), 0.6)
    model = load_model(ctx.inputs.get("robot", []), cell=12)
    ctx.log("Simulated camera: %s; arm model: %s." % ("on the wrist" if str(p.get("camera_mount", "table")).startswith("wrist") else "on a stand above the table",
                                                      "UFACTORY xArm 6 meshes" if model else "simple bars (connect xArm 6 model for the real shape)"))
    perfect = Perfect()
    rpy = [float(p.get("roll", 180)), float(p.get("pitch", 0)), float(p.get("yaw", 0))]
    xs, ys, frames_saved = [], [], 0
    found = tried = 0
    t0, last_pic, last_tel = time.time(), 0.0, 0.0
    q_seed = [0.0] * 6
    for g in range(games):
        # this game's paper, board position, camera and pens
        pg = dict(p)
        if vary:
            pg["board_x"] = float(p.get("board_x", 250)) + rng.normal(0, 6 * vary)
            pg["board_y"] = float(p.get("board_y", -90)) + rng.normal(0, 6 * vary)
            pg["board_rotation"] = float(p.get("board_rotation", 0)) + rng.normal(0, 3 * vary)
        sheet = Sheet(board_mm, float(p.get("marker_size", 25)), 4.0, int(rng.integers(215, 252)), rng)
        place = Place(pg)
        cam = SimCamera(pg, board_mm, rng if vary else None)
        geo = Board(dict(pg, board_size=board_mm))
        pen_w, pen_ink = rng.uniform(0.8, 1.6), int(rng.integers(10, 45))

        def robot_draw(strokes):
            for st in strokes:
                pts = [(x + rng.normal(0, 0.25), y + rng.normal(0, 0.25)) for x, y in st]
                sheet.stroke(pts, pen_w, pen_ink)

        # where the arm waits while the camera looks (parked, or holding the wrist camera over the board)
        lp = look_pose(pg, board_mm, rpy)
        lp = (lp[0] + rng.normal(0, 15 * vary), lp[1] + rng.normal(0, 15 * vary), lp[2] + rng.normal(0, 10 * vary))
        q, ep, _ = ik(lp, rpy, q_seed)
        if ep < 2:
            q_seed = q

        def shoot(state):
            nonlocal found, tried, frames_saved, last_pic
            img = cam.render(sheet, place, q_seed, model, float(p.get("pen_length", 100)), rng)
            tried += 1
            board, H, ids = find_board(img)
            now = time.time()
            if now - last_pic > 1.5:
                last_pic = now
                ctx.picture("camera", img, bgr=True, width=480, game=g + 1)
            if frames_saved < int(p.get("save_frames", 24)) and rng.random() < 0.3:
                cv2.imwrite(ctx.output("frames", "game%04d_%d.jpg" % (g + 1, sum(1 for v in state if v))), img)
                frames_saved += 1
            if board is None:
                return
            found += 1
            xs.append(cut_cells(board))
            ys.append(np.array([{0: 0, 1: 1, -1: 2}[v] for v in state], np.uint8))

        robot_draw(figure(geo, "tic-tac-toe grid", 5))
        state = [0] * 9
        shoot(state)
        robot = 1 if rng.random() < 0.5 else -1
        turn = 1
        while not winner(tuple(state)) and 0 in state:
            free = [i for i in range(9) if state[i] == 0]
            k = random.choice(perfect.best_moves(tuple(state), turn)) if rng.random() < 0.4 else random.choice(free)
            if turn == robot:
                robot_draw(figure(geo, SYMS[turn], k + 1))
            else:
                strokes, w, ink = hand_mark(SYMS[turn], k + 1, board_mm, rng)
                for st in strokes:
                    sheet.stroke(st, w, ink)
            state[k] = turn
            shoot(state)
            turn = -turn
        now = time.time()
        if now - last_tel > 0.5 or g == games - 1:
            last_tel = now
            ctx.telemetry("simulation", {"games": g + 1, "pictures": tried, "found": found, "found_rate": round(found / max(1, tried), 3),
                                         "cells": 9 * len(xs), "pictures_s": round(tried / max(1e-6, now - t0), 1)})
            ctx.progress(100 * (g + 1) / games, "game %d of %d" % (g + 1, games))
    if not xs:
        raise RuntimeError("The simulated camera never found the board: check the camera position (it must see the four markers).")
    x, y = np.concatenate(xs), np.concatenate(ys)
    np.savez_compressed(ctx.output("cells", "sim_cells_%02d.npz" % ((ctx.task.index if ctx.task else 0) + 1)), x=x, y=y, size=32, classes=np.array(["empty", "X", "O"]))
    k = min(len(x), 192)
    pick = np.linspace(0, len(x) - 1, k).astype(int)
    S = 32
    rows = math.ceil(k / 16)
    mos = np.full((rows * (S + 2), 16 * (S + 2)), 255, np.uint8)
    for i, j in enumerate(pick):
        r, c = divmod(i, 16)
        mos[r * (S + 2) + 1:r * (S + 2) + 1 + S, c * (S + 2) + 1:c * (S + 2) + 1 + S] = x[j]
    cv2.imwrite(ctx.output("samples", "sim_samples.png"), mos)
    secs = time.time() - t0
    res = {"games": games, "pictures": tried, "boards_found": found, "found_rate": round(found / max(1, tried), 3), "cells": int(len(x)),
           "empty": int((y == 0).sum()), "X": int((y == 1).sum()), "O": int((y == 2).sum()), "seconds": round(secs, 1)}
    ctx.write_json("metrics", "simulation.json", res)
    ctx.log("Simulated %d games: %d pictures, board found in %.1f%%, %d labelled cells, %.0f s." % (games, tried, 100 * res["found_rate"], len(x), secs))
    return res


# ---- shared with the other blocks of the tic-tac-toe demo ----

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
        self.x0, self.y0 = float(p.get("board_x", 250)), float(p.get("board_y", -90))
        self.rot = math.radians(float(p.get("board_rotation", 0)))
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


LINES = [(0, 1, 2), (3, 4, 5), (6, 7, 8), (0, 3, 6), (1, 4, 7), (2, 5, 8), (0, 4, 8), (2, 4, 6)]


def winner(b):
    for a, c, d in LINES:
        if b[a] != 0 and b[a] == b[c] == b[d]:
            return b[a]
    return 0


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


class Board:
    """Board coordinates (mm from its lower-left corner) to the arm's base coordinates."""

    def __init__(self, p):
        self.x0, self.y0 = float(p.get("board_x", 250)), float(p.get("board_y", -90))
        self.rot = math.radians(float(p.get("board_rotation", 0)))
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
