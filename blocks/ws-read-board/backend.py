# ── EsaSim block ──────────────────────────────────────────────────────
# Read board (ws-read-board) · version 2 · machine block, Python: def run(ctx)
# Finds the printed board in a camera picture by its markers, straightens it, cuts the 9 cells and reads them with the board reader.
# Workspace: Gianluca De Novi's workspace · Author: Gianluca De Novi
# Saved: 2026-11-07
#
# Inputs   ctx.inputs["id"] is a list of files: .path .name .size .read_text() .read_json()
#   pictures    Files (jpg, png)          Pictures · several · optional · Optional: pictures to read instead of the camera
#   models      Files (pt)                Models · several · optional · Optional: reader.pt from Train cell reader (or the models dataset)
# Outputs  ctx.output("id", "name.ext") is the path to write; ctx.write_json / ctx.write_text
#   board       Files (png)               Straightened board
#   frames      Files (jpg)               Camera pictures
#   cells       Files (npz)               Real cells · Cells with the reader's labels, for training
#   readings    Files (json)              Readings
#   telemetry   Live telemetry            ctx.telemetry("channel", data) while it runs
# Settings ctx.params["key"]
#   camera      text      Camera = "0"
#   shots       number    Pictures to take = 1
#   every       number    Every (seconds) = 1
#   width       number    Camera width (pixels) = 1280
#   height      number    Camera height (pixels) = 720
#   save_cells  bool      Save the cells read as training data = true
# Runs on  Any machine. Return a dict to fill the "result" output.
# Packages opencv-python-headless>=4.7, numpy, torch[cuda] (installed by the worker with pip when missing)
# EsaSim rewrites these lines from the block's definition each time it is saved: change the definition, not them.
# ── end of EsaSim block header ────────────────────────────────────────

"""Read board: takes a picture with a camera of the machine (or reads the pictures it receives), finds the printed
board by its four ArUco markers, straightens it, cuts the nine cells and, with a trained board reader, says what is
in each (empty, X, O) and how sure it is. For checking the camera, the light and the reader before a game, and for
collecting real cells (cells.npz, same format as Render cells) to train the reader on."""
import os
import sys
import time

CELL_PX = 96   # each cell of the straightened board, in pixels
MARGIN = 0.10  # share of a cell cut away at its edges (the grid lines)
CLASSES = ["empty", "X", "O"]


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


def run(ctx):
    import cv2
    import numpy as np

    p = ctx.params
    models = load_models(ctx, "models")
    reader = models.get("reader")
    if reader:
        ctx.log("Board reader: %s (%.1f%% on validation)" % (reader[1].get("file"), 100 * float(reader[1].get("val_acc", 0))))
    frames = []
    pics = [f for f in ctx.inputs.get("pictures", []) if f.name.lower().endswith((".jpg", ".jpeg", ".png", ".bmp", ".webp"))]
    if pics:
        frames = [(f.name, cv2.imread(f.path)) for f in pics[: int(p.get("max_pictures", 50))]]
    else:
        cam = Camera(ctx, p.get("camera", "0"), p.get("width", 1280), p.get("height", 720))
        try:
            for i in range(int(p.get("shots", 1))):
                frames.append(("camera_%02d.jpg" % (i + 1), cam.grab()))
                if i + 1 < int(p.get("shots", 1)):
                    time.sleep(float(p.get("every", 1)))
        finally:
            cam.close()

    all_cells, all_labels, readings, ok = [], [], [], 0
    for name, img in frames:
        if img is None:
            ctx.log("%s: not a picture OpenCV can read" % name)
            continue
        cv2.imwrite(ctx.output("frames", name if name.lower().endswith(".jpg") else name + ".jpg"), img)
        board, H, found = find_board(img)
        if board is None:
            ctx.log("%s: the board was not found (%s). Check that the four markers are in the picture, in focus and not covered." % (name, found))
            ctx.picture("camera", img, bgr=True, note="board not found: " + found)
            readings.append({"picture": name, "found": False, "reason": found})
            continue
        ok += 1
        cells = cut_cells(board)
        labels, probs = [""] * 9, None
        if reader:
            probs = read_cells(reader[0], cells)
            labels = [["", "X", "O"][int(i)] for i in probs.argmax(1)]
            all_cells.append(cells)
            all_labels.append(probs.argmax(1).astype(np.uint8))
        cv2.imwrite(ctx.output("board", "board_" + os.path.splitext(name)[0] + ".png"), board)
        view = overlay(board, labels, probs, name)
        ctx.picture("board", view, bgr=True, picture=name, markers=len(found))
        grid = [[labels[r * 3 + c] or "·" for c in range(3)] for r in range(3)]
        rd = {"picture": name, "found": True, "markers": found, "board": ["".join(r) for r in grid]}
        if probs is not None:
            rd["sure"] = [round(float(v), 3) for v in probs.max(1)]
            ctx.telemetry("read", {"matrix": [[round(float(probs[r * 3 + c].max()), 3) for c in range(3)] for r in range(3)], "board": rd["board"]})
        readings.append(rd)
        ctx.log("%s: %s%s" % (name, " | ".join(rd["board"]), ("  (least sure %.0f%%)" % (100 * min(rd["sure"]))) if probs is not None else ""))

    if all_cells and p.get("save_cells", True):
        # cells read with confidence, as training data (check them before training on them)
        x = np.concatenate(all_cells)
        y = np.concatenate(all_labels)
        np.savez_compressed(ctx.output("cells", "real_cells_%d.npz" % int(time.time())), x=x, y=y, size=32, classes=np.array(CLASSES))
    ctx.write_json("readings", "readings.json", readings)
    if not ok:
        raise RuntimeError("The board was not found in any picture. Print the board sheet (flow 1 · Print the board), keep its four markers visible, and check the camera.")
    return {"pictures": len(frames), "boards_found": ok, "reader": bool(reader)}


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


def make_player(nn):
    return nn.Sequential(nn.Linear(18, 96), nn.ReLU(), nn.Linear(96, 96), nn.ReLU(), nn.Linear(96, 1), nn.Tanh())
