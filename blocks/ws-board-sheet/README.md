# Board sheet

The printable tic-tac-toe board: four ArUco markers around a square, so a camera finds and straightens it by itself.

Print board-sheet.pdf on A4 at 100% (no scaling) and check the square with a ruler. The markers are ArUco 4x4 ids 0 (top left), 1 (top right), 2 (bottom right), 3 (bottom left); each touches the board with its inner corner. Use the same board size in the xArm and Tic-tac-toe game blocks. Board + 2 x marker must stay under 202 mm.

An [EsaSim](https://esacode.com/) blueprint block: it runs on a machine, in Python (`backend.py`, `def run(ctx)`).

| | |
|---|---|
| Block | `ws-board-sheet` · version 2 |
| Workspace | Gianluca De Novi's workspace |
| Author | Gianluca De Novi |
| Runs on | Any machine |

## Outputs

| Port | Type | Description |
|---|---|---|
| `sheet` Sheet | Files (pdf, png, json) | board-sheet.pdf to print, the same as PNG, and its measures |

## Settings

| Key | Type | Default | Description |
|---|---|---|---|
| `board_mm` Board size (mm) | number | `140` |  |
| `marker_mm` Marker size (mm) | number | `25` |  |
| `dpi` Resolution (dpi) | number | `300` |  |
| `title` Title on the sheet | text | `"EsaSim · tic-tac-toe with the xArm"` |  |

## Use it in EsaSim

In a workspace, open **Projects > Blocks > New block**, choose this kind of block, copy the ports and settings above and paste the code; or ask an AI assistant connected to EsaSim (MCP) to create it from this folder.
