# Render cells

Synthetic pictures of tic-tac-toe cells (empty, X, O) as a camera sees them on paper, to train the board reader.

Draws marks of many hands and pens on paper of many tones, with uneven light, small perspective, blur, noise and bits of the grid lines at the edges, like the crops the Tic-tac-toe game cuts from the camera. Writes one compressed file (cells_01.npz: x = N x 32 x 32 grey pictures, y = 0 empty, 1 X, 2 O; about 1 MB per 1,000 cells, so it stays on the machine) and a mosaic of examples. Live: cells per second and a mosaic every 2,000 cells. About 1,500 cells per second on one CPU core.

An [EsaSim](https://esacode.com/) blueprint block: it runs on a machine, in Python (`backend.py`, `def run(ctx)`).

| | |
|---|---|
| Block | `ws-render-cells` · version 1 |
| Workspace | Gianluca De Novi's workspace |
| Author | Gianluca De Novi |
| Runs on | Any machine |

## Outputs

| Port | Type | Description |
|---|---|---|
| `cells` Cells | Files (npz) | cells_01.npz: x (N x 32 x 32 uint8), y (0 empty, 1 X, 2 O) |
| `samples` Examples | Files (png) | A mosaic of the first 192 cells |

## Settings

| Key | Type | Default | Description |
|---|---|---|---|
| `count` Cells | number | `30000` |  |
| `size` Size (pixels) | number | `32` | 32 for Train cell reader |
| `seed` Seed | number | `1` |  |

## Use it in EsaSim

In a workspace, open **Projects > Blocks > New block**, choose this kind of block, copy the ports and settings above and paste the code; or ask an AI assistant connected to EsaSim (MCP) to create it from this folder.
