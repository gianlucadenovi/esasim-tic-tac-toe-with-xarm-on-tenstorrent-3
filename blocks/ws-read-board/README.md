# Read board

Finds the printed board in a camera picture by its markers, straightens it, cuts the 9 cells and reads them with the board reader.

Without Pictures it takes them with a camera of the machine (Camera: 0, 1 … or a device path or stream address). Connect the models (reader.pt) to read the cells: the log and the live picture show X, O or empty in each, with how sure the reader is. Without models it only checks that the board is found and straight: use it to place the camera and the light. Real cells read with the reader are saved as cells/real_cells_*.npz (same format as Render cells): check them before training on them. Live: board (straightened picture with the reading), camera (when the board is not found), read (matrix of confidences).

An [EsaSim](https://esacode.com/) blueprint block: it runs on a machine, in Python (`backend.py`, `def run(ctx)`).

| | |
|---|---|
| Block | `ws-read-board` · version 2 |
| Workspace | Gianluca De Novi's workspace |
| Author | Gianluca De Novi |
| Runs on | Any machine |

## Inputs

| Port | Type | Description |
|---|---|---|
| `pictures` Pictures | Files (jpg, png) | Optional: pictures to read instead of the camera Several connections. Optional. |
| `models` Models | Files (pt) | Optional: reader.pt from Train cell reader (or the models dataset) Several connections. Optional. |

## Outputs

| Port | Type | Description |
|---|---|---|
| `board` Straightened board | Files (png) |  |
| `frames` Camera pictures | Files (jpg) |  |
| `cells` Real cells | Files (npz) | Cells with the reader's labels, for training |
| `readings` Readings | Files (json) |  |

## Settings

| Key | Type | Default | Description |
|---|---|---|---|
| `camera` Camera | text | `"0"` | 0, 1 … a device path, or an rtsp:// or http:// address |
| `shots` Pictures to take | number | `1` |  |
| `every` Every (seconds) | number | `1` |  |
| `width` Camera width (pixels) | number | `1280` |  |
| `height` Camera height (pixels) | number | `720` |  |
| `save_cells` Save the cells read as training data | bool | `true` |  |

## Use it in EsaSim

In a workspace, open **Projects > Blocks > New block**, choose this kind of block, copy the ports and settings above and paste the code; or ask an AI assistant connected to EsaSim (MCP) to create it from this folder.
