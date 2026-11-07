# Train cell reader

Trains the board reader (PyTorch CNN: empty, X or O in a 32x32 cell) on the accelerator you choose: NVIDIA GPU, Tenstorrent or CPU.

Connect the cells of Simulate games (pictures of the digital twin) and/or Render cells, and optionally real cells (Read board, games). Accelerator: NVIDIA GPU (CUDA; on Windows the worker installs PyTorch with CUDA, EsaSim 0.12.13+), Tenstorrent (PyTorch/XLA from TT-Forge's tt-xla, installed on the Linux machine with the cards) or CPU. The one chosen is used or the block stops and says why: it never moves to another accelerator by itself. Flips and quarter turns augment the cells. Live: train (loss, acc, img_s), val (val_acc after each epoch), confusion (matrix: rows true, columns read), device. Writes model/reader.pt (+ reader.json) for Read board and the game.

An [EsaSim](https://esacode.com/) blueprint block: it runs on a machine, in Python (`backend.py`, `def run(ctx)`).

| | |
|---|---|
| Block | `ws-train-cell-reader` · version 2 |
| Workspace | Gianluca De Novi's workspace |
| Author | Gianluca De Novi |
| Runs on | Any machine, NVIDIA GPU, Tenstorrent |

## Inputs

| Port | Type | Description |
|---|---|---|
| `cells` Cells | Files (npz) | *.npz from Simulate games or Render cells Several connections. |
| `real` Real cells | Files (npz) | Optional: cells from real pictures (Read board, games), same format; they count more Several connections. Optional. |

## Outputs

| Port | Type | Description |
|---|---|---|
| `model` Board reader | Files (pt, json) | reader.pt (weights and facts) and reader.json |
| `metrics` Metrics | Metrics (json) |  |

## Settings

| Key | Type | Default | Description |
|---|---|---|---|
| `accelerator` Accelerator | select | `"NVIDIA GPU"` |  |
| `epochs` Epochs | number | `6` |  |
| `batch` Batch | number | `256` |  |
| `lr` Learning rate | number | `0.002` |  |
| `validation` Share for validation | number | `0.1` |  |
| `seed` Seed | number | `1` |  |

## Use it in EsaSim

In a workspace, open **Projects > Blocks > New block**, choose this kind of block, copy the ports and settings above and paste the code; or ask an AI assistant connected to EsaSim (MCP) to create it from this folder.
