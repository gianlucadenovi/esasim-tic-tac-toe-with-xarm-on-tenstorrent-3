# Train player by self-play

A small value network (PyTorch) learns tic-tac-toe by playing thousands of games against itself, on the accelerator you choose.

Games are played in batches side by side; after each move the network scores the position for the player who just moved, plays its best move (sometimes a random one, to explore) and learns from how games end and from its own look one move ahead. Tested as it learns against a random player, against the perfect player, and on every position of the game. Accelerator: NVIDIA GPU, Tenstorrent or CPU, as in Train cell reader (the one chosen is used, or the block stops and says why). Live: train (games, loss, explore, games_s, x_wins, o_wins, draws), eval (vs_random_win, vs_perfect_draw, vs_perfect_loss, optimal_moves), first_move (matrix: X's score for each first move). Writes model/player.pt for the game.

An [EsaSim](https://esacode.com/) blueprint block: it runs on a machine, in Python (`backend.py`, `def run(ctx)`).

| | |
|---|---|
| Block | `ws-train-player-by-self-play` · version 2 |
| Workspace | Gianluca De Novi's workspace |
| Author | Gianluca De Novi |
| Runs on | Any machine, NVIDIA GPU, Tenstorrent |

## Outputs

| Port | Type | Description |
|---|---|---|
| `model` Player | Files (pt, json) | player.pt (weights and facts) and player.json |
| `metrics` Metrics | Metrics (json) |  |

## Settings

| Key | Type | Default | Description |
|---|---|---|---|
| `accelerator` Accelerator | select | `"NVIDIA GPU"` |  |
| `games` Games | number | `60000` |  |
| `parallel` Games played side by side | number | `256` |  |
| `eval_every` Test every (games) | number | `4000` |  |
| `explore_start` Random moves at the start | number | `0.5` |  |
| `explore_end` Random moves at the end | number | `0.05` |  |
| `lookahead` Weight of the look ahead | number | `0.8` | 0: learn only from how games ended; 1: only from the network's look one move ahead |
| `updates` Training steps per batch of games | number | `8` |  |
| `lr` Learning rate | number | `0.002` |  |
| `seed` Seed | number | `1` |  |

## Use it in EsaSim

In a workspace, open **Projects > Blocks > New block**, choose this kind of block, copy the ports and settings above and paste the code; or ask an AI assistant connected to EsaSim (MCP) to create it from this folder.
