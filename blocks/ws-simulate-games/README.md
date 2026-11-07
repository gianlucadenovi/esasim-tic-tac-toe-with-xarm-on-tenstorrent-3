# Simulate games

Training data from the digital twin: simulated games on paper, seen by the simulated camera, cut into labelled cells.

In each simulated game the arm draws the grid and its marks (the strokes the real arm draws), a simulated visitor draws theirs by hand, and the simulated camera (on a stand above the table, or on the wrist) takes a picture after every move. Every picture goes through the same steps as a real game (markers found, board straightened, nine cells cut) and the game knows what is in each cell: labelled cells (sim_cells_01.npz, the format of Render cells) for Train cell reader. Paper, ink, pens, light, board position and camera position and lens change from game to game (Variety). Connect xArm 6 model so the arm appears in the pictures with its real shape. About 8 pictures per second on one CPU core; 200 games give about 15,000 cells. Use the same board, camera and pen settings as the game.

An [EsaSim](https://esacode.com/) blueprint block: it runs on a machine, in Python (`backend.py`, `def run(ctx)`).

| | |
|---|---|
| Block | `ws-simulate-games` · version 1 |
| Workspace | Gianluca De Novi's workspace |
| Author | Gianluca De Novi |
| Runs on | Any machine |

## Inputs

| Port | Type | Description |
|---|---|---|
| `robot` Robot model | Files (gz, json) | xarm6.robot.json.gz from xArm 6 model Optional. |

## Outputs

| Port | Type | Description |
|---|---|---|
| `cells` Cells | Files (npz) | sim_cells_01.npz: x (N x 32 x 32), y (0 empty, 1 X, 2 O) |
| `samples` Examples | Files (png) |  |
| `frames` Simulated pictures | Files (jpg) | A few of the camera's pictures |
| `metrics` Metrics | Metrics (json) |  |

## Settings

| Key | Type | Default | Description |
|---|---|---|---|
| `games` Games | number | `200` |  |
| `variety` Variety | select | `"some"` | How much paper, light, board and camera change from game to game |
| `camera_mount` Camera | select | `"table"` | On a stand above the table, or on the arm's wrist |
| `camera_pos` Camera position (mm) | text | `""` | x, y, z in the arm's base frame; empty: 20 cm past the board's centre, 45 cm above the table |
| `camera_look` Camera looks at (mm) | text | `""` | x, y, z; empty: the board's centre |
| `camera_hfov` Camera field of view (degrees) | number | `70` |  |
| `wrist_camera` Wrist camera offset (mm) | text | `"70, 0, -40"` | x, y, z in the flange frame |
| `wrist_height` Flange height to read (wrist camera, mm) | number | `330` |  |
| `park` Parked pen (mm) | text | `"150, -250, 250"` | Where the arm waits while a camera on a stand looks |
| `board_x` Board X (mm) | number | `250` |  |
| `board_y` Board Y (mm) | number | `-90` |  |
| `board_rotation` Board rotation (degrees) | number | `0` |  |
| `board_size` Board size (mm) | number | `140` |  |
| `table_z` Table height (mm) | number | `0` | The paper's height in the arm's base frame |
| `pen_length` Pen length below the flange (mm) | number | `100` |  |
| `sim_width` Picture width (pixels) | number | `1280` |  |
| `sim_height` Picture height (pixels) | number | `720` |  |
| `save_frames` Pictures to keep | number | `24` |  |
| `seed` Seed | number | `1` |  |

## Use it in EsaSim

In a workspace, open **Projects > Blocks > New block**, choose this kind of block, copy the ports and settings above and paste the code; or ask an AI assistant connected to EsaSim (MCP) to create it from this folder.
