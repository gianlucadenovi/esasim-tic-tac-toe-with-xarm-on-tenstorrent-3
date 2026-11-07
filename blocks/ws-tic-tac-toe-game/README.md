# Tic-tac-toe game

The xArm plays tic-tac-toe on paper against a visitor, in simulation (the digital twin) or with the real arm and camera.

Robot = simulation: the digital twin of the xArm 6 (UFACTORY's kinematics) draws with its pen on simulated paper, the simulated camera takes the pictures, and the visitor clicks a cell on the board in the Controls view (or a simulated visitor plays, Visitor = random or perfect): a simulated hand draws the mark and the game reads it with the camera and the board reader, so the whole chain runs and the reading is checked (live: reading). Robot = real xArm: the arm at Arm IP address (its Python SDK, straight over the network) and the camera of the machine; the visitor draws and presses Your move, or clicks the cell. Who starts: by default the robot and the visitor take turns, the robot starting the first game; each keeps its mark. Safety: every motion is planned whole and checked on the digital twin before anything moves (tool point inside the safe box and away from the base, pen not into the table, elbow and wrist above it, joints within their limits, no sudden swing of a joint), then runs smoothly with speed and acceleration capped (pen at most 150 mm/s, joints at most 60°/s, whatever the settings); with the real arm, its controller is told the limits too (reduced mode, the safe box as a fence). The arm starts at its rest position, goes back there after each move and when the session ends. The board lies with its top towards the arm (rotation 90°), as the visitor sitting opposite reads it. Controls (channel "control"): cell N, Your move, New game, Stop. Live: say, board, thinking, read, camera, paper, joints, pose, scene, limits, result.

An [EsaSim](https://esacode.com/) blueprint block: it runs on a machine, in Python (`backend.py`, `def run(ctx)`).

| | |
|---|---|
| Block | `ws-tic-tac-toe-game` · version 6 |
| Workspace | Gianluca De Novi's workspace |
| Author | Gianluca De Novi |
| Runs on | Any machine |

## Inputs

| Port | Type | Description |
|---|---|---|
| `models` Models | Files (pt) | reader.pt and player.pt (the models dataset, or the training blocks) Several connections. Optional. |
| `robot` Robot model | Files (gz, json) | xarm6.robot.json.gz from xArm 6 model Optional. |

## Outputs

| Port | Type | Description |
|---|---|---|
| `games` Games | Files (json, png) | game_001.json: who started, moves, times, result; the simulated paper |
| `cells` Real cells | Files (npz) | The cells seen in the games, labelled by the game |
| `log` Poses | Table (csv) | poses.csv: every pose (and, in simulation, the joints) |

## Settings

| Key | Type | Default | Description |
|---|---|---|---|
| `robot` Robot | select | `"simulation"` |  |
| `camera` Camera | text | `"simulated"` | simulated (with Robot = simulation), or 0, 1 … a device path or stream address; none = moves from the controls |
| `visitor` Visitor (simulation) | select | `"me (Controls)"` |  |
| `auto_games` Games with a simulated visitor | number | `5` |  |
| `sim_speed` Simulation speed | number | `1` | 1 = the arm's real speed |
| `first` Who starts | select | `"take turns"` | Taking turns, the robot starts the first game, the visitor the second, and so on |
| `robot_plays` The robot plays | select | `"X"` | The mark the robot draws; the visitor draws the other |
| `player` Player | select | `"learned, checked"` |  |
| `accelerator` Accelerator for the models | select | `"CPU"` | The models are tiny: the CPU answers at once |
| `camera_mount` Camera mount | select | `"table"` | On a stand above the table, or on the arm's wrist |
| `camera_pos` Camera position (mm) | text | `""` | x, y, z in the arm's base frame; empty: 20 cm past the board's centre, 45 cm above the table (where the simulated camera is, and where the 3D view draws the real one) |
| `camera_look` Camera looks at (mm) | text | `""` | Empty: the board's centre |
| `camera_hfov` Camera field of view (degrees) | number | `70` |  |
| `wrist_camera` Wrist camera offset (mm) | text | `"70, 0, -40"` | x, y, z in the flange frame |
| `wrist_height` Flange height to read (wrist camera, mm) | number | `330` |  |
| `park` Rest position of the pen (mm) | text | `"150, -250, 250"` | x, y, z where the arm waits: pen up, out of a stand camera's view. It starts there, goes back after each move and at the end. |
| `draw_grid` Draw the grid at the start of each game | bool | `true` |  |
| `keep_playing` Offer a new game after each one | bool | `true` |  |
| `watch` See the visitor's move without the button (real camera) | bool | `false` |  |
| `sure` Least confidence to accept a mark | number | `0.7` |  |
| `minutes` Longest session (minutes) | number | `30` |  |
| `channel` Controls channel | text | `"control"` |  |
| `ip` Arm IP address (real xArm) | text | `"192.168.1.200"` |  |
| `board_x` Board X (mm) | number | `390` | Lower-left corner of the board (marker 3) in the arm's base frame |
| `board_y` Board Y (mm) | number | `-70` |  |
| `board_rotation` Board rotation (degrees) | number | `90` | 90: the top of the sheet towards the arm, the way the visitor sitting opposite reads it |
| `board_size` Board size (mm) | number | `140` |  |
| `table_z` Table height (mm) | number | `0` | The paper's height in the arm's base frame |
| `pen_length` Pen length below the flange (mm) | number | `100` |  |
| `tcp` The arm's TCP is | select | `"flange"` | What the controller's tool point is: the draw and travel heights are of that point |
| `z_draw` Draw height (mm) | number | `100` |  |
| `z_up` Travel height (mm) | number | `130` |  |
| `roll` Tool roll (degrees) | number | `180` |  |
| `pitch` Tool pitch (degrees) | number | `0` |  |
| `yaw` Tool yaw (degrees) | number | `0` |  |
| `speed` Pen speed (mm/s) | number | `80` | The tool point's top speed, in strokes and joint moves alike; never more than 150 |
| `accel` Pen acceleration (mm/s²) | number | `400` | Never more than 800 |
| `joint_speed` Joint speed (degrees/s) | number | `30` | Joint moves (to and from the rest position); never more than 60 |
| `joint_accel` Joint acceleration (degrees/s²) | number | `80` | Never more than 120 |
| `safe_box` Safe box (mm) | text | `"100,600, -350,350, 20,450"` | x min, x max, y min, y max, z min, z max of the tool point: no motion that leaves it is sent, and the real arm's controller gets it as a fence |
| `pen_width` Pen width (mm, simulation) | number | `1.2` |  |
| `width` Camera width (pixels) | number | `1280` |  |
| `height` Camera height (pixels) | number | `720` |  |

## Use it in EsaSim

In a workspace, open **Projects > Blocks > New block**, choose this kind of block, copy the ports and settings above and paste the code; or ask an AI assistant connected to EsaSim (MCP) to create it from this folder.
