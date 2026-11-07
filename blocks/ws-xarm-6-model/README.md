# xArm 6 model

The UFACTORY xArm 6 for the digital twin: kinematics and 3D meshes (UFACTORY's own, BSD-3), for the twin view and the simulated camera.

Downloads UFACTORY's meshes of the xArm 6 (github.com/xArm-Developer/xarm_ros, BSD-3-Clause, Copyright UFACTORY Inc.) once into the worker's data folder (cache/xarm6), merges details smaller than Detail, and writes xarm6.robot.json.gz (about 220 KB: joints, limits, meshes in mm) with the license. Connect it to the xArm twin view and to Simulate games / Tic-tac-toe game (Robot model input), which draw the arm in the simulated camera. The machine needs to reach raw.githubusercontent.com the first time.

An [EsaSim](https://esacode.com/) blueprint block: it runs on a machine, in Python (`backend.py`, `def run(ctx)`).

| | |
|---|---|
| Block | `ws-xarm-6-model` · version 1 |
| Workspace | Gianluca De Novi's workspace |
| Author | Gianluca De Novi |
| Runs on | Any machine |

## Outputs

| Port | Type | Description |
|---|---|---|
| `model` Robot model | Files (gz, txt) | xarm6.robot.json.gz (esasim.robot/1) and LICENSE-xarm_ros.txt |

## Settings

| Key | Type | Default | Description |
|---|---|---|---|
| `detail` Detail (mm) | number | `3` | Details smaller than this merge: 3 mm looks like the arm and stays small |

## Use it in EsaSim

In a workspace, open **Projects > Blocks > New block**, choose this kind of block, copy the ports and settings above and paste the code; or ask an AI assistant connected to EsaSim (MCP) to create it from this folder.
