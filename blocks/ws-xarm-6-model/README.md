# xArm model

The UFACTORY xArm 6 or xArm 7 for the digital twin: kinematics and 3D meshes (UFACTORY's own, BSD-3), for the 3D view and the simulated camera.

Downloads UFACTORY's meshes of the arm (github.com/xArm-Developer/xarm_ros, BSD-3-Clause, Copyright UFACTORY Inc.) once into the worker's data folder (cache/xarm6 or cache/xarm7), merges details smaller than Detail, and writes xarm6.robot.json.gz or xarm7.robot.json.gz (esasim.robot/1: joints, limits, meshes in mm, about 200-300 KB) with the license. Connect it to a 3D view (posed live by the joints channel) and to the blocks that drive or simulate the arm (their Robot model input): connected to the block that drives the arm, it is made before that block starts, so the 3D view has the arm while the run goes on. The machine needs to reach raw.githubusercontent.com the first time.

An [EsaSim](https://esacode.com/) blueprint block: it runs on a machine, in Python (`backend.py`, `def run(ctx)`).

| | |
|---|---|
| Block | `ws-xarm-6-model` · version 2 |
| Workspace | Gianluca De Novi's workspace |
| Author | Gianluca De Novi |
| Runs on | Any machine |

## Outputs

| Port | Type | Description |
|---|---|---|
| `model` Robot model | Files (gz, txt) | xarm6.robot.json.gz or xarm7.robot.json.gz (esasim.robot/1) and LICENSE-xarm_ros.txt |

## Settings

| Key | Type | Default | Description |
|---|---|---|---|
| `arm` Arm | select | `"xArm 6"` |  |
| `detail` Detail (mm) | number | `3` | Details smaller than this merge: 3 mm looks like the arm and stays small |

## Use it in EsaSim

In a workspace, open **Projects > Blocks > New block**, choose this kind of block, copy the ports and settings above and paste the code; or ask an AI assistant connected to EsaSim (MCP) to create it from this folder.
