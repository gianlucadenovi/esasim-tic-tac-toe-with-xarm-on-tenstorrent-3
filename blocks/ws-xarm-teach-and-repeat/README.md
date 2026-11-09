# xArm teach and repeat

Record a movement by moving the xArm by hand, then have it repeat the movement any number of times, slower or faster, and go back to where it started.

Teaching by hand for the xArm. In a Controls view: Record puts the arm in manual mode (it holds its own weight and you move it by hand; the payload set in UFACTORY Studio must match what is mounted, or the arm drifts) and records its joints about 50 times a second; Stop ends the recording (the arm holds where it is; still moments at both ends are trimmed and the hand's tremor smoothed). Repeat plays it back Repeats times: the arm first goes to the pose where the movement started (a joint move at Move speed), then follows the recorded joints in the same rhythm (servo mode, 100 commands a second), and at the end goes back to that start pose. The Speed slider (25 % to 150 %) plays it slower or faster, also while it plays; no joint ever goes faster than Max joint speed (where the recording is faster, the playback slows down there). Stop stops the arm at once in any phase; Go to start moves to the start pose; End finishes. Clear the space around the arm: the move to the start pose is a straight joint move. Every recording is saved as recording.csv (t, j1…jN in degrees); connect one to Recording to play it again in another run. Robot model (optional): connect the xArm model block, so the 3D view shows the arm. Robot = simulation sends nothing: a pretend hand moves a pretend arm. Controls (channel "control"): Record, Stop, slider repeats, Repeat, slider speed, Go to start, End. Live: status (text, detail, state, progress), joints.

An [EsaSim](https://esacode.com/) blueprint block: it runs on a machine, in Python (`backend.py`, `def run(ctx)`).

| | |
|---|---|
| Block | `ws-xarm-teach-and-repeat` · version 1 |
| Workspace | Gianluca De Novi's workspace |
| Author | Gianluca De Novi |
| Runs on | Any machine |

## Inputs

| Port | Type | Description |
|---|---|---|
| `robot` Robot model | Files (gz, txt) | From xArm model: connected, the model is made before this block starts, so the 3D view shows the arm Optional. |
| `recording` Recording | Table (csv) | A recording.csv from an earlier run, to play again Optional. |

## Outputs

| Port | Type | Description |
|---|---|---|
| `recording` Recording | Table (csv) | recording.csv: t (s) and the joint angles (degrees), smoothed and trimmed |
| `log` Session log | Table (csv) | session.csv: recordings, repeats, speed changes, stops |

## Settings

| Key | Type | Default | Description |
|---|---|---|---|
| `robot` Robot | select | `"real xArm"` | simulation: nothing is sent to any arm |
| `arm` Arm | select | `"xArm 6"` |  |
| `ip` Arm IP address | text | `"192.168.1.200"` |  |
| `repeats` Repeats | number | `3` | Also the repeats slider |
| `speed` Speed (%) | number | `100` | Also the speed slider: 100 = as recorded |
| `max_speed` Max joint speed (degrees/s) | number | `60` | No joint goes faster during a repeat |
| `move_speed` Move speed (degrees/s) | number | `20` | Going to the start pose and back |
| `rate` Recording rate (per second) | number | `50` |  |
| `max_seconds` Longest recording (s) | number | `120` |  |
| `minutes` Longest session (minutes) | number | `60` |  |
| `channel` Controls channel | text | `"control"` |  |

## Use it in EsaSim

In a workspace, open **Projects > Blocks > New block**, choose this kind of block, copy the ports and settings above and paste the code; or ask an AI assistant connected to EsaSim (MCP) to create it from this folder.
