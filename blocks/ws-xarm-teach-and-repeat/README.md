# xArm teach and repeat

Free the xArm and move it by hand, record a movement between two moments, repeat it any number of times as one smooth motion, then send the arm to its zero position.

Teaching by hand for the xArm, from a Controls view. Start frees the arm (manual mode: it holds its own weight and you move it by hand; the payload set in UFACTORY Studio must match what is mounted, or it drifts). Record records the joints about 50 times a second from where the arm is at that moment; Stop recording ends the recording where the arm is then, and the arm holds still (the hand's tremor is smoothed; the first and last poses are kept as they were). Repeat plays the movement Repeats times as one continuous motion: a gentle transition takes the arm from where it is to the start of the movement, and between two repetitions another gentle transition joins the end of one to the start of the next (minimum jerk: starting and ending at rest, at most Move speed); after the last one the arm holds at the end of the movement. The Speed slider (25 % to 150 %) plays slower or faster, also while it plays; no joint ever goes faster than Max joint speed. End takes the arm gently to its zero position (every joint at 0°) and ends the session; pressed during a repeat, the arm stops first. Stop stops any motion at once (or locks the arm again when it is free). Transitions are straight joint moves: clear the space around the arm. Every recording is saved as recording.csv (t, j1…jN in degrees); connect one to Recording to play it in another run. Robot model (optional): connect the xArm model block, so the 3D view shows the arm. Robot = simulation sends nothing: a pretend hand moves a pretend arm. Controls (channel "control"): Start, Record, Stop recording, slider repeats, Repeat, slider speed, Stop, End. Live: status (text, detail, state), joints.

An [EsaSim](https://esacode.com/) blueprint block: it runs on a machine, in Python (`backend.py`, `def run(ctx)`).

| | |
|---|---|
| Block | `ws-xarm-teach-and-repeat` · version 2 |
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
| `recording` Recording | Table (csv) | recording.csv: t (s) and the joint angles (degrees), smoothed |
| `log` Session log | Table (csv) | session.csv: start, recordings, repeats, speed changes, stops, end |

## Settings

| Key | Type | Default | Description |
|---|---|---|---|
| `robot` Robot | select | `"real xArm"` | simulation: nothing is sent to any arm |
| `arm` Arm | select | `"xArm 6"` |  |
| `ip` Arm IP address | text | `"192.168.1.200"` |  |
| `repeats` Repeats | number | `3` | Also the repeats slider |
| `speed` Speed (%) | number | `100` | Also the speed slider: 100 = as recorded |
| `max_speed` Max joint speed (degrees/s) | number | `60` | No joint goes faster while the arm plays |
| `move_speed` Move speed (degrees/s) | number | `20` | The fastest joint during the gentle transitions (to the start, between repeats, to zero) |
| `rate` Recording rate (per second) | number | `50` |  |
| `max_seconds` Longest recording (s) | number | `120` |  |
| `minutes` Longest session (minutes) | number | `60` |  |
| `channel` Controls channel | text | `"control"` |  |

## Use it in EsaSim

In a workspace, open **Projects > Blocks > New block**, choose this kind of block, copy the ports and settings above and paste the code; or ask an AI assistant connected to EsaSim (MCP) to create it from this folder.
