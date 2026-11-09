# xArm joint test

Checks that the xArm is online and healthy, then moves it one joint at a time, slowly, from the Controls view.

Connects to the arm at Arm IP address and reports its health (live: health — connected, state, mode, error, warning, hottest joint; the log has the firmware and serial number). If the arm cannot be reached, the log says why: this machine has no address on the arm's network (and how to give it one), nothing answers at that address, or something else holds the connection. If the arm reports a joint error (C10 to C17), the status and the log name the joint, its own servo error code (S…) and UFACTORY's instruction for it (live: servo). Then each press of J1 − … J6 + in a Controls view moves that one joint by Step degrees, at most Joint speed (never more than 30°/s); select step changes the step (at most 15°). Every step is checked before it is sent: the joint stays 5° inside its limits, and the elbow, wrist, flange (and the tool, if Tool length is set) stay at least Clearance above the table. Presses made while the arm moves are dropped. Stop stops the motion at once; Back to start returns every joint to where it was when the test began; Clear error clears the arm's error after you fixed its cause; End test finishes. The controller is also asked to limit the joint speed by itself (reduced mode, restored at the end), but older firmware (1.4 and before) does not have reduced mode or self-collision detection: each call is checked and the log says what is actually in force rather than assuming it was taken. Whatever the controller accepts, every step is sent with its own speed and acceleration cap and checked before it leaves, so the test is safe on any firmware. The arm stays where it is when the test ends. Robot = simulation moves the digital twin only. Controls (channel "control"): J1 − … J6 +, step N, Stop, Back to start, Clear error, End test. Live: status, health, servo, joints, jog (one line per joint), scene.

An [EsaSim](https://esacode.com/) blueprint block: it runs on a machine, in Python (`backend.py`, `def run(ctx)`).

| | |
|---|---|
| Block | `ws-xarm-joint-test` · version 4 |
| Workspace | Gianluca De Novi's workspace |
| Author | Gianluca De Novi |
| Runs on | Any machine |

## Outputs

| Port | Type | Description |
|---|---|---|
| `log` Test log | Table (csv) | joint_test.csv: every move asked, refused and reached, with the joint angles |

## Settings

| Key | Type | Default | Description |
|---|---|---|---|
| `robot` Robot | select | `"real xArm"` | simulation: nothing is sent to any arm, the digital twin moves |
| `ip` Arm IP address | text | `"192.168.1.200"` |  |
| `step` Step (degrees per press) | number | `5` |  |
| `joint_speed` Joint speed (degrees/s) | number | `10` | Never more than 30 |
| `joint_accel` Joint acceleration (degrees/s²) | number | `40` |  |
| `table_z` Table height (mm) | number | `0` | The table's height in the arm's base frame (0: the arm stands on it) |
| `clearance` Clearance above the table (mm) | number | `40` |  |
| `tool_length` Tool length below the flange (mm) | number | `0` | The pen or gripper, if mounted: its tip is kept above the table too |
| `minutes` Longest test (minutes) | number | `30` |  |
| `channel` Controls channel | text | `"control"` |  |
| `sim_start` Start pose in simulation (degrees) | text | `"0,-20,-30,0,50,0"` |  |

## Use it in EsaSim

In a workspace, open **Projects > Blocks > New block**, choose this kind of block, copy the ports and settings above and paste the code; or ask an AI assistant connected to EsaSim (MCP) to create it from this folder.
