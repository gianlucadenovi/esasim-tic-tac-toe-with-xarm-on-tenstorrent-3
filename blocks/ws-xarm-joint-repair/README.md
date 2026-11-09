# xArm joint repair

UFACTORY's procedure for a joint's encoder error (S17, S18) and for setting a joint's zero again, step by step from the Controls view, without Studio's Joint debug page.

For an xArm whose UFACTORY Studio has no Joint debug page (before 2.4.0), or that cannot be updated. It connects to the arm at Arm IP address, reads which joint reports an error (S17 single-turn or S18 multi-turn encoder error) and guides you, one step at a time in the Controls view (live: guide), through UFACTORY's procedure: mark the joint; Clear the encoder error (H101 D0813 V2 I<joint>, or V256 for S18); E-stop; check the zero; if it was lost, Unlock the joint (H101 D0104 V1 I<joint>), turn it by hand to its zero, E-stop, Set this position as zero (D13 I<joint>, sent as the SDK's set_servo_zero), E-stop; Clear the arm's error. Every command that changes the arm is sent only on a second press of its button within 15 seconds, and is written to joint_repair.csv with the arm's answer. Joints 2 and 3 hold the arm up: unlocked, the arm falls unless someone holds it. Robot = rehearsal sends nothing (a pretend arm with an S17 error answers), to learn the steps first. Controls (channel "control"): select joint auto,1…7 · select clear S17-single-turn,S18-multi-turn · Check the arm · Clear the encoder error · I pressed and released the E-stop · Unlock the joint · Set this position as zero · Clear the arm's error · Finish. Live: guide (step, text, joint), health (connected, state, error, warning, servo, angles), joints. Use only as UFACTORY's support tells you: these are their debug commands.

An [EsaSim](https://esacode.com/) blueprint block: it runs on a machine, in Python (`backend.py`, `def run(ctx)`).

| | |
|---|---|
| Block | `ws-xarm-joint-repair` · version 1 |
| Workspace | Gianluca De Novi's workspace |
| Author | Gianluca De Novi |
| Runs on | Any machine |

## Outputs

| Port | Type | Description |
|---|---|---|
| `log` Repair log | Table (csv) | joint_repair.csv: every command sent, the arm's answer, its errors and joint angles at that moment |

## Settings

| Key | Type | Default | Description |
|---|---|---|---|
| `robot` Robot | select | `"rehearsal"` | rehearsal: nothing is sent to any arm; a pretend arm with an S17 error answers, to learn the steps |
| `ip` Arm IP address | text | `"192.168.1.200"` |  |
| `rehearsal_joint` Joint with the error (rehearsal) | number | `5` |  |
| `minutes` Longest session (minutes) | number | `60` |  |
| `channel` Controls channel | text | `"control"` |  |

## Use it in EsaSim

In a workspace, open **Projects > Blocks > New block**, choose this kind of block, copy the ports and settings above and paste the code; or ask an AI assistant connected to EsaSim (MCP) to create it from this folder.
