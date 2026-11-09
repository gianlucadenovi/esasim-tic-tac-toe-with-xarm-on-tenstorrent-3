# xArm joint repair

A wizard for UFACTORY's joint-encoder repair (S17, S18) and zero reset: one instruction at a time, a ✓ or ✗ after each, then the next step.

For an xArm whose UFACTORY Studio has no Joint debug page (before 2.4.0), or cannot be updated. Connects to the arm at Arm IP address, reads which joint reports an error, and guides you in a Controls view one step at a time (live: guide.step, guide.text, guide.press, guide.last, guide.command): read the instruction, do it, press the button it names (Done, next · Send this command · Yes · No); the block checks what it can on the arm, shows ✓ or ✗ (guide.last) and the next step. Steps: read the arm; mark the joint; send the clear command (H101 D0813 V2 I<joint>, or V256: select command); E-stop; clear the arm's error; is the zero right? If not: unlock the joint (H101 D0104 V1 I<joint>), turn it by hand to its zero, E-stop, make this position 0° (D13 I<joint>, sent as the SDK's set_servo_zero), E-stop, motors on, check. A command is sent only from the step that shows it; a button pressed at the wrong step does nothing and says which one the step needs. Back goes one step back, Start over reads the arm again, Finish ends. Every command and the arm's answer go to joint_repair.csv. The joints that hold the arm up (2 and 3; 2 and 4 on an xArm 7) make the arm fall when unlocked: their step says so. Robot = rehearsal sends nothing: a pretend xArm 7 with an S18 error answers. Controls (channel "control"): select joint auto,1…7 · select command V2,V256 · button Done, next · Send this command · Yes · No · Back · Start over · Finish. Live: guide, health (connected, error, servo, angles), joints. Use only as UFACTORY's support tells you: these are their debug commands.

An [EsaSim](https://esacode.com/) blueprint block: it runs on a machine, in Python (`backend.py`, `def run(ctx)`).

| | |
|---|---|
| Block | `ws-xarm-joint-repair` · version 2 |
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
| `robot` Robot | select | `"rehearsal"` | rehearsal: nothing is sent to any arm; a pretend xArm 7 with an S18 error answers, to learn the steps |
| `ip` Arm IP address | text | `"192.168.1.200"` |  |
| `rehearsal_joint` Joint with the error (rehearsal) | number | `1` |  |
| `minutes` Longest session (minutes) | number | `60` |  |
| `channel` Controls channel | text | `"control"` |  |

## Use it in EsaSim

In a workspace, open **Projects > Blocks > New block**, choose this kind of block, copy the ports and settings above and paste the code; or ask an AI assistant connected to EsaSim (MCP) to create it from this folder.
