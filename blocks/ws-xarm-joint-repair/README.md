# xArm joint repair

A wizard for UFACTORY's joint-encoder repair (S17, S18) and zero reset: one instruction at a time, a ✓ or ✗ after each, then the next step.

For an xArm whose UFACTORY Studio has no Joint debug page (before 2.4.0), or cannot be updated. Connects to the arm at Arm IP address, reads which joint reports an error, and leads you one step at a time in a Step-by-step guide view (live: guide): read the instruction, do it, press the button it shows; the block checks what it can on the arm, shows ✓ or ✗ and the next step. Steps: read the arm; mark the joint; send the clear command (H101 D0813 V2 I<joint>, or V256); E-stop; clear the arm's error; is the zero right? If not: unlock the joint (H101 D0104 V1 I<joint>), turn it by hand to its zero, E-stop, make this position 0° (D13 I<joint>, sent as the SDK's set_servo_zero), E-stop, motors on, check. A command is sent only from the step that shows it (the orange button); a press at the wrong step does nothing and says which one the step needs; an error at a step shows as ✗ and the run goes on. Back, Start over, Finish. Every command and the arm's answer go to joint_repair.csv. Arm: the arm's type (firmware 1.4 does not tell the SDK). The joints that hold the arm up (2 and 3; 2 and 4 on an xArm 7) make the arm fall when unlocked: their step says so. Robot model (optional): connect the xArm model block, so it is made before this block starts and the 3D view shows the arm during the repair. Robot = rehearsal sends nothing: a pretend arm with an S18 error answers. Live: guide, health (connected, error, servo, angles), joints. Use only as UFACTORY's support tells you: these are their debug commands.

An [EsaSim](https://esacode.com/) blueprint block: it runs on a machine, in Python (`backend.py`, `def run(ctx)`).

| | |
|---|---|
| Block | `ws-xarm-joint-repair` · version 4 |
| Workspace | Gianluca De Novi's workspace |
| Author | Gianluca De Novi |
| Runs on | Any machine |

## Inputs

| Port | Type | Description |
|---|---|---|
| `robot` Robot model | Files (gz, txt) | From xArm model: connected, the model is made before the repair starts, so the 3D view shows the arm Optional. |

## Outputs

| Port | Type | Description |
|---|---|---|
| `log` Repair log | Table (csv) | joint_repair.csv: every command sent, the arm's answer, its errors and joint angles at that moment |

## Settings

| Key | Type | Default | Description |
|---|---|---|---|
| `robot` Robot | select | `"rehearsal"` | rehearsal: nothing is sent to any arm; a pretend arm with an S18 error answers, to learn the steps |
| `arm` Arm | select | `"xArm 6"` | Older firmware (1.4) does not tell the SDK which arm it is |
| `ip` Arm IP address | text | `"192.168.1.200"` |  |
| `rehearsal_joint` Joint with the error (rehearsal) | number | `1` |  |
| `minutes` Longest session (minutes) | number | `60` |  |
| `channel` Controls channel | text | `"control"` |  |

## Use it in EsaSim

In a workspace, open **Projects > Blocks > New block**, choose this kind of block, copy the ports and settings above and paste the code; or ask an AI assistant connected to EsaSim (MCP) to create it from this folder.
