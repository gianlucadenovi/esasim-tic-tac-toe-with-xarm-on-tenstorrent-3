# Step-by-step guide

One step of a procedure that a running block leads: what to do, whether the last action worked (✓/✗), and only the buttons that step takes.

For procedures a person does with a machine block guiding them (repairs, calibrations, set-ups). Connect a Telemetry block's stream (from the block that leads) to Data. The block sends its step with ctx.telemetry("guide", {seq, step, text, last, buttons, extra, choices, busy}): seq grows with every item; step is the title; text the instruction (lines starting with 1. or • become a list; a line indented by 4 spaces shows as a command); last the result of the last action, starting with ✓ (green) or ✗ (red); buttons [{label, kind}] the step's buttons (kind send: orange, it sends a command to a device); extra small buttons under the step (Back, Start over, Finish); choices [{name, label, options, value}] drop-downs; busy a text while the block works (buttons off). A press sends {"control": label, "value": true} on the Channel (a choice: {"control": name, "value": option}); the view then says it waits until the next step comes. Facts: live values shown small under the step, one per line as CHANNEL.FIELD=Label.

An [EsaSim](https://esacode.com/) blueprint block: a view that draws in the browser, in JavaScript (`frontend.js`, `function render(ctx)`).

| | |
|---|---|
| Block | `ws-step-by-step-guide` · version 2 |
| Workspace | Gianluca De Novi's workspace |
| Author | Gianluca De Novi |

## Inputs

| Port | Type | Description |
|---|---|---|
| `data` Data | Stream | The live stream of a Telemetry block reading the block that leads the procedure Several connections. Optional. |

## Settings

| Key | Type | Default | Description |
|---|---|---|---|
| `channel` Channel | text | `"control"` | The presses go to the running blocks on this channel |
| `to` Only to the block | text | `""` | A block's id on the grid; empty = every running block |
| `guide_channel` Guide channel | text | `"guide"` |  |
| `facts` Facts | textarea | `""` | Live values under the step, one per line: CHANNEL.FIELD=Label |

## Use it in EsaSim

In a workspace, open **Projects > Blocks > New block**, choose this kind of block, copy the ports and settings above and paste the code; or ask an AI assistant connected to EsaSim (MCP) to create it from this folder.
