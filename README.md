# Tic-tac-toe with xArm on Tenstorrent

Investor demo: the xArm plays tic-tac-toe on paper with a marker; the board reader and the player are trained during the meeting on the worker's accelerators.

An [EsaSim](https://esacode.com/) project of the workspace Gianluca De Novi's workspace (Robotics & manipulation).

## Blueprints

- [0 · Robot test (online, joint by joint)](blueprints/0-robot-test-online-joint-by-joint.json) · 7 blocks · version 2
- [1 · Print the board](blueprints/1-print-the-board.json) · 2 blocks
- [2 · Train the board reader (simulation + rendered cells)](blueprints/2-train-the-board-reader-simulation-rendered-cells.json) · 14 blocks · version 1
- [3 · Train the player by self-play](blueprints/3-train-the-player-by-self-play.json) · 6 blocks · version 1
- [4 · Calibrate the arm and the camera](blueprints/4-calibrate-the-arm-and-the-camera.json) · 8 blocks
- [5 · Play (simulation or the real xArm)](blueprints/5-play-simulation-or-the-real-xarm.json) · 18 blocks · version 3

## Workspace blocks it uses

- [xArm joint test](blocks/ws-xarm-joint-test) · version 2
- [xArm 6 model](blocks/ws-xarm-6-model) · version 1
- [Board sheet](blocks/ws-board-sheet) · version 2
- [Simulate games](blocks/ws-simulate-games) · version 1
- [Render cells](blocks/ws-render-cells) · version 1
- [Train cell reader](blocks/ws-train-cell-reader) · version 2
- [Train player by self-play](blocks/ws-train-player-by-self-play) · version 2
- [Read board](blocks/ws-read-board) · version 2
- [Tic-tac-toe game](blocks/ws-tic-tac-toe-game) · version 6

Keys pasted in blocks (secret settings) are not saved here. Datasets, results and the team stay in EsaSim.
