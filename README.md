<div align="center">

<!-- Banner: upload a picture named banner.png under Data › Readme pictures; it shows here. -->

# Tic-tac-toe with xArm on Tenstorrent

**Investor demo: the xArm plays tic-tac-toe on paper with a marker; the board reader and the player are trained during the meeting on the worker's accelerators.**

![type](https://img.shields.io/badge/type-Robotics%20%26%20manipulation-blue) ![language](https://img.shields.io/badge/language-Python-3776AB) ![runs on](https://img.shields.io/badge/runs%20on-CPU%20%7C%20NVIDIA%20%7C%20Tenstorrent-6f42c1) [![made with](https://img.shields.io/badge/made%20with-EsaSim-0b7285)](https://esacode.com/)

<!-- More badges: ![licence](https://img.shields.io/badge/licence-MIT-green) — label, value and colour, written in the address. -->

</div>

## Flows

- **0 · Robot test (online, joint by joint)**: Checks that the xArm answers and is healthy, then moves it one joint at a time, slowly, from the View: J1 − … J6 +, Back to start, Stop.
- **0b · Repair a joint (encoder error, zero)**: UFACTORY's procedure for a joint's S17/S18 encoder error and its zero, step by step with buttons, for an arm without Studio's Joint debug page. Starts in rehearsal: set Robot to real xArm when ready.
- **1 · Print the board**: The A4 sheet with the four markers the camera finds the board by. Print it at 100%.
- **2 · Train the board reader (simulation + rendered cells)**: Synthetic cells into the cells-synthetic dataset, then a CNN that reads empty, X or O, trained on the accelerator; the model goes to the models dataset.
- **3 · Train the player by self-play**: Thousands of games against itself on the accelerator, tested against a random and a perfect player as it learns; the model goes to the models dataset.
- **4 · Calibrate the arm and the camera**: The pen touches the board's four corners (dry run first), and the camera checks it finds and straightens the board.
- **5 · Play (simulation or the real xArm)**: The robot plays the visitor: camera and board reader see the visitor's moves, the player network chooses the robot's, the arm draws. Your move, New game and Stop are in the View.
- **6 · Teach by hand and repeat**: Record moves the arm into manual mode: guide it by hand while the flow records. Stop ends the recording. Repeat plays it back N times at the chosen speed, then the arm returns to where it started.

## What you need

- A machine with the EsaSim worker (Project > Compute); it runs on any machine, faster with: NVIDIA GPU, Tenstorrent.
- Data it reads: the dataset **models** of the workspace, read by *models (reader.pt, player.pt)*.
- Python packages, installed by the worker the first time: `numpy`, `xarm-python-sdk`, `opencv-python-headless>=4.7`, `pillow`, `torch`.

## How to run it

1. Under **Compute**, check that a machine is available for this project.
2. Press **Run**, choose the machine, and follow the run on the flow's **View** page.
3. The files it made are under **Results**; they stay on the machine until you save or download them.

## What you get

- On the **View** page: *Arm data for the views* (0b · Repair a joint (encoder error, zero)), *Move the arm* (0 · Robot test (online, joint by joint)), *The arm as it is now* (0 · Robot test (online, joint by joint)), *Arm health (error, warning, state, hottest joint °C)* (0 · Robot test (online, joint by joint)), *Joints* (0b · Repair a joint (encoder error, zero)), *Repair, step by step* (0b · Repair a joint (encoder error, zero)), *The arm as it reads now* (0b · Repair a joint (encoder error, zero)), *The sheet* (1 · Print the board), *The simulated camera* (2 · Train the board reader (simulation + rendered cells)), *Cells cut from the simulated pictures* (2 · Train the board reader (simulation + rendered cells)), *Simulation* (2 · Train the board reader (simulation + rendered cells)), *Rendered cells as they are made* (2 · Train the board reader (simulation + rendered cells)).

## Data and licences

- the dataset **models** of the workspace, read by *models (reader.pt, player.pt)*

<!-- esasim: files of the project -->

---

An [EsaSim](https://esacode.com/) project of the workspace Gianluca De Novi's workspace (Robotics & manipulation).

### Blueprints in this repository

- [0 · Robot test (online, joint by joint)](blueprints/0-robot-test-online-joint-by-joint.json) · 7 blocks · version 4
- [0b · Repair a joint (encoder error, zero)](blueprints/0b-repair-a-joint-encoder-error-zero.json) · 6 blocks · version 3
- [1 · Print the board](blueprints/1-print-the-board.json) · 2 blocks
- [2 · Train the board reader (simulation + rendered cells)](blueprints/2-train-the-board-reader-simulation-rendered-cells.json) · 14 blocks · version 2
- [3 · Train the player by self-play](blueprints/3-train-the-player-by-self-play.json) · 6 blocks · version 1
- [4 · Calibrate the arm and the camera](blueprints/4-calibrate-the-arm-and-the-camera.json) · 8 blocks
- [5 · Play (simulation or the real xArm)](blueprints/5-play-simulation-or-the-real-xarm.json) · 18 blocks · version 4
- [6 · Teach by hand and repeat](blueprints/6-teach-by-hand-and-repeat.json) · 6 blocks

### Workspace blocks it uses

- [xArm joint test](blocks/ws-xarm-joint-test) · version 5
- [xArm model](blocks/ws-xarm-6-model) · version 2
- [xArm joint repair](blocks/ws-xarm-joint-repair) · version 5
- [Step-by-step guide](blocks/ws-step-by-step-guide) · version 2
- [Board sheet](blocks/ws-board-sheet) · version 2
- [Simulate games](blocks/ws-simulate-games) · version 1
- [Render cells](blocks/ws-render-cells) · version 1
- [Train cell reader](blocks/ws-train-cell-reader) · version 2
- [Train player by self-play](blocks/ws-train-player-by-self-play) · version 2
- [Read board](blocks/ws-read-board) · version 2
- [Tic-tac-toe game](blocks/ws-tic-tac-toe-game) · version 6
- [xArm teach and repeat](blocks/ws-xarm-teach-and-repeat) · version 1

Keys pasted in blocks (secret settings) are not saved here. Datasets, results and the team stay in EsaSim.
