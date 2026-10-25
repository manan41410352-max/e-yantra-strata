# Task 2A — Run instructions

Team 6131 · Ore perception, magnetic pickup and sorting

Start each attempt with a fresh world. Build the algorithms package after any source
edit, as described in the [workspace README](../README.md). For a portal recording,
start the screen recorder before launching and show Gazebo and the four terminals.

## Terminal 1 — World

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch eyantra_kepler_colony task2a.launch.py
```

Wait for `Task 2A world ready: Start your nodes now.`

## Terminal 2 — Evaluator

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src
./eyrc-sc-evaluator --task 2A --team-id 6131
```

Wait for `Ready` before starting either task node.

## Terminal 3 — Perception

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run algorithms task2A_perception.py
```

## Terminal 4 — Manipulation

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run algorithms task2A_manipulation.py
```

Wait for `All four ores have been picked and released in their containers.` Allow the
last ore to settle, then press **q** in the evaluator. Keep the world and nodes running
until scoring finishes. Stop recording after the score is visible.

The evaluator writes `~/ros2_ws/src/SC-6131-task-2A/result.zip`. Preserve this signed
file unchanged and package it with the two scripts used for the run, following the
[submission guide](SUBMISSION.md). Save it before another evaluation replaces the output.

During evaluation, let the nodes make the service calls and publish commands;
terminal `ros2 service call` and `ros2 topic pub` commands invalidate the attempt.
