# Task 2B — Run instructions

Team 6131 · Home → ore drop pose → arm pose → home

Start each attempt with a fresh world. Build the algorithms package after any source
edit, as described in the [workspace README](../README.md). For a portal recording,
start the screen recorder before launching and show Gazebo and the four terminals.

## Terminal 1 — World

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch eyantra_kepler_colony task2b.launch.py
```

Wait for `Task 2B world ready: Start your nodes now.`

## Terminal 2 — Evaluator

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src
./eyrc-sc-evaluator --task 2B --team-id 6131
```

Wait for `Ready` before starting either task node.

## Terminal 3 — Planner

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run algorithms task2B_planning.py
```

## Terminal 4 — Navigation

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run algorithms task2B_navigation.py
```

Let the eBot finish all three legs and come to a complete stop at home. The planner
requests the ore package at the drop pose automatically. Press **q** in the evaluator
after completion and keep the world and nodes running until scoring finishes.

The evaluator writes `~/ros2_ws/src/SC-6131-task-2B/result.zip`. Preserve this signed
file unchanged and package it with the two scripts used for the run, following the
[submission guide](SUBMISSION.md). Save it before another evaluation replaces the output.

During evaluation, let the nodes make the service calls and publish commands;
terminal `ros2 service call` and `ros2 topic pub` commands invalidate the attempt.
