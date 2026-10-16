# Task 2A — team 6131

## Before a run

Stop the launch, evaluator, perception and manipulation nodes from any previous
attempt with Ctrl+C. Every attempt needs a newly launched world.

The scripts are in `~/ros2_ws/src/algorithms/scripts/task2a/`.
They are already installed. After editing them, rebuild:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select algorithms
source ~/ros2_ws/install/setup.bash
```

For the submission video, start your screen recorder **before launching**. Show
Gazebo and all four terminals, including the final evaluator score. The task
requires this recording and an unlisted YouTube URL for the same submitted run.

## Terminal 1 — world

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch eyantra_kepler_colony task2a.launch.py
```

Wait for `Task 2A world ready: Start your nodes now.`

## Terminal 2 — evaluator

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src
./eyrc-sc-evaluator --task 2A --team-id 6131
```

Wait for the evaluator's `Ready` before starting either node.

## Terminal 3 — perception

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run algorithms task2A_perception.py
```

## Terminal 4 — manipulation

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run algorithms task2A_manipulation.py
```

Wait for `All four ores have been picked and released in their containers.`
Allow the last ore to settle, then press **q** in terminal 2. Keep the world and
both nodes running until scoring has finished. Stop the recorder after the score
is visible. The official evaluator determines the score; the controller's
progress messages are not a score.

Do not send ROS service calls or publish commands from another terminal during
evaluation. If a protective stop or incomplete-run message appears, save the logs
and restart with a fresh world for the next attempt.

## Package the recorded run

Run this after evaluation, using the unchanged scripts that produced that run:

```bash
cd ~/ros2_ws/src/SC-6131-task-2A
cp ~/ros2_ws/src/algorithms/scripts/task2a/task2A_perception.py .
cp ~/ros2_ws/src/algorithms/scripts/task2a/task2A_manipulation.py .
python3 - <<'PY'
from zipfile import ZipFile, ZIP_DEFLATED
from pathlib import Path
output = Path.home() / 'ros2_ws' / 'SC#6131_task2A.zip'
with ZipFile(output, 'w', ZIP_DEFLATED) as archive:
    for name in ('result.zip', 'task2A_perception.py', 'task2A_manipulation.py'):
        archive.write(name, arcname=name)
print(output)
PY
```

This creates `~/ros2_ws/SC#6131_task2A.zip`, with exactly three files at its top
level. It embeds the evaluator's `result.zip` unchanged. Do not unpack, edit or
rebuild that signed inner file. Copy the result elsewhere before another
evaluation: the evaluator replaces its output directory on each run.

Upload the submission zip and the unlisted video URL for that same run on the
eYRC portal.
