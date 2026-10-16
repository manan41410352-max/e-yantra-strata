# e-Yantra StrataCobot — Team 6131

ROS 2 Jazzy · Gazebo Harmonic

Task 2A scored **50/50 in two fresh-world evaluations**, with all eight pickups
succeeding on their first attempt and no protective stops. See the
[evaluation records](validation/task2a/README.md) and
[complete run and submission instructions](TASK2A_RUN.md).

## Workspace

| Task | Solution |
| --- | --- |
| 1A | [task1A.py](src/algorithms/scripts/task1a/task1A.py) |
| 1B | [task1B.py](src/algorithms/scripts/task1b/task1B.py) |
| 1C | [task1C.py](src/algorithms/scripts/task1c/task1C.py) |
| 2A perception | [task2A_perception.py](src/algorithms/scripts/task2a/task2A_perception.py) |
| 2A manipulation | [task2A_manipulation.py](src/algorithms/scripts/task2a/task2A_manipulation.py) |

The repository root is the ROS workspace. ROS packages and the evaluator are in
`src/`. The simulator assets and original history come from the official
[e-Yantra repository](https://github.com/eYantra-Robotics-Competition/eYRC_26-27_Strata-Cobot).
Existing notices and the [UR description license](src/ur_description/LICENSE)
are retained.

## Set up a fresh machine

Use Ubuntu 24.04 with ROS 2 Jazzy installed. Clone into a new `~/ros2_ws` directory:

```bash
git clone https://github.com/manan41410352-max/e-yantra-strata.git ~/ros2_ws
cd ~/ros2_ws
./requirements.sh
```

## Build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Source ROS and the workspace in each new terminal before running ROS commands.

## Run Task 2A

Start your screen recorder before the launch. Use four terminals and leave the world
running throughout the attempt.

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch eyantra_kepler_colony task2a.launch.py
```

Wait for `Task 2A world ready: Start your nodes now.`

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src
./eyrc-sc-evaluator --task 2A --team-id 6131
```

Wait for the evaluator's `Ready` message before starting either node.

Terminal 3:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run algorithms task2A_perception.py
```

Terminal 4:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run algorithms task2A_manipulation.py
```

Let both nodes run until the arm finishes sorting. Press `q` in the evaluator. It writes
`result.zip` under `src/SC-<YOUR_TEAM_ID>-task-2A/`; do not unpack or edit it. Submit
that untouched file and copies of both scripts together in
`SC#<YOUR_TEAM_ID>_task2A.zip`, with all three files at the archive root. Upload the
recording as an Unlisted YouTube video and provide its URL on the portal.
