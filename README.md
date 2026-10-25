# StrataCobot — Team 6131

e-Yantra Robotics Competition 2026–27 · Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic

**Authors:** Agrim Chauhan, Saksham Kaushal, Rayhan Khan, Manan Sood

Solutions for Tasks 1A, 1B, 1C, 2A and 2B. Saved official evaluator runs achieved
**50/50 for Task 2A** and **50/50 for Task 2B**. See the
[submission guide](docs/SUBMISSION.md) for the prepared archives and recording requirements.

## Workspace layout

```text
ros2_ws/
├── src/
│   ├── algorithms/             # Team solutions and original boilerplates
│   ├── ebot_description/       # Official eBot assets
│   ├── eyantra_kepler_colony/   # Official worlds, launches and arena plugin
│   ├── ur_description/         # Official UR7e assets and controller
│   ├── eyrc-sc-evaluator       # Official evaluator
│   └── requirements.sh        # Official dependency installer
├── docs/                      # Run and submission instructions
├── results/                   # Local evaluator results and score logs (not tracked)
├── submissions/               # Task 2A and 2B submission ZIPs
├── build/                     # Existing colcon build output
└── install/                   # Installed ROS packages
```

## Task scripts

| Task | Purpose | Source |
| --- | --- | --- |
| 1A | Ore detection and TF publication | [task1A.py](src/algorithms/scripts/task1a/task1A.py) |
| 1B | UR7e waypoint control | [task1B.py](src/algorithms/scripts/task1b/task1B.py) |
| 1C | eBot path following | [task1C.py](src/algorithms/scripts/task1c/task1C.py) |
| 2A | Ore perception | [task2A_perception.py](src/algorithms/scripts/task2a/task2A_perception.py) |
| 2A | Magnetic pickup and sorting | [task2A_manipulation.py](src/algorithms/scripts/task2a/task2A_manipulation.py) |
| 2B | Occupancy grid path planning | [task2B_planning.py](src/algorithms/scripts/task2b/task2B_planning.py) |
| 2B | eBot navigation | [task2B_navigation.py](src/algorithms/scripts/task2b/task2B_navigation.py) |

Each script includes the team, authors, filename, functions, node name, topics and
service clients used by that script.

## Build

On a machine with ROS 2 Jazzy installed, install the task dependencies:

```bash
cd ~/ros2_ws/src
./requirements.sh
```

Build from the workspace root:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

After editing a task script, rebuild the algorithms package:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select algorithms
source install/setup.bash
ros2 pkg executables algorithms
```

## Run and submit

- [Task 2A run instructions](docs/TASK2A_RUN.md)
- [Task 2B run instructions](docs/TASK2B_RUN.md)
- [Submission files and packaging](docs/SUBMISSION.md)
- [Saved results](results/README.md)

Source ROS and `~/ros2_ws/install/setup.bash` in every terminal. Start a fresh world,
wait for its ready message, start the evaluator, and start the task nodes after the
evaluator reports `Ready`.

## Upstream repository

The simulator, evaluator, controller, assets and boilerplates come from the official
[e-Yantra StrataCobot repository](https://github.com/eYantra-Robotics-Competition/eYRC_26-27_Strata-Cobot).
Compared with `origin/main` at `9c9f6d0` (`v2.1.0`) on 21 October 2026: all official
files are retained; simulator assets, binaries, boilerplates and the installer match
upstream byte for byte. The team additions are the task solutions, package metadata,
workspace documentation and saved results.

Original e-Yantra notices and the [UR description license](src/ur_description/LICENSE)
are retained. Local evaluator results, submission archives, build output and installed packages
are excluded from Git; the verified scores are documented in the submission guide.
