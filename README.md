# mazegen_gazebo

A **Gazebo Ignition Fortress** (`gz-sim 6`) world-system plugin that reads a
[micromouseonline](https://github.com/micromouseonline/mazefiles)-format
text maze file and spawns the corresponding walls, posts, and
start/goal markers into a running simulation world.

| | |
|---|---|
| Library (`filename=`) | `libmazegen_plugin.so` |
| Plugin class (`name=`) | `mazegen::MazegenPlugin` |
| Plugin type | World system (`ISystemConfigure + ISystemPreUpdate`) |
| Default cell size | 180 mm (classic micromouse) |
| Default wall | 12 mm thick × 50 mm tall, white sides, red top |
| Default posts | 12 × 12 × 50 mm at every grid intersection |

Geometry and colours follow the official
[micromouse maze rules](https://micromouseonline.com/micromouse-book/mazes-and-maze-solving/):
16 × 16 array of 180 mm unit squares, walls 50 mm high and 12 mm thick with
white sides and red tops, 12 × 12 mm lattice posts, non-gloss black floor.

---

## Gallery

| `mazes/alljapan-001-1980.txt` | `mazes/allamerica2013.txt` |
|---|---|
| ![All-Japan 1980](./documentation/images/alljapan-001-1980.png) | ![All-America 2013](./documentation/images/allamerica2013.png) |

---

## Table of contents

1. [Maze file format](#1-maze-file-format)
2. [Docker (recommended)](#2-docker-recommended)
3. [Build from source](#3-build-from-source)
4. [Run the bundled examples](#4-run-the-bundled-examples)
5. [Install system-wide](#5-install-system-wide)
6. [Using MazegenPlugin in your own world](#6-using-mazegenPlugin-in-your-own-world)
7. [Repository layout](#7-repository-layout)
8. [References & acknowledgements](#8-references--acknowledgements)
9. [Controlling the robot with ROS 2](#9-controlling-the-robot-with-ros-2)

---

## 1. Maze file format

Plain text, as used by
[`micromouseonline/mazefiles`](https://github.com/micromouseonline/mazefiles):

```
o---o---o---o---o---o
|       |       |   |
o   o   o   o   o   o
|   |       |   |   |
o   o   o   o   o   o
|   |   |   | G |   |
o---o   o   o---o   o
|       |       |   |
o   o   o---o   o   o
| S |       |       |
o---o---o---o---o---o
```

| Character | Meaning |
|---|---|
| `o` | Post at a grid intersection |
| `---` | Horizontal wall segment |
| `\|` | Vertical wall segment |
| `S` | Start cell (green floor tile) |
| `G` | Goal cell (red floor tile; may be multiple) |

Row 0 is the bottom-most text row; column 0 is the left-most column.

---

## 2. Docker (recommended)

The fastest way to run the project - no host dependencies beyond Docker and an
X server. Full details in **[docker/README.md](./docker/README.md)**; essentials below.

### Build

```bash
# Run from repo root
docker build -f docker/Dockerfile -t mazegen-gazebo .
```

### Run (two mazes - default)

```bash
xhost +local:docker
docker compose -f docker/docker-compose.yml up
```

### Run (single maze)

```bash
xhost +local:docker
docker compose -f docker/docker-compose.yml run --rm mazegen single
```

GPU options (Intel/AMD `--device /dev/dri --group-add video`, NVIDIA `--gpus all`)
and full `docker run` examples are in [docker/README.md](./docker/README.md).

---

## 3. Build from source

### Prerequisites (Ubuntu 20.04 / 22.04)

```bash
sudo apt install \
  libignition-gazebo6-dev libignition-plugin-dev \
  libignition-common4-dev libignition-math6-dev \
  libignition-msgs8-dev libignition-transport11-dev \
  libsdformat12-dev cmake build-essential
```

### Build plugin

```bash
git clone https://github.com/svaibhav101/mazegen_gazebo.git
cd mazegen_gazebo
cmake -S . -B build
cmake --build build -j
```

Produces `build/libmazegen_plugin.so`.

### Run unit tests

```bash
ctest --test-dir build --output-on-failure
```

### Generate API docs (optional)

```bash
cmake --build build --target doc
# output: build/docs/html/index.html
```

---

## 4. Run the bundled examples

```bash
# Source env vars so Ignition finds the plugin and maze assets
source setup.bash

# Single maze
# -r     : autoplay when gazebo starts
# -v (4) : verbose level
ign gazebo -r -v 4 worlds/maze.sdf

# Two mazes side by side
ign gazebo -r -v 4 worlds/two_mazes.sdf

```

`setup.bash` exports:

```bash
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$PWD/build:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH
export IGN_GAZEBO_RESOURCE_PATH=$PWD:$IGN_GAZEBO_RESOURCE_PATH
```

---

## 5. Install system-wide

```bash
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build -j
sudo cmake --install build
```

Add to `~/.bashrc`:

```bash
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/usr/local/lib:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH
```

For maze files referenced by relative path, also export a resource directory:

```bash
export IGN_GAZEBO_RESOURCE_PATH=/path/to/mazegen_gazebo:$IGN_GAZEBO_RESOURCE_PATH
```

---

## 6. Using MazegenPlugin in your own world

### Single maze

```xml
<plugin filename="libmazegen_plugin.so"
        name="mazegen::MazegenPlugin">
  <maze_file>/absolute/path/to/maze.txt</maze_file>
  <!-- All remaining params are optional; defaults shown -->
  <model_name>maze</model_name>
  <cell_size>0.18</cell_size>             <!-- m -->
  <wall_thickness>0.012</wall_thickness>  <!-- m -->
  <wall_height>0.05</wall_height>         <!-- m -->
  <post_size>0.012</post_size>            <!-- m, default = wall_thickness -->
  <wall_color>1 1 1</wall_color>          <!-- R G B in [0,1] -->
  <cap_color>0.9 0.05 0.05</cap_color>    <!-- R G B in [0,1] -->
  <origin>0 0 0 0 0 0</origin>            <!-- x y z (m)  roll pitch yaw (rad) -->
</plugin>
```

### Multiple mazes

```xml
<plugin filename="libmazegen_plugin.so"
        name="mazegen::MazegenPlugin">
  <!-- Shared params applied to all mazes (all optional) -->
  <cell_size>0.18</cell_size>
  <wall_color>1 1 1</wall_color>
  <cap_color>0.9 0.05 0.05</cap_color>

  <maze>
    <file>mazes/alljapan-001-1980.txt</file>
    <model_name>japan1980</model_name>
    <origin>0 0 0 0 0 0</origin>
  </maze>
  <maze>
    <file>mazes/allamerica2013.txt</file>
    <model_name>america2013</model_name>
    <origin>3 0 0 0 0 0</origin>   <!-- offset 3 m along X -->
  </maze>
</plugin>
```

### Parameters

#### Top-level / shared

| Tag | Type | Default | Description |
|---|---|---|---|
| `<maze_file>` | string | **required** | Path to `.txt` maze: absolute, or relative to `IGN_GAZEBO_RESOURCE_PATH`. |
| `<model_name>` | string | `maze` | Name of the spawned Gazebo model. |
| `<cell_size>` | double | `0.18` | Grid cell pitch (m). |
| `<wall_thickness>` | double | `0.012` | Wall thickness (m). |
| `<wall_height>` | double | `0.05` | Wall height (m). |
| `<post_size>` | double | `wall_thickness` | Square corner post side (m). |
| `<wall_color>` | vec3 | `1 1 1` | Wall body colour R G B ∈ [0,1]. |
| `<cap_color>` | vec3 | `0.9 0.05 0.05` | Top-cap colour R G B ∈ [0,1]. |
| `<origin>` | 3–6 doubles | `0 0 0 0 0 0` | World pose of the SW corner: x y z (m) roll pitch yaw (rad). |

#### Per `<maze>` block (multi-maze)

| Tag | Type | Default | Description |
|---|---|---|---|
| `<file>` | string | **required** | Path to the maze `.txt` file. |
| `<model_name>` | string | `maze_<N>` | Unique model name per world. |
| `<origin>` | 3–6 doubles | `0 0 0 0 0 0` | World pose of this maze's SW corner. |
| geometry/colour | same as above | inherited | Any shared param can be overridden per block. |

### Transport services

Once loaded, the plugin advertises two persistent Ignition transport services
per maze, keyed by `<model_name>`.

#### `/mazegen/<model_name>/spawn_pose`

Returns `ignition::msgs::Pose`:
- **Position**: world-frame centre of the start cell (`S` marker).
- **Orientation**: yaw from the first open side of the start cell (E -> N -> W -> S priority).

```bash
ign service -s /mazegen/allamerica2013/spawn_pose \
    --reqtype ignition.msgs.Empty \
    --reptype ignition.msgs.Pose \
    -r "" --timeout 2000
```

#### `/mazegen/<model_name>/goal_poses`

Returns `ignition::msgs::Pose_V` - one entry per `G` cell in maze-file order:
- **Position**: world-frame centre of the goal cell.
- **Orientation**: identity (goal cells have no facing direction).

```bash
ign service -s /mazegen/allamerica2013/goal_poses \
    --reqtype ignition.msgs.Empty \
    --reptype ignition.msgs.Pose_V \
    -r "" --timeout 2000
```

---

## 7. Repository layout

```
.
├── build/                          # CMake build tree (generated)
├── CMakeLists.txt
├── setup.bash                      # Sets IGN_GAZEBO_SYSTEM_PLUGIN_PATH + RESOURCE_PATH
├── docker/
│   ├── Dockerfile
│   ├── docker-compose.yml
│   ├── entrypoint.sh
│   └── README.md
├── documentation/
│   ├── images/
│   ├── api_reference.md
│   ├── architecture.md
│   ├── ros2_control_guide.md       # ROS 2 bridge setup, topics, algorithm skeleton
│   └── Doxyfile
├── include/
│   ├── mazegen_plugin.h            # MazegenPlugin class declaration
│   ├── maze_parse.h                # Maze struct + ParseMazeFile()
│   ├── maze_params.h               # Params struct + SDF parsing helpers
│   ├── maze_sdf_builder.h          # BuildMazeSdf() declaration
│   └── maze_spawn_utils.h          # CellCenter, SpawnYaw, LogSpawnInfo
├── mazes/                          # Micromouseonline competition mazes
├── src/
│   ├── mazegen_plugin.cpp          # Plugin lifecycle: Configure + PreUpdate
│   ├── maze_parse.cpp              # Maze file parser
│   ├── maze_params.cpp             # SDF parameter parsing
│   ├── maze_sdf_builder.cpp        # Wall merging + SDF XML builder
│   └── maze_spawn_utils.cpp        # Coordinate math + spawn logging
├── test/
│   └── test_maze_parser.cpp        # CTest unit tests for the parser
└── worlds/
    ├── maze.sdf                    # Single-maze example world
    └── two_mazes.sdf               # Two mazes side by side
```

---

## 8. Controlling the robot with ROS 2

The plugin spawns a differential-drive micromouse robot for each maze. You can
read its sensors and send drive commands from any ROS 2 node via the
**ROS–Ignition bridge** (`ros_gz-bridge`).

Full details, bridge launch files, and a complete algorithm skeleton are in
**[documentation/ros2_control_guide.md](./documentation/ros2_control_guide.md)**.

### Quick overview

**Install the bridge**

```bash
sudo apt install ros-humble-ros-gz-bridge
```

**Start a bridge** (replace `allamerica2013_robot` with your `<model_name>_robot`)

```bash
ros2 run ros_gz_bridge parameter_bridge \
    /micromouse/allamerica2013_robot/ir@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan \
    /micromouse/allamerica2013_robot/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU \
    /micromouse/allamerica2013_robot/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry \
    /allamerica2013_robot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist
```

**Drive the robot**

```python
from geometry_msgs.msg import Twist

twist = Twist()
twist.linear.x  = 0.1    # m/s forward
twist.angular.z = 0.0    # rad/s turn
publisher.publish(twist)
```

### Topic reference

| Signal | Direction | ROS 2 topic / type |
|---|---|---|
| IR array (5 rays) | Ign -> ROS | `/micromouse/<robot>/ir` - `sensor_msgs/LaserScan` |
| IMU (100 Hz) | Ign -> ROS | `/micromouse/<robot>/imu` - `sensor_msgs/Imu` |
| Wheel odometry | Ign -> ROS | `/micromouse/<robot>/odom` - `nav_msgs/Odometry` |
| Wheel encoders | Ign -> ROS | `/world/<world>/model/<robot>/joint_state` - `sensor_msgs/JointState` |
| Drive command | ROS -> Ign | `/<robot>/cmd_vel` - `geometry_msgs/Twist` |

IR ray index layout: `[0] right · [1] front-right · [2] front-centre · [3] front-left · [4] left`

> The maze start/goal services (`/mazegen/<model>/spawn_pose` and
> `/mazegen/<model>/goal_poses`) are Ignition transport services - see
> [section 6 above](#6-using-mazegenPlugin-in-your-own-world), or
> [section 3 of the ROS 2 guide](./documentation/ros2_control_guide.md#3-calling-maze-services-from-ros-2)
> for calling them from a Python node.


---

## 9. References & acknowledgements

### Maze format & rules

- **[micromouseonline/mazefiles](https://github.com/micromouseonline/mazefiles)** - maze file format and source of the bundled competition mazes.
- **[Micromouse maze rules](https://micromouseonline.com/micromouse-book/mazes-and-maze-solving/)** - official geometry and colour specification (cell size, wall dimensions, colours).
- **[UK Micromouse Classic rules](https://ukmars.org/contests/contest-rules/micromouse-classic/)** - supplementary rule reference used for post and floor geometry.

### Simulator & libraries

- **[Ignition Gazebo Fortress](https://gazebosim.org/docs/fortress)** - simulator and plugin/transport APIs (`gz-sim 6`, `ignition-transport`, `ignition-msgs`, `sdformat12`).
- **[SDFormat](http://sdformat.org/)** - scene description format used to build and spawn maze models at runtime.

### Prior work & inspiration

- **[PeterMitrano/gzmaze](https://github.com/PeterMitrano/gzmaze)** - maze generator for Gazebo Classic; inspired the wall-merging approach used here.

### ROS 2 integration

- **[ros_gz_bridge](https://github.com/gazebosim/ros_gz)** - ROS–Ignition topic and service bridge used to expose robot sensors and drive commands to ROS 2 nodes.
- **[ROS 2 Humble](https://docs.ros.org/en/humble/)** - target ROS 2 distribution for the navigation stack and bridge packages.

## License

`mazegen_gazebo` is released under the `Apache License 2.0`. See [LICENSE](./LICENSE).
