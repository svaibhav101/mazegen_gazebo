# ROS 2 Control Guide - Micromouse in Gazebo

This guide explains how to interact with the spawned micromouse robot and maze
from a ROS 2 node using the **ROS–gazebo bridge** (`ros_gz_bridge`).

All examples assume:

- **Ignition Fortress** (`ign-gazebo6`) running with `worlds/maze.sdf` and `<spawn_robot>true</spawn_robot>`
- **ROS 2 Humble** (or later) sourced alongside Ignition
- World name: `maze_world` (as set in `worlds/maze.sdf`)
- Maze model name: `allamerica2013` (adjust if you changed `<model_name>`)
- Robot model name: `allamerica2013_robot` (always `<model_name>_robot`)

Each robot's topics are namespaced under its own robot name so that multiple
robots spawned in the same world never clash.

---

## 1. Prerequisites

### 1.1 Install the bridge

```bash
sudo apt install ros-humble-ros-gz-bridge   # Humble+; use ros-humble-ros-ign-bridge for older installs
```

### 1.2 Source both environments

```bash
source /opt/ros/humble/setup.bash
source /home/$USER/ign_ws/install/setup.bash   # adjust to your workspace
```

### 1.3 Launch Gazebo

```bash
cd /path/to/mazegen_gazebo
source setup.bash
ign gazebo -r worlds/maze.sdf
```

---

## 2. Topic naming convention

When a robot is spawned for maze `<model_name>`, its robot model is named
`<model_name>_robot` and **all topics are namespaced under that robot name**:

| Signal | Ignition topic |
|---|---|
| IR sensor array | `/micromouse/<model_name>_robot/ir` |
| IR point cloud | `/micromouse/<model_name>_robot/ir/points` |
| IMU | `/micromouse/<model_name>_robot/imu` |
| Wheel odometry | `/micromouse/<model_name>_robot/odom` |
| TF | `/micromouse/<model_name>_robot/tf` |
| Drive command | `/<model_name>_robot/cmd_vel` |
| Wheel encoders | `/world/<world>/model/<model_name>_robot/joint_state` |

For the `allamerica2013` maze the concrete topics are:

```
/micromouse/allamerica2013_robot/ir
/micromouse/allamerica2013_robot/ir/points
/micromouse/allamerica2013_robot/imu
/micromouse/allamerica2013_robot/odom
/micromouse/allamerica2013_robot/tf
/allamerica2013_robot/cmd_vel
/world/maze_world/model/allamerica2013_robot/joint_state
```

> When running **multiple mazes** each robot gets its own isolated namespace -
> no bridge or algorithm changes are needed between robots.

---

## 3. Calling maze services from ROS 2

The maze plugin exposes pure **Ignition transport** services - they are not
natively visible to ROS 2. The bridge can forward them, but for one-shot
service calls it is simplest to call them directly with `ign service` from a
shell, or from Python using `sdformat` / `ignition-transport` Python bindings.

The recommended pattern for a ROS 2 node is to call them once at startup
using the Ignition transport C++ or Python API, then work with the result.

### 3.1 From the terminal (quickest verification)

```bash
# Start pose of the robot (position + yaw)
ign service -s /mazegen/allamerica2013/spawn_pose \
    --reqtype ignition.msgs.Empty \
    --reptype ignition.msgs.Pose \
    -r "" --timeout 2000

# All goal cell centres
ign service -s /mazegen/allamerica2013/goal_poses \
    --reqtype ignition.msgs.Empty \
    --reptype ignition.msgs.Pose_V \
    -r "" --timeout 2000
```

### 3.2 From a ROS 2 Python node (using ignition-transport Python bindings)

```python
#!/usr/bin/env python3
"""Query maze start/goal poses at node startup."""
import rclpy
from rclpy.node import Node

from ignition.transport import Node as IgnNode
from ignition.msgs.pose_pb2 import Pose as IgnPose
from ignition.msgs.pose_v_pb2 import Pose_V as IgnPoseV
from ignition.msgs.empty_pb2 import Empty


class MazePoseClient(Node):
    def __init__(self):
        super().__init__('maze_pose_client')
        ign = IgnNode()

        maze_name = 'allamerica2013'

        #  Spawn pose 
        ok, rep = ign.request(
            f'/mazegen/{maze_name}/spawn_pose',
            Empty(), IgnPose, timeout_ms=2000)
        if ok:
            self.get_logger().info(
                f'Start pose: x={rep.position.x:.4f}  '
                f'y={rep.position.y:.4f}  '
                f'yaw={rep.orientation.z:.4f}')
        else:
            self.get_logger().error('spawn_pose service call failed')

        #  Goal poses 
        ok, rep = ign.request(
            f'/mazegen/{maze_name}/goal_poses',
            Empty(), IgnPoseV, timeout_ms=2000)
        if ok:
            for i, pose in enumerate(rep.pose):
                self.get_logger().info(
                    f'Goal {i}: x={pose.position.x:.4f}  '
                    f'y={pose.position.y:.4f}')
        else:
            self.get_logger().error('goal_poses service call failed')


def main():
    rclpy.init()
    rclpy.spin_once(MazePoseClient(), timeout_sec=5.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3.3 Pose message fields

| Field | Type | Description |
|---|---|---|
| `position.x` | float64 | World X of cell centre (metres) |
| `position.y` | float64 | World Y of cell centre (metres) |
| `position.z` | float64 | World Z (ground level = 0) |
| `orientation` | quaternion | Yaw-only rotation; for `spawn_pose` this is the heading into the first open corridor |

> `goal_poses` orientation is always identity - goal cells have no preferred facing direction.

---

## 4. Bridging topics to ROS 2

Start a bridge process for each topic you need. Replace `allamerica2013_robot`
with your actual robot name (`<model_name>_robot`) throughout.

### 4.1 IR sensor array

All five beams are published in a **single `LaserScan`** on
`/micromouse/allamerica2013_robot/ir`.

```bash
ros2 run ros_gz_bridge parameter_bridge \
    /micromouse/allamerica2013_robot/ir@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan
```

Fixed ray layout (index -> direction):

| Index | Direction | Angle |
|---|---|---|
| `ranges[0]` | right | −90° |
| `ranges[1]` | front-right | −45° |
| `ranges[2]` | front-centre | 0° |
| `ranges[3]` | front-left | +45° |
| `ranges[4]` | left | +90° |

Reading sensor distances in a ROS 2 node:

```python
from sensor_msgs.msg import LaserScan

IR_RIGHT        = 0
IR_FRONT_RIGHT  = 1
IR_FRONT_CENTRE = 2
IR_FRONT_LEFT   = 3
IR_LEFT         = 4

ROBOT = 'allamerica2013_robot'

def ir_callback(msg: LaserScan):
    r = msg.ranges
    print(f'right={r[IR_RIGHT]:.3f}  '
          f'fr={r[IR_FRONT_RIGHT]:.3f}  '
          f'fwd={r[IR_FRONT_CENTRE]:.3f}  '
          f'fl={r[IR_FRONT_LEFT]:.3f}  '
          f'left={r[IR_LEFT]:.3f}')
```

Practical thresholds for a 180 mm cell, 12 mm walls:

| Ray | Open corridor | Wall present |
|---|---|---|
| `right` / `left` (index 0, 4) | > 0.12 m | < 0.10 m |
| `front_right` / `front_left` (index 1, 3) | > 0.10 m | < 0.07 m |
| `front_centre` (index 2) | > 0.15 m | < 0.05 m |

### 4.2 IMU

```bash
ros2 run ros_gz_bridge parameter_bridge \
    /micromouse/allamerica2013_robot/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU
```

Reading yaw rate and heading in a ROS 2 node:

```python
from sensor_msgs.msg import Imu
import math

def imu_callback(msg: Imu):
    # Yaw rate (rad/s) - most useful for turn control
    yaw_rate = msg.angular_velocity.z

    # Heading from fused orientation quaternion
    q = msg.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)

    # Forward acceleration (m/s²) - detects wall impacts, spin-up
    accel_x = msg.linear_acceleration.x
```

> Update rate is 100 Hz - faster than the IR array (20 Hz), suitable for
> closed-loop turn control and slip detection.

### 4.3 Wheel encoder (joint state)

The `JointStatePublisher` plugin publishes on the Ignition internal topic
`/world/maze_world/model/allamerica2013_robot/joint_state`.

```bash
ros2 run ros_gz_bridge parameter_bridge \
    /world/maze_world/model/allamerica2013_robot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model
```

Reading joint velocities for odometry:

```python
from sensor_msgs.msg import JointState

WHEEL_RADIUS = 0.02   # metres
TRACK_WIDTH  = 0.10   # metres (distance between wheel centres)

def joint_state_callback(msg: JointState):
    # joint order matches model definition: joint_right, joint_left
    idx_r = msg.name.index('joint_right')
    idx_l = msg.name.index('joint_left')

    v_right = msg.velocity[idx_r] * WHEEL_RADIUS   # m/s
    v_left  = msg.velocity[idx_l] * WHEEL_RADIUS   # m/s

    v_robot = (v_right + v_left) / 2.0             # forward speed  (m/s)
    omega   = (v_right - v_left) / TRACK_WIDTH     # yaw rate       (rad/s)
```

> `msg.position[i]` gives cumulative wheel angle in radians - use for distance
> travelled: `distance = angle × WHEEL_RADIUS`.

### 4.4 Drive commands (cmd_vel -> differential drive)

The `DiffDrive` plugin subscribes to `/<model_name>_robot/cmd_vel` and publishes
odometry on `/micromouse/<model_name>_robot/odom` at 50 Hz.

```bash
ros2 run ros_gz_bridge parameter_bridge \
    /allamerica2013_robot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist
```

```python
from geometry_msgs.msg import Twist

twist = Twist()
twist.linear.x  = 0.1    # m/s forward
twist.angular.z = 0.0    # rad/s turn
publisher.publish(twist)
```

### 4.5 Wheel odometry and ground-truth pose

**Wheel-model odometry** from the `DiffDrive` plugin (fuses encoder velocities
into a pose estimate - use this for algorithm feedback):

```bash
ros2 run ros_gz_bridge parameter_bridge \
    /micromouse/allamerica2013_robot/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry
```

**Ground-truth pose** from the physics engine (useful for debugging; not
available from real hardware):

```bash
ros2 run ros_gz_bridge parameter_bridge \
    /world/maze_world/model/allamerica2013_robot/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose
```

> Use `/micromouse/<robot>/odom` in your algorithm. Reserve the ground-truth topic for
> visualisation and accuracy validation only.

### 4.6 Launch file (all bridges together)

`launch/micromouse_bridge.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

WORLD = 'maze_world'
ROBOT = 'allamerica2013_robot'   # <model_name>_robot

def generate_launch_description():
    bridge_args = [
        # Sensors (Ignition -> ROS)
        f'/micromouse/{ROBOT}/ir@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        f'/micromouse/{ROBOT}/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        # Odometry from DiffDrive plugin (Ignition -> ROS)
        f'/micromouse/{ROBOT}/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        # Wheel encoders raw (Ignition -> ROS)
        f'/world/{WORLD}/model/{ROBOT}/joint_state'
            f'@sensor_msgs/msg/JointState[ignition.msgs.Model',
        # Ground-truth pose for debugging (Ignition -> ROS)
        f'/world/{WORLD}/model/{ROBOT}/pose'
            f'@geometry_msgs/msg/Pose[ignition.msgs.Pose',
        # Drive command (ROS -> Ignition)
        f'/{ROBOT}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
    ]

    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='micromouse_bridge',
            arguments=bridge_args,
            output='screen',
        )
    ])
```

#### Multi-robot launch (one bridge node per robot)

When two mazes are running, spawn a separate bridge node for each robot:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

WORLD  = 'maze_world'
ROBOTS = ['japan1980_robot', 'america2013_robot']

def _bridge_node(robot):
    args = [
        f'/micromouse/{robot}/ir@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        f'/micromouse/{robot}/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        f'/micromouse/{robot}/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        f'/world/{WORLD}/model/{robot}/joint_state'
            f'@sensor_msgs/msg/JointState[ignition.msgs.Model',
        f'/{robot}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
    ]
    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'bridge_{robot}',
        arguments=args,
        output='screen',
    )

def generate_launch_description():
    return LaunchDescription([_bridge_node(r) for r in ROBOTS])
```

---

## 5. Topic / service reference

| What | Direction | Ignition topic | ROS 2 type |
|---|---|---|---|
| IR array (5 rays) | Ign -> ROS | `/micromouse/<robot>/ir` | `sensor_msgs/LaserScan` |
| IR point cloud | Ign -> ROS | `/micromouse/<robot>/ir/points` | `sensor_msgs/PointCloud2` |
| IMU (100 Hz) | Ign -> ROS | `/micromouse/<robot>/imu` | `sensor_msgs/Imu` |
| Wheel odometry | Ign -> ROS | `/micromouse/<robot>/odom` | `nav_msgs/Odometry` |
| TF | Ign -> ROS | `/micromouse/<robot>/tf` | `tf2_msgs/TFMessage` |
| Wheel encoders (raw) | Ign -> ROS | `/world/<world>/model/<robot>/joint_state` | `sensor_msgs/JointState` |
| Ground-truth pose | Ign -> ROS | `/world/<world>/model/<robot>/pose` | `geometry_msgs/Pose` |
| Drive command | ROS -> Ign | `/<robot>/cmd_vel` | `geometry_msgs/Twist` |
| Start pose | Ign service | `/mazegen/<maze_name>/spawn_pose` | `ignition.msgs.Pose` |
| Goal poses | Ign service | `/mazegen/<maze_name>/goal_poses` | `ignition.msgs.Pose_V` |

Where `<robot>` = `<model_name>_robot` and `<maze_name>` = `<model_name>`.

---

## 6. Minimal algorithm node skeleton

```python
#!/usr/bin/env python3
"""
Minimal ROS 2 node skeleton for a micromouse maze-solving algorithm.
Assumes the bridge launch file from [section 4.6](#46-launch-file-all-bridges-together) is already running.

Set ROBOT to your robot name (<model_name>_robot).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

ROBOT        = 'allamerica2013_robot'   # change to match your maze's model_name
WHEEL_RADIUS = 0.02   # metres
TRACK_WIDTH  = 0.10   # metres


class MicromouseSolver(Node):
    def __init__(self):
        super().__init__('micromouse_solver')

        # IR sensor array - single topic, 5 rays
        self.create_subscription(LaserScan,
                                 f'/micromouse/{ROBOT}/ir',
                                 self._cb_ir, 10)

        # IMU - 100 Hz, gyro + fused orientation
        self.create_subscription(Imu,
                                 f'/micromouse/{ROBOT}/imu',
                                 self._cb_imu, 10)

        # DiffDrive odometry - 50 Hz, wheel-model pose estimate
        self.create_subscription(Odometry,
                                 f'/micromouse/{ROBOT}/odom',
                                 self._cb_odom, 10)

        # Raw wheel encoders - position (rad) and velocity (rad/s) per joint
        self.create_subscription(
            JointState,
            f'/world/maze_world/model/{ROBOT}/joint_state',
            self._cb_joints, 10)

        # Drive command publisher
        self._cmd_pub = self.create_publisher(Twist, f'/{ROBOT}/cmd_vel', 10)

        # Latest sensor state
        self.ir       = [0.2, 0.2, 0.2, 0.2, 0.2]  # [right, fr, fwd, fl, left]
        self.yaw_rate = 0.0   # rad/s  (IMU gyro Z)
        self.yaw_imu  = 0.0   # rad    (IMU fused heading)
        self.x        = 0.0   # m      (DiffDrive odometry)
        self.y        = 0.0   # m      (DiffDrive odometry)
        self.yaw_odom = 0.0   # rad    (DiffDrive odometry)
        self.v_robot  = 0.0   # m/s    (encoder odometry, manual)
        self.omega    = 0.0   # rad/s  (encoder odometry, manual)

        # Control loop at 20 Hz
        self.create_timer(0.05, self._control_loop)

    #  IR callback 
    def _cb_ir(self, msg: LaserScan):
        self.ir = list(msg.ranges)   # index 0=right … 4=left

    #  IMU callback 
    def _cb_imu(self, msg: Imu):
        self.yaw_rate = msg.angular_velocity.z
        q = msg.orientation
        self.yaw_imu = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    #  DiffDrive odometry callback 
    def _cb_odom(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw_odom = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    #  Raw encoder callback (optional) 
    def _cb_joints(self, msg: JointState):
        try:
            vr = msg.velocity[msg.name.index('joint_right')] * WHEEL_RADIUS
            vl = msg.velocity[msg.name.index('joint_left')]  * WHEEL_RADIUS
            self.v_robot = (vr + vl) / 2.0
            self.omega   = (vr - vl) / TRACK_WIDTH
        except ValueError:
            pass

    #  Control loop 
    def _control_loop(self):
        cmd = Twist()
        # TODO: replace with your maze algorithm
        cmd.linear.x  = 0.05   # m/s forward
        cmd.angular.z = 0.0    # rad/s turn
        self._cmd_pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(MicromouseSolver())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 7. Coordinate conventions

- World frame: **+X east, +Y north** (matches Gazebo default top-down view).
- Robot body frame: **+X forward, +Y left, +Z up**.
- `spawn_pose` yaw is in radians, measured from world +X axis, positive counter-clockwise.
- Cell (0, 0) is the **south-west corner** of the maze; row increases northward, column eastward.

---

## 8. References

### ROS 2 & bridge

- **[ROS 2 Humble](https://docs.ros.org/en/humble/)** - target ROS 2 distribution; all package names and API examples use Humble.
- **[ros_gz_bridge](https://github.com/gazebosim/ros_gz)** - ROS–Ignition bridge providing `parameter_bridge` and the `@` topic mapping syntax used throughout this guide.
- **[geometry_msgs/Twist](https://docs.ros2.org/humble/api/geometry_msgs/msg/Twist.html)** - drive command message type (`linear.x` m/s, `angular.z` rad/s).
- **[sensor_msgs/LaserScan](https://docs.ros2.org/humble/api/sensor_msgs/msg/LaserScan.html)** - IR sensor array message; `ranges[]` indexed right→left.
- **[nav_msgs/Odometry](https://docs.ros2.org/humble/api/nav_msgs/msg/Odometry.html)** - wheel-model pose estimate published by the `DiffDrive` plugin.

### Ignition Gazebo

- **[ignition-transport](https://gazebosim.org/libs/transport)** - pub/sub and request/reply transport layer; used directly for `spawn_pose` / `goal_poses` service calls from ros 2 node.
- **[DiffDrive system plugin](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1DiffDrive.html)** - drives the robot and publishes `/odom` from wheel encoder velocities.

### Project documentation

- **[Main README](../README.md)** - plugin parameters, SDF usage, Docker setup, and repository layout.
- **[mazegen_gazebo on GitHub](https://github.com/svaibhav101/mazegen_gazebo)** - source repository.
