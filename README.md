# Pilz Robot Programming - ROS2 Jazzy

Python API for easy programming of Pilz PRBT manipulator with industrial motion commands.
Migrated from ROS1 Noetic to **ROS2 Jazzy** with MoveIt2 and `pilz_industrial_motion_planner`.

## Table of Contents

- [Requirements](#requirements)
- [Installation](#installation)
- [Building the Workspace](#building-the-workspace)
- [Launching](#launching)
- [API Overview](#api-overview)
  - [Robot](#robot)
  - [Ptp](#ptp)
  - [Lin](#lin)
  - [Circ](#circ)
  - [Sequence](#sequence)
  - [Gripper](#gripper)
  - [from_euler](#from_euler)
- [Reference Frames](#reference-frames)
- [Relative Movements](#relative-movements)
- [Motion Control](#motion-control)
- [Example Program](#example-program)
- [Simulation Notes](#simulation-notes)
- [Workspace Structure](#workspace-structure)

---

## Requirements

- **OS:** Ubuntu 24.04 (Noble)
- **ROS2:** Jazzy Jalisco
- **MoveIt2** with Pilz Industrial Motion Planner
- **Python:** 3.12+

## Installation

### 1. Install ROS2 Jazzy

Follow the official guide: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

```bash
sudo apt update && sudo apt upgrade -y

sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop-full
```

### 2. Install all dependencies

```bash
# install basic apt packages
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-psutil python3-pip

# clone the repository
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
git clone -b jazzy https://github.com/LRMPUT/pilz_ros2.git src/pilz_ros2

# install dependencies with rosdep
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src -y --ignore-src
```

<!-- ```bash
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-psutil \
  python3-pip \
  ros-jazzy-moveit \
  ros-jazzy-moveit-planners \
  ros-jazzy-moveit-ros-move-group \
  ros-jazzy-moveit-ros-planning \
  ros-jazzy-moveit-ros-planning-interface \
  ros-jazzy-moveit-ros-visualization \
  ros-jazzy-moveit-simple-controller-manager \
  ros-jazzy-moveit-kinematics \
  ros-jazzy-moveit-configs-utils \
  ros-jazzy-pilz-industrial-motion-planner \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-control-msgs \
  ros-jazzy-xacro \
  ros-jazzy-rviz2 \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-tf-transformations \
  ros-jazzy-geometry-msgs \
  ros-jazzy-sensor-msgs \
  ros-jazzy-shape-msgs \
  ros-jazzy-std-msgs \
  ros-jazzy-std-srvs \
  ros-jazzy-trajectory-msgs \
  ros-jazzy-builtin-interfaces \
  ros-jazzy-rosidl-default-generators \
  ros-jazzy-rosidl-default-runtime \
  ros-jazzy-ament-cmake \
  ros-jazzy-ament-cmake-python \
  ros-jazzy-action-msgs \
  ros-jazzy-launch \
  ros-jazzy-launch-ros
``` -->

<!-- ### 3. Initialize rosdep (if not done yet)

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
``` -->

## Building the Workspace

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

To rebuild a single package after changes:

```bash
colcon build --packages-select pilz_robot_programming
source install/setup.bash
```

## Launching

### Without gripper

```bash
# Terminal 1: Launch MoveIt + controllers + RViz
ros2 launch pilz_tutorial my_application.launch.py
```

### With PG70 gripper (simulation)

```bash
# Terminal 1: Launch with gripper
ros2 launch pilz_tutorial my_application.launch.py gripper:=pg70
```

### Running a program

```bash
# Terminal 2: Run your robot program
ros2 run pilz_tutorial example_node
```

---

## API Overview

All motion commands are imported from the `pilz_robot_programming` package:

```python
from pilz_robot_programming import *
```

This imports: `Robot`, `Ptp`, `Lin`, `Circ`, `Sequence`, `Gripper`, `from_euler`.

### Robot

The main class that manages all robot motion. Only one instance is allowed at a time.

```python
import rclpy
from pilz_robot_programming import *

rclpy.init()
robot = Robot("1")  # API version string
```

**Constructor:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `version` | str | Required API version for compatibility checking |

**Methods:**

| Method | Description |
|--------|-------------|
| `move(cmd)` | Execute a motion command. Blocks until complete. |
| `get_current_pose()` | Returns the current TCP pose as `geometry_msgs/Pose` |
| `get_current_joint_states()` | Returns current joint values as a list of floats |
| `pause()` | Pause the currently executing motion |
| `resume()` | Resume a paused motion |
| `stop()` | Cancel the currently executing motion |
| `shutdown()` | Clean up resources and shut down the robot API |

### Ptp

Point-to-Point motion. The robot moves to the goal position as fast as possible.
The trajectory is planned in joint space - the path is not necessarily a straight line in Cartesian space.

```python
# Joint space goal (6 joint values in radians)
robot.move(Ptp(goal=[0.0, 0.5, 0.5, 0.0, 0.0, 0.0], vel_scale=0.4))

# Cartesian goal
from geometry_msgs.msg import Pose, Point, Quaternion
robot.move(Ptp(goal=Pose(position=Point(x=0.6, y=-0.3, z=0.2),
                          orientation=from_euler(0, math.pi, 0)),
               vel_scale=0.4))

# With reference frame (goal expressed in the "pnoz" frame)
robot.move(Ptp(goal=Pose(position=Point(x=0.0, y=0.0, z=0.1),
                          orientation=from_euler(0, math.pi, 0)),
               vel_scale=0.3, reference_frame="pnoz"))

# Relative motion
robot.move(Ptp(goal=Pose(position=Point(x=0.1, y=0.0, z=0.0)), vel_scale=0.2, relative=True))
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `goal` | list / Pose | required | Target as joint values or Cartesian pose |
| `vel_scale` | float | 1.0 | Velocity scaling factor (0, 1] |
| `acc_scale` | float | vel_scale^2 | Acceleration scaling factor (0, 1] |
| `reference_frame` | str | planning frame | TF frame for the goal pose |
| `relative` | bool | False | If True, goal is relative to current pose |

### Lin

Linear motion. The robot TCP moves along a straight line in Cartesian space.

```python
robot.move(Lin(goal=Pose(position=Point(x=0.2, y=0.0, z=0.8),
                          orientation=from_euler(0, math.pi, 0)),
               vel_scale=0.1, acc_scale=0.1))

# Relative linear motion
robot.move(Lin(goal=Pose(position=Point(x=0.0, y=0.0, z=-0.1)), vel_scale=0.05, relative=True))
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `goal` | Pose | required | Target Cartesian pose |
| `vel_scale` | float | 0.1 | Velocity scaling factor (0, 1] |
| `acc_scale` | float | vel_scale | Acceleration scaling factor (0, 1] |
| `reference_frame` | str | planning frame | TF frame for the goal pose |
| `relative` | bool | False | If True, goal is relative to current pose |

### Circ

Circular motion. The robot TCP moves along a circular arc in Cartesian space.
The arc is defined by the current position, a via-point (`interim` or `center`), and the goal position.

```python
# Using interim point (point on the arc)
robot.move(Circ(goal=Pose(position=Point(x=0.0, y=-0.6, z=0.08)),
                interim=Point(x=0.11, y=-0.49, z=0.08),
                vel_scale=0.1, acc_scale=0.1))

# Using center point (center of the circle)
robot.move(Circ(goal=Pose(position=Point(x=0.12, y=-0.5, z=0.08)),
                center=Point(x=0.0, y=-0.5, z=0.08),
                vel_scale=0.1, acc_scale=0.1))
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `goal` | Pose | required | Target Cartesian pose (end of arc) |
| `interim` | Point | None | Point on the arc (mutually exclusive with center) |
| `center` | Point | None | Center of the circle (mutually exclusive with interim) |
| `vel_scale` | float | 0.1 | Velocity scaling factor (0, 1] |
| `acc_scale` | float | vel_scale | Acceleration scaling factor (0, 1] |
| `reference_frame` | str | planning frame | TF frame for the goal and via-point |

> Only one of `interim` or `center` may be specified, not both.

### Sequence

Concatenates multiple motion commands into a single trajectory that is planned and executed as one unit.
If planning of any command in the sequence fails, none of them are executed.

```python
pose1 = Pose(position=Point(x=0.2, y=-0.2, z=0.5), orientation=from_euler(0, math.pi, 0))
pose2 = Pose(position=Point(x=0.2, y=0.2, z=0.5), orientation=from_euler(0, math.pi, 0))

sequence = Sequence()
sequence.append(Lin(goal=pose1, vel_scale=0.05, reference_frame="prbt_tcp"), blend_radius=0.01)
sequence.append(Lin(goal=pose2, vel_scale=0.05, reference_frame="prbt_tcp"))
robot.move(sequence)
```

Blending connects consecutive motions smoothly. The last command in a sequence must have `blend_radius=0` (the default).

```python
pose1 = Pose(position=Point(x=0.2, y=-0.2, z=0.5), orientation=from_euler(0, math.pi, 0))
pose2 = Pose(position=Point(x=0.2, y=0.0, z=0.5), orientation=from_euler(0, math.pi, 0))
pose3 = Pose(position=Point(x=0.2, y=0.2, z=0.5), orientation=from_euler(0, math.pi, 0))
pose4 = Pose(position=Point(x=0.3, y=0.2, z=0.5), orientation=from_euler(0, math.pi, 0))
inter = Point(x=0.25, y=0.1, z=0.5)

sequence = Sequence()
sequence.append(Ptp(goal=pose1, vel_scale=0.3))
sequence.append(Lin(goal=pose2, vel_scale=0.1), blend_radius=0.1)
sequence.append(Circ(goal=pose3, interim=inter, vel_scale=0.05), blend_radius=0.01)
sequence.append(Ptp(goal=pose4, vel_scale=0.3))  # last item, blend_radius=0
robot.move(sequence)
```

### Gripper

Controls the Schunk PG70 gripper. Sends goals directly to `gripper_trajectory_controller`.

```python
# Open gripper (half-width 0.03m = fully open for PG70)
robot.move(Gripper(goal=0.03, vel_scale=0.2))

# Close gripper (half-width 0.02m)
robot.move(Gripper(goal=0.02, vel_scale=0.2))
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `goal` | float | required | Half of the opening width in meters (0 to 0.03) |
| `vel_scale` | float | 0.1 | Velocity scaling factor (0, 1] |

### from_euler

Utility function to convert Euler angles to a `geometry_msgs/Quaternion`.
Uses **intrinsic ZYZ** convention (same as KUKA/Pilz convention).

```python
from pilz_robot_programming import from_euler
import math

orientation = from_euler(a, b, c)
```

| Parameter | Description |
|-----------|-------------|
| `a` | Rotation around the Z-axis (radians) |
| `b` | Rotation around the new Y-axis (radians) |
| `c` | Rotation around the new Z-axis (radians) |

---

## Reference Frames

By default, goals are specified in the planning frame (typically `world`).
You can specify any valid TF frame using the `reference_frame` parameter:

```python
# Goal in the "pnoz" frame (e.g. a fixture on the table)
robot.move(Ptp(goal=Pose(position=Point(x=0.0, y=0.0, z=0.1),
                          orientation=from_euler(0, math.pi, 0)),
               vel_scale=0.4, reference_frame="pnoz"))

# Goal relative to the current TCP
robot.move(Lin(goal=Pose(position=Point(x=0.0, y=0.0, z=-0.1)),
               vel_scale=0.1, reference_frame="prbt_tcp"))
```

Common frames for PRBT:
- `prbt_base_link` - robot base
- `prbt_tcp` - tool center point
- `world` / planning frame - default
- Custom frames published via TF (e.g. `pnoz`)

## Relative Movements

When `relative=True`, the goal is interpreted as an offset from the current robot pose.
Position offsets are added directly. Orientation offsets are added as Euler angle increments.

```python
# Move 10cm down from current position
robot.move(Lin(goal=Pose(position=Point(x=0.0, y=0.0, z=-0.1)),
               vel_scale=0.1, relative=True))

# Relative in a specific reference frame
robot.move(Ptp(goal=Pose(position=Point(x=0.0, y=0.1, z=0.0)),
               vel_scale=0.3, reference_frame="prbt_tcp"))
```

## Motion Control

Control services are available for multithreaded programs:

```bash
# Pause the current motion
ros2 service call /pause_movement std_srvs/srv/Trigger

# Resume a paused motion
ros2 service call /resume_movement std_srvs/srv/Trigger

# Stop (abort) the current motion
ros2 service call /stop_movement std_srvs/srv/Trigger
```

Or from Python in a separate thread:

```python
robot.pause()
robot.resume()
robot.stop()
```

---

## Example Program

```python
#!/usr/bin/env python3
from geometry_msgs.msg import Pose, Point
from pilz_robot_programming import *
import math
import rclpy

def main():
    rclpy.init()
    robot = Robot("1")

    # Print current state
    print(robot.get_current_pose())
    print(robot.get_current_joint_states())

    # Move to joint configuration
    robot.move(Ptp(goal=[0.0, 0.5, 0.5, 0.0, 0.0, 0.0], vel_scale=0.4))

    # Linear move to Cartesian pose
    robot.move(Lin(goal=Pose(position=Point(x=0.2, y=0.0, z=0.65),
                              orientation=from_euler(0, math.pi, 0)),
                   vel_scale=0.1, acc_scale=0.1))

    # Open gripper
    robot.move(Gripper(goal=0.03, vel_scale=0.2))

    # Circular motion using interim point
    robot.move(Circ(goal=Pose(position=Point(x=0.0, y=-0.6, z=0.08)),
                    interim=Point(x=0.11, y=-0.49, z=0.08),
                    vel_scale=0.1, acc_scale=0.1))

    # Sequence with blending
    seq = Sequence()
    seq.append(Lin(goal=Pose(position=Point(x=0.0, y=0.0, z=-0.1)),
                   vel_scale=0.05, reference_frame="prbt_tcp"), blend_radius=0.01)
    seq.append(Lin(goal=Pose(position=Point(x=-0.1, y=0.0, z=-0.1)),
                   vel_scale=0.05, reference_frame="prbt_tcp"))
    robot.move(seq)

    robot.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

## Simulation Notes

This workspace is configured for **simulation using mock_components** (no real hardware).

Joint velocity and acceleration limits in URDF and `joint_limits.yaml` are elevated above
hardware values to provide headroom for the LIN/CIRC planners near singularities:

| Parameter | Hardware | Simulation | File |
|-----------|----------|------------|------|
| Joint velocity | 1.57 rad/s | 5.0 rad/s | `prbt_macro.xacro`, `joint_limits.yaml` |
| Joint acceleration | 3.49 rad/s^2 | 10.0 rad/s^2 | `joint_limits.yaml` |

PTP commands automatically apply a correction factor to maintain realistic speeds:

```
effective_ptp_vel = vel_scale * (1.57 / sim_limit) * sim_limit = vel_scale * 1.57
```

Before deploying on real hardware, restore the original values in:
- `prbt_support/urdf/prbt_macro.xacro` - velocity limits
- `prbt_moveit_config/config/joint_limits.yaml` - velocity and acceleration
- `prbt_pg70_support/config/joint_limits.yaml` - velocity and acceleration (with gripper)

## Workspace Structure

```
pilz_ws/
  src/pilz_ros2/
    pilz_industrial_motion/       # Motion planning API
      pilz_robot_programming/     #   Python API (Robot, Ptp, Lin, Circ, Sequence, Gripper)
    pilz_msgs/                    # Custom message/service definitions (GetSpeedOverride)
    pilz_tutorial/                # Example programs and launch files
      launch/
        my_application.launch.py  #   Main launch file
      pilz_tutorial/              #   Python module (ament_python package)
        example.py                #   Example program (node: example_node)
      setup.py                    #   Declares console_scripts (e.g. example_node)
    prbt_support/                 # PRBT robot description (URDF/xacro, meshes)
    prbt_moveit_config/           # MoveIt2 configuration (SRDF, kinematics, limits, controllers)
    prbt_grippers/                # Gripper support packages
      prbt_pg70_support/          #   Schunk PG70 gripper config and description
    prbt_hardware_support/        # Hardware support services
    schunk_description/           # Schunk gripper meshes and URDF
```

## License

Original packages: Pilz GmbH & Co. KG, Apache 2.0 / LGPL v3.
ROS2 migration: adapted for ROS2 Jazzy Jalisco.
