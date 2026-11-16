# Horizon Robot

A modular ROS 2 and Gazebo simulation, localization, mapping, and autonomous navigation stack for the **Horizon Bot** 4-wheel mobile platform.

The repository is designed around a single parameterized bringup architecture that allows switching between **Differential (Skid-Steer)** and **Omnidirectional (Mecanum)** kinematics, **Gazebo Native** and **`ros2_control`** actuation backends, **Extended Kalman Filter (EKF)** sensor fusion, **2D LiDAR SLAM**, and **Nav2 MPPI** autonomous navigation from the command line.

---

## Table of Contents

1. [Repository Structure](#1-repository-structure)
2. [Branches and ROS 2 Distribution Compatibility](#2-branches-and-ros-2-distribution-compatibility)
3. [Robot Physical and Kinematic Specifications](#3-robot-physical-and-kinematic-specifications)
4. [Prerequisites and Installation](#4-prerequisites-and-installation)
5. [Launch Architecture and Arguments Reference](#5-launch-architecture-and-arguments-reference)
6. [Launch Commands by Setup](#6-launch-commands-by-setup)
7. [Teleoperation Guide](#7-teleoperation-guide)
8. [SLAM Map Serialization Notes](#8-slam-map-serialization-notes)

---

## 1. Repository Structure

```text
horizon_robot/
├── horizon_description/          # Robot URDF/Xacro, STL meshes, Gazebo sensor & friction tags
│   ├── launch/
│   │   └── display.launch.py     # Standalone URDF + Joint State Publisher GUI + RViz viewer
│   ├── meshes/                   # Chassis, wheel, sensor, and manipulator STL meshes
│   └── urdf/
│       ├── horizon.urdf.xacro    # Top-level parameterized Xacro entry point
│       ├── base/                 # base_footprint and base_link definitions
│       ├── wheels/               # 4-wheel joint and link definitions
│       ├── sensors/              # 2D LiDAR, IMU, and RGB-D camera Xacros
│       ├── gazebo/               # Gazebo plugins and wheel surface friction (<fdir1>, <mu>)
│       └── ros2_control/         # ros2_control hardware interfaces and velocity limits
├── horizon_bringup/              # Launch files, controller YAMLs, EKF, SLAM, and Nav2 configs
│   ├── config/
│   │   ├── horizon_bridge_gazebo.yaml        # ros_gz_bridge config for drive:=gz
│   │   ├── horizon_bridge_ros2_control.yaml  # ros_gz_bridge config for drive:=ros2_control
│   │   ├── horizon_mecanum_controllers.yaml  # ros2_control params (enable_odom_tf: true)
│   │   ├── horizon_ekf_controllers.yaml      # ros2_control params when ekf:=true (enable_odom_tf: false)
│   │   ├── horizon_ekf_params.yaml           # robot_localization EKF configuration
│   │   ├── mapper_params_online_async.yaml   # slam_toolbox online async configuration
│   │   ├── nav2_diff_params.yaml             # Nav2 MPPI configuration for Differential Drive
│   │   ├── nav2_mecanum_params.yaml          # Nav2 MPPI configuration for Omnidirectional Mecanum Drive
│   │   └── sim.rviz                          # Preconfigured RViz2 layout
│   ├── launch/
│   │   └── horizon_mecanum.launch.py         # Unified parameterized simulation & navigation bringup
│   ├── maps/room/                            # Saved occupancy grid and serialized pose-graph maps
│   └── worlds/                               # Gazebo Sim (.sdf) test environments
│       ├── room.sdf                          # 10 m x 10 m indoor room with 3 box obstacles
│       ├── maze.sdf                          # 20 m x 20 m multi-corridor indoor maze (TurtleBot 4)
│       ├── office.sdf                        # 16 m x 12 m multi-room office with doorways and desks
│       ├── tb3_sandbox.sdf                   # 8 m x 8 m cylindrical pillar obstacle arena (TurtleBot 3)
│       ├── depot.sdf                         # Industrial depot environment (Gazebo Fuel)
│       └── warehouse.sdf                     # Logistics warehouse with shelves and barriers (Gazebo Fuel)
└── horizon_robot/                # Metapackage definition
```

---

## 2. Branches and ROS 2 Distribution Compatibility

This repository maintains two active branches targeting different ROS 2 and Gazebo releases:

| Branch | ROS 2 Distro | Gazebo Version | SDFormat | Key Distro-Specific Details |
| :--- | :--- | :--- | :--- | :--- |
| **`main`** | **ROS 2 Jazzy Jalisco** | Gazebo Harmonic (`gz-sim8`) | SDFormat 1.11 | Uses `<gz_frame_id>` in sensor Xacros, `libgz_ros2_control-system.so`, and Jazzy `slam_toolbox` / `nav2_bringup` defaults. |
| **`lyrical`** | **ROS 2 Lyrical Luth** | Gazebo Jetty (`gz-sim10`) | SDFormat 1.12 | Uses official `<frame_id>` sensor tags, `gz_ros2_control-system`, Lyrical `.posegraph` serialization format, `bringup_launch.py` lifecycle management, and `pluginlib` MPPI motion models (`mppi::DiffDriveMotionModel` / `mppi::OmniMotionModel`). |

---

## 3. Robot Physical and Kinematic Specifications

Dimensions extracted directly from `base_link.stl` and `horizon_wheels.xacro`:

| Parameter | Value | Notes / Where Used |
| :--- | :--- | :--- |
| **Chassis Length ($X$)** | `0.4578 m` (`45.78 cm`) | Spans `-0.2328 m` (rear) to `+0.2250 m` (front) relative to `base_link` |
| **Chassis Breadth ($Y$)** | `0.3660 m` (`36.60 cm`) | Spans `-0.1830 m` (right) to `+0.1830 m` (left) relative to `base_link` |
| **Chassis Height ($Z$)** | `0.1755 m` (`17.55 cm`) | `base_footprint` to `base_link` Z-offset is `0.0425 m` |
| **Circumscribed Radius** | `0.2822 m` (mesh) / `0.2961 m` (box) | Configured as `robot_radius: 0.31` in Nav2 costmaps |
| **Wheel Radius ($r$)** | `0.0425 m` | Configured in `horizon_wheels.xacro` and controller YAMLs |
| **Wheelbase ($L = 2 l_x$)** | `0.3200 m` (`l_x = 0.16 m`) | Longitudinal distance between front and rear wheel axles |
| **Track Width ($W = 2 l_y$)** | `0.3100 m` (`l_y = 0.155 m`) | Lateral distance between left and right wheel centers |
| **Mecanum Geometry Sum ($l_x + l_y$)** | `0.3150 m` | `sum_of_robot_center_projection_on_X_Y_axis` in `mecanum_drive_controller` |
| **Effective Skid-Steer Separation ($W_{\text{eff}}$)** | `0.6400 m` | $W_{\text{eff}} = W + L^2 / W = 0.31 + 0.32^2 / 0.31 = 0.6403\text{ m}$ (used by `diff_drive_controller`) |
| **Wheel Joint Velocity Limit** | `±10.0 rad/s` (`0.425 m/s`) | Enforced in `horizon_ros2_control.xacro`; YAML controllers cap chassis speed to `0.2 m/s` and `0.2 rad/s` |

---

## 4. Prerequisites and Installation

### 4.1 Install Required ROS 2 Packages

Replace `$ROS_DISTRO` with `jazzy` (on `main`) or `lyrical` (on `lyrical`):

```bash
sudo apt update
sudo apt install -y \
  ros-$ROS_DISTRO-ros-gz \
  ros-$ROS_DISTRO-ros-gz-sim \
  ros-$ROS_DISTRO-ros-gz-bridge \
  ros-$ROS_DISTRO-gz-ros2-control \
  ros-$ROS_DISTRO-ros2-control \
  ros-$ROS_DISTRO-ros2-controllers \
  ros-$ROS_DISTRO-robot-localization \
  ros-$ROS_DISTRO-slam-toolbox \
  ros-$ROS_DISTRO-navigation2 \
  ros-$ROS_DISTRO-nav2-bringup \
  ros-$ROS_DISTRO-twist-stamper \
  ros-$ROS_DISTRO-teleop-twist-keyboard \
  ros-$ROS_DISTRO-xacro \
  ros-$ROS_DISTRO-joint-state-publisher-gui
```

### 4.2 Clone and Build the Workspace

```bash
# 1. Create workspace directory
mkdir -p ~/mobile_ws/src
cd ~/mobile_ws/src

# 2. Clone the repository (choose -b main for Jazzy or -b lyrical for Lyrical)
git clone -b lyrical git@github.com:praiseorji4/horizon_robot.git

# 3. Build the workspace
cd ~/mobile_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build
source install/setup.bash
```

---

## 5. Launch Architecture and Arguments Reference

The primary bringup file is `horizon_mecanum.launch.py` in `horizon_bringup`. It accepts the following launch arguments:

| Argument | Allowed Values | Default | Description |
| :--- | :--- | :--- | :--- |
| **`drive`** | `gz`, `ros2_control` | `gz` | Selects actuation backend: Gazebo's built-in `DiffDrive` plugin (`gz`) or `ros2_control` hardware interfaces (`ros2_control`). |
| **`system`** | `diff`, `mecanum` | `diff` | Selects wheel surface friction, controller spawner (`diff_drive_controller` vs. `mecanum_drive_controller`), and Nav2 parameter file (`nav2_diff_params.yaml` vs. `nav2_mecanum_params.yaml`). |
| **`ekf`** | `true`, `false` | `false` | When `true`, loads `horizon_ekf_controllers.yaml` (disabling controller TF broadcast) and starts `ekf_filter_node` (`robot_localization`) to fuse wheel odometry (`vx`, `vy`) with IMU (`yaw`, `vyaw`) and publish `odom -> base_footprint` + `/odometry/filtered`. |
| **`slam`** | `true`, `false` | `false` | Waits for the first odometry message (`/odom` or `/odometry/filtered`) and launches `slam_toolbox` (`online_async_launch.py`). |
| **`nav`** | `true`, `false` | `false` | Waits for `/map` to be published and launches the Nav2 navigation stack with the parameter file matching `system`. |
| **`use_arm`** | `true`, `false` | `false` | Includes the top-mounted manipulator arm in the URDF when `true`. |
| **`x`, `y`, `z`, `yaw`** | float | `0.0, 0.0, 0.1, 0.0` | Initial spawn coordinates and heading (in radians) of the robot in Gazebo. |

### Automatic Configuration Routing

Based on the arguments above, `horizon_mecanum.launch.py` automatically wires the following components:
* **Bridge Configuration**: Uses `horizon_bridge_ros2_control.yaml` when `drive:=ros2_control`, or `horizon_bridge_gazebo.yaml` when `drive:=gz`.
* **Controller Parameters**: Uses `horizon_ekf_controllers.yaml` (`enable_odom_tf: false`) when `ekf:=true`, or `horizon_mecanum_controllers.yaml` (`enable_odom_tf: true`) when `ekf:=false`.
* **Nav2 Parameters**: Uses `nav2_diff_params.yaml` (`DiffDrive` motion model) when `system:=diff`, or `nav2_mecanum_params.yaml` (`Omni` motion model + lateral velocity limits + `TwirlingCritic`) when `system:=mecanum`.
* **Velocity Command Stamping**: Runs `twist_stamper` to convert unstamped `geometry_msgs/msg/Twist` on `/cmd_vel` into stamped `geometry_msgs/msg/TwistStamped` on `/cmd_vel_drive` for `ros2_control` controllers.

---

## 6. Launch Commands by Setup

Before running any launch command, source your workspace:

```bash
source ~/mobile_ws/install/setup.bash
```

### Setup 1: Inspect URDF in RViz Only (No Gazebo)

Launches `robot_state_publisher`, `joint_state_publisher_gui`, and RViz to inspect links, joints, and TF frames:

```bash
ros2 launch horizon_description display.launch.py
```

### Setup 2: Gazebo Native Drive (`drive:=gz`)

Uses Gazebo's native `DiffDrive` system plugin (bridged directly over `/cmd_vel` and `/odom`):

```bash
ros2 launch horizon_bringup horizon_mecanum.launch.py drive:=gz system:=diff
```

### Setup 3: `ros2_control` — Differential / Skid-Steer Drive (`system:=diff`)

Spawns `joint_state_broadcaster` and `diff_drive_controller` with isotropic tire friction:

```bash
ros2 launch horizon_bringup horizon_mecanum.launch.py drive:=ros2_control system:=diff
```

### Setup 4: `ros2_control` — Omnidirectional Mecanum Drive (`system:=mecanum`)

Spawns `joint_state_broadcaster` and `mecanum_drive_controller` with 45-degree directional roller friction (`<fdir1>`), enabling full 3-DOF $(v_x, v_y, \omega_z)$ motion:

```bash
ros2 launch horizon_bringup horizon_mecanum.launch.py drive:=ros2_control system:=mecanum
```

### Setup 5: `ros2_control` + EKF Sensor Fusion (`ekf:=true`)

Fuses wheel odometry linear velocities with IMU orientation and angular rate via `robot_localization`:

```bash
# Differential Drive + EKF
ros2 launch horizon_bringup horizon_mecanum.launch.py drive:=ros2_control system:=diff ekf:=true

# Mecanum Drive + EKF
ros2 launch horizon_bringup horizon_mecanum.launch.py drive:=ros2_control system:=mecanum ekf:=true
```

### Setup 6: Online Asynchronous SLAM (`slam:=true`)

Starts Gazebo, `ros2_control`, EKF, and `slam_toolbox` (automatically waiting for `/odometry/filtered` before activating SLAM):

```bash
# Differential Drive + EKF + SLAM
ros2 launch horizon_bringup horizon_mecanum.launch.py drive:=ros2_control system:=diff ekf:=true slam:=true

# Mecanum Drive + EKF + SLAM
ros2 launch horizon_bringup horizon_mecanum.launch.py drive:=ros2_control system:=mecanum ekf:=true slam:=true
```

### Setup 7: Full Autonomous Navigation Stack (`slam:=true nav:=true`)

Launches the complete pipeline (Gazebo + `ros2_control` + EKF + `slam_toolbox` + `Nav2`). Once the `/map` topic is live and `lifecycle_manager_nav2` activates the servers, use the **2D Goal Pose** tool in RViz to send navigation goals:

```bash
# Full Stack — Differential Drive (uses nav2_diff_params.yaml)
ros2 launch horizon_bringup horizon_mecanum.launch.py drive:=ros2_control system:=diff ekf:=true slam:=true nav:=true

# Full Stack — Omnidirectional Mecanum Drive (uses nav2_mecanum_params.yaml)
ros2 launch horizon_bringup horizon_mecanum.launch.py drive:=ros2_control system:=mecanum ekf:=true slam:=true nav:=true
```

---

## 7. Teleoperation Guide

In a second terminal, launch `teleop_twist_keyboard`:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

*(Note: `teleop_twist_keyboard` publishes unstamped `geometry_msgs/msg/Twist` on `/cmd_vel`. The `twist_stamper` node launched by `horizon_mecanum.launch.py` automatically converts `/cmd_vel` to `geometry_msgs/msg/TwistStamped` on `/cmd_vel_drive` for the `ros2_control` controllers.)*

### Standard Controls (`system:=diff` and `system:=mecanum`)
* **`i`** : Move forward ($+v_x$)
* **`,`** : Move backward ($-v_x$)
* **`j`** : Rotate counter-clockwise ($+\omega_z$)
* **`l`** : Rotate clockwise ($-\omega_z$)
* **`u` / `o` / `m` / `.`** : Combined forward/backward arc turns
* **`k`** : Stop ($v_x = 0, v_y = 0, \omega_z = 0$)
* **`q` / `z`** : Increase / decrease max speeds by 10%

### Omnidirectional Holonomic Controls (`system:=mecanum` only)
Hold **`Shift`** (or enable Caps Lock) for lateral strafing and diagonal translation without rotating the chassis:
* **`J` (`Shift + j`)** : Strafe left ($+v_y$)
* **`L` (`Shift + l`)** : Strafe right ($-v_y$)
* **`U` (`Shift + u`)** : Diagonal forward-left ($+v_x, +v_y$)
* **`O` (`Shift + o`)** : Diagonal forward-right ($+v_x, -v_y$)
* **`M` (`Shift + m`)** : Diagonal backward-left ($-v_x, +v_y$)
* **`>` (`Shift + .`)** : Diagonal backward-right ($-v_x, -v_y$)

---

## 8. SLAM Map Serialization Notes

In `horizon_bringup/config/mapper_params_online_async.yaml`:

1. **Loading a Pre-Saved Serialized Map vs. Mapping from Scratch**:
   * To load the pre-saved room map on startup, ensure `map_file_name` points to the absolute path of `room_map_serial` on your machine (without the `.posegraph` or `.data` extension) and set `map_start_at_dock: true`:
     ```yaml
     map_file_name: /home/<your_username>/mobile_ws/src/horizon_robot/horizon_bringup/maps/room/room_map_serial
     map_start_at_dock: true
     ```
   * To build a new map from scratch, comment out both `map_file_name` and `map_start_at_dock`, then rebuild with `colcon build`.

2. **Converting `.posegraph` Files Between ROS 2 Jazzy (`main`) and ROS 2 Lyrical (`lyrical`)**:
   Between ROS 2 Jazzy and ROS 2 Lyrical, `slam_toolbox` removed the 17-byte `m_Listeners` vector from `karto::Mapper::serialize()` right before the final 174 bytes of `Parameter*` pointers in `.posegraph`.
   * **Jazzy `.posegraph` to Lyrical `.posegraph`**:
     ```bash
     python3 -c '
     p = "horizon_bringup/maps/room/room_map_serial.posegraph"
     data = open(p, "rb").read()
     assert data[-191:-174] == b"\x00" * 17, "Already in Lyrical format!"
     open(p, "wb").write(data[:-191] + data[-174:])
     '
     ```
   * **Lyrical `.posegraph` to Jazzy `.posegraph`**:
     ```bash
     python3 -c '
     p = "horizon_bringup/maps/room/room_map_serial.posegraph"
     data = open(p, "rb").read()
     open(p, "wb").write(data[:-174] + (b"\x00" * 17) + data[-174:])
     '
     ```
