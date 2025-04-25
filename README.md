# ABB IRB140 Simulation in ROS 2

This package provides a basic RViz2-based simulation of the ABB IRB140 industrial robot using ROS 2. It includes the robot's URDF, 3D meshes, materials, and a preconfigured RViz2 environment.

---

## Features

- URDF/XACRO description of the IRB140 robot
- Visual and collision meshes
- RViz2 visualization
- Joint control using GUI sliders (`joint_state_publisher_gui`)

---

## 🚀 Quick Start

### 1. Clone this Repository into a ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/jstoquica/ros2_irb140.git
```
### 2. Build the Package

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 3. Launch the Simulation in RViz2

```bash
ros2 launch irb140 irb140.launch.py 
```
