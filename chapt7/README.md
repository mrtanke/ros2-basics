# Autonomous Inspection Robot with ROS 2 and Navigation 2

## 1. Project Overview

This project provides a simulation-ready autonomous inspection robot powered by ROS 2 and Navigation 2.

The robot loops through multiple goal points. At each location it plays a spoken message announcing the waypoint, captures a real-time image with its camera, and saves the snapshot locally.

Package summary:
- **fishbot_description** – robot description and simulation configuration
- **fishbot_navigation2** – navigation configuration assets
- **fishbot_application** – Python application for navigation demos
- **fishbot_application_cpp** – C++ navigation application
- **autopatrol_interfaces** – interfaces related to the inspection workflow
- **autopatrol_robot** – autonomous inspection implementation package

## 2. Usage

Development environment:

- OS: Ubuntu 22.04
- ROS: ROS 2 Humble

### 2.1 Installation

This workspace uses slam-toolbox for mapping, Navigation 2 for navigation, Gazebo for simulation, and ros2-control for motion control. Install the dependencies before building:

1. SLAM and Navigation 2

```
sudo apt install ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-slam-toolbox
```

2. Simulation packages

```
sudo apt install ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-xacro
```

3. Speech synthesis and image processing packages

```
sudo apt install python3-pip -y
sudo apt install espeak-ng -y
sudo pip3 install espeakng
sudo apt install ros-$ROS_DISTRO-tf-transformations
sudo pip3 install transforms3d
```

### 2.2 Running

After installing the prerequisites, build and launch with colcon.

Build the workspace:

```
colcon build
```

Run the simulation:

```
source install/setup.bash
ros2 launch fishbot_description gazebo_sim.launch.py
```

Launch navigation:

```
source install/setup.bash
ros2 launch fishbot_navigation2 navigation2.launch.py
```

Start the autonomous inspection workflow:

```
source install/setup.bash
ros2 launch autopatrol_robot autopatrol.launch.py
```

## 3. Original Author

- [fishros](https://github.com/fishros)