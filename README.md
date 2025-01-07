# ROS2 Robot Mover

This package contains a ROS2 node that allows you to move a robot in a simulation environment using user input for linear and angular velocities.

## Prerequisites

Ensure you have ROS2 installed on your system. You can follow the official [ROS2 installation guide](https://docs.ros.org/en/foxy/Installation.html).

### Install Required Packages

First of all, run:
```sh
sudo apt-get update
sudo apt-get upgrade  # it may take a while!
sudo apt-get install ros-<distro>-xacro ros-<distro>-joint-state-publisher ros-<distro>-gazebo*
```
Replace `<distro>` with your ROS2 distribution (e.g., foxy, galactic, humble).

## Clone the Repositories

We are going to use this repo: [robot_urdf](https://github.com/CarmineD8/robot_urdf.git)

Switch to the ROS2 branch:
```sh
cd ~/ros2_ws/src
git clone -b ros2 https://github.com/CarmineD8/robot_urdf.git
```

Clone the `ros2_robot_mover_python` repository:
```sh
git clone https://github.com/mohamedeyaad/ros2_robot_mover_python.git
```

## Build the Package

Navigate to your ROS2 workspace and build the package:
```sh
cd ~/ros2_ws
colcon build 
```

Source the setup file:
```sh
source install/setup.bash
```

## Launch the Simulation

Navigate to the cloned `robot_urdf` repository and launch the Gazebo simulation:
```sh
ros2 launch robot_urdf gazebo.launch.py
```

## Running the Mover Node

### 1. Python Node

Navigate to the `ros2_robot_mover_python` package directory and run the node:
```sh
ros2 run ros2_robot_mover_python mover_node
```

### 2. C++ Node
If you prefer to run the C++ node, follow these steps:

Switch to the C++ branch:
```sh
git checkout cpp
```

Navigate to the `ros2_robot_mover_cpp` package directory and run the node:
```sh
ros2 run ros2_robot_mover_cpp mover_node
```

## Node Description

### MoverNode (Python/C++)

- **Publisher**: `/cmd_vel` (geometry_msgs/Twist)
- **Features**:
  - Initializes the ROS 2 node and creates a publisher for the `/cmd_vel` topic.
  - Continuously listens to odometry data `/odom` and logs the robot's position and orientation.
  - Allows the user to input linear and angular velocities from the terminal.
  - Publishes the entered velocity commands for **1 second**, during which odometry data is logged.
  - Automatically stops the robot by publishing zero velocity after 1 second of movement.