# Robot Arm Digital Twin - Communication Latency Analysis

## Overview

A ROS-based digital twin system for analyzing communication latency and accuracy between Unity virtual robot arm, Gazebo simulation, and real Franka robot arm.

## Project Structure


dissertation-main/
├── unity_script/                    # Unity scripts
│   ├── JointStatePublisher.cs
│   ├── FrankaJointStateSubscriber.cs
│   ├── FPSLogger.cs
│   ├── RosConnectionTest.cs
│   └── PublishFloat_backup.cs
├── franka_ws/                      # Franka ROS workspace
│   └── src/
│       ├── franka_ros/
│       ├── libfranka/
│       ├── ros_control_boilerplate/
│       └── rosparam_shortcuts/
├── catkin_ws/                      # Main ROS workspace
│   └── src/
│       ├── franka_digital_twin/
│       ├── joint_publisher_pkg/
│       ├── my_robot_msgs/
│       ├── panda_moveit_config/
│       ├── ROS-TCP-Endpoint/
│       └── franka_ros/
└── *.m                            # MATLAB analysis scripts


## MATLAB Analysis Scripts

- `analyze_communication_latency.m` - Communication delay analysis
- `robot_accuracy_comparison.m` - Accuracy comparison between real, Gazebo, and Unity
- `calculate_unity_real_metrics.m` - Unity vs real robot metrics
- `trajectory_comparison.m` - Trajectory visualization and comparison
- `latency.m`, `latency2.m` - Latency calculation tools
- `Mse_rmse_lentency.m` - MSE/RMSE error analysis

## Unity Scripts

- `JointStatePublisher.cs` - Publishes joint states to ROS topics
- `FrankaJointStateSubscriber.cs` - Subscribes to Franka joint states
- `FPSLogger.cs` - Performance monitoring
- `RosConnectionTest.cs` - ROS connection testing
- `PublishFloat_backup.cs` - Float data publisher backup

## Setup

```bash
# Clone repository
git clone https://github.com/yuan18844-afk/dissertation.git
cd dissertation-main

# Build Franka workspace
cd franka_ws
catkin_make
source devel/setup.bash

# Build main workspace
cd ../catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

### Data Analysis
```matlab
% Run analysis scripts
analyze_communication_latency_fixed()
robot_accuracy_comparison_final()
calculate_unity_real_metrics_complete()
trajectory_comparison
```

## Notes

- Ensure proper timestamp synchronization
- Update file paths in MATLAB scripts
- Verify ROS topic names match between Unity and ROS
- Consider network latency impact

