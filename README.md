# Custom Odometry & Control Stack (ROS 2 Humble)

## Overview
This project is a complete robotics software pipeline built from scratch in **ROS 2 Humble**. It simulates a differential drive robot, estimates its position using dead reckoning (odometry), and autonomously navigates to target coordinates using a closed-loop controller.

The primary goal of this project is to demonstrate the **limitations of Dead Reckoning** by visualizing Sensor Noise and Integration Drift in real-time without relying on external simulators like Gazebo.

## Features
* **Custom Physics Engine:** A lightweight Python-based simulator that replaces heavy tools like Gazebo.
  * Implements differential drive kinematics.
  * Simulates **Gaussian Sensor Noise** on wheel encoders ($\mu=0, \sigma=0.05$).
* **Odometry System:** Real-time state estimation using raw encoder data and Euler integration.
* **Autonomous Controller:** A Proportional (P) Controller capable of:
  * "Turn-then-Move" logic for precise navigation.
  * Dynamic heading correction.
  * Distance-based arrival checking.

### Drift Analysis (Ground Truth vs Estimation)
![PlotJuggler graph showing drift between odom and ground truth](https://github.com/rajindulkar22/ros2-odometry-control/blob/main/Screenshot%20from%202026-01-18%2015-08-41.png)
*The red line (odometry estimate) slowly diverges from the blue line (ground truth) due to accumulated sensor noise.*

## Tech Stack
* **Framework:** ROS 2 Humble
* **Language:** Python 3.10
* **Tools:** Rviz2, PlotJuggler, TF2

## How to Run
1. **Simulation:** `ros2 run odometry_control physics_node`
2. **Estimation:** `ros2 run odometry_control odometry`
3. **Control:** `ros2 run odometry_control goal_controller`

## Future Improvements
* **Extended Kalman Filter (EKF):** Implement sensor fusion with IMU data to correct drift.
* **SLAM Integration:** Add Lidar simulation to enable Simultaneous Localization and Mapping.
* **Launch Files:** Migrate to ROS 2 Launch files to start all nodes with a single command.
