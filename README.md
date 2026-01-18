# Custom Odometry & Control Stack (ROS 2 Humble)

## Overview
This project is a custom implementation of a differential drive robot simulation and control stack, built entirely from scratch in Python and ROS 2. It demonstrates the full robotics pipeline: Kinematics -> Simulation -> Estimation -> Control -> Analysis.

## Features
* **Physics Engine:** Custom kinematic simulator simulating wheel slip and Gaussian sensor noise.
* **Odometry System:** Real-time Dead Reckoning estimation using encoder data.
* **Closed-Loop Control:** Proportional controller with heading correction and "turn-then-move" logic.
* **Analysis:** Real-time drift visualization using PlotJuggler (Ground Truth vs. Estimation).

## Tech Stack
* **Framework:** ROS 2 Humble
* **Language:** Python 3.10
* **Tools:** Rviz2, PlotJuggler, TF2

## How to Run
1. **Simulation:** `ros2 run odometry_control physics_node`
2. **Estimation:** `ros2 run odometry_control odometry`
3. **Control:** `ros2 run odometry_control goal_controller`
