# **6-DOF Robotic Arm in ROS 2**

Author: Ilknur Koparır
Date: 17.12.2025

## **Project Overview**

Over the project we built and simulated a 6-Degree-of-Freedom (6-DOF) robotic manipulator in a virtual industrial environment using the Robot Operating System (ROS2). The goal was to take a robotic arm from digital modeling all the way to intelligent motion control — an end-to-end robotic development pipeline.

## **Watch The Video**

[![Watch the video](https://github.com/user-attachments/assets/a45f994a-9dfb-4276-964f-0161fc12020b)](https://drive.google.com/file/d/10RQ2HRr-g1OwPaWHpk6bmsdDzKO48CQ1/view?usp=sharing)

**HOW TO RUN**

**Build the Workspace**
```bash
colcon build
source install/setup.bash

**Launch the Robot Description**
ros2 launch my_robot_arm_description robot_arm.launch.py

**Launch MoveIt 2**
ros2 launch my_robot_bringup my_robot.launch.py
