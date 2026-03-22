# **6-DOF Robotic Arm in ROS 2**

Author: Ilknur KOPARIR


## **Project Overview**

Over the project we built and simulated a 6-Degree-of-Freedom (6-DOF) robotic manipulator in a virtual industrial environment using the Robot Operating System (ROS2). The goal was to take a robotic arm from digital modeling all the way to intelligent motion control — an end-to-end robotic development pipeline.


<div align="center">

## 📺 Demo Video

<div align="center">
<a href="https://youtu.be/KmvKYmH3s50" target="_blank">
  <img src="https://img.youtube.com/vi/KmvKYmH3s50/maxresdefault.jpg" alt="ROS 2 Moveit2 - Control a Robotic Arm - Watch on YouTube" width="700"/>
</a>

<br/>

[![▶ Watch on YouTube](https://img.shields.io/badge/▶%20Watch%20on-YouTube-FF0000?style=for-the-badge&logo=youtube&logoColor=white)](https://youtu.be/eUiBadoHhME)

</div>

<a href="https://youtu.be/eUiBadoHhME" target="_blank">
  <img src="https://img.youtube.com/vi/eUiBadoHhME/maxresdefault.jpg" alt="ROS 2 Moveit2 - Control a Robotic Arm - Watch on YouTube" width="700"/>
</a>

<br/>

[![▶ Watch on YouTube](https://img.shields.io/badge/▶%20Watch%20on-YouTube-FF0000?style=for-the-badge&logo=youtube&logoColor=white)](https://youtu.be/eUiBadoHhME)

</div>

**HOW TO RUN**

**Build the Workspace**
```bash
colcon build
source install/setup.bash

**Launch the Robot Description**
ros2 launch my_robot_arm_description robot_arm.launch.py

**Launch MoveIt 2**
ros2 launch my_robot_bringup my_robot.launch.py
