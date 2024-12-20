---
layout: post
title: SLAM From Scratch
date: March 19th, 2024
image: slam_unknown_good.gif
toc: false
math: true
# featured: true
tags: [cpp, ros2, slam, navigation, lidar, machine learning, kalman filter]
---

This project implements SLAM from scratch on a turtlebot3 hamburger using wheel
odometry, LIDAR, an Extended Kalman Filter (EKF), and machine learning.

![slam_gif](/public/SLAM-from-scratch/slam_unknown_good.gif)

## [Link to this project's Github](https://github.com/gjcliff/EKFSLAM-from-scratch)

## Table of Contents
* [Implementation](#implementation)
* [Gallery](#gallery)

## Implementation
This project consists of several ROS packages
- **nuturtle_description** - This package contains urdf files and basic debugging, testing, and visualization code for turtlebot3s.
- **nusim** - This package contains ros nodes and launchfiles that display turtlebots inside a user-defined arena with user-defined obstacles.
- **nuturtle_control** - This package enables control of the turtlebot via geometry_msgs/msg/Twist messages on the cmd_vel topic.
    - This package contains three nodes:
        * **turtle_control** - Manage the flow of information between all nodes. Convert twists into wheel command values, and motor encoder values into joint states.
        * **odometry** - Perform odometry calculations for the turtlebot, and publish them for other nodes to see
        * **circle** - Command the robot to move in a circle, and offer some additional movement related services (stopping and reversing)
- **nuslam** - This package contains a node and a launch file that performs EKF SLAM using the turtlebot.

This project also includes its own 2D geometry library, **turtlelib** which
provides objects for points, vectors, twists, and transformation matrices. It
also includes helper functions for normalizing angles and normalizing vectors,
which can be quite helpful when implementing SLAM.

A high level overview of the program's structure can be seen below:
![slam_block_diagram](/public/SLAM-from-scratch/slam_block_diagram.png)

<!-- ## [Link to this project's Github](https://github.com/gjcliff/SlamFromScratch) -->

<!-- ## How to Run -->
<!-- **To run SLAM with fake sensor data**: -->
<!-- ```bash -->
<!-- $ ros2 launch nuslam slam.launch.py -->
<!-- ``` -->
<!---->
<!-- **To run SLAM with unknown data association (simulated LIDAR data)**:   -->
<!-- ```bash -->
<!-- $ ros2 launch nuslam unknown_data_assoc.launch.xml -->
<!-- ``` -->
<!---->
<!-- **To run SLAM on the turtlebot**:   -->
<!-- You can follow these instructions to get your code onto the turtlebot:   -->
<!-- https://nu-msr.github.io/ros_notes/ros2/turtlebot3.html -->
<!---->
<!-- on the turtlebot, type in the following command:   -->
<!-- ```bash -->
<!-- $ ros2 launch nuslam turtlebot_bringup.launch.py -->
<!-- ``` -->
<!---->
<!-- on your computer:   -->
<!-- ```bash -->
<!-- $ ros2 launch nuslam pc_bringup.launch.py -->
<!-- ``` -->
<!---->

The **nusim** node is a placeholder for the real turtlebot. It simulates the
turtlebot's encoders and LIDAR, and controls the position of a red turtlebot
in the RVIZ simulation. This robot represents the ground truth of the system.

The **nuturtlebot_control** node captures commands from the user and sends
them to the turtlebot to tell it how to move. This node also captures encoder
data from the turtlebot and transforms it into joint states of the wheels, which
are absolute measurements of how far the wheels have turned since the node started
running, which are then sent to the odometry node.

The **odometry** nodes receives these joint states from the turtlebot, and uses them to
figures out the left and right wheel velocity. These wheel velocities are then
used to calculate a body twist of the robot for the current timestep using
forward kinematics. This body twist is then integrated to produce an updated
state estimate for the robot.

The **landmarks** node reads LIDAR data produced by either the real turtlebot
or the simulated turtlebot, and divides the data into clusters, and then performs
circular regression on each of the clusters to determine which of the clusters
should be counted as landmarks. This data is sent to the SLAM node.

The **slam** node performs EKF slam using the state estimates and body twist
calculated by the odometry node. It receives measurements of possible landmarks
at a rate of 5Hz, and odometry state estimates at a rate of 100Hz. In between
measurements, the state estimates are accumulated and uncertainty from them is
propagated through the covariance matrix. When a measurement is received, the
Kalman gain is calculated and an updated state estimate is calculated. In
simulation, there are two modes you can run this node in. In one mode, fake
landmark data with built-in noise is generated and used to perform SLAM. In the
other mode, the simulated LIDAR data is used to perform circular regression
and data association.

## Gallery
EKF SLAM in the real world with unknown data association and circular regression:
<iframe width="560" height="315" src="https://www.youtube.com/embed/HyAhQ1MOPjE?si=2DXvevmM8yIqDphF" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

EKF SLAM with unknown data association and circular regression:
<iframe width="560" height="315" src="https://www.youtube.com/embed/B4iBtdST0zI?si=YTuaZNBG4CpbqJ0D" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

EKF SLAM with simulated landmark positions:
<iframe width="560" height="315" src="https://www.youtube.com/embed/SA2kviWRW3M?si=uzDI45YVCwwfVhLB" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
