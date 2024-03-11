---
layout: post
title: SLAM From Scratch
date: February 28th, 2024
image: slam_cover.png
toc: true
math: true
---

This project implements SLAM from scratch on a turtlebot3 hamburger using wheel
odometry, LIDAR, an Extended Kalman Filter (EKF), and machine learning.

### **This project is currently under construction, and will be available by March 16th**

Check out some of the results!  
<iframe width="560" height="315" src="https://www.youtube.com/embed/SA2kviWRW3M?si=uzDI45YVCwwfVhLB" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

<!-- ## [Link to this project's Github](https://github.com/gjcliff/SlamFromScratch) -->

![slam arena](/public/SLAM-from-scratch/slam_arena.gif)

<!-- ## How to Run -->
<!---->
<!-- ## Implementation -->
<!-- This project includes its own 2D geometry library, which provides objects for -->
<!-- points, vectors, twists, and transformation matrices. It also includes helper -->
<!-- functions for normalizing angles and normalizing vectors, which can be quite -->
<!-- helpful when implementing SLAM. -->
<!---->
<!-- A high level overview of the program's structure can be seen below: -->
<!---->
<!-- Every circle represents a ROS2 node. -->
<!---->
<!-- The nusim node is a placeholder for the real turtlebot. It simulates the -->
<!-- turtlebot's encoders and LIDAR, the first of which is used to perform wheel -->
<!-- odometry and the second of which is used in the SLAM calculations. -->
<!---->
<!-- The nuturtlebot_control node captures commands from the user and sends -->
<!-- them to the turtlebot to tell it how to move. This node also captures encoder -->
<!-- data from the turtlebot and transforms it into joint states of the wheels, which -->
<!-- are absolute measurements of how far the wheels have turned since the node started -->
<!-- running, which are then sent to the odometry node. -->
<!---->
<!-- The odometry node receives these joint states from the turtlebot and figures out -->
<!-- the left and right wheel velocity using them. These wheel velocities are then -->
<!-- used to calculate a body twist of the robot for the current timestep using -->
<!-- forward kinematics. The forward kinematics algorithm is below: -->
<!---->
<!-- Next, the node integrates this twist to come up with an estimate of the current -->
<!-- state of the robot. This is a 2D vector that looks like this: -->
<!---->
<!-- $$ q_t = \begin{bmatrix} x_t \\ y_t \\ \theta_t \end{bmatrix} $$ -->
<!---->
<!-- And represents the state of the robot at time $t$. -->
<!---->
<!-- In this vector, x and y are the robot's position in the world frame, and $$ \theta $$ is -->
<!-- the robot's orientation in the world frame about the z axis. -->
<!---->
<!-- The odometry node publishes this state inside an odometry ROS message, which is -->
<!-- received by the SLAM node. -->
<!---->
<!-- ### SLAM with EKF -->
<!-- The odometry node publishes state estimates at a frequency of 100hz, however -->
<!-- the SLAM node only runs at a frequency of 5hz, and receives measurement data -->
<!-- at the same frequency. State estimates are accumulated by the SLAM node -->
<!-- in-between measurements. When a state estimate from the odomatry node is  -->
<!-- received,  -->


<!-- ## Gallery -->

