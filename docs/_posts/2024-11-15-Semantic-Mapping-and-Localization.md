---
layout: post
title: Semantic Mapping with RTABMap and ORB_SLAM3 for Localizing an Autonomous Wheelchair
date: November 15th, 2024
image: under_construction.png
toc: true
math: true
featured: true
---

This project implements semantic mapping in tandem with RTABMap and ORB_SLAM3
to create occupancy grid maps and localize an autonomous wheelchair within them.

I create two repositories for this project:
- [ORB_SLAM3_ROS2](https://github.com/gjcliff/ORB_SLAM3_ROS2)
- [RTABMap_Semantic_Mapping](https://github.com/gjcliff/RTABMap_Semantic_Mapping)

And worked with researchers from the [argallab](https://github.com/argallab) to integrate my code with the LUCI
autonomous wheelchair.

## Table of Contents
- [Implementation](#implementation)
- [Semantic Mapping](#semantic-mapping)
- [RTABMap](#rtabmap)
- [ORB_SLAM3](#orb-slam3)
- [How to Run](#how-to-run)
- [Results](#results)
- [Future Work](#future-work)
- [Acknowledgements](#acknowledgements)

## Implementation
The two major components in this project are semantic mapping with a YOLOv8
model and localization with Adaptive Monte-Carlo Localization (AMCL).

I provide two methods for semantic mapping: ORB_SLAM3 and RTABMap. The main
difference between the two methods is the data capture device.

For ORB_SLAM3, I created a ROS2 Humble package that performs visual-inertial odometry
(VIO) SLAM in real-time while at the same time publishing PointCloud2,
OccupancyGrid, Odometry, and Pose-Array messages. I also publish the dynamic
transform from the odom frame to the base_link frame. This package is designed
to perform VIO SLAM with a RealSense D435i camera through a singular monocular
image at 30Hz and IMU data at 200Hz.

For RTABMap, I created CMake package that processes the information from saved
RTABMap database files to obtain a 3D point cloud in pcl format, RGB images, depth
images, and camera calibration information. This package is specifically designed
to process database files created by the RTABMap iPhone app without LIDAR. After
extracting and saving the information from the database, object detection is performed
using a YOLOv8 model loaded into OpenCV C++'s DNN module. The detected objects are
then projected onto the 3D point cloud to create a semantic map.

Both packages can either be run in a docker container for ease of use or built
from source on your host system.

## Semantic Mapping

## RTABMap

## ORB_SLAM3

## How to Run

Please visit this project's [github page](https://github.com/gjcliff/ORB_SLAM3_ROS2) for instructions on how to run the code.

## Results

## Future Work

## Acknowledgements

Thank you to Larisa, Joel, Matt, and Professor Argall for their help and guidance on this project.
