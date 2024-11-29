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
- [RTABMap](#rtabmap)
- [ORB_SLAM3](#orb-slam3)
- [Semantic Mapping](#semantic-mapping)
- [How to Run](#how-to-run)
- [Results](#results)
- [Future Work](#future-work)
- [Acknowledgements](#acknowledgements)

## Implementation
The two major components in this project are semantic mapping with a YOLOv8
model and localization with Adaptive Monte-Carlo Localization (AMCL).

I provide two methods for semantic mapping: ORB_SLAM3 and RTABMap. The main
difference between the two methods is the data capture device.

Both packages can either be run in a docker container for ease of use or built
from source on your host system.

## RTABMap
For RTABMap, I created a CMake package that processes the information from
saved RTABMap database files to obtain a 3D point cloud in PCL format, RGB
images, depth images, and camera calibration information. This package is
specifically designed to process database files created by the RTABMap iPhone
app without LIDAR.

RTABMap itself only produces the 3D point cloud and the RGB images through VIO
SLAM. The depth images are created by using the iPhone's intrinsic camera matrix
to project points in the 3D point cloud onto the image plane at each step in the
camera's pose graph. Pixel information from the RGB image is also used to give
color to the 3D point cloud.

## ORB_SLAM3
For ORB_SLAM3, I created a ROS2 Humble package that performs visual-inertial
odometry (VIO) SLAM in real-time and publishes PointCloud2, OccupancyGrid,
Odometry, and Pose-Array messages. I also publish the dynamic transform from
the odom frame to the base_link frame. This package is designed to perform VIO
SLAM with a RealSense D435i camera through a singular monocular image at 30Hz
and IMU data at 200Hz.

## Semantic Mapping
After extracting and saving the information from the database, object detection
is performed using a YOLOv8 model loaded into OpenCV C++'s DNN module. The
detected objects are then projected onto the 3D point cloud to create a
semantic map.

## How to Run

Please visit each project's github page for instructions on how to run the code.
* [RTABMap Semantic Mapping](https://github.com/gjcliff/RTABMap_Semantic_Mapping)
* [ORB_SLAM3 ROS2](https://github.com/gjcliff/ORB_SLAM3_ROS2)

## Results

## Future Work

## Acknowledgements

Thank you to Larisa, Joel, Matt, and Professor Argall for their help and guidance on this project.
