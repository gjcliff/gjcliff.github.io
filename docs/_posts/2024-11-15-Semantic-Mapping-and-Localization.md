---
layout: post
title: Semantic Mapping with RTABMap and ORB_SLAM3 for Localization and Navigation
date: December 3rd, 2024
image: localization_combined_20241202.gif
toc: true
math: true
featured: true
---
**Post under construction, please check back on December 11th, 2024 for the complete post.**

This project implements semantic mapping in tandem with RTABMap and ORB_SLAM3
to create occupancy grid maps and localize an autonomous wheelchair within them.

![localization-20241202](/public/Semantic_Mapping/localization_combined_20241202.gif)

I created two repositories for this project:
- [ORB_SLAM3_ROS2](https://github.com/gjcliff/ORB_SLAM3_ROS2)
- [RTABMap_Semantic_Mapping](https://github.com/gjcliff/RTABMap_Semantic_Mapping)

I also worked with researchers from the [argallab](https://github.com/argallab) to integrate my code with the LUCI
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


![Localization](/public/Semantic_Mapping/localization_combined-20241125.gif)

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

Here's an example of what it looks like to overlay points from the point cloud
onto an image:
<center>
  <img src="/public/Semantic_Mapping/rgb_vs_emulated_depth.gif" alt="Emulated Depth" width="400"/>
</center>
<!-- ![Emulated Depth](/public/Semantic_Mapping/rgb_vs_emulated_depth.gif) -->

## ORB_SLAM3
For ORB_SLAM3, I created a ROS2 Humble package that performs visual-inertial
odometry (VIO) SLAM in real-time and publishes PointCloud2, OccupancyGrid,
Odometry, and Pose-Array messages. I also publish the dynamic transform from
the odom frame to the base_link frame. This package is designed to perform VIO
SLAM with a RealSense D435i camera through a singular monocular image at 30Hz
and IMU data at 200Hz.

## Semantic Mapping
After extracting and saving the information from the database, object detection
is performed using a YOLOv8 model loaded into OpenCV C++'s DNN module at each
pose in the camera's pose graph. Points from the 3D point cloud are projected
onto the image plane using the iPhone's intrinsic camera matrix, and points that
are within the bounding box of an object are assigned to and labeled with the
object's class. Objects are represented by 3D point clouds, and the centroid
of each point cloud is used to represent the object's position in the map.

## Localization
Adaptive Monte-Carlo Localization (AMCL) is used to localize the LUCI wheelchair
in the semantic maps created by RTABMap and ORB_SLAM3. This strategy requires a
2D lidar scan, a 2D occupancy grid map, and the initial pose of the robot in the
map. The 2D lidar scan is created from the front right and front left infrared
cameras on the LUCI wheelchair. The 3D point clouds from the infrared cameras
are collapsed down and filtered into 2D lidar scans using the ROS2 package
[pointcloud_to_laserscan](https://github.com/ros-perception/pointcloud_to_laserscan).

## Navigation
The LUCI wheelchair is navigated through the semantic maps using a ROS2 package
created by alumni MSR student [Rintaroh Shima](https://www.linkedin.com/in/rintaroh-shima/). This package uses a user-friendly
PyQt interface to allow the wheelchair operator to select a destination on the
semantic map from a list of loaded landmarks. The wheelchair then navigates to
the selected destination by sending an action-goal request to Nav2's controller
server.

## How to Run

Please visit each project's github page for instructions on how to run the code.
* [RTABMap Semantic Mapping](https://github.com/gjcliff/RTABMap_Semantic_Mapping)
* [ORB_SLAM3 ROS2](https://github.com/gjcliff/ORB_SLAM3_ROS2)

## Results

### RTABMap Semantic Mapping

### ORB_SLAM3 Semantic Mapping

### Localization and Navigation

## Future Work


## Acknowledgements

Thank you to Larisa, Joel, Matt, and Professor Argall for their help and guidance on this project.
