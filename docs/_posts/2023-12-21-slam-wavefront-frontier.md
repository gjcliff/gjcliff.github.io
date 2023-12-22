---
layout: post
title: SLAM Frontier Exploration with Visualized Wavefront Planning
date: December 21st, 2023
image: nubot.png
toc: true
math: true
---
Using the ROS2 slam_toolbox and NAV2 packages to help a robot explore and map its environment.

## [Link to this project's Github](https://github.com/gjcliff/SLAM-Frontier-Exploration)

## Introduction

This package uses frontier exploration and wavefront planning to autonomously drive a differential-drive robot named "nubot"
around its environment, while simultaneously mapping that environment using slam_toolbox. NAV2 is used to plan paths for nubot.

Here are links to the packages I used in this project:
- [slam_toolbox](https://index.ros.org/p/slam_toolbox/github-SteveMacenski-slam_toolbox/#iron)
- [NAV2](https://navigation.ros.org/index.html)

## How to Run
1. Clone nubot from https://github.com/m-elwin/nubot into the same workspace as this repository.
2. Build and source the workspace.
3. To have nubot map the environment autonomously, run:
    ```
    ros2 launch nubot_nav explore.launch.xml
    ```
4. To have nubot map the environment manually, with input from you, run:
    ```
    ros2 launch nubot_nav manual_gexplore.launch.xml
    ```

## Implementation

The simulated robot maps its surrounding environment using the ROS2 slam_toolbox package, and then navigates within the map that's created using the ROS2 NAV2 package.

The exploration algorithm used is [Frontier Exploration](https://web.archive.org/web/20230531203647/http://robotfrontier.com/frontier/index.html)

Frontier Exploration is supplemented by [Wavefront Planning](https://www.cs.tufts.edu/comp/150IR/labs/wavefront.html). Wavefront planning goes and seeks out portions of the  map that are unknown, and once it finds one the Frontier Exploration logic determines if the unknown area should be explored.

The flow logic within the code is as follows:
1. Wavefront planning algorithm will rapidly search the area surrounding the robot. There are three types of cells the wavefront algorithm looks for: open, unknown, and occupied. In the graphics that follow, open cells will be white, unknown will be grey, and occupied will be black.

    ![wavefront-gif](/public/SLAM_NUBOT/wavefront.gif "wavefront gif")

The algorithm takes a step of size 2 to speed up the searching. Visualized, it looks like this:

<img src="/public/SLAM_NUBOT/wavefront_demo1.png" alt="Centered Image" style="display: block; margin: 0 auto;">

Unfortunately, this leads to situations like the one below. The wavefront algorithm can "jump" over walls, which is not a behavior that we want:

<img src="/public/SLAM_NUBOT/wavefront_demo2.png" alt="Centered Image" style="display: block; margin: 0 auto;">

Luckily, it's usually the case that there are unknown cells right behind wall cells. If the wavefront planning algorithm finds an unknown cell, it will immediately stop and begin to determine if that unknown cell is a valid frontier. 

Frontiers are determined by searching a small area around the unknown cell thoroughly. If there are any occupied cells at all inside this small area, the frontier is deemed invalid. If there are less than a certain number of other unknown cell within the small area, the frontier is deemed invalid.

Let's say that in the case above, there are a group of unknown cells behind the wall. Even though this is a nice big unknown area, the frontier would be deemed invalid because of the wall cells. 

<img src="/public/SLAM_NUBOT/wavefront_demo3.png" alt="Centered Image" style="display: block; margin: 0 auto;">

The reason this has to be the case is because the nubot determines that it has reached its goal if the center of its chassis has the same coordinates and orientation as the original goal. If the code decided that this was a valid frontier despite the wall cells being there, the nubot would never be able to reach its goal because it's too close to the wall.

The nubot will continue exploring unknown areas until there are none left to explore. 

## Gallery

<iframe width="560" height="315" src="https://www.youtube.com/embed/K1LOrBtIQ58?si=OiT6pcVKn2PE_AO2" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
