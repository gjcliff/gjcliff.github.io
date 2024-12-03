---
layout: post
title: Teleoperating a Robot Arm with Computer Vision
date: March 15, 2024
image: franka_combined.png
toc: true
math: true
featured: true
repo: gjcliff/FrankaTeleop
---

This project controls the Franka Emika Panda robot arm via teleoperation and computer vision.

![franka_servo gif](/public/Franka-Teleop/franka_servo.gif)

To see a full length video demonstration, head over to the [Gallery](#gallery) section.

## [Link to this project's Github](https://github.com/gjcliff/FrankaTeleop) - ![GitHub Repo stars](https://img.shields.io/github/stars/gjcliff/FrankaTeleop?style=social)

## Introduction
The system is composed of three main nodes: handcv, cv_franka_bridge, and franka_teleop.
The handcv node is responsible for capturing the 3D position of the user's hands and providing
gesture recognition. The cv_franka_bridge is responsible for processing the information
provided from handcv, and sending commands to the franka_teleop node. The franka_teleop
node is running an implementation of the moveit_servo package, which allows for
smooth real-time control of the franka robot.

For specific setup instructions, please see the README.md file in the project's
github repository.

## Implementation
Here's a basic block diagram of the system:
![block diagram](/public/Franka-Teleop/block_diagram.png)

Right now, gestures from both hands are captured, but only
gestures from the right hand are used to control the robot. In a future implementation
of this project, I plan to use the left hand to control the orientation of the end effector.
It's also possible to train the mediapipe model to recognize custom gestures in addition to the default
gestures.

Here's a list of the gestures the system recognizes and what they do:

* **Thumbs Up (Start/Stop Tracking)**: This is one of the gestures used to tell the code to start/stop
tracking the position of your right hand. You also use this gesture to adjust
your hand in the camera frame without moving the robot. While the camera sees
you giving a thumbs up the robot won't move, but once you release your hand
the robot will start tracking your hand.
![thumbs_up](/public/Franka-Teleop/thumbs_up.png)

* **Thumbs Down (Shutdown)**: This gesture is used to tell the code to stop tracking your hand,
and to end the program. You will not be able to give the thumbs up gesture
anymore, and will have to restart the program to start tracking your hand again.
![thumbs_down](/public/Franka-Teleop/thumbs_down.png)

* **Close Fist (Close Gripper)**: This gesture will close the gripper of the robot.
![closed_fist](/public/Franka-Teleop/closed_fist.png)

* **Open Palm (Open Gripper)**: This gesture will open the gripper of the robot.
![open_palm](/public/Franka-Teleop/open_palm.png)

If you'd like to see a demonstration of these gestures, you can watch the youtube
video the [Gallery](#gallery) section.

## Lessons Learned
**Integrating MoveIt and Franka**
I spent a lot of time integrating the franka robot with moveit. The launch files,
URDFs, and SRDF files distributed by Franka themselves were not compatible with
any of the MoveIt tutorials on moveit_cpp or moveit_servo, both of which I wanted
to get working. I ended up having to modify many of the configuration files provided
by Franka by hand to get everything to work. This was a great way to learn in-depth
about getting a robot setup and integrated wth moveit.
**Teleoperating a Robot**
I wasn't sure how to go about achieving teleoperation at first. I ended up deciding
on a PD control loop, where the difference between the desired position of the
end effector and the current position of the end effector is used to calculate
the magnitude of a linear output vector which tells the robot how to move at
each time step. I also learned that when using the moveit_servo package the
robot joints tend to drift, so I had to enforce both the 3D position of the
end effector that I wanted as well as the orientation that I wanted at every
time step.

## Future Work
The features that I would most like to add to this project are:
* 6 dof control of the end-effector
    * Right now I can only control 3 dof (xyz position). To control the orientation,
    I could calculate a vector between the bottom of my left
    palm and the tip of my left middle finger. Doing this would allow me to construct
    a quaternion and use it to control the orientation of the end effector.
* Force control in the gripper
    * I think that this project would greatly benefit from granular force control
    in the gripper. The system is quite dexterous and precise, however I was not
    able to figure out how to get the gripper to grip arbirarily shaped objects
    with specific amounts of force. One day I would like to add this feature.
* Smoother servoing
    * Right now there is this issue where if I move too fast the robot's movement
    starts to get jittery. I would like to write my own ros2_control plugin to allow for smooth control
    of the end-effector. I know that packages and control plugins already exist to do this and have
    been used successfully by others to achieve very smooth servoing, however
    I've been wanting to learn how to write my own ros2_control plugin and this
    would be a great opportunity to do so.

## Gallery
<iframe width="560" height="315" src="https://www.youtube.com/embed/6R6WPQre0Jg?si=jX77sXLAA7iwiiMV" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
