---
layout: post
title: Robot Arm Teleoperation Through Computer Vision Hand-Tracking
date: March 15, 2024
image: franka_combined.png
toc: true
math: true
---

This project controls the Franka Emika Panda robot arm via teleoperation and computer vision.

<!-- ![franka_servo gif](/public/Franka-Teleop/franka_servo.gif) -->

## [Link to this project's Github](https://github.com/gjcliff/FrankaTeleop)

## Gallery
<iframe width="560" height="315" src="https://www.youtube.com/embed/6R6WPQre0Jg?si=jX77sXLAA7iwiiMV" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

## Introduction
This repository consists of several ROS packages
- franka_teleop: this custom package contains a ROS2 node which implements the
MoveIt! Servo package. It's reponsible for controlling to the movement of the robot
using linear and angular increments.
- handcv: a Python ROS2 node combining Google Mediapipe gesture recognition and
hand tracking to provide hand waypoints and state information.
- cv_franka_bridge: A Python ROS2 node that subscribes to the information handcv
provides, and processes it into commands for the franka_servo node.
- numsr_franka_moveit_config: this custom package contains necessary configuration
files for the Franka robot.
- hand_interfaces: A package containing custom ROS2 messages and services.

## How to Run
### Necessary packages:
* Media Pipe:  
```$ pip install mediapipe```
* RealSense Camera
    * follow setup instructions here: https://github.com/IntelRealSense/realsense-ros/tree/ros2-development
* MoveIt  
```$ sudo apt install ros-{ROS_DISTRO}-moveit```

### If you're in the MSR Lab at Northwestern
The Franka robots we have DO NOT have moveit_servo installed on their control
computers. How unfortunate! Because of this, we'll have to copy over and build
moveit_servo and any other dependencies we're missing.
#### Install Necessary Packages  
```$ git clone git@github.com:gjcliff/FrankaTeleop.git```  
```$ git clone git@github.com:ros-planning/moveit2.git```  
```$ git clone git@github.com:ros2/ros_testing.git```  
* install these to some temporary location on your local drive.

#### Copy all packages over to the Franka control computer
* Choose a reasonable and considerate directory on the Franka control computer
to copy the moveit_servo, ros_testing, and Franka-Teleop repositories into.
* **IMPORTANT** only copy the moveit_servo direectory from the moveit2 package.
The computer already has most of the moveit directories you'll need, just not
moveit_servo. You can find moveit_servo in "moveit2/moveit_ros/moveit_servo"
* Copy our packages by running these commands (you must have an active ethernet
connection to the Franka):  
```$ scp FrankaTeleop/* student@station:/home/Documents/your/directory/here```  
```$ scp moveit_servo/* student@station:/home/Documents/your/directory/here```  
```$ scp ros_testing/* student@station:/home/Documents/your/directory/here```  
* build each of these packages and then source their install/setup.bash files
```$ source FrankaTeleop/install/setup.bash```  
```$ source moveit_servo/install/setup.bash```  
```$ source ros_testing/install/setup.bash```  

You might not have to do this, as I installed these files myself on two of the
robots under ~/Documents/graham_winter_project.  

#### Running Commands
**Real Robot**:
Run on robot: ros2 launch cv_franka_bridge integrate_servo.launch.py use_fake_hardware:=false use_rviz:=false robot_ip:=panda0.robot use_realsense:=false run_franka_teleop:=true
Run on your computer: ros2 launch cv_franka_bridge integrate_servo.launch.py use_fake_hardware:=false use_rviz:=true robot_ip:=panda0.robot use_realsense:=true run_franka_teleop:=false

**Simulated Robot**:
Run on your computer: ros2 launch cv_franka_bridge integrate_servo.launch.py

### If you're not at the MSR Lab
You can definitely get the simulation working. You might need to fiddle with
the camera setup if you don't have a D435i RealSense. I will leave it up to you
to figure out how to get the files working correctly on your robot. What's the old
saying? "The rest is left as an exercise for the reader."

## Implementation
The system is composed of three main nodes: handcv, cv_franka_bridge, and franka_teleop.
The handcv node is responsible for capturing the 3D position of the user's hands and providing
gesture recognition. The cv_franka_bridge is responsible for processing the information
provided from handcv, and sending commands to the franka_teleop node. The franka_teleop
node is running an implementation of the moveit_servo package, which allows for
smooth real-time control of the franka robot.

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
* **Thumbs Down (Shutdown)**: This gesture is used to tell the code to stop tracking your hand,
and to end the program. You will not be able to give the thumbs up gesture
anymore, and will have to restart the program to start tracking your hand again.
* **Close Fist (Close Gripper)**: This gesture will close the gripper of the robot.
* **Open Palm (Open Gripper)**: This gesture will open the gripper of the robot.

If you'd like to see a demonstration of these gestures, you can watch the youtube
video at the top of the page.

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
