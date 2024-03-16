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

#### Running Commands
Run on robot: ros2 launch cv_franka_bridge integrate_servo.launch.py use_fake_hardware:=false use_rviz:=false robot_ip:=panda0.robot use_realsense:=false run_franka_teleop:=true
Run on your computer: ros2 launch cv_franka_bridge integrate_servo.launch.py use_fake_hardware:=false use_rviz:=true robot_ip:=panda0.robot use_realsense:=true run_franka_teleop:=false

## Implementation
Here's a basic block diagram of the system:
![block diagram](/public/Franka-Teleop/block_diagram.png)

The system is composed of three main nodes: handcv, cv_franka_bridge, and franka_teleop.
handcv is responsible for capturing the 3D position of the user's hands and providing
gesture recognition. Right now, gestures from both hands are captured but only
gestures from the right hand are used to control the robot. It's also possible 
to train the model to recognize extra gestures in addition to the default mediapipe
gestures.

To begin teleoperation, give the camera a thumbs up with your right hand. The
robot will immediately start tracking the location of your right hand. To be
exact, the center of your palm is used as the center of your hand. As you move
your hand, the robot end effector will move with you. By default, camera is
meant to be facing you, and the robot is also meant to be facing you. This will
make it so when you move your hand to the right, the robot will also move to the
right, and so on. There is no logic to determine the orientation or location of 
the camera in regards to the robot.

This is something that I want to add in the future, although I think it may
involve some sort of hardcoding since the camera can't see the location of the robot.

If you want to stop teleoperation permanently, give the camera a thumbs down with
your right hand. This won't kill the cv_franka_bridge node, but it will stop
tracking your hand until the node is killed and restarted.

If you want to readjust your hand in the frame of the camera without moving the
robot, give the camera another thumbs up with your right hand. As long as you
are giving a thumbs up, the robot won't track the postion of your hand in the
camera frame. Once you stop giving a thumbs up, the robot will start tracking
your hand again.

If any of this confuses you, I assure you it's not your fault. I suggest watching
the youtube video of me demoing the system. The link to this video is at the
bottom of this page in the gallery section.

You can close the gripper by making a fist with your right hand. The gesture
recognition is a little finicky and it's not perfect, so it's somewhat important
to try and make a fist with your palm facing the camera. The gripper will open
when you unclench your fist and show the camera your open palm.

The current implementation of the gripper is quite simple. It will grab any
object that can fit between the gripper's fingers, however it cannot grip with
precise forces. This is a limitation of the Franka's ROS2 gripper node and the
Grasp Action message (I'm 99% sure), but in the future I would love to be able
to objects of any width with specific amounts of force. If I am wrong about this,
please don't hesitate to contact me and correct me. I would actually really
appreciate it if you did.

## Lessons Learned
I learned a lot about how robots are integrated with moveit in this project. I
spent the first four weeks digging deep into moveit and franka documentation and
trying to understand how each of them were integrated with each other.
What I learned was that even though moveit uses the franka robot arm in all of
its tutorials, the files moveit provides for the arm will not work on the real
robot. I also learned that the configuration files provided by the franka arm's
team don't play very well with moveit_cpp or moveit_servo. I had to go deep into
both packages' configuration files and painstakingly combine them to get my project
to work. The resulting package is the msr_franka_moveit_config package.

## Future Work
The features that I would most like to add to this project are:
* 6 dof control of the end-effector
    * Right now I can only control 3 dof (xyz position). I would really love
    to be able to control the orientation of the end effector as well. I would use
    my left hand to do this by calculating a vector between the bottom of my left
    palm and the tip of my left middle finger. Doing this would allow me to construct
    a quaterion and use it to control the orientation of the end effector.
* Force control in the gripper
    * I think that this project would greatly benefit from granular force control
    in the gripper. The system is quite dexterous and precise, and it really bugs
    me that the gripper is not as precise as the rest of the system.
* Smoother servoing
    * I would like to write my own ros2_control plugin to allow for smooth control
    of the end-effector. I know that packages already exist to do this and have
    been used successfully by others to achieve very smooth servoing, however
    I've been wanting to learn how to write my own ros2_control plugin and this
    would be a great opportunity to do so.
* Behavior Cloning
    * I can see how this interface could be leveraged to train a behavior cloning
    model, and I think this would be a great learning opportunity.
