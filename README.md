# Dynamic Terrain Plugins for Gazebo

* [Introduction](#introduction)
* [Requirements](#requirements)
* [Usage](#usage)
* [Demo](#demo)

## Introduction
A demo of dynamic terrain within ROS/Gazebo environment

![](./misc/scene_dynamic_terrain_no_smooth_1.gif) ![](./misc/scene_dynamic_terrain_no_smooth_2.gif)

## Requirements

* ROS distro: melodic
* Gazebo v 9.13 or higher
* ROS Packages:
  - [ros-melodic-husky-desktop](https://wiki.ros.org/husky_desktop)
  - [ros-melodic-husky-simulator](http://wiki.ros.org/husky_gazebo)
  - [ros-melodic-teleop-twist-keyboard](http://wiki.ros.org/teleop_twist_keyboard)

## Usage
Launch demo world using ```roslaunch dynamic_terrain terrain_world.launch```.  
To control the husky robot using a keyboard execute ```rosrun teleop_twist_keyboard teleop_twist_keyboard.py``` in another terminal.

## Demo
![](https://www.youtube.com/watch?v=owiQa95QXVU)
