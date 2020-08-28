# Beetle-Bot-Arm
BeetleBot is aimed to be an open source learning and tinkering platform for anyone interested in control of Robotic Manipulators and Computer Vision. Currently the project is under initial development and configurations. 

# About
Beetlebot is a 5-DoF robotic manipulator designed in Solidworks. Currently the arm is capable of basic manipulation and path motions. The robotic arm can either be printed using the provided .STL files and assembled with addition to some hardware and an Arduino board. Otherwise, it can be simulated in Rviz/Gazebo environment. For anyone looking to assemble the robot please find the list of required parts below:

* 3D printable Parts provided in the Repository
* Arduino Mega/Uno Board (Prefer Mega)
* MG996R Servo x2 - For base and end effector bracket (continous rotation/unlocked servo)
* MG995 Servo x2 - For both arms 
* SG90 Servo x2 - For Gripper Attachment
* OV7670 Camera Module
* DC to DC Step down Buck Converter (LM2596)
* 16 Channel 12 Bit Servo Driver Board (PCA9685)
* 12V DC Power Source
* Ball Bearings x2 (8x22x7 MM)
* Screws and Nuts
* Jumper Cables

Note: All these parts are easily available on Amazon/Other Retailers

# Installation
This part assumes that you are running Ubuntu 18.04 and have the following dependencies installed:
* ROS Melodic
* Rviz
* MoveIt
* Gazebo
* Rosserial
* OpenCV

Setup your environment
```shell
$ mkdir -p ~/workspace/src
$ cd ~/workspace
$ catkin_make
$ source devel/setup.bash
```
Clone this repository
```console
$ cd ~/workspace/src
$ git clone https://github.com/Kartik-Singhal26/Beetlebot-Arm
$ cd ~/workspace
```
To run Rviz Simulation:
```console
$ roslaunch beetlebot_moveit_config demo.launch
```
To run Gazebo Simulation:
```console
$ roslaunch beetlebot_moveit_config gazebo.launch
```

# Project Status
## Tasks 
* Added: Gripper
* Solved: Orientation mismatch between Solidworks and Rviz/Gazebo
* Created: Moveit Package
* Created: URDF File
* Completed: Design and Assembly in Solidworks

## Current Issues
* None

## Planned Tasks
* Blob Detection with OpenCV
* Motion Planning with the real robot


