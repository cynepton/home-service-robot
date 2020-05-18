# HOME SERVICE ROBOT
A Home Service Robot that will autonomously map an environment and navigate to pickup and deliver objects!

## Working with this project
### Requirements
- KNowledge on Localization, Mapping, SLAM, ROS & Path Planning.
- ROS (It comes with Gazebo installed)
  - For Ubuntu 16.04, install [ROS Kinetic Kame](http://wiki.ros.org/kinetic/Installation/Ubuntu)
  - For Ubuntu 18.04, install [ROS Melodic Morenia](http://wiki.ros.org/melodic/Installation)

1. Upgrade the system, use this command in the terminal
   ```
   sudo apt-get update && apt-get upgrade
   ```

2. 

## Project Build Process

### Shell Scripts
A shell script is a file containing a series of commands and could be executed. It is commonly used to set up environment, run a program, etc.

You already know how to build a `roslaunch` file. It is very convenient to launch multiple ROS nodes and set parameters from a single `roslaunch` command. However, when developing robotic software with different packages, it might get harder to track errors and bugs generated from different nodes.

That's when shell scripts come in handy! After you create a shell script file to launch one or many nodes each in separate terminals, you will have the power to track the output of different nodes and keep the convenience of running a single command to launch all nodes.

#### The launch.sh Script
Let us start by creating this launch.sh script in the Udacity Workspace. Its goal is to launch Gazebo and Rviz in separate instances of terminals. Note that we are using xterm terminal in the script here.