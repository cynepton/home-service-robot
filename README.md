# HOME SERVICE ROBOT
A Home Service Robot that will autonomously map an environment and navigate to pickup and deliver objects!

## Github Repository Link
https://github.com/cynepton/home-service-robot

## Working with this project
### Requirements
- Knowledge on Localization, Mapping, SLAM, ROS & Path Planning.
- ROS (It comes with Gazebo installed)
  - For Ubuntu 16.04, install [ROS Kinetic Kame](http://wiki.ros.org/kinetic/Installation/Ubuntu)
  - For Ubuntu 18.04, install [ROS Melodic Morenia](http://wiki.ros.org/melodic/Installation)

1. **Upgrade the system, use this command in the terminal**
   ```
   sudo apt-get update && apt-get upgrade
   ```

2. **Clone the repository**
```
git clone https://github.com/cynepton/home-service-robot.git
```

3. **Build the workspace**
   In the root of the `catkin_home` directory from the terminal, run
   ```
   catkin make
   ```
   and source it
   ```
   source devel/setup.bash
   ```

## Project Build Process

### Here's a list of the steps in this project - use it to track your progress!

- Simulation setup
- SLAM Testing
- Wall Follower Node
- Navigation Testing
- Waypoint Node
- Virtual Objects
- Put it all Together

## Shell Scripts
A shell script is a file containing a series of commands and could be executed. It is commonly used to set up environment, run a program, etc.

You already know how to build a `roslaunch` file. It is very convenient to launch multiple ROS nodes and set parameters from a single `roslaunch` command. However, when developing robotic software with different packages, it might get harder to track errors and bugs generated from different nodes.

That's when shell scripts come in handy! After you create a shell script file to launch one or many nodes each in separate terminals, you will have the power to track the output of different nodes and keep the convenience of running a single command to launch all nodes.

### The launch.sh Script
Let us start by creating this `launch.sh` script in the Workspace. Its goal is to launch Gazebo and Rviz in separate instances of terminals. Note that we are using `xterm` terminal in the script here.

```
#!/bin/sh
xterm  -e  " gazebo " &
sleep 5
xterm  -e  " source /opt/ros/kinetic/setup.bash; roscore" & 
sleep 5
xterm  -e  " rosrun rviz rviz" 
```

Note: To use xterm with another terminal, install `xterm` with: 

```
sudo apt install xterm
```
or 

```
sudo apt-get install xterm
```


The `launch.sh` shell script launches three terminals and issues one or multiple commands in each terminal. Let’s break down this script to understand the meaning of each line.


### Code Breakdown

`#!/bin/sh`

This statement is called a `shebang`. It must be included in every shell script you write since it specifies the full path of the UNIX interpreter to execute it.

`xterm -e " gazebo " &`

With the `xterm -e` statement, we launch a new instance of an `xterminal`. Inside this terminal, we launch gazebo using the command `"gazebo"`. Then we add an ampersand `&` to indicate that another instance of an xterm terminal will be created in a separate statement.

`sleep 5`

We pause this script for 5 seconds using `sleep`.

`xterm -e " source /opt/ros/kinetic/setup.bash; roscore" &`

We launch a second instance of the xterm terminal. Inside this terminal, we source the ROS workspace and launch the ROS master node.

`sleep 5`

We pause this script for another 5 seconds.

`xterm -e " rosrun rviz rviz"`

We are launching a third instance of the xterm terminal, and running rviz.

Save your script file and give it `execute` pemission by `chmod +x launch.sh`. Then launch the shell script with `./launch.sh`.

After launching this script, we’ll have three open xterm terminals, and we will be able to track any errors or bugs that occur. To recap, this script will open the first terminal and launch gazebo. Then it will pause for 5 seconds and open a second terminal to launch the ROS master. It will pause for another 5 seconds and, finally, open a third terminal to launch RVIZ.

Try to launch your script in the Workspace and verify its functions!

## Simulation Setup
### Catkin Workspace
To program your home service robot, you will need to interface it with different ROS packages. Some of these packages are official ROS packages which offer great tools and others are packages that you’ll create. The goal of this section is to prepare and build your `catkin workspace`.

Here’s the list of the official ROS packages that you will need to grab, and other packages and directories that you’ll need to create at a later stage as you go through the project. 
In your `workspaces` folder,

```
mkdir catkin_ws && cd catkin_ws
```
The `catkin_ws` name is arbitrary

```
mkdir src && cd src/
```
```
catkin_init_workspace
catkin_make
```


### Official ROS packages
Import these packages now and install them in the `src` directory of your `catkin workspace`. Be sure to clone the full GitHub directory and not just the package itself.

1. [gmapping](http://wiki.ros.org/gmapping): With the gmapping_demo.launch file, you can easily perform SLAM and build a map of the environment with a robot equipped with laser range finder sensors or RGB-D cameras.
2. [turtlebot_teleop](http://wiki.ros.org/turtlebot_teleop): With the keyboard_teleop.launch file, you can manually control a robot using keyboard commands.
3. [turtlebot_rviz_launchers](http://wiki.ros.org/turtlebot_rviz_launchers): With the view_navigation.launch file, you can load a preconfigured rviz workspace. You’ll save a lot of time by launching this file, because it will automatically load the robot model, trajectories, and map for you.
4. [turtlebot_gazebo](http://wiki.ros.org/turtlebot_gazebo): With the turtlebot_world.launch you can deploy a turtlebot in a gazebo environment by linking the world file to it.

### Your Packages and Directories
You’ll install these packages and create the directories as you go through the project.
First navigate to the `src/` folder

1. **map**: Inside this directory, you will store your gazebo world file and the map generated from SLAM.

2. **scripts**: Inside this directory, you’ll store your shell scripts.

3. **rvizConfig**: Inside this directory, you’ll store your customized rviz configuration files.

  Create the directory with:
  
  ```
  cd src/
  catkin_create_pkg pick_objects roscpp std_msgs message_generation
  ```

4. **pick_objects**: You will write a node that commands your robot to drive to the pickup and drop off zones.
  Create the package with:
  
  ```
  catkin_create_pkg pick_objects roscpp std_msgs message_generation
  ```
  We will be writing nodes in C++. Since we already know in advance that this package will contain C++ source code and messages, I create the package with those dependencies.

5. **add_markers**: You will write a node that model the object with a marker in rviz.
  Create the package with:
  
  ```
  catkin_create_pkg add_markers roscpp std_msgs message_generation
  ```
  We will be writing nodes in C++. Since we already know in advance that this package will contain C++ source code and messages, I create the package with those dependencies.

Your `catkin_ws/src` directory should look as follows:


    ├──                                # Official ROS packages
    |
    ├── slam_gmapping                  # gmapping_demo.launch file                   
    │   ├── gmapping
    │   ├── ...
    ├── turtlebot                      # keyboard_teleop.launch file
    │   ├── turtlebot_teleop
    │   ├── ...
    ├── turtlebot_interactions         # view_navigation.launch file      
    │   ├── turtlebot_rviz_launchers
    │   ├── ...
    ├── turtlebot_simulator            # turtlebot_world.launch file 
    │   ├── turtlebot_gazebo
    │   ├── ...
    ├──                                # Your packages and direcotries
    |
    ├── map                          # map files
    │   ├── ...
    ├── scripts                   # shell scripts files
    │   ├── ...
    ├──rvizConfig                      # rviz configuration files
    │   ├── ...
    ├──pick_objects                    # pick_objects C++ node
    │   ├── src/pick_objects.cpp
    │   ├── ...
    ├──add_markers                     # add_marker C++ node
    │   ├── src/add_markers.cpp
    │   ├── ...
    └──
   

## SLAM Testing
The next task of this project is to autonomously map the environment you designed earlier with the Building Editor in Gazebo. But before you tackle autonomous mapping, it’s important to test if you are able to manually perform SLAM by teleoperating your robot. The goal of this step is to manually test SLAM.

Write a shell script test_slam.sh that will deploy a turtlebot inside your environment, control it with keyboard commands, interface it with a SLAM package, and visualize the map in rviz. We will be using turtlebot for this project but feel free to use your personalized robot to make your project stand out!