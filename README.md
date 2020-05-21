# HOME SERVICE ROBOT
A Home Service Robot that will autonomously map an environment and navigate to pickup and deliver objects! This is the final project for my Udacity Robotics Software Engineer Nanodegree

## Github Repository Link
https://github.com/cynepton/home-service-robot

## Overview on ROS packages
**The following ROS packages are used in the project:**

[gmapping](http://wiki.ros.org/gmapping): The ROS gmapping package provides laser-based SLAM, by using the slam_gmapping node. It generates a 2D Occupancy grid map by subscribing to the laser and position data collected by the robot's sensors. This map is published to the `map` topic which can then be retrieved by using the `map_saver` node from the ROS  [`map_server`](http://wiki.ros.org/map_server) package. The `map_saver` node creates a map.pgm file which is an image format of the map and a map.yaml file which contains the necessary details of the map.

[turtlebot_teleop](http://wiki.ros.org/turtlebot_teleop): The turtlebot_teleop provides launch files that allow the user to control a robot's movement with keyboard, PS3 joystick or XBOX joystick inputs. For this project, the `keyboard_teleop.launch` is used to control the turtlebot_with the keyboard commands, this is especially useful when performing SLAM. 

[turtlebot_rviz_launchers](http://wiki.ros.org/turtlebot_rviz_launchers?distro=kinetic): This package reduces the time taken to manually subscribe to each topic in RViz by launching Rviz based on a pre-configured rviz config file. 

[turtlebot_gazebo](http://wiki.ros.org/turtlebot_gazebo): The turtlebot_gazebo package contains a number of launch files that allow the robot to perform localization and navigation functions.  It contains the `turtlebot_world.launch` that launches the turtlebot in a predefined gazebo world. The turtlebot can be replaced with a personalised robot, and the default world with another world file. This launch file is necessary for SLAM, localization and navigation as it provides the world and robot that the other functions would interact with.
It also contains the `gmapping_demo.launch` file that launches the `slam_gmapping` node from the `gmapping` package to perform localization and mapping.
Finally, it contains the `amcl_demo.launch` file that launches three nodes: 
- **`map_server`** node from the [`map_server`](http://wiki.ros.org/map_server) package: The map_server node reads a map from the disk (in this case, our saved `map.yaml` generated durring the mapping operation), and publishes this map to the `map` topic.
- **`amcl`** node from the [`amcl`](http://wiki.ros.org/amcl) package: The amcl node uses Adaptive Monte Carlo to localise the robot by subscribing to the laser scans from the robot and the `map` topic (published by the map_server node in this case).
- **`move_base`** node from the [`move_base`](http://wiki.ros.org/move_base?distro=kinetic) package which is a submodule of the ROS [`navigation`](http://wiki.ros.org/navigation?distro=kinetic) package. It provides an interface for interacting with the ROS Navigation stack. The move_base node subscribes to a goal defined by the user and publishes commands to the `cmd/vel` topic to navigate the robot towards the goal.

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

### Here's a list of the steps in this project - use it to track the progress!

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
mkdir catkin_home && cd catkin_ws
```
The `catkin_home` name is arbitrary

```
mkdir src && cd src/
```
Initialise the workspace and build it.
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

- **map**: Inside this directory, you will store your gazebo world file and the map generated from SLAM.
  
  ```
  mkdir map
  ```

- **scripts**: Inside this directory, you’ll store your shell scripts.
  ```
  mkdir scripts
  ```

- **rvizConfig**: Inside this directory, you’ll store your customized rviz configuration files.

  Create the directory with:
  
  ```
  mkdir rvizConfig
  ```

- **pick_objects**: You will write a node that commands your robot to drive to the pickup and drop off zones.
  Create the package with:
  
  ```
  catkin_create_pkg pick_objects roscpp move_base_msgs actionlib
  ```
  We will be writing nodes in C++. Since I already know in advance that this package will contain C++ source code and messages, I created the package with those dependencies.

- **add_markers**: You will write a node that model the object with a marker in rviz.
  Create the package with:
  
  ```
  catkin_create_pkg add_markers roscpp std_msgs message_generation
  ```
  We will be writing nodes in C++. Since we already know in advance that this package will contain C++ source code and messages, I created the package with those dependencies.

Your `catkin_home/src` directory should look as follows:


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

Write a shell script test_slam.sh that will deploy a turtlebot inside your environment, control it with keyboard commands, interface it with a SLAM package, and visualize the map in rviz. A personalised robot can also be used instead, and also a custom world.

### SLAM Testing Task List
To manually test SLAM, create a test_slam.sh shell script that launches these files:
- The `turtlebot_world.launch` file to deploy a turtlebot in your enviroment
- The `gmapping_demo.launch` or run slam_gmapping to perform SLAM
- The `view_navigation.launch` to observe themap in rviz
- The `keyboard_teleop.launch` to manually control the robot with keyboard commands

### Run and Test
Launch your test_slam.sh file, search for the `xterminal`  running the `keyboard_teleop` node, and start controlling your robot. There is no need to fully map your environment but just make sure everything is working fine. 
Make sure to create a functional map of the environment which would then be used for localization and navigation tasks.

Note that the `catkin_path` variable should be adjusted to your location of the catkin workspace folder

### test_xlam.sh code 
```
#!/bin/sh

#Set workspace location here 
#######################################################################################
catkin_path="~/workspace/catkin_home"
#######################################################################################

#The new_world.launch file to deploy the turtlebot in a custom world set by the user
xterm -e "cd ${catkin_path} && source devel/setup.bash && roslaunch turtlebot_gazebo new_world.launch" &
sleep 5

#The gmapping_demo.launch to perform SLAM
xterm -e "cd ${catkin_path} && source devel/setup.bash && roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5

#The view_navigation.launch to view the map in Rviz
xterm -e "cd ${catkin_path} && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

#The keyboard_teleop.launch to manually control the robot with the computer keyboard
xterm -e "cd ${catkin_path} && source devel/setup.bash && roslaunch turtlebot_teleop keyboard_teleop.launch"
```

Save the file in the `src/scripts` directory of the `catkin_home` folder
Navigate to the scripts folder and run 

```
chmod +x test_slam.sh
```

Finally, run the shell script with 
```
./test_slam.sh
```

Confirm that everything works fine, remember, there is no need to fully map the enviroment

## Localization and Navigation Testing
The next task of this project is to pick two different goals and test your robot's ability to reach them and orient itself with respect to them. We will refer to these goals as the pickup and drop off zones. This section is only for testing purposes to make sure our robot is able to reach these positions before autonomously commanding it to travel towards them.

We will be using the ROS Navigation stack, which is based on the Dijkstra's, a variant of the Uniform Cost Search algorithm, to plan our robot trajectory from start to goal position. The ROS navigation stack permits your robot to avoid any obstacle on its path by re-planning a new trajectory once your robot encounters them. You are familiar with this navigation stack from the localization project where you interfaced with it and sent a specific goal for your robot to reach while localizing itself with AMCL. 

### Navigation Test Task List
Write a `test_navigation.sh` shell script that launches these files:
- Add `turtlebot_world.launch` to deploy a turtlebot in the enviroment
- Add `amcl_demo.launch` to localize the turtlebot
- Add `view_navigation.launch` to observe the map in rviz

### test_navigation.sh code 

```
#!/bin/sh

#Set workspace location here 
#######################################################################################
catkin_path="~/workspace/catkin_home"
#######################################################################################

#The new_world.launch file to deploy the turtlebot in a custom world set by the user
xterm -e "cd ${catkin_path} && source devel/setup.bash && roslaunch turtlebot_gazebo new_world.launch" &
sleep 5
#The amcl_demo.launch to loalize the turtlebot
xterm -e "cd ${catkin_path} && source devel/setup.bash && roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
#The view_navigation.launch to view the map in Rviz
xterm -e "cd ${catkin_path} && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch" &
```

Save the file in the `src/scripts` directory of the `catkin_home` folder
Navigate to the scripts folder and run 

```
chmod +x test_navigation.sh
```

Finally, run the shell script with 
```
./test_navigation.sh
```

### Test it
Once you launch all the nodes, you will initially see the particles around your robot, which means that AMCL recognizes the initial robot pose. Now, manually point out to two different goals, one at a time, and direct your robot to reach them and orient itself with respect to them. 

## Navigation Goal Node

**Reaching Multiple Goals**
Earlier, you tested your robot capabilities in reaching multiple goals by manually commanding it to travel with the 2D NAV Goal arrow in rviz. Now, you will write a node that will communicate with the ROS navigation stack and autonomously send successive goals for your robot to reach. As mentioned earlier, the ROS navigation stack creates a path for your robot based on **Dijkstra's** algorithm, a variant of the **Uniform Cost Search** algorithm, while avoiding obstacles on its path.

There is an official ROS tutorial that teaches you how to send a single goal position and orientation to the navigation stack. You are already familiar with this code from the Localization project where you used it to send your robot to a pre-defined goal. Check out the [tutorial](http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals) and go through its documentation.

Here’s the C++ code of this node which sends a **single goal** for the robot to reach. 

```
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
}
```

### Customize the code
You will need to modify this code and edit its node name to **pick_objects**. Then, edit the **frame_id** to `map`, since your fixed frame is the map and not base_link. After that, you will need to modify the code and include an extra goal position and orientation for your robot to reach.

The first goal should be your desired pickup goal and the second goal should be your desired drop off goal. The robot has to travel to the desired pickup zone, display a message that it reached its destination, wait 5 seconds, travel to the desired drop off zone, and display a message that it reached the drop off zone.

### Reaching Multiple Goals
Follow these instructions to autonomously command the robot to travel to both desired pickup and drop off zones:
- Create a pick_objects package with `move_base_msgs`, `actionlib`, and `roscpp` deependencies
- Create a `pick_objects` C++ node
- Edit C++ node and modify its `node name` and `frame_id`
- Modify the C++ node and publish a second goal for the robot to reach
- Display messsages to track if robot sucessfully reached both zones
- Pause 5 seconds after reaching the pickup zone
- Edit the `CMakeLists.txt` file and add `directories`, `executable` and `target_link_libraries`
- Build the `catkin_home`
- Create a `pick_objects.sh` script file that launches the **turtlebot**, **AMCL**, ********rviz** and the **pick_objects** node.
