# HOME SERVICE ROBOT
A Home Service Robot that will autonomously map an environment and navigate to pickup and deliver objects!

## Working with this project
### Requirements
- Knowledge on Localization, Mapping, SLAM, ROS & Path Planning.
- ROS (It comes with Gazebo installed)
  - For Ubuntu 16.04, install [ROS Kinetic Kame](http://wiki.ros.org/kinetic/Installation/Ubuntu)
  - For Ubuntu 18.04, install [ROS Melodic Morenia](http://wiki.ros.org/melodic/Installation)

1. Upgrade the system, use this command in the terminal
   ```
   sudo apt-get update && apt-get upgrade
   ```

2. 

## Project Build Process

## Here's a list of the steps in this project - use it to track your progress!

- Simulation setup
- SLAM Testing
- Wall Follower Node
- Navigation Testing
- Waypoint Node
- Virtual Objects
- Put it all Together

### Shell Scripts
A shell script is a file containing a series of commands and could be executed. It is commonly used to set up environment, run a program, etc.

You already know how to build a `roslaunch` file. It is very convenient to launch multiple ROS nodes and set parameters from a single `roslaunch` command. However, when developing robotic software with different packages, it might get harder to track errors and bugs generated from different nodes.

That's when shell scripts come in handy! After you create a shell script file to launch one or many nodes each in separate terminals, you will have the power to track the output of different nodes and keep the convenience of running a single command to launch all nodes.

#### The launch.sh Script
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


#### Code Breakdown

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