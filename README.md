# HOME SERVICE ROBOT
A Home Service Robot that will autonomously map an environment and navigate to pickup and deliver objects! This is the final project for my Udacity Robotics Software Engineer Nanodegree

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
