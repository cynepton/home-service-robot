#!/bin/sh

#Set workspace location here 
#######################################################################################
catkin_path="~/workspace/catkin_home"
#######################################################################################

#The new_world.launch file to deploy the turtlebot in a custom world set by the user
xterm -e "cd ${catkin_path} && source devel/setup.bash && roslaunch turtlebot_gazebo new_world.launch" &
sleep 5

#The gmapping_demo.launch to perform SLAM
xterm -e "cd ${catkin_path} && source devel/setup.bash && roslaunch turtlebot_gazebo new_gmapping_demo.launch" &
sleep 5

#The view_navigation.launch to view the map in Rviz
xterm -e "cd ${catkin_path} && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

#The keyboard_teleop.launch to manually control the robot with the computer keyboard
xterm -e "cd ${catkin_path} && source devel/setup.bash && roslaunch turtlebot_teleop keyboard_teleop.launch"
