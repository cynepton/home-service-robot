#!/bin/sh

#Set workspace location here 
#######################################################################################
catkin_path="~/workspace/catkin_home"
#######################################################################################

#The new_world.launch file to deploy the turtlebot in a custom world set by the user
xterm -e "cd ${catkin_path} && source devel/setup.bash && roslaunch turtlebot_gazebo new_world.launch" &
sleep 15
#The new_amcl_demo.launch to localize the turtlebot
xterm -e "cd ${catkin_path} && source devel/setup.bash && roslaunch turtlebot_gazebo new_amcl_demo.launch" &
sleep 15
#The view_navigation.launch to view the map in Rviz
xterm -e "cd ${catkin_path} && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 10
#The add_markers node
xterm -e "cd ${catkin_path} && source devel/setup.bash && rosrun add_markers add_markers" 