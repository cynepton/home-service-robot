// http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals
// http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams
// http://wiki.ros.org/rosparam
// http://wiki.ros.org/Parameter%20Server
// http://wiki.ros.org/roscpp/Overview/Parameter%20Server
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the node to set  the pickup and drop off locations
  // Set the ROS node name to the preferred name, in this case pick_objects
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Add robot position to the parameter server
  ros::param::set("/robot_pose", "initial_position");

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // Set number of goals here
  //int goalCount = 2;
  // Set goal poses in a 2D array with 3 for the x & y positions columns and rows equal to the number of goal positions
  //int goalPose[goalCount][3] = {{-9.393530,0.760470,1.0},{6.34,2.1,1.0}}

  // set up the frame parameters
  // set the frame_id to the robot base frame
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  //Pickup point

  // Define a position and orientation for the robot to reach
  //https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
  goal.target_pose.pose.position.x = -9.4;
  goal.target_pose.pose.position.y = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Approaching pickup point");
  ac.sendGoal(goal);

  // Set robot position as moving to pickup location
  ros::param::set("/robot_pose", "approaching_pickup");

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("The robot has arrived at the pickup point!");
    // wait for 5 seconds
    // Set robot position to indicate arrival at pickup location
    ros::param::set("/robot_pose", "arrived_at_pickup");
    sleep(5);
  }
  else{
    ROS_INFO("Robot unable to reach it's pickup point!!");
    ros::param::set("/robot_pose", "could_not_pickup");
  }

//Drop-Off point

  // set up the frame parameters
  // set the frame_id to the robot base frame
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();
  
  ROS_INFO("Picking up object...");
  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 6.0;
  goal.target_pose.pose.position.y = -1.0;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Approaching drop-off point");
  ac.sendGoal(goal);

  // Set robot position as moving to drop-off location
  ros::param::set("/robot_pose", "approaching_drop-off");

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("The robot has arrived at the drop-off point!");
    // Set robot position as moving to drop-off location
    ros::param::set("/robot_pose", "arrived_at_drop-off");
    // wait for 5 seconds
    sleep(5);
  }
  else{
    ROS_INFO("Robot unable to reach it's drop-off point!!");
    ros::param::set("/robot_pose", "could_not_reach_drop-off");
  }

  
  return 0;
}
