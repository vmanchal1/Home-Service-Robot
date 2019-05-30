#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Define the start position
  move_base_msgs::MoveBaseGoal startPosition;
  
  // set up the frame parameters
  startPosition.target_pose.header.frame_id = "map";
  startPosition.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach at start goal
  startPosition.target_pose.pose.position.x = 3.95;
  startPosition.target_pose.pose.position.y = 7.5;
  startPosition.target_pose.pose.orientation.w = 1.0;

   // Send the first goal position and orientation for the robot to reach
  ROS_INFO("Sending to the start position");
  ac.sendGoal(startPosition);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Robot reached the Start Position");

    //Wait for 5 seconds before continuing 
    sleep(5);
  }
  else
  {
    ROS_INFO("The robot failed to reach the first zone");
    return 0;
  }
  
  // define the end position
  move_base_msgs::MoveBaseGoal endPosition;

  // set up the frame parameters
  endPosition.target_pose.header.frame_id = "map";
  endPosition.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach at end goal
  endPosition.target_pose.pose.position.x = -3.5;
  endPosition.target_pose.pose.position.y = 5.0;
  endPosition.target_pose.pose.orientation.w = 1.0;

  // Reached first goal, let's move on!
  ROS_INFO("Sending to the end position");
  ac.sendGoal(endPosition);
  
  // Wait an infinite time for the results
  ac.waitForResult();
  
  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Robot reached the Destrination");
  }
  else
  {
    ROS_INFO("The robot failed to reach the second zone");
  }

  sleep(5);
  
  return 0;
}
