#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true); //This line constructs an action client that we'll use to communicate with the action named "move_base" that adheres to the MoveBaseAction interface. It also tells the action client to start a thread to call ros::spin() so that ROS callbacks will be processed by passing "true" as the second argument of the MoveBaseClient constructor.
  
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up!");
  }
  //ros::Duration(15).sleep();
  move_base_msgs::MoveBaseGoal goal;
  ros::Duration(10).sleep();
  
  //we'll send a goal to the robot to move 1 meter forward
  
  goal.target_pose.header.frame_id = "map"; //We'll just tell the base to move 1 meter forward in the "map" coordinate frame. 
  //while(ros::ok()){}
  goal.target_pose.header.stamp = ros::Time::now();
  
  goal.target_pose.pose.position.x = 8.1;
  goal.target_pose.pose.position.y =  -1.18;
  goal.target_pose.pose.position.z =  0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;
  
  ROS_INFO("Sending goal...");
  ac.sendGoal(goal);
  
  ac.waitForResult();//ros::Duration(10));
  
  
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved to goal!");
  else
    ROS_INFO("The base failed to move to goal for some reason!");
  ros::Duration(2).sleep();
  
  return 0;
}
