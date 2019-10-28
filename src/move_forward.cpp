#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "move_forward", ros::init_options::AnonymousName);
  
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true); //This line constructs an action client that we'll use to communicate with the action named "move_base" that adheres to the MoveBaseAction interface. It also tells the action client to start a thread to call ros::spin() so that ROS callbacks will be processed by passing "true" as the second argument of the MoveBaseClient constructor.
  
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up!");
  }
  ROS_INFO("Robot will move 5 meters forward after 30 seconds!");
  ros::Duration(30).sleep();
  
  move_base_msgs::MoveBaseGoal goal;
  
  
  goal.target_pose.header.frame_id = "map"; 
  goal.target_pose.header.stamp = ros::Time::now();
  
  //Go Home
  goal.target_pose.pose.position.x = 5.0;
  goal.target_pose.pose.position.y =  0.0;
  goal.target_pose.pose.position.z =  0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;
  
  ROS_INFO("Move Forward...");
  ac.sendGoal(goal);
  
  ac.waitForResult();//ros::Duration(10));
  
  
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base reach goal!");
  else
    ROS_INFO("The base failed to move 5 meters forward for some reason!");
  ros::Duration(2).sleep();
  
  return 0;
}
