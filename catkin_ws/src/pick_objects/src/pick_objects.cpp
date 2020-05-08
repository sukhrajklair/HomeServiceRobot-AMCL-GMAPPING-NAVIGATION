#include "ros/ros.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "add_markers/DisplayMarker.h"

// Define a client for to send drop requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_drops node
  ros::init(argc, argv, "simple_navigation_drops");
  
	//display marker service client
	ros::ServiceClient displayMarkerClient = n.serviceClient<add_markers::DisplayMarker>("/add_markers/add_marker");
	
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
	move_base_msgs::MoveBaseGoal pick;
  move_base_msgs::MoveBaseGoal drop;
	
  // set up the frame parameters
  pick.target_pose.header.frame_id = "map";
  pick.target_pose.header.stamp = ros::Time::now();
  drop.target_pose.header.frame_id = "map";
  drop.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  pick.target_pose.pose.position.x = 3.34;
  pick.target_pose.pose.position.y = 2.57;
  pick.target_pose.pose.orientation.z = 0.88;
	drop.target_pose.pose.position.x = -2.91;
	drop.target_pose.pose.position.y = 3.28;
  drop.target_pose.pose.orientation.z = 0.77;
  
  //display marker at pick position
  display_markers::DisplayMarker marker;
  marker.request.display = true;
  marker.request.x = pick.target_pose.pose.position.x;
  marker.request.x = pick.target_pose.pose.position.y;
  displayMarkerClient.call(marker);
  
  // Send the pick position and orientation for the robot to reach
  ROS_INFO("Sending pick position");
  ac.sendGoal(pick);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its pick pose
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the robot has reached its pickup spot");
    //hide marker if the robot has reached its drop location
		marker.request.display = false;
		displayMarkerClient.call(marker);
  else
    ROS_INFO("The robot failed to reach its pickup spot");
	
	//wait for 5 seconds
	ros::Duration(5.0).sleep();
	
	// Send the pick position and orientation for the robot to reach
  ROS_INFO("Sending drop position");
  ac.sendGoal(drop);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its drop pose
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the robot has reached its drop location");
    //display marker at drop position
		marker.request.display = true;
		marker.request.x = drop.target_pose.pose.position.x;
		marker.request.x = drop.target_pose.pose.position.y;
		displayMarkerClient.call(marker);
  else
    ROS_INFO("The robot failed to reach its drop location");
    
  return 0;
}
