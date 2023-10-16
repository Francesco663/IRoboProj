#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#define PI 3.1416

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool createGoal(MoveBaseClient& ac, float goal_x, float goal_y) {
  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = goal_x;
  goal.target_pose.pose.position.y = goal_y;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Goal reached successfully!");
      return true;
  }
  else {
      ROS_INFO("The base failed to reach the goal.");
      return false;
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  if (!createGoal(ac, 1.0, 0.0)) return 1;
  if (!createGoal(ac, 0.0, 1.0)) return 1;
  if (!createGoal(ac, 0.0, 1.5)) return 1;
  if (!createGoal(ac, 1.2, -1.0)) return 1;
  if (!createGoal(ac, -2.2, -1.5)) return 1;

  return 0;
}