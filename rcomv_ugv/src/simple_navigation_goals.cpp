#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private_("~");

  // initialize the parameters
  //double x, y, z, w, nx, ny, nz;
  std::vector<double> position;  // [x, y, z]
  std::vector<double> euler;    // [roll, pitch, yaw]

  nh_private_.param("position", position, std::vector<double>());
  nh_private_.param("euler", euler, std::vector<double>());

  ROS_INFO("read position: [%f, %f, %f]", position[0], position[1], position[2]);
  ROS_INFO("read orientation: [%f, %f, %f]", euler[0], euler[1], euler[2]);

  // transform euler angles to quaternions
  // quaternion type: [nx,ny,nz,w]
  tf::Quaternion qt;
  qt = tf::createQuaternionFromRPY(euler[0], euler[1], euler[2]);
  ROS_INFO("transformed qt: [%f, %f, %f, %f]", qt[0], qt[1], qt[2], qt[3]);
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }


  move_base_msgs::MoveBaseGoal goal;


  //we'll send a goal to the robot to move to a desired location
  goal.target_pose.header.frame_id = "odom";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = position[0];
  goal.target_pose.pose.position.y = position[1];
  goal.target_pose.pose.orientation.w = qt[3];
  goal.target_pose.pose.orientation.z = qt[2];


  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  actionlib::SimpleClientGoalState state = ac.getState();
  ROS_INFO("Action [x=%f,y=%f]: %s", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, state.toString().c_str());
  ac.waitForResult();
  ROS_INFO("goal finished");

  // print client state after finishing
  state = ac.getState();
  ROS_INFO("Action: %s",state.toString().c_str());

  // send a second goal back to origin
  goal.target_pose.header.frame_id = "odom";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 0;
  goal.target_pose.pose.position.y = 0;
  goal.target_pose.pose.orientation.w = qt[3];
  goal.target_pose.pose.orientation.z = qt[2];

  ROS_INFO("Sending second goal");
  ac.sendGoal(goal);
  state = ac.getState();
  ROS_INFO("Action [x=%f,y=%f]: %s", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, state.toString().c_str());
  ac.waitForResult();
  ROS_INFO("second goal finished");

  //
  state = ac.getState();
  ROS_INFO("Action: %s",state.toString().c_str());

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
}
