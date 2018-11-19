#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

#include <stdlib.h>
#include <math.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

move_base_msgs::MoveBaseFeedback feedback_data;

void generatedPath(std::vector<move_base_msgs::MoveBaseGoal> &goal, std::vector<double> path, int n, int init_x, int init_y) {
  double cx, cy, radius, theta;

  cx = path[0];
  cy = path[1];
  radius = path[2];

  theta = 0;
  for (int i=0; i<n; i++) {
    goal[i].target_pose.header.frame_id = "odom";
    goal[i].target_pose.pose.position.x = cx + radius * cos(theta) - init_x;
    goal[i].target_pose.pose.position.y = cy + radius * sin(theta) - init_y;
    goal[i].target_pose.pose.orientation =
    // update the orientation angle
    tf::createQuaternionMsgFromRollPitchYaw(0, 0, (theta+M_PI/2));
    theta += (M_PI*2) / n;
  }
}

bool arrivalCheck(const move_base_msgs::MoveBaseGoal goal, const move_base_msgs::MoveBaseFeedback curr_pose) {
  double diff_x = goal.target_pose.pose.position.x - curr_pose.base_position.pose.position.x;
  double diff_y = goal.target_pose.pose.position.y - curr_pose.base_position.pose.position.y;
  double distance = sqrt(pow(diff_x,2) + pow(diff_y,2));
  double tolerance = 0.5;

  if (distance < tolerance)
    return true;
  else
    return false;
}

void feedbackCallback(const move_base_msgs::MoveBaseFeedback::ConstPtr& msgs) {
  feedback_data.base_position = msgs->base_position;
  ROS_INFO("receive feedback data...");
}

int main(int argc, char** argv){
  ros::init(argc, argv, "circular_path_goals");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private_("~");

  std::vector<double> path;
  int init_x, init_y;
  int n;

  // read path data from launch file, default is [0,0,0]
  nh_private_.param("path", path, std::vector<double>(3,0));
  // read initial pose data from launch file, default is [0,0]
  nh_private_.param("x", init_x, 0);
  nh_private_.param("y", init_y, 0);

  // generate goal locations for the path
  nh_private_.param("number_segements", n, 4);
  std::vector<move_base_msgs::MoveBaseGoal> goal(n);
  generatedPath(goal, path, n, init_x, init_y);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // initialize the move_base feedback Subscriber
  //ros::Subscriber sub_mbFeedback = nh.subscribe("/move_base/feedback", 1000, feedbackCallback);
  //ros::spin();

  // while the action server is connected
  int count = 0;
  while (ac.isServerConnected () && ros::ok()) {

    goal[count].target_pose.header.stamp = ros::Time::now();

    // publish the waypoint
    ROS_INFO("Publishing waypoint[%d] : [%f, %f]", count, goal[count].target_pose.pose.position.x,
    goal[count].target_pose.pose.position.y);
    ROS_INFO("with orientation[%d] : [%f, %f]", count, goal[count].target_pose.pose.orientation.w,
    goal[count].target_pose.pose.orientation.z);

    ac.sendGoal(goal[count]);

    // wait until it reaches the goal
    ac.waitForResult();

    //update
    count = (count + 1) % n;
  }

  return 0;
}
