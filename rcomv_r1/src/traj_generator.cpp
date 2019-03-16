
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <stdlib.h>

void topicCallback(const geometry_msgs::Twist::ConstPtr& msg, const int list_idx)
{
  twist_list[list_idx].pose=msgs->pose;
  twist_list[list_idx].orientation=msgs->orientation;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
  std::vector<geometry_msgs::Twist> twist_list;

  ros::Subscriber sub = n.subscribe("chatter", 1000, topicCallback);
  ros::spin();

  return 0;
}
