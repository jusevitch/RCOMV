
#ifndef PATH_WMSR_NODE_H
#define PATH_WMSR_NODE_H

#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include <rcomv_r1/CubicPath.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>

#include <stdlib.h>
#include <math.h>
#include <vector>
#include <string>
#include <algorithm>

// define aliases for msgs types
typedef rcomv_r1::CubicPath path_msgs;

// define the WMSR Node class
class WMSRNode
{
public:
  WMSRNode();
  ~WMSRNode();

private:
  // ROS NodeHandle
  ros::NodeHandle nh;
  ros::NodeHandle nh_private_;

  // ROS subscriber and topic for switch function
  ros::Subscriber switch_sub; // receive switch signal from upper level node
  std_msgs::Bool switch_signal;
  void switch_subCallback(const std_msgs::Bool::ConstPtr& msg);

  // ROS Publisher and Timer
  ros::Publisher ref_pub;  // publish the reference path to other neighbor WMSR Nodes
  ros::Publisher output_pub; // publish the reference path to the controller
  ros::Timer ref_pub_timer, out_pub_timer;

  // ROS Subscribers
  std::vector<ros::Subscriber> ref_subs; // subscribe references from neighbor WMSR nodes

  // messages
  path_msgs inform_center_path;  // reference path of the formation center
  std::vector<path_msgs> ref_lists; // reference center location received from neighbor agents
  path_msgs mali_path; // malicous path (for cyber attack, physical attack, or both)

  // Callback Functions
  void ref_subCallback(const path_msgs::ConstPtr& msgs, const int list_idx);
  void ref_pubCallback(const ros::TimerEvent& event);
  void out_pubCallback(const ros::TimerEvent& event);

  // private variables for intermediate step calculations
  //int weight_x, weight_y, weight_z;   // weights for the neighbor agents
  int n, k;   // number of agents and number of neighbors in the graph
  int idx;    // the index of the current agent, range from 1 to n
  int role;   // the role of hte current agents: Malicious=1, Normal=2, Leader=3
  int attack; // type of attack: cyber attack=1, physical attack=2
  //std::vector<std::vector<int>> L; // comunication graph
  int F;    // allowed maximum number of adversaries
  double x0, y0, theta0; // inital pose
  int attack_type; // 1: cyber attack, 2: physical attack

  // Some Helper functions
  // WMSR Algorithm: output the filtered reference path using WMSR algorithm
  path_msgs WMSRAlgorithm(const std::vector<path_msgs> &list);


}; // end of class

// Helper functions
double FilterOutlier(std::vector<double> &list, const int k, const double inform_center_path, const int F);

#endif
