
#ifndef WMSR_NODE_H
#define WMSR_NODE_H

#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <stdlib.h>
#include <math.h>
#include <vector>
#include <string>
#include <algorithm>


// define aliases for msgs types, topic names
//typedef Eigen::Vector3d ref_msgs;
typedef geometry_msgs::PoseStamped ref_msgs;

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

  // ROS pub sub and topics for switch function
  ros::Subscriber switch_sub; // receive switch signal from upper level node
  std_msgs::Bool switch_signal;
  void switch_subCallback(const std_msgs::Bool::ConstPtr& msg);


  // ROS  Publishers and Timers
  ros::Publisher ref_pub;  // publish reference to other neighbor WMSR Nodes
  ros::Publisher output_pub; // publish the goal to the robot
  ros::Timer ref_pub_timer, out_pub_timer;

  // ROS  Subscribers
  std::vector<ros::Subscriber> ref_subs; // subscribe references from neighbor WMSR nodes

  // messages
  ref_msgs inform_states; // reference center location
  ref_msgs inform_formation_states; // reference formation location
  std::vector<ref_msgs> ref_lists; // reference center location from neighbor agents
  ref_msgs mali_states; // reference location for malicious agents

  // Callback Functions
  void ref_subCallback(const ref_msgs::ConstPtr& msgs, const int list_idx);
  void ref_pubCallback(const ros::TimerEvent& event);
  void out_pubCallback(const ros::TimerEvent& event);

  // private variables for intermediate calculations
  //int weight_x, weight_y, weight_z;   // weights for the neighbor agents
  int n, k;   // number of agents and number of neighbors in the graph
  int idx;    // the index of the current agent, range from 1 to n
  int role;   // the role of hte current agents: Malicious=1, Normal=2, Leader=3
  std::vector<std::vector<int>> L; // comunication graph
  int F;    // allowed maximum number of adversaries
  double x0, y0; // inital pose
  int demo; // 1: x dir 1D motion,  2: y dir 1D motion, 3: 2D motion
  double cx, cy;

  // Some Helper functions
  ref_msgs WMSRAlgorithm(const std::vector<ref_msgs> &list);
  void Formation();

}; // end of class

// Helper functions
double FilterOutlier(std::vector<double> &list, const int k, const double inform_state, const int F);


#endif
