
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
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <stdlib.h>
#include <math.h>
#include <vector>
#include <string>
#include <algorithm>


// define aliases for msgs types, topic names
//typedef Eigen::Vector3d ref_msgs;
typedef geometry_msgs::PoseStamped ref_msgs;
typedef geometry_msgs::Point tiny_msgs;
typedef nav_msgs::Odometry state_msgs;
typedef geometry_msgs::Twist twist_msgs;
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


  // ROS  Publishers and Timers
  ros::Publisher ref_pub;  // publish reference to other neighbor WMSR Nodes
  ros::Publisher output_pub; // publish the goal to the robot
  ros::Timer ref_pub_timer, out_pub_timer;

  // ROS  Subscribers
  std::vector<ros::Subscriber> ref_subs; // subscribe references from neighbor WMSR nodes

  //ROS Subscriber to build the adjacency matrix
  std::vector<ros::Subscriber> states_subs;

  // messages
  ref_msgs inform_states; // reference center location
  ref_msgs inform_formation_states; // reference formation location
  std::vector<ref_msgs> ref_lists; // reference center location from neighbor agents
  ref_msgs mali_states; // reference location for malicious agents

  //message for states
  std::vector<state_msgs> state_lists;
  //std::vector<twist_msgs> twist_lists;
  std::vector<int> role_list;

  // Callback Functions
  void ref_subCallback(const ref_msgs::ConstPtr& msgs, const int list_idx);
  void ref_pubCallback(const ros::TimerEvent& event);
  void out_pubCallback(const ros::TimerEvent& event);
  void state_subCallback(const state_msgs::ConstPtr& msgs, const int list_idx);

  // vector for odometry in the barrier functions
  std::vector<tiny_msgs> swarm_odom;
  std::vector<tiny_msgs> swarm_tau;
  std::vector<tiny_msgs> prev_odom;
  std::vector<tiny_msgs> prev_tau;
  std::vector<tiny_msgs> barrier_out;

  // private variables for intermediate calculations
  //int weight_x, weight_y, weight_z;   // weights for the neighbor agents
  int n, k;   // number of agents and number of neighbors in the graph
  int idx;    // the index of the current agent, range from 1 to n
  int role;   // the role of hte current agents: Malicious=1, Normal=2, Leader=3
  std::vector<std::vector<int>> L; // communication graph
  int F;    // allowed maximum number of adversaries
  double x0, y0; // inital pose
  int demo; // 1: x dir 1D motion,  2: y dir 1D motion, 3: 2D motion
  double cx, cy;

  float rc, rp; // communication radius and proximity radius
  float ds, dc; // safety distance for collision avoidance & distance where barrier function starts being applied
  std::vector<std::vector<float>> tau;
  float umax;

  // Some Helper functions
  // WMSR Algorithm
  ref_msgs WMSRAlgorithm(const std::vector<ref_msgs> &list);
  // Compute the acutal location of the agent by adding its offset to the center location
  void Formation();

}; // end of class

// Helper functions
double FilterOutlier(std::vector<double> &list, const int k, const double inform_state, const int F);


#endif
