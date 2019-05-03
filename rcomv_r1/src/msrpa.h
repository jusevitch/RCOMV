#ifndef MSRPA_NODE_H
#define MSRPA_NODE_H

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
//#include <state_graph_builder/graph.h>/
//#include <state_graph_builder/posegraph.h>

#include <rcomv_r1/MSRPA.h>

#include <stdlib.h>
#include <math.h>
#include <vector>
#include <string>
#include <algorithm>

// define aliases for msgs types, topic names
//typedef Eigen::Vector3d ref_msgs;
typedef rcomv_r1::MSRPA ref_msgs;
typedef geometry_msgs::Point tiny_msgs;
typedef geometry_msgs::Pose pose_msgs;
typedef geometry_msgs::PoseStamped sref_msgs;
typedef nav_msgs::Odometry state_msgs;
typedef geometry_msgs::Twist twist_msgs;
typedef std::vector<std::vector<int>> Matrix;
typedef std::vector<std::vector<double>> FMatrix;


// define the MSRPA Node class
class MSRPA
{
public:
  MSRPA();
  ~MSRPA();

private:
  // ROS NodeHandle
  ros::NodeHandle nh;
  ros::NodeHandle nh_private_;

  // ROS subscriber and topic for switch function
  ros::Subscriber switch_sub; // receive switch signal from upper level node
  std_msgs::Bool switch_signal;
  void switch_subCallback(const std_msgs::Bool::ConstPtr& msg);

  ros::Subscriber leader_cmd_sub;
  void leader_subCallback(const ref_msgs::ConstPtr& msgs);

  // ROS  Publishers and Timers
  ros::Publisher ref_pub;  // publish reference to other neighbor MSRPA Nodes
  ros::Publisher out_pub; // publish the reference to I/O controller
  ros::Timer ref_pub_timer, out_pub_timer;

  // ROS  Subscribers
  std::vector<ros::Subscriber> ref_subs; // subscribe references from neighbor MSRPA nodes

  rcomv_r1::MSRPA ref_msg;

  // messages
  ref_msgs inform_states; // reference center location
  //message for states
  
  // Callback Functions
  void ref_subCallback(const ref_msgs::ConstPtr& msgs, const int list_idx);
  void ref_pubCallback(const ros::TimerEvent& event);
  void out_pubCallback(const ros::TimerEvent& event);
  
  
  // private variables for intermediate calculations
  //int weight_x, weight_y, weight_z;   // weights for the neighbor agents
  int n, k;   // number of agents and number of neighbors in the graph
  int idx;    // the index of the current agent, range from 1 to n
  int role;   // the role of hte current agents: Malicious=1, Normal=2, Leader=3
  Matrix L; //Laplacian
  Matrix Anan; //helper for calculating B
  int F;    // allowed maximum number of adversaries

  double Rf, t0, xc, yc, Rad, wd, phi0, Leng, psi, v, start_L;  // parameters to define the trajectory

  uint iteration=0;

  void Calc_Laplacian();
  
  std::string common_namespace;
  std::string trajectory_type;
  std::vector<ref_msgs> cvec;
  void Consensus(int i);
  std::vector<FMatrix> BFunc();
  std::vector<FMatrix> B;
  void initialize_cvec();
  int eta;
  sref_msgs get_leader_reference(uint t);
  ref_msgs get_malicious_reference();
  ref_msgs control;
  ref_msgs reset_message;
  sref_msgs update_reference(const sref_msgs reference, const sref_msgs control);
  sref_msgs castToPoseAndSubtract(const tiny_msgs point, const sref_msgs pose);
  ref_msgs NANMSG;
  std::vector<double> multiply_scalar_vec(const double val, const std::vector<double> vec);

}; // end of class


#endif
