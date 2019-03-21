
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
#include <nav_msgs/Odometry.h>


#include <stdlib.h>
#include <math.h>
#include <vector>
#include <string>
#include <algorithm>
#include <iterator>

// define aliases for msgs types, topic names
//typedef Eigen::Vector3d ref_msgs;
typedef geometry_msgs::PointStamped ref_msgs;
typedef geometry_msgs::Point tiny_msgs;
typedef std::vector<std::vector<int>> Matrix;
struct Neigh{
  double val;
  int id;
};
struct NLists{
  std::vector<int> f_neigh;
  std::vector<int> u_neigh;
};
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

  //ROS Subscriber to odometry to build the adjacency matrix
  std::vector<ros::Subscriber> states_subs;

  // messages for ref
  ref_msgs inform_states; // reference center location
  ref_msgs inform_formation_states; // reference formation location
  std::vector<ref_msgs> ref_lists; // reference center location from neighbor agents
  ref_msgs mali_states; // reference location for malicious agents

  //message for state
  std::vector<ref_msgs> state_lists;
  std::vector<int> role_list;

  // Callback Functions
  void ref_subCallback(const ref_msgs::ConstPtr& msgs, const int list_idx);
  void ref_pubCallback(const ros::TimerEvent& event);
  void out_pubCallback(const ros::TimerEvent& event);
  void state_subCallback(const ref_msgs::ConstPtr& msgs, const int list_idx);
  void new_pubCallback(const ros::TimerEvent& event);

  // vector for odometry

  std::vector<tiny_msgs> swarm_odom;
  std::vector<tiny_msgs> swarm_tau;
  std::vector<tiny_msgs> prev_odom;
  std::vector<tiny_msgs> prev_tau;
  tiny_msgs barrier_out;

  // private variables for intermediate calculations
  //int weight_x, weight_y, weight_z;   // weights for the neighbor agents
  int n, k;   // number of agents and number of neighbors in the graph
  int idx;    // the index of the current agent, range from 1 to n
  int role;   // the role of hte current agents: Malicious=1, Normal=2, Leader=3
  std::vector<Matrix> G; // comunication graph use for adjacency matrix
  int F;    // allowed maximum number of adversaries
  double x0, y0, z0; // inital pose
  int demo; // 1: x dir 1D motion,  2: z dir 1D motion, 3: 3D motion
  double cx, cy, cz;
  float rc, rp; // communication radius and proximity radius
  float ds, dc; // safety distance for collision avoidance & distance where barrier function starts being applied
  std::vector<std::vector<float>> tau;
  float umax;

  // Some Helper functions
  ref_msgs WMSRAlgorithm(const std::vector<ref_msgs> &list);
  void Formation();

  void Calc_Adjacency();
  double calculate_norm(const ref_msgs &state1, const ref_msgs &state2);
  std::vector<int> get_in_neighbours(const Matrix &Q, int agent);
  tiny_msgs calc_vec(const tiny_msgs &state1,const tiny_msgs &state2);
  void populate_state_vector();
  void save_state_vector();
  void filtered_barrier_function(int iteration, int idx);
  void filtered_barrier_collision(int iteration, int idx);

  float psi_helper(const tiny_msgs &m_agent, const tiny_msgs &n_agent, const tiny_msgs &tau_ij);
  tiny_msgs psi_gradient(int m_agent, int n_agent, const tiny_msgs &tau_ij);
  float psi_col_helper(const tiny_msgs &m_agent, const tiny_msgs  &n_agent); //internally uses ds,dc
  tiny_msgs psi_col_gradient(int m_agent, int n_agent);

  double self_norm(const tiny_msgs &tiny);
  void populate_velocity_vector(std::vector<tiny_msgs> &yidot);
  std::vector<Neigh> multiply_vectors(const std::vector<tiny_msgs> &vec1,const std::vector<tiny_msgs> &vec2, const std::vector<int> neigh);
  NLists velocity_filter(int i, const std::vector<tiny_msgs> &yidot);
  void make_tau_vector();
  tiny_msgs add_vectors(tiny_msgs &a, tiny_msgs &b);
  tiny_msgs subtract_vectors(tiny_msgs &a, tiny_msgs &b);
  tiny_msgs multiply_scalar_vec(const float gain, const tiny_msgs &vec);
  tiny_msgs calc_fvec(const std::vector<float> &state1, const std::vector<float> &state2);
}; // end of class

// Helper functions
double FilterOutlier(std::vector<double> &list, const int k, const double inform_state, const int F);


#endif
