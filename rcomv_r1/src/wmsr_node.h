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
#include <state_graph_builder/graph.h>
#include <state_graph_builder/posegraph.h>

#include <stdlib.h>
#include <math.h>
#include <vector>
#include <string>
#include <algorithm>


// define aliases for msgs types, topic names
//typedef Eigen::Vector3d ref_msgs;
typedef geometry_msgs::PoseStamped ref_msgs;
typedef geometry_msgs::Point tiny_msgs;
typedef geometry_msgs::Pose pose_msgs;
typedef nav_msgs::Odometry state_msgs;
typedef geometry_msgs::Twist twist_msgs;
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

  // ROS subscriber and topic for switch function
  ros::Subscriber switch_sub; // receive switch signal from upper level node
  std_msgs::Bool switch_signal;
  void switch_subCallback(const std_msgs::Bool::ConstPtr& msg);


  // ROS  Publishers and Timers
  ros::Publisher ref_pub;  // publish reference to other neighbor WMSR Nodes
  ros::Publisher output_pub; // publish the goal to the robot
  ros::Publisher new_pub;
  ros::Timer ref_pub_timer, out_pub_timer, new_pub_timer;

  // ROS  Subscribers
  std::vector<ros::Subscriber> ref_subs; // subscribe references from neighbor WMSR nodes

  //ROS Subscriber to build the adjacency matrix
  ros::Subscriber states_sub;

  // messages
  ref_msgs inform_states; // reference center location
  ref_msgs inform_formation_states; // reference formation location
  std::vector<ref_msgs> ref_lists; // reference center location from neighbor agents
  ref_msgs mali_states; // reference location for malicious agents

  //message for states
  std::vector<pose_msgs> state_lists;
  //std::vector<twist_msgs> twist_lists;

  // Callback Functions
  void ref_subCallback(const ref_msgs::ConstPtr& msgs, const int list_idx);
  void ref_pubCallback(const ros::TimerEvent& event);
  void out_pubCallback(const ros::TimerEvent& event);
  void new_pubCallback(const ros::TimerEvent& event);
  void state_subCallback(const state_msgs::ConstPtr& msgs, const int list_idx);
  void graph_subCallback(const state_graph_builder::posegraph::ConstPtr& msgs);

  // vector for odometry in the barrier functions
  std::vector<tiny_msgs> swarm_odom;
  std::vector<tiny_msgs> swarm_tau;
  std::vector<tiny_msgs> prev_odom;
  std::vector<tiny_msgs> prev_tau;
  std::vector<tiny_msgs> yidot;
  tiny_msgs barrier_out;

  // private variables for intermediate calculations
  //int weight_x, weight_y, weight_z;   // weights for the neighbor agents
  int n, k;   // number of agents and number of neighbors in the graph
  int idx;    // the index of the current agent, range from 1 to n
  int role;   // the role of hte current agents: Malicious=1, Normal=2, Leader=3
  std::vector<Matrix> G; // communication graph
  int F;    // allowed maximum number of adversaries
  double x0, y0; // inital pose
  int demo; // 1: x dir 1D motion,  2: y dir 1D motion, 3: 2D motion
  double cx, cy;

  float rc, rp; // communication radius and proximity radius
  float ds, dc; // safety distance for collision avoidance & distance where barrier function starts being applied
  std::vector<std::vector<float>> tau;
  float umax;
  uint iteration=0;

  // Some Helper functions
  // WMSR Algorithm
  ref_msgs WMSRAlgorithm(const std::vector<ref_msgs> &list);
  // Compute the acutal location of the agent by adding its offset to the center location
  void Formation();

  void Calc_Adjacency();
   double calculate_norm(const pose_msgs &state1, const pose_msgs &state2);
  std::vector<int> get_in_neighbours(int rad_type, int agent);
  tiny_msgs calc_vec(const tiny_msgs& state1, const tiny_msgs& state2);
  void populate_state_vector();
  void save_state_vector();
  void filtered_barrier_function(int iteration, int idx);
  void filtered_barrier_collision(int idx);

  float psi_helper(const tiny_msgs &m_agent, const tiny_msgs &n_agent, const tiny_msgs &tau_ij);
  tiny_msgs psi_gradient(int m_agent, int n_agent, const tiny_msgs &tau_ij);
  float psi_col_helper(const tiny_msgs &m_agent, const tiny_msgs  &n_agent); //internally uses ds,dc
  tiny_msgs psi_col_gradient(int m_agent, int n_agent);

  double self_norm(const tiny_msgs &tiny);
  void populate_velocity_vector();
  std::vector<Neigh> multiply_vectors(const std::vector<tiny_msgs> &vec1,const std::vector<tiny_msgs> &vec2, const std::vector<int> neigh);
  NLists velocity_filter(int i);
  void make_tau_vector();
  tiny_msgs add_vectors(const tiny_msgs &a, const tiny_msgs &b);
  tiny_msgs subtract_vectors(const tiny_msgs &a, const tiny_msgs &b);
  tiny_msgs multiply_scalar_vec(const float gain, const tiny_msgs &vec);
  tiny_msgs calc_fvec(int i, int j);

}; // end of class

// Helper functions
double FilterOutlier(std::vector<double> &list, const int k, const double inform_state, const int F);


#endif
