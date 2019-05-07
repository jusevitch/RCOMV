
#ifndef IO_COLLISION
#define IO_COLLISION

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>

#include <rcomv_r1/CubicPath.h>
#include <rcomv_r1/MSRPA.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <stdlib.h>
#include <math.h>
#include <string>
#include <vector>
#include <regex>

#include <state_graph_builder/graph.h>
#include <state_graph_builder/posegraph.h>
#include <state_graph_builder/posestampedgraph.h>


// pose structure
struct pose {
  double x;
  double y;
  double theta;
};

struct control_cmd{
  double v;
  double w;
  double gamma;
};

struct PoseStamped_Radius{
  geometry_msgs::PoseStamped pose;
  double r_safety; 
};

class IO_control_collision
{
public:
  IO_control_collision();
  ~IO_control_collision();
private:
  // ROS node handle  
  ros::NodeHandle nh;
  ros::NodeHandle nh_private_;

  // msgs
  geometry_msgs::Twist cmd_vel;
  nav_msgs::Odometry state;

  // publisher
  ros::Publisher pub;
  ros::Timer pub_timer;
  ros::Timer dis_timer;  // display timer, for debug
  ros::Timer update_param_timer;

  // subscriber
  ros::Subscriber odom_sub;
  ros::Subscriber trajectory_sub;
  ros::Subscriber states_sub;
  ros::Subscriber msrpa_sub;
  ros::Subscriber obstacle_sub;

  // Callback functions
  void pubCallback(const ros::TimerEvent& event);
  void disCallback(const ros::TimerEvent& event);  // display callback function
  void odom_subCallback(const nav_msgs::Odometry::ConstPtr& msgs);
  void posestamped_subCallback(const geometry_msgs::PoseStamped::ConstPtr& msgs);
  void trajectory_subCallback(const rcomv_r1::CubicPath::ConstPtr& msgs);
  void graph_subCallback(const state_graph_builder::posegraph::ConstPtr& msgs);
  void graph_subCallback_PoseStamped(const state_graph_builder::posestampedgraph::ConstPtr& msgs);
  void obstacle_Callback(const gazebo_msgs::ModelStates::ConstPtr& msgs);
  void change_trajectories(const ros::TimerEvent& event);

  // Callback functions for the MS-RPA algorithm. This callback function updates the trajectory parameters when it receives values from MS-RPA nodes.
  void msrpa_Callback(const rcomv_r1::MSRPA::ConstPtr& msgs);



  std::vector<geometry_msgs::Pose> collision_neighbors(const std::vector<geometry_msgs::Pose> &other_agents, const geometry_msgs::Pose &current_state);
  std::vector<geometry_msgs::Pose> collision_neighbors(const std::vector<geometry_msgs::PoseStamped> &other_agents, const geometry_msgs::PoseStamped &current_state);
  double psi_col_helper(const geometry_msgs::Point &m_agent, const  geometry_msgs::Point &n_agent);
  geometry_msgs::Vector3 psi_col_gradient(const geometry_msgs::Pose &m_agent, const geometry_msgs::Pose &n_agent);
  geometry_msgs::Vector3 psi_col_gradient(const geometry_msgs::PoseStamped &m_agent, const geometry_msgs::Pose &n_agent);
  geometry_msgs::Vector3 calc_vec(const geometry_msgs::Point& state1, const geometry_msgs::Point& state2);
  double self_norm(const geometry_msgs::Vector3 &tiny);
  control_cmd collision_avoid();
  double difference_norm(const geometry_msgs::Pose &v1, const geometry_msgs::Pose &v2);
  double difference_norm(const geometry_msgs::PoseStamped &v1, const geometry_msgs::Pose &v2);

  // private variables


  // List of obstacles
  std::vector<PoseStamped_Radius> obstacles;

  // controller paramters
  double b; // a longitudinal distance ahead of the unicycle model
  double k1, k2, k3; // control gains
  double vmax, wmax; // maximum velocity, and angular velocity

  double Ri, alphai; // offset from the center (in the body fixed frame)

  std::string path_type;
  double t0; // initial time

  // parametric path paramters
  double xc, yc; // center location
  double R; // radius for cirular path
  double R1, R2; // radius for eight_shaped path
  double wd; // reference turning rate
  double phi0; // Initial starting point on circle trajectory w.r.t. the zero angle position in the global frame.

  // Safety parameters
  double ds; // Safety radius; must not be crossed 
  double dc; // Radius where collision avoidance function is activated
  double mu2; // Parameter for collision avoidance barrier function

  // Add square parameters

  // cubic ploynomials path paramters
  pose qi, qf; // initial, final pose
  double poly_k;
  double T;  // total travel time
  bool endless_flag; // true: switch the final and inital locations every T secs
  bool even_cycle; // true: it's in a even cycle, false: it's in a odd cycle


  // Parameters for the trajectory in the queue:

  std::string type_q;
  double t0_q;
  double xc_q;
  double yc_q;
  double R_q;
  double wd_q;
  double phi0_q;

  // Add square q parameters



  bool odometry_connected; // flag of odometry
  double initial_time;
  state_graph_builder::posegraph all_states;
  int n;
  std::vector<geometry_msgs::PoseStamped> state_lists;
  int agent_index;

  int gazebo_msgs; // Must be put as true if you're running Gazebo simulations
  int rover_number; // The number of the rover. This should correspond with the VICON topic the state is published to.
  std::string sub_topic;
  std::string pub_topic;



  // helper functions
  void CubePolyPath(pose qi, pose qf, double k, double T, double t,
                    double &xd, double &yd, double &vd, double &wd);

geometry_msgs::Vector3 add_vectors(const geometry_msgs::Vector3 &a, const geometry_msgs::Vector3 &b);

}; // end of class

// helper function
double QuaternionToYaw(const nav_msgs::Odometry &msgs);
// helper function
double findDifference(double init_psi, double goal_psi);
// helper function




#endif
