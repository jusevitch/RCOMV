
#ifndef IO_COLLISION
#define IO_COLLISION

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>

#include <rcomv_r1/CubicPath.h>

#include <tf/transform_broadcaster.h>

#include <stdlib.h>
#include <math.h>
#include <string>
#include <vector>

#include <state_graph_builder/graph.h>
#include <state_graph_builder/posegraph.h>


// pose structure
struct pose {
  double x;
  double y;
  double theta;
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

  // subscriber
  ros::Subscriber odom_sub;
  ros::Subscriber trajectory_sub;
  ros::Subscriber states_sub;

  // callback funcrions
  void pubCallback(const ros::TimerEvent& event);
  void disCallback(const ros::TimerEvent& event);  // display callback function
  void odom_subCallback(const nav_msgs::Odometry::ConstPtr& msgs);
  void trajectory_subCallback(const rcomv_r1::CubicPath::ConstPtr& msgs);
  void graph_subCallback(const state_graph_builder::posegraph::ConstPtr& msgs);

  geometry_msgs::Vector3 collision_neighbors(const state_graph_builder::posegraph::ConstPtr& graph);
  double psi_col_helper(const geometry_msgs::Vector3 &m_agent, const  geometry_msgs::Vector3 &n_agent);
  geometry_msgs::Vector3 psi_col_gradient(int m_agent, int n_agent);
  geometry_msgs::Vector3 calc_vec(const geometry_msgs::Point& state1, const geometry_msgs::Point& state2);
  double IO_control_collision::self_norm(const geometry_msgs::Vector3 &tiny);
  
  // private variables
  // controller paramters
  double b; // a longitudinal distance ahead of the unicycle model
  double k1, k2; // control gains
  double vmax, wmax; // maximum velocity, and angular velocity

  double Ri, alphai; // offset from the center (in the body fixed frame)

  std::string path_type;
  double t0; // initial time

  // parametric path paramters
  double xc, yc; // center location
  double R; // radius for cirular path
  double R1, R2; // radius for eight_shaped path
  double wd; // reference turning rate

  // cubic ploynomials path paramters
  pose qi, qf; // initial, final pose
  double poly_k;
  double T;  // total travel time
  bool endless_flag; // true: switch the final and inital locations every T secs
  bool even_cycle; // true: it's in a even cycle, false: it's in a odd cycle

  bool odometry_connected; // flag of odometry
  double initial_time;
  state_graph_builder::posegraph all_states;
  int n;
  std::vector<pose_msgs> state_lists;
  int agent_index;




  // helper functions
  void CubePolyPath(pose qi, pose qf, double k, double T, double t,
                    double &xd, double &yd, double &vd, double &wd);


}; // end of class

// helper function
double QuaternionToYaw(const nav_msgs::Odometry &msgs);
// helper function
double findDifference(double init_psi, double goal_psi);


#endif
