
#ifndef INPUTOUTPUT_CONTROLLER_H
#define INPUTOUTPUT_CONTROLLER_H

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include <rcomv_r1/CubicPath.h>
#include <rcomv_r1/MSRPA.h>

#include <tf/transform_broadcaster.h>

#include <stdlib.h>
#include <math.h>
#include <string>


// pose structure
struct pose {
  double x;
  double y;
  double theta;
};

class InOutLinController
{
public:
  InOutLinController();
  ~InOutLinController();
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
  ros::Subscriber indoors_sub;
  ros::Subscriber msrpa_sub;


  // callback funcrions
  void pubCallback(const ros::TimerEvent& event);
  void disCallback(const ros::TimerEvent& event);  // display callback function
  void odom_subCallback(const nav_msgs::Odometry::ConstPtr& msgs);
  void indoor_subCallback(const geometry_msgs::PoseStamped::ConstPtr& msgs);
  void trajectory_subCallback(const rcomv_r1::CubicPath::ConstPtr& msgs);
  void msrpa_subCallback(const rcomv_r1::MSRPA& msgs);

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
  double phi0; // angle for trajectory
  double L, psi, V, startLIdx; // parameters for square path

  // cubic ploynomials path paramters
  pose qi, qf; // initial, final pose
  double poly_k;
  double T;  // total travel time
  bool endless_flag; // true: switch the final and inital locations every T secs
  bool even_cycle; // true: it's in a even cycle, false: it's in a odd cycle

  bool odometry_connected; // flag of odometry
  double initial_time;

  bool indoors_rover_bool; // Must be put as true if you're running an AION rover indoors
  int rover_number; // The number of the rover. This should correspond with the VICON topic the state is published to.
  std::string indoor_sub_topic; // Indoors subscriber topic
  std::string indoor_pub_topic; // Indoors publisher topic


  // helper functions
  void CubePolyPath(pose qi, pose qf, double k, double T, double t,
                    double &xd, double &yd, double &vd, double &wd);

}; // end of class

// helper function
double QuaternionToYaw(const nav_msgs::Odometry &msgs);
// helper function
double findDifference(double init_psi, double goal_psi);


#endif
