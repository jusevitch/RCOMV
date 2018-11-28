
#ifndef INPUTOUTPUT_CONTROLLER_H
#define INPUTOUTPUT_CONTROLLER_H

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Eigen>
#include <stdlib.h>
#include <math.h>
#include <string>


// path parameters structure


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

  // callback funcrions
  void pubCallback(const ros::TimerEvent& event);
  void disCallback(const ros::TimerEvent& event);  // display callback function
  void odom_subCallback(const nav_msgs::Odometry::ConstPtr& msgs);
  void trajectory_subCallback(); // to be determined)

  // private variables
  double b; // a longitudinal distance ahead of the unicycle model
  double k1, k2; // control gains
  double vmax, wmax; // maximum velocity, and angular velocity
  std::string path_type;
  double t0; // initial time
  double xc, yc; // center location
  double R; // radius for cirular path
  double R1, R2; // radius for eight_shaped path
  double wd; // reference turning rate
  bool odometry_connected; // flag of odometry
  double initial_time;




  // helper functions
  //void controller(double &v, double &w, string path_type);

}; // end of class

// helper function
double QuaternionToYaw(const nav_msgs::Odometry &msgs);

#endif
