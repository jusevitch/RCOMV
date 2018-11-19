
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Eigen>
#include <stdlib.h>
#include <math.h>

// structure of error
struct errorStr
{
  double dis;
  double yaw;
};

// struct of goal location
struct goalStr
{
    double x;
    double y;
    double psi;
};

class PIDController
{
public:
  PIDController();
  ~PIDController();
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
  ros::Timer dis_timer;

  // subscriber
  ros::Subscriber odom_sub;
  ros::Subscriber path_sub;

  // callback funcrions
  void pubCallback(const ros::TimerEvent& event);
  void disCallback(const ros::TimerEvent& event);
  void odom_subCallback(const nav_msgs::Odometry::ConstPtr& msgs);
  void path_subCallback(const geometry_msgs::PoseStamped::ConstPtr& msgs);

  //private variables
  errorStr lst_error; // derivative of errors
  errorStr int_error; // integral of errors
  errorStr error; // current error
  double Kp1, Ki1, Kd1, Kp2, Ki2, Kd2; // PID gains
  double threshold; // error threshold
  double last_pub_time, initial_time;
  goalStr goal;
  double MAX_LIN_V, MAX_ANG_V;
  bool odometry_connected;

  //EigenOdometry odometry; //

}; // end of class


// helper functions
double findDifference(double init_psi, double goal_psi);
double QuaternionToEuler(const nav_msgs::Odometry &msgs);

#endif
