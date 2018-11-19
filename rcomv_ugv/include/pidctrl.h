
#ifndef PIDCTRL_H
#define PIDCTRL_H


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <stdlib.h>
#include <math.h>

// struct of goal location
struct goalstr
{
    double x;
    double y;
    double psi;
};
// struct of error
struct error
{
   double dis;
   double psi;
};


class SubscribeAndPublish
{
public:
  SubscribeAndPublish();
  ~SubscribeAndPublish();

private:
  // ROS NodeHandle
  ros::NodeHandle nh;
  ros::NodeHandle nh_private_;
  // ROS Topic Publishers and Timers
  ros::Publisher output_pub;
  ros::Timer pub_timer;
  // ROS Topic Subscribers
  ros::Subscriber input_sub;

  // messages to publish or subscribe on topics
  nav_msgs::Odometry input;
  geometry_msgs::Twist output;

  // Callback Functions
  void subCallback(const nav_msgs::Odometry::ConstPtr& msgs);
  void pubCallback(const ros::TimerEvent& event);

  // private variables for the PID algorithm
  error last_err;
  error err_integral;
  goalstr goal;
  double initial_time;
  double last_sub_time;
  // PID constants
  double Kp1, Kp2;
  double Ki1, Ki2;
  double Kd1, Kd2;
  // I term tolerance threshold
  double err_threshold;
}; // end of class


// helper functions
double findDifference (double init_psi, double goal_psi);
double QuaternionToEuler(const nav_msgs::Odometry &msgs);


#endif
