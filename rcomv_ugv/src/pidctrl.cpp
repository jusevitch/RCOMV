
#include "pidctrl.h"

// constructor
SubscribeAndPublish::SubscribeAndPublish()
  : nh_private_("~")
  {
    // Publisher := husky_velocity_controller/cmd_vel
    output_pub = nh.advertise<geometry_msgs::Twist>(
          "husky_velocity_controller/cmd_vel", 1000);
    // Set a timer for the Output Publisher callback function
    // frequency: 50 Hz
    pub_timer = nh.createTimer(ros::Duration(0.1),
                &SubscribeAndPublish::pubCallback, this);

    // Subscriber := husky_velocity_controller/odom
    input_sub = nh.subscribe("husky_velocity_controller/odom", 1000,
               &SubscribeAndPublish::subCallback, this);

    // Initialize the variables used for computing derivative term
    last_sub_time = ros::Time::now().toSec();
    initial_time = ros::Time::now().toSec();
    last_err.dis = 0; last_err.psi = 0;
    err_integral.dis = 0; err_integral.psi = 0;

    // Initialize the goal location
    nh_private_.param<double>("goal_x", goal.x, 1);
    nh_private_.param<double>("goal_y", goal.y, 1);


    // Initialize the error threshold
    nh_private_.param<double>("threshold", err_threshold, 0.05);
    // Set control Constant
    nh_private_.param<double>("Kp1", Kp1, 0); nh_private_.param<double>("Kp2", Kp2, 0);
    nh_private_.param<double>("Kd1", Kd1, 0); nh_private_.param<double>("Kd2", Kd2, 0);
    nh_private_.param<double>("Ki1", Ki1, 0); nh_private_.param<double>("Ki2", Ki2, 0);
  }

// Destructor
SubscribeAndPublish::~SubscribeAndPublish()
  {
      output.linear.x = 0;
      output.angular.z = 0;
      output_pub.publish(output);
      ros::shutdown();
  }

// subscriber callback function
void SubscribeAndPublish::subCallback(const nav_msgs::Odometry::ConstPtr& msgs)
{
       // assign the callback input to private class variable "input"
       /*input.pose.pose.position.x = msgs->pose.pose.position.x;
       input.pose.pose.position.y = msgs->pose.pose.position.y;
       input.pose.pose.orientation.x = msgs->pose.pose.orientation.x;
       input.pose.pose.orientation.y = msgs->pose.pose.orientation.y;
       input.pose.pose.orientation.z = msgs->pose.pose.orientation.z;
       input.pose.pose.orientation.w = msgs->pose.pose.orientation.w;
       */
       input.pose.pose = msgs->pose.pose;
}


// Publisher callback function
void SubscribeAndPublish::pubCallback(const ros::TimerEvent& event)
{
  //PID control algorithm
  // ------------------------ P term ---------------------------
    error err;
    double psi_g, psi_cur;

    // find the current yaw angle from quaternions
    psi_cur = QuaternionToEuler(input);
    // compute the direction toward the goal position
    psi_g   = atan2(goal.y - input.pose.pose.position.y, goal.x - input.pose.pose.position.x);
    psi_g   = fmod(psi_g + 2*M_PI, 2*M_PI); // transform from [-pi,pi] to [0,2pi]

    err.dis = sqrt(pow(goal.x - input.pose.pose.position.x, 2) +
                   pow(goal.y - input.pose.pose.position.y, 2));
    err.psi = findDifference(psi_cur, psi_g);

    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //err.dis = err.dis * cos(err.psi);

    // ------------------------ D term --------------------------
    double current_sub_time = ros::Time::now().toSec();
    double dt = current_sub_time - last_sub_time;
    error err_derivative;
    // compute the derivative of the error
    err_derivative.dis = (err.dis - last_err.dis) / dt;
    err_derivative.psi = (err.psi - last_err.psi) / dt;
    // update
    last_sub_time = current_sub_time;
    last_err = err;

    // ---------------------- I term ----------------------
    // compute the integral of the error
    err_integral.dis += err.dis * dt;
    err_integral.psi += err.psi * dt;

    // compute output
    output.linear.x  = Kp1 * err.dis +
                      Kd1 * err_derivative.dis;
    output.angular.z = Kp2 * err.psi +
                      Kd2 * err_derivative.psi;
    //ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"P: "<<err.psi<<"I: "<<err_integral.psi<<"D: "<<err_derivative.psi<<"and dt is: "<<dt);

    // publish the output
    output_pub.publish(output);


    // display robot info
    ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"Time: "
      <<current_sub_time-initial_time<<"s");
    ROS_INFO_STREAM(std::setprecision(2)<<std::fixed
      << "GOAL: x: " << goal.x<< " | y: " << goal.y);
    ROS_INFO_STREAM(std::setprecision(2)<<std::fixed
      << "Pose: x: " << input.pose.pose.position.x<< " | y: " << input.pose.pose.position.y<< " | dir: "<<psi_cur);
    ROS_INFO_STREAM(std::setprecision(2)<<std::fixed
      << "cmd_vel_lin_x: " << output.linear.x<< " | cmd_vel_ang_z: " << output.angular.z);
      ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"heading error"<<err.psi);

    ROS_INFO_STREAM("-------------------------------------------------");

}

// helper function
// Given two heading angles, compute the smaller angle
// between those two. Note: two angles must have the
// range : [0, 2*Pi].
double findDifference(double init_psi, double goal_psi)
{
  double err  = goal_psi - init_psi;
  int sign = (std::signbit(err) == 0) ? 1 : -1;
  err  = (fabs(err) <= M_PI) ? err :
                  sign * fmod(fabs(err) - 2*M_PI, M_PI);
  return err;
}


// helper function
// giving quaternion readings from Odometry, convert them to euler angles
double QuaternionToEuler(const nav_msgs::Odometry &msgs)
{
  tf::Quaternion q(msgs.pose.pose.orientation.x, msgs.pose.pose.orientation.y,
     msgs.pose.pose.orientation.z, msgs.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}
