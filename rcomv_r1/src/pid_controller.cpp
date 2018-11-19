

#include "pid_controller.h"

//constructor
PIDController::PIDController()
:nh_private_("~")
{
  // initialize the variables
  last_pub_time = ros::Time::now().toSec();
  initial_time = ros::Time().toSec();
  lst_error.dis = 0; lst_error.yaw = 0;
  int_error.dis = 0; int_error.yaw = 0;
  error.dis = 0; error.yaw = 0;
  odometry_connected = false;

  // Initialize the goal location
  // default location: (1,1)
  nh_private_.param<double>("goal_x", goal.x, 1);
  nh_private_.param<double>("goal_y", goal.y, 1);
  // Initialize the error threshold
  nh_private_.param<double>("threshold", threshold, 0.01);
  // Initialize the maximum absolute velocities
  nh_private_.param<double>("max_linear_velocity", MAX_LIN_V, 2);
  nh_private_.param<double>("max_angular_velocity",MAX_ANG_V, 2);
  // Set control Constant
  nh_private_.param<double>("Kp1", Kp1, 0); nh_private_.param<double>("Kp2", Kp2, 0);
  nh_private_.param<double>("Kd1", Kd1, 0); nh_private_.param<double>("Kd2", Kd2, 0);
  nh_private_.param<double>("Ki1", Ki1, 0); nh_private_.param<double>("Ki2", Ki2, 0);


  // Publisher := cmd_vel_mux/input/teleop
  pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 10);
  // frequency: 50 Hz
  pub_timer = nh.createTimer(ros::Duration(0.02), &PIDController::pubCallback, this);
  // frequency: 1 Hz
  dis_timer = nh.createTimer(ros::Duration(1), &PIDController::disCallback, this);

  //Subscriber := current states
  odom_sub = nh.subscribe("odom", 10, &PIDController::odom_subCallback, this);
  //Subscriber := reference states
  path_sub = nh.subscribe("ref", 10, &PIDController::path_subCallback, this);
}

// destructor
PIDController::~PIDController()
{
  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0;
  pub.publish(cmd_vel);
  ros::shutdown();
}

// odometry subscriber callback function
void PIDController::odom_subCallback(const nav_msgs::Odometry::ConstPtr& msgs)
{
  odometry_connected = true;
  state.header = msgs->header;
  state.child_frame_id = msgs->child_frame_id;
  state.pose = msgs->pose;
  state.twist = msgs->twist;
}

// trajectory subscriber callback function
void PIDController::path_subCallback(const geometry_msgs::PoseStamped::ConstPtr& msgs)
{
  goal.x = msgs->pose.position.x;
  goal.y = msgs->pose.position.y;
}

// callback function to display the odometry reading
void PIDController::disCallback(const ros::TimerEvent& event) {
  // displays the current pose and velocity
  double x = state.pose.pose.position.x;
  double y = state.pose.pose.position.y;
  double yaw = QuaternionToEuler(state);
  double vx = state.twist.twist.linear.x;
  double vy = state.twist.twist.linear.y;
  double dot_yaw = state.twist.twist.angular.z;

  ROS_INFO("-----------------------------------------");
  ROS_INFO("Odom Reading: ");
  ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"Time: "
    <<ros::Time::now().toSec()-initial_time<<"s");
  ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"position at ["<<x<<", "<<y<<"]"<<"|| orientation at "<<yaw);
  ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"goal position at ["<<goal.x<<", "<<goal.y<<"]");
  ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"cmd lin vel: "<<cmd_vel.linear.x<<"|| cmd ang vel: "<<cmd_vel.angular.z);
  ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"velocity is "<<vx<<"|| yaw rate is "<<dot_yaw);
}

// publisher callback function
void PIDController::pubCallback(const ros::TimerEvent& event)
{
  // wait until the odometry topic is received
  if(!odometry_connected)
    return;

  // PID position control algorithm
  double x = state.pose.pose.position.x;
  double y = state.pose.pose.position.y;

  // P term
  double yaw_g, yaw;
  errorStr error;
  // find the current yaw angle from quaternions
  yaw = QuaternionToEuler(state);
  // compute the directin toward the goal postion
  yaw_g = atan2(goal.y - y, goal.x - x);
  yaw_g = fmod(yaw_g + 2*M_PI, 2*M_PI); // transform from [-pi,pi] to [0,2pi]
  // compute the error
  error.dis = sqrt( pow(goal.x - x, 2) + pow(goal.y - y, 2) );
  error.yaw = findDifference(yaw, yaw_g);

  // D term
  double current_pub_time = ros::Time::now().toSec();
  double dt = current_pub_time - last_pub_time;
  errorStr d_error;
  // compute the derivative of the error
  d_error.dis = (error.dis - lst_error.dis) / dt;
  d_error.yaw = (error.yaw - lst_error.yaw) / dt;
  //update
  last_pub_time = current_pub_time;
  lst_error = error;

  // I term
  // compute the integral of the error
  int_error.dis += (error.dis) * dt;
  int_error.yaw += (error.yaw) * dt;

  // compute the output
  cmd_vel.linear.x = Kp1*error.dis + Ki1*int_error.dis + Kd1*d_error.dis;
  cmd_vel.angular.z = Kp2*error.yaw + Ki2*int_error.yaw + Kd2*d_error.yaw;
  // limit the output
  cmd_vel.linear.x = std::max(-MAX_LIN_V, std::min(cmd_vel.linear.x, MAX_LIN_V));
  cmd_vel.angular.z = std::max(-MAX_ANG_V, std::min(cmd_vel.angular.z, MAX_ANG_V));
  // wait for turning
  if (std::abs(error.yaw) > M_PI/3) {
    cmd_vel.linear.x = 0;
  }
  // stop when the distance error is less than threshold
  if (error.dis < threshold) {
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
  }

  //ROS_INFO_STREAM(dt);
  //ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<error.dis<<", "<<int_error.dis<<", "<<d_error.dis);
  //ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<error.yaw<<", "<<int_error.yaw<<", "<<d_error.yaw);
  //ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<Kp1*error.dis+Ki1*int_error.dis+Kd1*d_error.dis);
  //ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<Kp2*error.yaw+Ki2*int_error.yaw+Kd2*d_error.yaw);
  //ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<Kp1*error.dis<<", "<<Ki1*int_error.dis <<", "<<Kd1*d_error.dis);
  //ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<Kp2*error.yaw<<", "<<Ki2*int_error.yaw <<", "<<Kd2*d_error.yaw);

  // publish the output
  pub.publish(cmd_vel);
}



// helper function
// Given two heading angles, compute the smaller angle
// between those two. Note: two angles must have the
// range [0, 2*Pi].
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
// return range: [0, 2*pi]
double QuaternionToEuler(const nav_msgs::Odometry &msgs)
{
  tf::Quaternion q(msgs.pose.pose.orientation.x, msgs.pose.pose.orientation.y,
     msgs.pose.pose.orientation.z, msgs.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  yaw = fmod(yaw + 2*M_PI, 2*M_PI); // transform from [-pi,pi] to [0,2pi]

  return yaw;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "pid_controller_node");

  PIDController pid_controller_node;

  ros::spin();

  return 0;
}
