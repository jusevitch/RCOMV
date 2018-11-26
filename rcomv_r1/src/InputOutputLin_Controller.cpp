
#include "InputOutputLin_Controller.h"

// constructor
InOutLinController::InOutLinController()
:nh_private_("~")
{
  // Initialize the parameters (constant, from the launch file)
  // syntax: nh_private_.param<type>
  // ("parameter name in launch file", variable to be assigned, default value);
  nh_private_.param<double>("b", b, 0.05);
  nh_private_.param<double>("k1", k1, 4);
  nh_private_.param<double>("k2", k2, 4);
  nh_private_.param<double>("vmax", vmax, 6);
  nh_private_.param<double>("wmax", wmax, 4);

  // Initialize the paramters (varying, will be updated by subscribers)
  nh_private_.param<std::string>("path_type", path_type, "circular");
  nh_private_.param<double>("t0", t0, 0);
  nh_private_.param<double>("xc", xc, 0);
  nh_private_.param<double>("yc", yc, 0);
  nh_private_.param<double>("R", R, 4);
  nh_private_.param<double>("wd", wd, 0.5);

  //
  odometry_connected = false;
  initial_time = ros::Time().toSec();


  // Publisher := cmd_vel_mux/input/teleop
  pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 10);
  // frequency: 50 Hz
  pub_timer = nh.createTimer(ros::Duration(0.02), &InOutLinController::pubCallback, this);
  // frequency: 1 Hz
  dis_timer = nh.createTimer(ros::Duration(1), &InOutLinController::disCallback, this);

  //Subscriber := current states
  odom_sub = nh.subscribe("odom", 10, &InOutLinController::odom_subCallback, this);
  //Subscriber := reference trajectory parameters
  //trajectory_sub = nh.subscribe("ref", 10, &InOutLinController::trajectory_subCallback, this);

}

// destructor
InOutLinController::~InOutLinController()
{
  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0;
  pub.publish(cmd_vel);
  ros::shutdown();
}

// odometry subscriber callback function
void InOutLinController::odom_subCallback(const nav_msgs::Odometry::ConstPtr& msgs)
{
  // set the intial time when odometry is connected
  if (odometry_connected == false)
  {
      odometry_connected = true;
      t0 = ros::Time::now().toSec(); // intial time
  }
  state.header = msgs->header;
  state.child_frame_id = msgs->child_frame_id;
  state.pose = msgs->pose;
  state.twist = msgs->twist;
}

//void trajectory_subCallback() {
//
//} // to be determined)

// callback function to display the odometry reading
void InOutLinController::disCallback(const ros::TimerEvent& event) {
  // displays the current pose and velocity
  double x = state.pose.pose.position.x;
  double y = state.pose.pose.position.y;
  double yaw = QuaternionToYaw(state);
  double vx = state.twist.twist.linear.x;
  double vy = state.twist.twist.linear.y;
  double dot_yaw = state.twist.twist.angular.z;

  ROS_INFO("-----------------------------------------");
  ROS_INFO_STREAM(path_type<<" path");
  ROS_INFO("Odom Reading: ");
  ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"Time: "
    <<ros::Time::now().toSec()-initial_time<<"s");
  ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"position at ["<<x<<", "<<y<<"]"<<"|| orientation at "<<yaw);
  //ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"goal position at ["<<goal.x<<", "<<goal.y<<"]");
  ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"cmd lin vel: "<<cmd_vel.linear.x<<"|| cmd ang vel: "<<cmd_vel.angular.z);
  ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"velocity is "<<vx<<"|| yaw rate is "<<dot_yaw);
}

// publisher callback function
void InOutLinController::pubCallback(const ros::TimerEvent& event)
{
  // Input/Output Linearization Algorithm
  double xd, yd, vd;
  double y1d, y2d, vy1d, vy2d;
  double y1, y2;
  double x = state.pose.pose.position.x;
  double y = state.pose.pose.position.y;
  double theta = QuaternionToYaw(state);
  double c_th, s_th;
  double e_y1, e_y2;
  double u1, u2;
  double t = ros::Time::now().toSec() - t0;
  // the reference states and velocity: circular path
  if (path_type.compare(std::string("circular")) == 0) {
     xd = xc + R*cos(wd*t);
     yd = yc + R*sin(wd*t);
     vd = R*wd;
  }

  // the reference output
  c_th = cos(theta);
  s_th = sin(theta);
  y1d = xd + b*c_th;
  y2d = yd + b*s_th;
  // the time derivative of the reference output
  vy1d = c_th * vd - b * s_th * wd;
  vy2d = s_th * vd + b * c_th * wd;

  // the output error
  y1 = x + b*c_th;
  y2 = y + b*s_th;
  e_y1 = y1d - y1;
  e_y2 = y2d - y2;

  // compute the control inputs for the linearized output model
  u1 = vy1d + k1 * e_y1;
  u2 = vy2d + k2 * e_y2;
  // transform to the actual control inputs for the unicycle model
  cmd_vel.linear.x = c_th * u1 + s_th * u2;
  cmd_vel.angular.z = -s_th/b * u1 + c_th/b * u2;
  // saturation
  cmd_vel.linear.x = std::max(-vmax, std::min(cmd_vel.linear.x, vmax));
  cmd_vel.angular.z = std::max(-wmax, std::min(cmd_vel.angular.z, wmax));

  // publish
  pub.publish(cmd_vel);
}

// helper function
// giving quaternion readings from Odometry, convert them to euler angles
// return range: [0, 2*pi]
double QuaternionToYaw(const nav_msgs::Odometry &msgs)
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
  ros::init(argc, argv, "InOutLinController_node");

  InOutLinController InOutLinController_node;

  ros::spin();

  return 0;
}
