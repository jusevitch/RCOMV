
#include "InputOutputLin_Controller.h"

// constructor
InOutLinController::InOutLinController()
:nh_private_("~")
{
  // ------------------------------ Set Parameters -----------------------------
  // Initialize the parameters (constant, from the launch file)
  // syntax: nh_private_.param<type>
  // ("parameter name in launch file", variable to be assigned, default value);
  nh_private_.param<double>("b", b, 0.05);
  nh_private_.param<double>("k1", k1, 4);
  nh_private_.param<double>("k2", k2, 4);
  nh_private_.param<double>("vmax", vmax, 6);
  nh_private_.param<double>("wmax", wmax, 4);
  nh_private_.param<double>("Ri", Ri, 0);
  nh_private_.param<double>("alphai", alphai, 0);

  // Initialize the paramters (varying, will be updated by subscribers)
  nh_private_.param<std::string>("path_type", path_type, "circular");
  //
  // parametric path paramters
  nh_private_.param<double>("xc", xc, 0);
  nh_private_.param<double>("yc", yc, 0);
  nh_private_.param<double>("R", R, 4);
  nh_private_.param<double>("wd", wd, 0.5);
  nh_private_.param<double>("t0", t0, ros::Time().toSec());
  nh_private_.param<double>("R1", R1, 4);
  nh_private_.param<double>("R2", R2, 4);
  // cubic ploynomials path paramters
  nh_private_.param<double>("qi_x", qi.x, 5); nh_private_.param<double>("qi_y", qi.y, 0); nh_private_.param<double>("qi_theta", qi.theta, 1.57);
  nh_private_.param<double>("qf_x", qf.x, -5); nh_private_.param<double>("qf_y", qf.y, 0); nh_private_.param<double>("qf_theta", qf.theta, 4.71);
  nh_private_.param<double>("poly_k", poly_k, 20);
  nh_private_.param<double>("T", T, 15);
  nh_private_.param<bool>("endless", endless_flag, false);

  // For indoor use
  nh_private_.param<bool>("indoors_rover_bool", indoors_rover_bool, true); // Tests whether we're running it indoors or not
  nh_private_.param<int>("rover_number", rover_number, 0); // gives Rover number; 0 throws an error.

  even_cycle = false;
  //
  odometry_connected = false;
  initial_time = ros::Time().toSec();


  // ------------------------------ Set Pubs/Subs -----------------------------
  // Publisher := cmd_vel_mux/input/teleop
  if(indoors_rover_bool){
    indoor_pub_topic = "/R" + std::to_string(rover_number) + "/cmd_vel";
    pub = nh.advertise<geometry_msgs::Twist>(indoor_pub_topic, 10);
  } else {
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel/", 10); //  cmd_vel_mux/input/teleop
  }
  // frequency: 50 Hz
  pub_timer = nh.createTimer(ros::Duration(0.02), &InOutLinController::pubCallback, this);
  // frequency: 1 Hz
  dis_timer = nh.createTimer(ros::Duration(1), &InOutLinController::disCallback, this);

  //Subscriber := current states
  if(indoors_rover_bool){
    indoor_sub_topic = "/R" + std::to_string(rover_number);
    indoors_sub = nh.subscribe(indoor_sub_topic, 10, &InOutLinController::indoor_subCallback, this);
  } else{
    odom_sub = nh.subscribe("odom", 10, &InOutLinController::odom_subCallback, this);
  }

  //Subscriber := reference trajectory parameters
  trajectory_sub = nh.subscribe("ref", 10, &InOutLinController::trajectory_subCallback, this);

  //Subscriber := update parameters based on trajectory
  msrpa_sub = nh.subscribe("msrpa", 10, &InOutLinController::msrpa_subCallback, this);

  // ROS_INFO("this worked in constructor \n");
}

// destructor
InOutLinController::~InOutLinController()
{
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  pub.publish(cmd_vel);
  ros::shutdown();
}

// callback function to update the parameters for this class
void InOutLinController::msrpa_subCallback(const rcomv_r1::MSRPA& msg)
{

  if(msg.type=="circular") {
    // update trajectory type
    this->path_type = msg.type;
    
    // update trajectory parameters
    this->t0 = msg.trajectory[0];
    this->xc = msg.trajectory[1]; 
    this->yc = msg.trajectory[2];
    this->R = msg.trajectory[3]; 
    this->wd = msg.trajectory[4];
    this->phi0 = msg.trajectory[5];


    // update formation parameters
    this->Ri = msg.formation[0];
  }

  else if (msg.type=="square") {
    // update trajectory type
    this->path_type = msg.type;

    // update trajectory parameters
    this->t0 = msg.trajectory[0];
    this->xc = msg.trajectory[1]; 
    this->yc = msg.trajectory[2];
    this->L = msg.trajectory[3];
    this->psi = msg.trajectory[4];
    this->V = msg.trajectory[5];
    this->startLIdx = msg.trajectory[6];
    this->T = this->L / this->V;

    // update formation parameters
    this->Ri = msg.formation[0];

  }

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

void InOutLinController::indoor_subCallback(const geometry_msgs::PoseStamped::ConstPtr& msgs)
{
  if (odometry_connected == false)
  {
      odometry_connected = true;
      t0 = ros::Time::now().toSec(); // intial time
  }
  state.header = msgs->header;
  // state.child_frame_id = msgs->child_frame_id;
  state.pose.pose.position.x = msgs->pose.position.x;
  state.pose.pose.position.y = msgs->pose.position.y;
  state.pose.pose.position.z = msgs->pose.position.z;

  state.pose.pose.orientation = msgs->pose.orientation;

}

void InOutLinController::trajectory_subCallback(const rcomv_r1::CubicPath::ConstPtr& msgs)
{

  path_type = msgs->path_type;
  qi.x = msgs->qi_x;
  qi.y = msgs->qi_y;
  qi.theta = msgs->qi_theta;
  qf.x = msgs->qf_x;
  qf.y = msgs->qf_y;
  qf.theta = msgs->qf_theta;
  T = msgs->T;
  poly_k = msgs->poly_k;

  // if endless_flag is true and it's in a even number cycle, switch the qi and qf
  if (endless_flag && even_cycle) {
    pose temp_q = qi;
    qi = qf;
    qf = temp_q;
    ROS_INFO("Even Cycle...");
  }
}

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
  
  ROS_INFO("Theta: %lf \n", theta);

  // the reference states and velocity: circular path
  if (path_type.compare(std::string("circular")) == 0) {
     xd = xc + R*cos(wd*t) + Ri*cos(wd*t+alphai);
     yd = yc + R*sin(wd*t) + Ri*sin(wd*t+alphai);
     vd = hypot(wd*(-R*sin(wd*t) - Ri*sin(theta+alphai)),
                wd*(R*cos(wd*t) + Ri*cos(theta+alphai)));
     ROS_INFO("(xd, yd) : (%lf, %lf)", xd, yd);
  }
  // the reference states and velocity: eight-shaped path
  else if (path_type.compare(std::string("eight_shaped")) == 0) {
     xd = xc + R1*sin(2*wd*t) + Ri*cos(wd*t+alphai);
     yd = yc + R2*sin(wd*t) + Ri*sin(wd*t+alphai);
     //vd = hypot((2*R1*wd*cos(2*wd*t)), (R2*wd*cos(wd*t)));
     vd = hypot((2*wd*R1*cos(2*wd*t) - wd*Ri*sin(theta+alphai)),
                wd*(R2*cos(wd*t) + Ri*cos(theta+alphai)));
  }
  // the reference states and velocity: cubic polynomial path
  else if (path_type.compare(std::string("cubic")) == 0) {
      CubePolyPath(qi, qf, poly_k, T, t, xd, yd, vd, wd);
  }

  double xddot, yddot, theta_d, xdoubledot, ydoubledot, thetaddot;

  // the reference output
  if(path_type.compare(std::string("circular")) == 0) {
    theta_d = fmod(wd*t + (M_PI / 2.0) + 2*M_PI, 2*M_PI);
  } else if(path_type.compare(std::string("eight_shaped")) == 0) {
    xddot = wd*(2*R1*cos(2*wd*t) - Ri*sin(wd*t + alphai)); // First derivative of xd
    yddot = wd*(R2*cos(wd*t) + Ri*cos(wd*t + alphai)); // First derivative of yd
    theta_d = atan2(yddot,xddot);
  }
  double c_thd = cos(theta_d);
  double s_thd = sin(theta_d);
  y1d = xd + b*c_thd;
  y2d = yd + b*s_thd;
  // the time derivative of the reference output
  if(path_type.compare(std::string("circular")) == 0) {
    vy1d = -wd*(R*sin(wd*t) + Ri*sin(wd*t + alphai) + b*sin(theta_d)); // c_thd * vd - b * s_thd * wd;
    vy2d = wd*(R*cos(wd*t) + Ri*cos(wd*t + alphai) + b*cos(theta_d)); // s_thd * vd + b * c_thd * wd;
  } 
  else if(path_type.compare(std::string("eight_shaped")) == 0) {
    double xdoubledot = -pow(wd,2)*(4*R1*sin(2*wd*t) + Ri*cos(wd*t + alphai)); // Second derivative of xd
    double ydoubledot = -pow(wd,2)*(R2*sin(wd*t) + Ri*sin(wd*t+alphai)); // Second derivative of yd
    double thetaddot = (xddot / (pow(xddot,2) + pow(yddot,2)))*ydoubledot - (yddot / (pow(xddot,2) + pow(yddot,2)))*xdoubledot;
    vy1d = xddot - b*sin(theta_d)*thetaddot;
    vy2d = yddot + b*cos(theta_d)*thetaddot;
  }
  else if( path_type.compare(std::string("square")) == 0) {
      // modulo the time for infinite looping
      double t_corrected = (t >= 8*T) ? fmod(t, 8*T) : t;

      // parameters for square
      if ( 0 <= t_corrected && t_corrected < T) {
        y1d = xc + L;
        y2d = yc + V*t_corrected;
        vy1d = 0;
        vy2d = V;
      } 
      else if ( T <= t_corrected && t_corrected < 3*T) {
        y1d = xc + L - V*(t_corrected-T);
        y2d = yc + L;
        vy1d = -V;
        vy2d = 0;
      }
      else if ( 3*T <= t_corrected && t_corrected < 5*T) {
        y1d = xc - L;
        y2d = yc + L -V*(t_corrected-3*T);
        vy1d = 0;
        vy2d = -V;
      }
      else if ( 5*T <= t_corrected && t_corrected < 7*T) {
        y1d = xc - L + V*(t_corrected-5*T);
        y2d = yc - L;
        vy1d = V;
        vy2d = 0;
      }
      else if ( 7*T <= t_corrected && t_corrected < 8*T) {
        y1d = xc + L;
        y2d = yc - L + V*(t_corrected-8*T);
        vy1d = 0;
        vy2d = V;
      }
  }
  
  // the output error
  c_th = cos(theta);
  s_th = sin(theta);
  y1 = x + b*c_th;
  y2 = y + b*s_th;
  e_y1 = y1d - y1;
  e_y2 = y2d - y2;

  // compute the control inputs for the linearized output model
  u1 = vy1d + k1 * e_y1;
  u2 =  vy2d + k2 * e_y2;
  // transform to the actual control inputs for the unicycle model
  cmd_vel.linear.x = c_th * u1 + s_th * u2;
  cmd_vel.angular.z = -s_th/b * u1 + c_th/b * u2;
  // saturation
  cmd_vel.linear.x = std::max(-vmax, std::min(cmd_vel.linear.x, vmax));
  cmd_vel.angular.z = std::max(-wmax, std::min(cmd_vel.angular.z, wmax));

  // endless_flag is false: stop when t > T for cubic polynomial path
  // endless_flag is true: reset t0 and odd_cycle flag
  if (path_type.compare(std::string("cubic")) == 0) {
      if (t > T && !endless_flag) {
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
      }
      if (t > T && endless_flag && odometry_connected) {
        if (even_cycle)
            even_cycle = false;
        else
            even_cycle = true;

        pose temp_q = qi;
        qi = qf;
        qf = temp_q;
        t0 = ros::Time::now().toSec(); // intial time
      }
  }

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

// helper function
// return the reference trajectory and velocities for any arbitrary cubic polynomial trajectory
void InOutLinController::CubePolyPath(pose qi, pose qf, double k, double T, double t,
                  double &xd, double &yd, double &vd, double &wd) {
  // uniform time law
  double s = t/T;
  // parameters of the polynomial
  double xi = qi.x, yi = qi.y, thetai = qi.theta;
  double xf = qf.x, yf = qf.y, thetaf = qf.theta;
  double alpha_x = k*cos(thetaf) - 3*xf;
  double alpha_y = k*sin(thetaf) - 3*yf;
  double beta_x = k*cos(thetai) + 3*xi;
  double beta_y = k*sin(thetai) + 3*yi;

  // center position
  xd = pow(s,3.0)*xf - pow((s-1),3.0)*xi + alpha_x*pow(s,2.0)*(s-1) + beta_x*s*pow((s-1),2.0);
  yd = pow(s,3.0)*yf - pow((s-1),3.0)*yi + alpha_y*pow(s,2.0)*(s-1) + beta_y*s*pow((s-1),2.0);
  // center velocity
  // dx/ds and dy/ds
  double vel_x = 3*pow(s,2.0)*xf - 3*pow((s-1),2.0)*xi + alpha_x*(3*pow(s,2.0)-2*s) + beta_x*(3*pow(s,2.0)-4*s+1);
  double vel_y = 3*pow(s,2.0)*yf - 3*pow((s-1),2.0)*yi + alpha_y*(3*pow(s,2.0)-2*s) + beta_y*(3*pow(s,2.0)-4*s+1);
  // dx/dt = dx/ds * ds/dt
  vel_x = vel_x / T;
  vel_y = vel_y / T;
  // center acceleration
  // dv/ds
  double accel_x = 6*s*xf - 6*(s-1)*xi + alpha_x*(6*s-2) + beta_x*(6*s-4);
  double accel_y = 6*s*yf - 6*(s-1)*yi + alpha_y*(6*s-2) + beta_y*(6*s-4);
  // dv/dt = dv/ds * ds/dt
  accel_x = accel_x / (T*T);
  accel_y = accel_y / (T*T);
  // center longitudinal velocity and turning rate
  vd = hypot(vel_x, vel_y);
  wd = (accel_y*vel_x - accel_x*vel_y) / pow(vd,2.0);

  // transform the states from the center location to the agent's location
  //double theta = findDifference(atan2(vel_y, vel_x), thetai);
  //double theta = atan2(vel_y, vel_x) - thetai;
  double theta = atan2(vel_y, vel_x) ;
  xd = xd + Ri*cos(theta+alphai);
  yd = yd + Ri*sin(theta+alphai);
  vel_x = vel_x - wd*Ri*sin(theta+alphai);
  vel_y = vel_y + wd*Ri*cos(theta+alphai);
  accel_x = accel_x - wd*wd*Ri*cos(theta+alphai); // ignore d(wd) / dt
  accel_y = accel_y - wd*wd*Ri*sin(theta+alphai); // ignore d(wd) / dt
  // update accel
  vd = hypot(vel_x, vel_y);
  wd = (accel_y*vel_x - accel_x*vel_y) / pow(vd,2.0);
}

// helper function
double findDifference(double init_psi, double goal_psi)
{
 double err  = goal_psi - init_psi;
 int sign = (std::signbit(err) == 0) ? 1 : -1;
 err  = (fabs(err) <= M_PI) ? err :
                 sign * fmod(fabs(err) - 2*M_PI, M_PI);
 return err;
}


// main function: create a InOutLinController class type that handles everything
int main(int argc, char** argv) {
  ros::init(argc, argv, "InOutLinController_node");

  InOutLinController InOutLinController_node;

  ros::spin();

  return 0;
}
