
#include "IO_control_collision.h"

// constructor
IO_control_collision::IO_control_collision()
:nh_private_("~")
{
  // ------------------------------ Set Parameters -----------------------------
  // Initialize the parameters (constant, from the launch file)
  // syntax: nh_private_.param<type>
  // ("parameter name in launch file", variable to be assigned, default value);
  nh_private_.param<double>("b", b, 0.05);
  nh_private_.param<double>("k1", k1, 4.0);
  nh_private_.param<double>("k2", k2, 4.0);
  nh_private_.param<double>("k3", k3, 10.0); // Proportional gain for collision avoidance direction tracking
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

  // Parameters for collision avoidance
  nh_private_.param<double>("ds", ds, 1); // Safety radius; agents must be twice this far away to not crash.
  nh_private_.param<double>("dc", dc, 2); // Radius where collision avoidance function becomes active. Must be strictly greater than ds.
  nh_private_.param<int>("agent_index", agent_index, 0);

  even_cycle = false;
  //
  odometry_connected = false;
  initial_time = ros::Time().toSec();
  n = 0; // This is changed in the callback function all_states_sub

  // ------------------------------ Set Pubs/Subs -----------------------------
  // Publisher := cmd_vel_mux/input/teleop
  pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 10);
  // frequency: 50 Hz
  pub_timer = nh.createTimer(ros::Duration(0.02), &IO_control_collision::pubCallback, this);
  // frequency: 1 Hz
  dis_timer = nh.createTimer(ros::Duration(1), &IO_control_collision::disCallback, this);

  //Subscriber := current states
  odom_sub = nh.subscribe("odom", 10, &IO_control_collision::odom_subCallback, this);

  //Subscriber := reference trajectory parameters
  trajectory_sub = nh.subscribe("ref", 10, &IO_control_collision::trajectory_subCallback, this);

}

// destructor
IO_control_collision::~IO_control_collision()
{
  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0;
  pub.publish(cmd_vel);
  ros::shutdown();
}

// odometry subscriber callback function
void IO_control_collision::odom_subCallback(const nav_msgs::Odometry::ConstPtr& msgs)
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

void IO_control_collision::trajectory_subCallback(const rcomv_r1::CubicPath::ConstPtr& msgs)
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
void IO_control_collision::disCallback(const ros::TimerEvent& event) {
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
void IO_control_collision::pubCallback(const ros::TimerEvent& event)
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
     xd = xc + R*cos(wd*t) + Ri*cos(theta+alphai);
     yd = yc + R*sin(wd*t) + Ri*sin(theta+alphai);
     vd = hypot(wd*(-R*sin(wd*t) - Ri*sin(theta+alphai)),
                wd*(R*cos(wd*t) + Ri*cos(theta+alphai)));
  }
  // the reference states and velocity: eight-shaped path
  if (path_type.compare(std::string("eight_shaped")) == 0) {
     xd = xc + R1*sin(2*wd*t) + Ri*cos(theta+alphai);
     yd = yc + R2*sin(wd*t) + Ri*sin(theta+alphai);
     //vd = hypot((2*R1*wd*cos(2*wd*t)), (R2*wd*cos(wd*t)));
     vd = hypot((2*wd*R1*cos(2*wd*t) - wd*Ri*sin(theta+alphai)),
                wd*(R2*cos(wd*t) + Ri*cos(theta+alphai)));
  }
  // the reference states and velocity: cubic polynomial path
  if (path_type.compare(std::string("cubic")) == 0) {
      CubePolyPath(qi, qf, poly_k, T, t, xd, yd, vd, wd);
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

  // compute collision avoidance term


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
void IO_control_collision::CubePolyPath(pose qi, pose qf, double k, double T, double t,
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

// helper function: determine collision avoidance term
control_cmd IO_control_collision::collision_avoid(){
  // Initialize the output
  control_cmd out_cmd; out_cmd.v = 0.0; out_cmd.w = 0.0;

  // Collect list of in-neighbors
  std::vector<geometry_msgs::Pose> all_states = state_lists; // Freezes the state list at a certain time
  geometry_msgs::Pose current_state = all_states[agent_index - 1]; // This agent's current state (pose)
  all_states.erase(all_states.begin() + agent_index - 1); // Remove the agent's state from the list
  std::vector<geometry_msgs::Pose> collision_states = collision_neighbors(all_states, current_state); // Need to define this

  if (!collision_states.empty()){
    // Get the collision avoidance gradient term
    geometry_msgs::Vector3 psi_collision_sum; psi_collision_sum.x = 0.0; psi_collision_sum.y = 0.0; psi_collision_sum.z = 0.0;
    for (int j=0; j<collision_states.size(); j++){
      geometry_msgs::Vector3 grad_vector = psi_col_gradient(current_state, collision_states[j]);
      psi_collision_sum = add_vectors(psi_collision_sum,grad_vector);
    }

    // Convert the collision avoidance gradient term into linear and angular velocity commands
    double PI = 3.141592653589793;
    double theta_d = fmod(atan2(psi_collision_sum.y,psi_collision_sum.x) + 2*PI, 2*PI);
    double theta = tf::getYaw(state.pose.pose.orientation);
    double angle_error = findDifference(theta,theta_d); // Change angle error to be in [-PI,PI]

    out_cmd.w = k3*angle_error; // Proportional gain must be tuned

    // Only set a linear velocity if angular error is less than PI/4
    if(abs(angle_error) < PI/4.0){
      out_cmd.v = sqrt(pow(psi_collision_sum.x,2) + pow(psi_collision_sum.y,2));
    }
  }

  return out_cmd;
}

// Helper function: update states of other agents
void IO_control_collision::graph_subCallback(const state_graph_builder::posegraph::ConstPtr& msgs){
  // The posegraph->poses attribute returns a vector of pose objects
  // state_lists should be an array of pose objects
  state_lists = msgs->poses;
}

// Helper function: calculate neighbors within dc of the agent
std::vector<geometry_msgs::Pose> IO_control_collision::collision_neighbors(const std::vector<geometry_msgs::Pose> &other_agents, const geometry_msgs::Pose &current_state){
  std::vector<geometry_msgs::Pose> close_poses;
  for(int ii=0; ii < other_agents.size(); ii++){
    double distance = std::sqrt(std::pow(current_state.position.x - other_agents[ii].position.x,2) +\
      std::pow(current_state.position.y - other_agents[ii].position.y,2) + std::pow(current_state.position.z - other_agents[ii].position.z,2));
    if(distance < dc){
      // Save the close poses
      close_poses.push_back(other_agents[ii]);
    }
  }
  return close_poses;
}

double IO_control_collision::psi_col_helper(const geometry_msgs::Point &m_agent, const  geometry_msgs::Point &n_agent){
  geometry_msgs::Vector3 vecij = calc_vec(m_agent,n_agent);
  double val=self_norm(vecij);
  double mu2=10000;
  double output;

  // //Reference MATLAB code
  // if norm(xij,2) <= norm(dc,2)
  //   mu2 = 10000; % What should this be? Not sure.

  //   outscalar = (norm(xij,2) - dc)^2 / (norm(xij,2) - ds + (ds - dc)^2/mu2);
  // elseif norm(xij,2) < ds
  //    outscalar = mu2;
  // else
  //    outscalar = 0;
  // end
  double dc2 = 2*dc; // Line 123 of MATLAB code -- 2*dc is used.
  if (val <= dc2){
    if (val >=ds)
      output =  ((val - dc2)*(val-dc2)) / (val - ds + ((ds-dc2)*(ds-dc2))/mu2);
    else
      output = mu2;
  }
  else
    output = 0.0;
}

geometry_msgs::Vector3 IO_control_collision::psi_col_gradient(const geometry_msgs::Pose &m_agent, const geometry_msgs::Pose &n_agent){ //this is supposed to only take the state vector
  double h=0.001;
  std::vector<geometry_msgs::Point> perturb;
  for (int i=0; i<6; i++){
    perturb.push_back(m_agent.position);
  }
  perturb[0].x+=h;
  perturb[1].x-=h;
  perturb[2].y+=h;
  perturb[3].y-=h;
  perturb[4].z+=h;
  perturb[5].z-=h;


  geometry_msgs::Vector3 output;
  output.x = (psi_col_helper(perturb[0],n_agent.position) - psi_col_helper(perturb[1],n_agent.position))/(2*h);

  output.y = (psi_col_helper(perturb[2],n_agent.position) - psi_col_helper(perturb[3],n_agent.position))/(2*h);

  output.z = (psi_col_helper(perturb[4],n_agent.position) - psi_col_helper(perturb[5],n_agent.position))/(2*h);
  //std::cout << "Output" << output << std::endl;
  return output;

}

geometry_msgs::Vector3 IO_control_collision::calc_vec(const geometry_msgs::Point& state1, const geometry_msgs::Point& state2){
  geometry_msgs::Vector3 outVector3;
  outVector3.x = state1.x - state2.x;
  outVector3.y = state1.y - state2.y;
  outVector3.z = state1.z - state2.z;
  return outVector3;
}

double IO_control_collision::self_norm(const geometry_msgs::Vector3 &tiny){
  double val;
  val = sqrt((tiny.x*tiny.x) + (tiny.y*tiny.y) + (tiny.z*tiny.z));
  return val;
}

geometry_msgs::Vector3 IO_control_collision::add_vectors(const geometry_msgs::Vector3 &a, const geometry_msgs::Vector3 &b){
  geometry_msgs::Vector3 result;
  result.x = a.x + b.x;
  result.y = a.y + b.y;
  result.z = a.z + b.z;
  return result;
}

// main function: create a IO_control_collision class type that handles everything
int main(int argc, char** argv) {
  ros::init(argc, argv, "IO_control_collision_node");

  IO_control_collision IO_control_collision_node;

  ros::spin();

  return 0;
}
