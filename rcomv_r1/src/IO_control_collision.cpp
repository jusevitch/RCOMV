
#include "IO_control_collision.h"

// constructor
IO_control_collision::IO_control_collision()
:nh_private_("~")
{
  // ------------------------------ Set Parameters -----------------------------
  // Initialize the parameters (constant, from the launch file)
  // syntax: nh_private_.param<type>
  // ("parameter name in launch file", variable to be assigned, default value);
  nh_private_.param<int>("n", n, 0);
  nh_private_.param<double>("b", b, 0.05);
  nh_private_.param<double>("k1", k1, 0.0);
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
  nh_private_.param<double>("t0", t0, ros::Time().toSec());
  nh_private_.param<double>("xc", xc, 0);
  nh_private_.param<double>("yc", yc, 0);
  nh_private_.param<double>("R", R, 1);
  nh_private_.param<double>("wd", wd, 0.5);
  nh_private_.param<double>("phi0", phi0, 0.0);
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

  // Distinguishes between running in simulation and running in hardware
  nh_private_.param<int>("gazebo", gazebo, 1); // Tests whether we're running it indoors or not
  nh_private_.param<int>("rover_number", rover_number, 0); // gives Rover number; 0 throws an error.

  mu2=10000;
  even_cycle = false;
  //
  odometry_connected = false;
  initial_time = ros::Time().toSec();

  // ------------------------------ Set Pubs/Subs -----------------------------
  // Publisher := cmd_vel_mux/input/teleop
  if(gazebo){
    pub_topic = "cmd_vel_mux/input/teleop";
  } else {
    pub_topic = "cmd_vel";
  }
  pub = nh.advertise<geometry_msgs::Twist>(pub_topic, 10);
  // frequency: 50 Hz
  pub_timer = nh.createTimer(ros::Duration(0.02), &IO_control_collision::pubCallback, this);
  // frequency: 1 Hz
  dis_timer = nh.createTimer(ros::Duration(1), &IO_control_collision::disCallback, this);
  // frequency: 50 Hz
  update_param_timer = nh.createTimer(ros::Duration(0.02), &IO_control_collision::change_trajectories, this);

  // Subscriber: gets all current states from state_graph_builder
  if(gazebo){
    states_sub = nh.subscribe("/graph", 1, &IO_control_collision::graph_subCallback_PoseStamped, this);
  } else {
    states_sub = nh.subscribe("/graph", 1, &IO_control_collision::graph_subCallback_PoseStamped, this);
  }

  //Subscriber := current states
  if(gazebo){
    sub_topic = "/R" + std::to_string(rover_number) + "/odom";
    odom_sub = nh.subscribe(sub_topic, 10, &IO_control_collision::odom_subCallback, this);
  } else {
    sub_topic = "/R" + std::to_string(rover_number); // Need to remap the topic name in the launch files
    odom_sub = nh.subscribe(sub_topic,10, &IO_control_collision::posestamped_subCallback, this);
  }

  //Subscriber := reference trajectory parameters
  // WARNING: THE NAME OF THIS TOPIC CONFLICTS WITH THE MSRPA TOPIC. Fix before using.
  // trajectory_sub = nh.subscribe("ref", 10, &IO_control_collision::trajectory_subCallback, this);


  // Subscriber : MS-RPA callback function
  
  msrpa_sub = nh.subscribe("ref", 1, &IO_control_collision::msrpa_Callback, this);


  //Subscriber := update parameters based on trajectory
  msrpa_sub = nh.subscribe("msrpa", 10, &IO_control_collision::msrpa_subCallback, this);

}

// destructor
IO_control_collision::~IO_control_collision()
{
  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0;
  pub.publish(cmd_vel);
  ros::shutdown();
}





// callback function to update the parameters for this class
void IO_control_collision::msrpa_subCallback(const rcomv_r1::MSRPA& msg)
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




// Pose Stamped Callback
void IO_control_collision::posestamped_subCallback(const geometry_msgs::PoseStamped::ConstPtr& msgs){
  if (odometry_connected == false)
  {
      odometry_connected = true;
      t0 = ros::Time::now().toSec(); // intial time
  }

  state.header = msgs->header;
  state.pose.pose.position.x = msgs->pose.position.x;
  state.pose.pose.position.y = msgs->pose.position.y;
  state.pose.pose.position.z = msgs->pose.position.z;

  state.pose.pose.orientation.x = msgs->pose.orientation.x;
  state.pose.pose.orientation.y = msgs->pose.orientation.y;
  state.pose.pose.orientation.z = msgs->pose.orientation.z;
  state.pose.pose.orientation.w = msgs->pose.orientation.w;
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
    // ROS_INFO("Even Cycle...");
  }
}





// callback function to display the odometry reading
// !!! WARNING: THIS WILL NOT WORK PROPERLY FOR NON-GAZEBO TESTS. Real-world data comes as PoseStamped messages instead of Odom, which
//  does not come with a Twist element.
void IO_control_collision::disCallback(const ros::TimerEvent& event) {
  // displays the current pose and velocity
  double x = state.pose.pose.position.x;
  double y = state.pose.pose.position.y;
  double yaw = QuaternionToYaw(state);
  double vx = state.twist.twist.linear.x;
  double vy = state.twist.twist.linear.y;
  double dot_yaw = state.twist.twist.angular.z;

  // ROS_INFO("-----------------------------------------");
  // ROS_INFO_STREAM(path_type<<" path");
  // ROS_INFO("Odom Reading: ");
  // ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"Time: "
  //   <<ros::Time::now().toSec()-initial_time<<"s");
  // ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"position at ["<<x<<", "<<y<<"]"<<"|| orientation at "<<yaw);
  // //ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"goal position at ["<<goal.x<<", "<<goal.y<<"]");
  // ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"cmd lin vel: "<<cmd_vel.linear.x<<"|| cmd ang vel: "<<cmd_vel.angular.z);
  // ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"velocity is "<<vx<<"|| yaw rate is "<<dot_yaw);
}



// -------------------------
// Control Command Generating function
// -------------------------

// publisher callback function
void IO_control_collision::pubCallback(const ros::TimerEvent& event)
{
  // ROS_INFO("FOOOOOOBAAAARRRRRRR TIMER");
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

  // ROS_INFO("gazebo: %d", gazebo ? 1 : 0);

  double t = ros::Time::now().toSec() - t0;

  // the reference states and velocity: circular path
  if (path_type.compare(std::string("circular")) == 0) {
    xd = xc + R*cos(wd*t + this->phi0 ) + Ri*cos(wd*t + this->phi0 + alphai);
    yd = yc + R*sin(wd*t + this->phi0 ) + Ri*sin(wd*t + this->phi0 + alphai);
    vd = hypot(wd*(-R*sin(wd*t + this->phi0 ) - Ri*sin(theta+alphai)),
                wd*(R*cos(wd*t + this->phi0 ) + Ri*cos(theta+alphai)));
  }
  // the reference states and velocity: eight-shaped path
  if (path_type.compare(std::string("eight_shaped")) == 0) {
     xd = xc + R1*sin(2*wd*t) + Ri*cos(wd*t+alphai);
     yd = yc + R2*sin(wd*t) + Ri*sin(wd*t+alphai);
     //vd = hypot((2*R1*wd*cos(2*wd*t)), (R2*wd*cos(wd*t)));
     vd = hypot((2*wd*R1*cos(2*wd*t) - wd*Ri*sin(theta+alphai)),
                wd*(R2*cos(wd*t) + Ri*cos(theta+alphai)));
  }
  // the reference states and velocity: cubic polynomial path
  if (path_type.compare(std::string("cubic")) == 0) {
      CubePolyPath(qi, qf, poly_k, T, t, xd, yd, vd, wd);
  }

  double xddot, yddot, theta_d, xdoubledot, ydoubledot, thetaddot;
  // ROS_INFO("wd : %lf", wd);
  // ROS_INFO("agent_index, rover_number: %d, %d", agent_index, rover_number);

  // the reference output
  if(path_type.compare(std::string("circular")) == 0) {
    theta_d = fmod(wd*t + this->phi0 + (M_PI / 2.0) + 2*M_PI, 2*M_PI);
  } else if(path_type.compare(std::string("eight_shaped")) == 0) {
    xddot = wd*(2*R1*cos(2*wd*t) - Ri*sin(wd*t + alphai)); // First derivative of xd
    yddot = wd*(R2*cos(wd*t) + Ri*cos(wd*t + alphai)); // First derivative of yd
    theta_d = atan2(yddot,xddot);
  }
  ROS_INFO("xd, yd : [%lf, %lf]", xd, yd);
  ROS_INFO("xc, yc : [%lf, %lf]", xc, yc);
  double c_thd = cos(theta_d);
  double s_thd = sin(theta_d);
  y1d = xd + b*c_thd;
  y2d = yd + b*s_thd;
  // the time derivative of the reference output
  if(path_type.compare(std::string("circular")) == 0) {
    vy1d = -wd*(R*sin(wd*t + this->phi0 ) + Ri*sin(wd*t + this->phi0 + alphai) + b*sin(theta_d)); // c_thd * vd - b * s_thd * wd;
    vy2d = wd*(R*cos(wd*t + this->phi0 ) + Ri*cos(wd*t + this->phi0 + alphai) + b*cos(theta_d)); // s_thd * vd + b * c_thd * wd;
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
  // ROS_INFO("xd, yd, thetad: [%lf, %lf, %lf]", xd, yd, theta_d);

  c_th = cos(theta);
  s_th = sin(theta);

  // the output error
  y1 = x + b*c_th;
  y2 = y + b*s_th;
  e_y1 = y1d - y1;
  e_y2 = y2d - y2;

  // compute the control inputs for the linearized output model
  u1 = vy1d + k1 * e_y1;
  u2 = vy2d + k2 * e_y2;

  // compute collision avoidance term
  
  control_cmd coll_avoid = IO_control_collision::collision_avoid(); // inputs are x,y,theta? Delays may cause problems

  ROS_INFO("Gamma, v_coll, w_coll: [%lf, %lf, %lf]", coll_avoid.gamma, coll_avoid.v, coll_avoid.w);

  // transform to the actual control inputs for the unicycle model
  cmd_vel.linear.x =  (1.0 - coll_avoid.gamma)*(c_th * u1 + s_th * u2) + coll_avoid.gamma*coll_avoid.v; // 
  cmd_vel.angular.z =  (1.0 - coll_avoid.gamma)*(-s_th/b * u1 + c_th/b * u2) + coll_avoid.gamma*coll_avoid.w; // 

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



// -------------------------
// Collision Avoidance Term Generating Function
// -------------------------


// helper function: determine collision avoidance term
control_cmd IO_control_collision::collision_avoid(){
  // Initialize the output
  control_cmd out_cmd; out_cmd.v = 0.0; out_cmd.w = 0.0;

  ROS_INFO("state_lists.size(): %d", state_lists.size());
  ROS_INFO("n: %d", n);
  ROS_INFO("state_lists.size() == n: %d", (state_lists.size() == n));


  // Collect list of in-neighbors
  if(!state_lists.empty() && state_lists.size() == n){ // Keeps the node from crashing before the list is populated
    // ROS_INFO("FOOOOOOBAAAAARRRR");
    std::vector<geometry_msgs::PoseStamped> all_states = state_lists; // Freezes the state list at a certain time
    geometry_msgs::PoseStamped current_state = all_states[agent_index - 1]; // This agent's current state (pose)
    // ROS_INFO("x,y,z for rover_number %d, agent_index %d: [%lf, %lf, %lf]", rover_number, agent_index,\
      current_state.pose.position.x, current_state.pose.position.y, current_state.pose.position.z);
    // ROS_INFO("current_state x,y,z: [%lf, %lf, %lf]", current_state.position.x, current_state.position.y, current_state.position.z);
    all_states.erase(all_states.begin() + agent_index - 1); // Remove the agent's state from the list
    std::vector<geometry_msgs::Pose> collision_states = collision_neighbors(all_states, current_state); 

    ROS_INFO("collision_states.size(): %d", collision_states.size());

    if (!collision_states.empty()){
      
      ROS_INFO("Collision_states is not empty");
      
      // Get the collision avoidance gradient term
      geometry_msgs::Vector3 psi_collision_sum; psi_collision_sum.x = 0.0; psi_collision_sum.y = 0.0; psi_collision_sum.z = 0.0;
      geometry_msgs::Vector3 grad_vector; grad_vector.x = 0.0; grad_vector.y = 0.0; grad_vector.z = 0.0;
      for (int j=0; j<collision_states.size(); j++){
        
        // ROS_INFO("current state: [%lf, %lf, %lf] \
        // collision_states[j]: [%lf, %lf, %lf]\
        // difference_norm: %lf, \
        // ds: %lf", \
        // current_state.position.x, current_state.position.y, current_state.position.z, \
        // collision_states[j].position.x, collision_states[j].position.y, collision_states[j].position.z, \
        // difference_norm(current_state,collision_states[j]));
        // ROS_INFO("BARFOO TO THE MIN!!!!");
        if(difference_norm(current_state,collision_states[j]) < ds){
          // ROS_INFO("FOOBAR TO THE MAX!!!!");
          // Return maximum norm gradient in direction opposite of other agent.
          // This is necessary because value is capped at mu2. If agents are less than ds apart, the gradient will be zero.
          // !!! THE FOLLOWING CODE ONLY WORKS FOR 2D GROUND ROVERS
          double grad_norm = std::abs(2*mu2/(ds - dc) - pow(mu2,2)/pow(ds - dc,2));
          double grad_angle = std::atan2(current_state.pose.position.y - collision_states[j].position.y,current_state.pose.position.x - collision_states[j].position.x);
          grad_angle = std::fmod(grad_angle + 2*M_PI, 2*M_PI);
          ROS_INFO("grad_angle: %lf", grad_angle);
          // ROS_INFO("grad_angle for agent %d: %lf", agent_index, grad_angle);
          
          grad_vector.x = grad_norm*std::cos(grad_angle);
          grad_vector.y = grad_norm*std::sin(grad_angle);

          // ROS_INFO("grad_vector x,y : [%lf, %lf]", grad_vector.x, grad_vector.y);
        } else {        
          grad_vector = psi_col_gradient(current_state, collision_states[j]);
        }

        psi_collision_sum = add_vectors(psi_collision_sum,grad_vector);
      }

      // ROS_INFO("psi_collision_sum x,y: [%lf, %lf]", psi_collision_sum.x, psi_collision_sum.y);
      // Convert the collision avoidance gradient term into linear and angular velocity commands
      double PI = 3.141592653589793;
      double theta_d = fmod(atan2(psi_collision_sum.y,psi_collision_sum.x) + 2*PI, 2*PI);
      // ROS_INFO("theta_d: %lf",theta_d);
      double theta = tf::getYaw(state.pose.pose.orientation);
      double angle_error = findDifference(theta,theta_d); // Change angle error to be in [-PI,PI]

      out_cmd.w = k3*angle_error; // Proportional gain must be tuned

      // Only set a linear velocity if angular error is less than PI/4
      if(abs(angle_error) < PI/4.0){
        out_cmd.v = sqrt(pow(psi_collision_sum.x,2) + pow(psi_collision_sum.y,2));
      }
      // Calculate the variable gamma which interpolates between tracking the trajectory and the collision avoidance
    
      // Find out the smallest Euclidean distance between this agent and any one of the agents in collision_states
      double min_distance;
      // ROS_INFO("FOOBAR TO THE MAX!!! %d", agent_index);
      if(agent_index > 2){
        // ROS_INFO("collision_states.size(): %d", collision_states.size());
      }
      for (int jj=0; jj < collision_states.size(); jj++){
        double xi = current_state.pose.position.x; double yi = current_state.pose.position.y; double zi = current_state.pose.position.z;
        double xj = collision_states[jj].position.x; double yj = collision_states[jj].position.y; double zj = collision_states[jj].position.z;
        double xij = std::sqrt(std::pow(xi - xj,2) + std::pow(yi - yj,2) + std::pow(zi - zj,2));
      
        if(jj == 0 || xij < min_distance){
          min_distance = xij;
        }
      }
      // ROS_INFO("min_distance: %lf", min_distance);
      if (min_distance > dc){
        out_cmd.gamma = 0.0;
      } else if(min_distance < ds){
        out_cmd.gamma = 1.0;
      } else{
        out_cmd.gamma = 1.0 - (min_distance - ds) / std::abs(dc - ds); // Convex interpolation between 1 and 0
      }
    } else {
      out_cmd.gamma = 0.0;
      out_cmd.v = 0.0;
      out_cmd.w = 0.0;
    }
  }


  return out_cmd;
}






// -------------------------
// Updates the states of all the other agents from state_graph_builder
// -------------------------


// Helper function: update states of other agents
void IO_control_collision::graph_subCallback(const state_graph_builder::posegraph::ConstPtr& msgs){
  // The posegraph->poses attribute returns a vector of pose objects
  // state_lists should be an array of pose objects
  // ROS_INFO("FOOBAR graph_subCallback");
  for(int ii=1; ii < msgs->poses.size(); ii++){
    state_lists[ii].pose = msgs->poses[ii];
  }
}




void IO_control_collision::graph_subCallback_PoseStamped(const state_graph_builder::posestampedgraph::ConstPtr& msgs) {
  // ROS_INFO("FOOBAR graph_subCallback");
  state_lists = msgs->poses;
  // ROS_INFO("[0] x,y,z: [%lf, %lf, %lf],\n [1] x,y,z: [%lf, %lf, %lf]",\
  state_lists[0].pose.position.x, state_lists[0].pose.position.y, state_lists[0].pose.position.z,\
  state_lists[1].pose.position.x, state_lists[1].pose.position.y, state_lists[1].pose.position.z);
}




void IO_control_collision::msrpa_Callback(const rcomv_r1::MSRPA::ConstPtr& msgs){
  
  // Need to add square trajectory functionality. To be created.

  // Add new variables to the queue. Their values will be passed to the controller once t >= t0_q.
  if(msgs->type.compare("circular") == 0){
    type_q = msgs->type;
    t0_q = msgs->trajectory[0];
    xc_q = msgs->trajectory[1];
    yc_q = msgs->trajectory[2];
    R_q = msgs->trajectory[3];
    wd_q = msgs->trajectory[4];
    phi0_q = msgs->trajectory[5];
    ROS_INFO("ms_rpa callback worked");
  }
}


void IO_control_collision::change_trajectories(const ros::TimerEvent& event){

  // Check the current time against the next t0 in the queue: t0_q.
  // If t >= t0_q, store t0_q -> t0 and change variables to the parameters of the next trajectory

  double t = ros::Time::now().toSec();

  if(t >= t0_q){
    if(type_q.compare("circular") == 0){
      path_type = type_q;
      t0 = t0_q;
      xc = xc_q;
      yc = yc_q;
      R = R_q;
      wd = wd_q;
      phi0 =phi0_q;
      ROS_INFO("Parameters switched: \n t0, xc, yc, R, wd, phi0: [%lf,%lf, %lf, %lf, %lf, %lf]", t0, xc, yc, R, wd, phi0);
    }

    // Add square trajectories here

  }

}




// Helper function: calculate neighbors within dc of the agent
std::vector<geometry_msgs::Pose> IO_control_collision::collision_neighbors(const std::vector<geometry_msgs::Pose> &other_agents, const geometry_msgs::Pose &current_state){
  double distance = 0.0;
  std::vector<geometry_msgs::Pose> close_poses;
  // ROS_INFO("other_agents.size() not overload: %d", other_agents.size());
  for(int ii=0; ii < other_agents.size(); ii++){
    distance = std::sqrt(std::pow(current_state.position.x - other_agents[ii].position.x,2) +\
      std::pow(current_state.position.y - other_agents[ii].position.y,2) + std::pow(current_state.position.z - other_agents[ii].position.z,2));
    if(distance < dc){
      // Save the close poses
      close_poses.push_back(other_agents[ii]);
    }
  }
  ROS_INFO("dc, distance, close_poses.size(): [%lf, %lf, %d]", dc, distance, close_poses.size());
  return close_poses;
}

// Overloaded for PoseStamped
// Helper function: calculate neighbors within dc of the agent
std::vector<geometry_msgs::Pose> IO_control_collision::collision_neighbors(const std::vector<geometry_msgs::PoseStamped> &other_agents, const geometry_msgs::PoseStamped &current_state){
  double distance = 0.0;
  std::vector<geometry_msgs::Pose> close_poses;
  // ROS_INFO("other_agents.size() for overload: %d", other_agents.size());
  for(int ii=0; ii < other_agents.size(); ii++){
    distance = std::sqrt(std::pow(current_state.pose.position.x - other_agents[ii].pose.position.x,2) +\
      std::pow(current_state.pose.position.y - other_agents[ii].pose.position.y,2) + std::pow(current_state.pose.position.z - other_agents[ii].pose.position.z,2));
    // ROS_INFO("other_agents.x, otheragents.y, other agents.z: [%lf, %lf, %lf], \n \
    current_state.x, current_state.y, current_state.z: [%lf, %lf, %lf]",\
    other_agents[ii].pose.position.x, other_agents[ii].pose.position.y, other_agents[ii].pose.position.z,\
    current_state.pose.position.x, current_state.pose.position.y, current_state.pose.position.z);
    
    if(distance < dc){
      // Save the close poses
      close_poses.push_back(other_agents[ii].pose);
    }
  }
  ROS_INFO("dc, distance, close_poses.size(): [%lf, %lf, %d]", dc, distance, close_poses.size());
  return close_poses;
}




double IO_control_collision::psi_col_helper(const geometry_msgs::Point &m_agent, const  geometry_msgs::Point &n_agent){
  geometry_msgs::Vector3 vecij = calc_vec(m_agent,n_agent);
  double val=self_norm(vecij);
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
  if (val <= dc){
    if (val >=ds) {
      // ROS_INFO("The if happened for val - dc / garbage for agent %d", agent_index);
      // !!! WATCH OUT FOR IF YOU'RE USING dc OR dc2 !!!
      output =  ((val - dc)*(val-dc)) / (val - ds + ((ds-dc)*(ds-dc))/mu2);
    } else {
      // ROS_INFO("The mu2 else happened for agent %d", agent_index);
      output = mu2;
    }
  } else {
    // ROS_INFO("The zero else happened for agent %d", agent_index);
    output = 0.0;
  }

  // ROS_INFO("Output was %lf", output);
  return output;
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


  // Testing
  // ROS_INFO("m_agent x,y,z: [%lf, %lf, %lf]", m_agent.position.x, m_agent.position.y, m_agent.position.z);
  // ROS_INFO("n_agent x,y,z: [%lf, %lf, %lf]", n_agent.position.x, n_agent.position.y, n_agent.position.z);

  // ROS_INFO("perturb[0] x,y,z: [%lf, %lf, %lf]", perturb[0].x, perturb[0].y, perturb[0].z);

  geometry_msgs::Vector3 output;
  output.x = -(psi_col_helper(perturb[0],n_agent.position) - psi_col_helper(perturb[1],n_agent.position))/(2*h);
  // double temp_var_plusx = (psi_col_helper(perturb[0],n_agent.position));
  // ROS_INFO("temp_var_plusx is %lf", temp_var_plusx);
  // double temp_var_minusx = psi_col_helper(perturb[1],n_agent.position);
  // ROS_INFO("temp_var_minusx is %lf", temp_var_minusx);

  output.y = -(psi_col_helper(perturb[2],n_agent.position) - psi_col_helper(perturb[3],n_agent.position))/(2*h);
  // double temp_var_plusy = (psi_col_helper(perturb[2],n_agent.position));
  // ROS_INFO("temp_var_plusy is %lf", temp_var_plusy);
  // double temp_var_minusy = psi_col_helper(perturb[3],n_agent.position);
  // ROS_INFO("temp_var_minusy is %lf", temp_var_minusy);

  output.z = -(psi_col_helper(perturb[4],n_agent.position) - psi_col_helper(perturb[5],n_agent.position))/(2*h);
  //std::cout << "Output" << output << std::endl;
  return output;

}




geometry_msgs::Vector3 IO_control_collision::psi_col_gradient(const geometry_msgs::PoseStamped &m_agent, const geometry_msgs::Pose &n_agent){ //this is supposed to only take the state vector
  double h=0.001;
  std::vector<geometry_msgs::Point> perturb;
  for (int i=0; i<6; i++){
    perturb.push_back(m_agent.pose.position);
  }
  perturb[0].x+=h;
  perturb[1].x-=h;
  perturb[2].y+=h;
  perturb[3].y-=h;
  perturb[4].z+=h;
  perturb[5].z-=h;


  // Testing
  // ROS_INFO("m_agent x,y,z: [%lf, %lf, %lf]", m_agent.position.x, m_agent.position.y, m_agent.position.z);
  // ROS_INFO("n_agent x,y,z: [%lf, %lf, %lf]", n_agent.position.x, n_agent.position.y, n_agent.position.z);

  // ROS_INFO("perturb[0] x,y,z: [%lf, %lf, %lf]", perturb[0].x, perturb[0].y, perturb[0].z);

  geometry_msgs::Vector3 output;
  output.x = -(psi_col_helper(perturb[0],n_agent.position) - psi_col_helper(perturb[1],n_agent.position))/(2*h);
  // double temp_var_plusx = (psi_col_helper(perturb[0],n_agent.position));
  // ROS_INFO("temp_var_plusx is %lf", temp_var_plusx);
  // double temp_var_minusx = psi_col_helper(perturb[1],n_agent.position);
  // ROS_INFO("temp_var_minusx is %lf", temp_var_minusx);

  output.y = -(psi_col_helper(perturb[2],n_agent.position) - psi_col_helper(perturb[3],n_agent.position))/(2*h);
  // double temp_var_plusy = (psi_col_helper(perturb[2],n_agent.position));
  // ROS_INFO("temp_var_plusy is %lf", temp_var_plusy);
  // double temp_var_minusy = psi_col_helper(perturb[3],n_agent.position);
  // ROS_INFO("temp_var_minusy is %lf", temp_var_minusy);

  output.z = -(psi_col_helper(perturb[4],n_agent.position) - psi_col_helper(perturb[5],n_agent.position))/(2*h);
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





double IO_control_collision::difference_norm(const geometry_msgs::Pose &v1, const geometry_msgs::Pose &v2){
  double out_double = pow(v1.position.x - v2.position.x,2) + pow(v1.position.y - v2.position.y,2) + pow(v1.position.z - v2.position.z,2);
  return sqrt(out_double);
}

double IO_control_collision::difference_norm(const geometry_msgs::PoseStamped &v1, const geometry_msgs::Pose &v2){
  double out_double = pow(v1.pose.position.x - v2.position.x,2) + pow(v1.pose.position.y - v2.position.y,2) + pow(v1.pose.position.z - v2.position.z,2);
  return sqrt(out_double);
}



// main function: create a IO_control_collision class type that handles everything
int main(int argc, char** argv) {
  ros::init(argc, argv, "IO_control_collision_node");

  IO_control_collision IO_control_collision_node;

  ros::spin();

  return 0;
}
