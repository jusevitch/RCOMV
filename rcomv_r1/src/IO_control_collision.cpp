
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
  nh_private_.param<double>("vmax", vmax, 0.5);
  nh_private_.param<double>("wmax", wmax, 1.0);
  nh_private_.param<double>("Ri", Ri, 0);
  nh_private_.param<double>("alphai", alphai, 0);
  nh_private_.param<double>("mu2", mu2, 100);

  // Initialize the paramters (varying, will be updated by subscribers)
  nh_private_.param<std::string>("path_type", path_type, "NaN");
  //
  // parametric path paramters
  nh_private_.param<double>("t0", t0, ros::Time().toSec());
  nh_private_.param<double>("xc", xc, 0);
  nh_private_.param<double>("yc", yc, 0);
  nh_private_.param<double>("R", R, 1);
  nh_private_.param<double>("wd", wd, 0.0);
  nh_private_.param<double>("phi0", phi0, 0.0); // For circular trajectories. Initial angle of the trajectory that formation starts from.
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

  // Parameter to determine number of obstacles to watch for. HARDWARE ONLY.
  nh_private_.param<int>("number_of_obstacles", number_of_obstacles,0);
  nh_private_.getParam("obstacle_radii", obstacle_radii);


  even_cycle = false;
  //
  odometry_connected = false;
  initial_time = ros::Time().toSec();

  ROS_INFO("number_of_obstacles: %d", number_of_obstacles);
  ROS_INFO("obstacle_radii.size(): %lu", obstacle_radii.size());

  obstacles.resize(number_of_obstacles);
  for(int ii = 0; ii < obstacles.size(); ii++)
  {
    // Set it to some ridiculous number so that the obstacles don't interfere with the agents until their actual position is obtained
    obstacles[ii].pose.pose.position.x = -1000;
    obstacles[ii].pose.pose.position.y = -1000;
    obstacles[ii].pose.pose.position.z = -1000;
    obstacles[ii].r_safety = obstacle_radii[ii]; // Set the safety radius.
  }
  

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
  
  // Obstacle subscriber
  std::string obs_sub_name;
  if (!gazebo){
    ROS_INFO("number_of_obstacles: %d", number_of_obstacles);
    for(int ii = 0; ii < number_of_obstacles; ii++)
    {
      obs_sub_name = "/vicon/obs" + std::to_string(ii) + "/obs" + std::to_string(ii);
      ROS_INFO("obs_sub_name: %s", obs_sub_name.c_str());
      obstacle_subs.push_back(nh.subscribe<geometry_msgs::TransformStamped>(obs_sub_name, 1, boost::bind(&IO_control_collision::vicon_obstacle, this, _1, ii)));
    }
    
  }

}

// destructor
IO_control_collision::~IO_control_collision()
{
  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0;
  pub.publish(cmd_vel);
  ros::shutdown();
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
      ROS_INFO("2: odometry callback worked");
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

for(int ii = 0; ii < number_of_obstacles; ii++)
    {
      std::string obs_sub_name = "/vicon/obs" + std::to_string(ii) + "/obs" + std::to_string(ii);
      ROS_INFO("obs_sub_name: %s", obs_sub_name.c_str());
    }

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
  double xddot, yddot, thetaddot; //// 1st and 2nd derivatives of xd, yd, thetad
  double xdoubledot, ydoubledot; // 2nd derivatives of xd, yd
  double y1d, y2d;
  double vy1d, vy2d;
  double y1, y2;
  double x = state.pose.pose.position.x;
  double y = state.pose.pose.position.y;
  double theta = QuaternionToYaw(state);
  double c_th, s_th;
  double e_y1, e_y2;
  double u1, u2;
  double thetad;
  double c_thd, s_thd;

  // ROS_INFO("gazebo: %d", gazebo ? 1 : 0);

  double t = ros::Time::now().toSec() - t0;
  double thetaf = wd*t + phi0; // Angle which determines the center of formation position. Note that d/dt(thetaf) = wd.

  // the reference states and velocity: circular path
  if (path_type.compare(std::string("circular")) == 0) {
     xd = xc + R*cos(thetaf) + Ri*cos(thetaf+alphai); // xd = x desired
     yd = yc + R*sin(thetaf) + Ri*sin(thetaf+alphai); // yd = y desired
     xddot = -wd*(R*sin(thetaf) + Ri*sin(thetaf + alphai));
     yddot = wd*(R*cos(thetaf) + Ri*sin(thetaf + alphai));
     thetad = atan2(yddot,xddot); // Desired direction

     xdoubledot = -pow(wd,2)*(R*cos(thetaf) + Ri*cos(thetaf + alphai));
     ydoubledot = -pow(wd,2)*(R*sin(thetaf) + Ri*sin(thetaf + alphai));

     thetaddot = xddot/(pow(xddot,2) + pow(yddot,2))*ydoubledot - yddot/(pow(xddot,2) + pow(yddot,2))*xdoubledot;

     vd = hypot(wd*(-R*sin(wd*t+phi0) - Ri*sin(wd*t+phi0+alphai)),
                wd*(R*cos(wd*t) + Ri*cos(wd*t+phi0+alphai))); // Do we use this?? This is the velocity of the position, but not the IO linearization point.
  }
  // the reference states and velocity: eight-shaped path
  if (path_type.compare(std::string("eight_shaped")) == 0) {
     xd = xc + R1*sin(2*wd*t) + Ri*cos(wd*t+alphai);
     yd = yc + R2*sin(wd*t) + Ri*sin(wd*t+alphai);

    xddot = wd*(2*R1*cos(2*wd*t) - Ri*sin(wd*t + alphai)); // First derivative of xd
    yddot = wd*(R2*cos(wd*t) + Ri*cos(wd*t + alphai)); // First derivative of yd
    thetad = atan2(yddot,xddot);

     xdoubledot = -pow(wd,2)*(4*R1*sin(2*wd*t) + Ri*cos(wd*t + alphai)); // Second derivative of xd
     ydoubledot = -pow(wd,2)*(R2*sin(wd*t) + Ri*sin(wd*t+alphai)); // Second derivative of yd
     thetaddot = (xddot / (pow(xddot,2) + pow(yddot,2)))*ydoubledot - (yddot / (pow(xddot,2) + pow(yddot,2)))*xdoubledot;


     //vd = hypot((2*R1*wd*cos(2*wd*t)), (R2*wd*cos(wd*t)));
     vd = hypot((2*wd*R1*cos(2*wd*t) - wd*Ri*sin(theta+alphai)),
                wd*(R2*cos(wd*t) + Ri*cos(theta+alphai)));

  }
  
  
  // the reference states and velocity: cubic polynomial path
  if (path_type.compare(std::string("cubic")) == 0) {
      CubePolyPath(qi, qf, poly_k, T, t, xd, yd, vd, wd);
  }

  // ROS_INFO("wd : %lf", wd);
  // ROS_INFO("agent_index, rover_number: %d, %d", agent_index, rover_number);

  // the reference output

  // ROS_INFO("xd, yd : [%lf, %lf]", xd, yd);
  // ROS_INFO("xc, yc : [%lf, %lf]", xc, yc);

  ROS_INFO("path_type: %s", path_type.c_str());

  if (path_type == "circular") {
    c_thd = cos(thetad);
    s_thd = sin(thetad);
    y1d = xd + b*c_thd;
    y2d = yd + b*s_thd;
    vy1d = xddot - b*s_thd*thetaddot;
    vy2d = yddot + b*c_thd*thetaddot;
  } 

  else if( path_type.compare(std::string("square")) == 0) { // Square path
    
    // Square path without rounded edges
    //
    //    3 ___ 2 ___ 1
    //    |           |
    //    4           0
    //    |           |
    //    5 ___ 6 ___ 7
    //
    //    T = Leng / V

    if (fillet == 0) {

      // modulo the time for infinite looping
      double t_corrected = (t >= 8*T) ? fmod(t, 8*T) : t;

      if(startLIdx == 0) t_corrected+=0;
      else if(startLIdx >= 1 && startLIdx < 3) t_corrected += T;
      else if(startLIdx >= 3 && startLIdx < 5) t_corrected += 3*T;
      else if(startLIdx >= 5 && startLIdx < 7) t_corrected += 5*T;
      else t_corrected += 7*T;

      ROS_INFO("t_corrected: %f", t_corrected );
      // parameters for square
      if ( 0 <= t_corrected && t_corrected < T) {
        ROS_INFO("state1");
        vy1d = 0;
        vy2d = V;
        y1d = Leng;
        y2d = V*t_corrected;
        
      } 
      else if ( T <= t_corrected && t_corrected < 3*T) {
        ROS_INFO("state2");
        vy1d = -V;
        vy2d = 0;
        y1d = Leng - V*(t_corrected-T);
        y2d = Leng;
        
      }
      else if ( 3*T <= t_corrected && t_corrected < 5*T) {
        ROS_INFO("state3");
        vy1d = 0;
        vy2d = -V;
        y1d = -Leng;
        y2d = Leng -V*(t_corrected-3*T);
      }
      else if ( 5*T <= t_corrected && t_corrected < 7*T) {
        ROS_INFO("state4");
        vy1d = V;
        vy2d = 0;
        y1d = -Leng + V*(t_corrected-5*T);
        y2d = -Leng;
      }
      else if ( 7*T <= t_corrected && t_corrected < 8*T) {
        ROS_INFO("state5");
        vy1d = 0;
        vy2d = V;
        y1d = Leng;
        y2d = -Leng + V*(t_corrected-7*T);
      }

      double y1d_temp = y1d * cos(psi) - y2d * sin(psi) + xc;
      double y2d_temp = y1d * sin(psi) + y2d * cos(psi) + yc;
      double vy1d_temp = vy1d * cos(psi) - vy2d * sin(psi);
      double vy2d_temp = vy1d * sin(psi) + vy2d * cos(psi);

      y2d = y2d_temp; y1d = y1d_temp; vy1d = vy1d_temp; vy2d = vy2d_temp;
    }
    

    // Square path with rounded edges
    //
    //      4 ___ 3 ___ 2
    //    5               1
    //    |               |
    //    6               0
    //    |               |
    //    7               11
    //      8 ___ 9 ___ 10
    //
    //    T = (Leng - fillet) / V

    else {
     // modulo the time for infinite looping
      double t_corrected = (t >= 12*T) ? fmod(t, 12*T) : t;
      double thetadelta, pi = M_PI;

      if(startLIdx == 0) t_corrected+=0;
      else if(startLIdx >= 1 && startLIdx < 3) t_corrected += 2*T;
      else if(startLIdx >= 3 && startLIdx < 5) t_corrected += 5*T;
      else if(startLIdx >= 5 && startLIdx < 7) t_corrected += 8*T;
      else t_corrected += 11*T;

     // parameters for square
      if ( 0 <= t_corrected && t_corrected < T) {
        vy1d = 0; vy2d = V;
        y1d = Leng; y2d = V*t_corrected;
      }

      else if ( T <= t_corrected && t_corrected < 2*T) {
        thetadelta = 0 + (t_corrected - T) * (pi/(2*T));
        y1d = (Leng - fillet) + fillet * cos(thetadelta)  ; y2d = (Leng - fillet) + fillet * sin(thetadelta);
        vy1d = V*sin(-thetadelta); vy2d = V*cos(-thetadelta);
      }

      else if ( 2*T <= t_corrected && t_corrected < 4*T) {
        vy1d = -V; vy2d = 0;
        y1d = Leng - V*(t_corrected-2*T); y2d = Leng;
      }

      else if ( 4*T <= t_corrected && t_corrected < 5*T) {
        thetadelta = pi/2 + (t_corrected - 4*T) * (pi/(2*T));
        y1d = (Leng - fillet) + fillet * cos(thetadelta)  ; y2d = (Leng - fillet) + fillet * sin(thetadelta);
        vy1d = V*sin(-thetadelta); vy2d = V*cos(-thetadelta);
      }

      else if ( 5*T <= t_corrected && t_corrected < 7*T) {
        vy1d = 0; vy2d = -V;
        y1d = -Leng; y2d = Leng -V*(t_corrected-5*T);
      }

      else if ( 7*T <= t_corrected && t_corrected < 8*T) {
        thetadelta = pi + (t_corrected - 7*T) * (pi/(2*T));
        y1d = (Leng - fillet) + fillet * cos(thetadelta)  ; y2d = (Leng - fillet) + fillet * sin(thetadelta);
        vy1d = V*sin(-thetadelta); vy2d = V*cos(-thetadelta);
      }

      else if ( 8*T <= t_corrected && t_corrected < 10*T) {
        vy1d = V; vy2d = 0;
        y1d = -Leng + V*(t_corrected-8*T); y2d = -Leng;
      }

      else if ( 10*T <= t_corrected && t_corrected < 11*T) {
        thetadelta = 3*pi/2 + (t_corrected - 10*T) * (pi/(2*T));
        y1d = (Leng - fillet) + fillet * cos(thetadelta)  ; y2d = (Leng - fillet) + fillet * sin(thetadelta);
        vy1d = V*sin(-thetadelta); vy2d = V*cos(-thetadelta);
      }

      else if ( 11*T <= t_corrected && t_corrected < 12*T) {
        vy1d = 0; vy2d = V;
        y1d = Leng; y2d = -Leng + V*(t_corrected-11*T);
      }

      double y1d_temp = y1d * cos(psi) - y2d * sin(psi) + xc;
      double y2d_temp = y1d * sin(psi) + y2d * cos(psi) + yc;
      double vy1d_temp = vy1d * cos(psi) - vy2d * sin(psi);
      double vy2d_temp = vy1d * sin(psi) + vy2d * cos(psi);

      y1d = y1d_temp; y2d = y2d_temp; vy1d = vy1d_temp; vy2d = vy2d_temp;

    }

  }

  ROS_INFO("y1d, y2d: [%lf, %lf]", y1d, y2d);
  ROS_INFO("vy1d, vy2d: [%lf, %lf]", vy1d, vy2d);
  ROS_INFO("psi, cos(psi), sin(psi): [%lf, %lf, %lf]", psi, cos(psi), sin(psi)  );

  // the time derivative of the reference output
  
  // if(path_type.compare(std::string("circular")) == 0) {
  //   vy1d = -wd*(R*sin(wd*t + phi0) + Ri*sin(wd*t + phi0 + alphai) + b*sin(theta_d)); // c_thd * vd - b * s_thd * wd;
  //   vy2d = wd*(R*cos(wd*t) + phi0 + Ri*cos(wd*t + phi0 + alphai) + b*cos(theta_d)); // s_thd * vd + b * c_thd * wd;
  // } else if(path_type.compare(std::string("eight_shaped")) == 0) {
  //   double xdoubledot = -pow(wd,2)*(4*R1*sin(2*wd*t) + Ri*cos(wd*t + alphai)); // Second derivative of xd
  //   double ydoubledot = -pow(wd,2)*(R2*sin(wd*t) + Ri*sin(wd*t+alphai)); // Second derivative of yd
  //   double thetaddot = (xddot / (pow(xddot,2) + pow(yddot,2)))*ydoubledot - (yddot / (pow(xddot,2) + pow(yddot,2)))*xdoubledot;
  //   vy1d = xddot - b*sin(theta_d)*thetaddot;
  //   vy2d = yddot + b*cos(theta_d)*thetaddot;
  // } else 
  

  // ROS_INFO("xd, yd, thetad: [%lf, %lf, %lf]", xd, yd, theta_d);

  if(path_type == "circular" || path_type == "square") { // Add other types later

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

  ROS_INFO("Agent %d", rover_number);
  // ROS_INFO("Gamma, v_coll, w_coll: [%lf, %lf, %lf]", coll_avoid.gamma, coll_avoid.v, coll_avoid.w);
  // ROS_INFO("mu2, k3: [%lf, %lf]", mu2, k3);

  // transform to the actual control inputs for the unicycle model
  cmd_vel.linear.x =  (1.0 - coll_avoid.gamma)*(c_th * u1 + s_th * u2) + coll_avoid.gamma*coll_avoid.v; // 
  cmd_vel.angular.z =  (1.0 - coll_avoid.gamma)*(-s_th/b * u1 + c_th/b * u2) + coll_avoid.gamma*coll_avoid.w; // 

  // saturation
  cmd_vel.linear.x = std::max(-vmax, std::min(cmd_vel.linear.x, vmax));
  cmd_vel.angular.z = std::max(-wmax, std::min(cmd_vel.angular.z, wmax));
  } else {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
  }

  // ROS_INFO("After saturation: v, w: [%lf, %lf]", cmd_vel.linear.x, cmd_vel.angular.z);

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

  // ROS_INFO("state_lists.size(): %d", state_lists.size());
  // ROS_INFO("n: %d", n);
  // ROS_INFO("state_lists.size() == n: %d", (state_lists.size() == n));

  // Collect list of in-neighbors
  if(!state_lists.empty() && state_lists.size() == n){ // Keeps the node from crashing before the list is populated
    // ROS_INFO("FOOOOOOBAAAAARRRR");
    std::vector<geometry_msgs::PoseStamped> all_states = state_lists; // Freezes the state list at a certain time
    geometry_msgs::PoseStamped current_state = all_states[agent_index - 1]; // This agent's current state (pose)
    // ROS_INFO("x,y,z for rover_number %d, agent_index %d: [%lf, %lf, %lf]", rover_number, agent_index,\
      current_state.pose.position.x, current_state.pose.position.y, current_state.pose.position.z);
    // ROS_INFO("current_state x,y,z: [%lf, %lf, %lf]", current_state.position.x, current_state.position.y, current_state.position.z);
    all_states.erase(all_states.begin() + agent_index - 1); // Remove the agent's state from the list
    std::vector<PoseStamped_Radius> collision_states = collision_neighbors(all_states, current_state); 
    std::vector<PoseStamped_Radius> obstacle_collision_states = collision_neighbors(obstacles, current_state);

    // ROS_INFO("# collision agents, # obstacles for agent R%d: [%lu, %lu]", rover_number, collision_states.size(), obstacle_collision_states.size());

    collision_states.insert(collision_states.end(), obstacle_collision_states.begin(), obstacle_collision_states.end());
    // ROS_INFO("Size of collision_states after insertion: %lu", collision_states.size());

    if (!collision_states.empty()){
      
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
        if(difference_norm(current_state,collision_states[j]) < ds + collision_states[j].r_safety){
          // ROS_INFO("FOOBAR TO THE MAX!!!!");
          // Return maximum norm gradient in direction opposite of other agent.
          // This is necessary because value is capped at mu2. If agents are less than ds apart, the gradient will be zero.
          // !!! THE FOLLOWING CODE ONLY WORKS FOR 2D GROUND ROVERS
          double grad_norm = std::abs(2*mu2/(ds - dc) - pow(mu2,2)/pow(ds - dc,2)); // This equation may need to change due to the r_safety changes.
          double grad_angle = std::atan2(current_state.pose.position.y - collision_states[j].pose.pose.position.y,current_state.pose.position.x - collision_states[j].pose.pose.position.x);
          grad_angle = std::fmod(grad_angle + 2*M_PI, 2*M_PI);
          // ROS_INFO("grad_angle: %lf", grad_angle);
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
      if(abs(angle_error) < PI/8.0){
        out_cmd.v = sqrt(pow(psi_collision_sum.x,2) + pow(psi_collision_sum.y,2));
      }

      // Calculate the variable gamma which interpolates between tracking the trajectory and the collision avoidance
      out_cmd.gamma = 0.0;

      for (int jj = 0; jj < collision_states.size(); jj++)
      {
        double xi = current_state.pose.position.x; double yi = current_state.pose.position.y; double zi = current_state.pose.position.z;
        double xj = collision_states[jj].pose.pose.position.x; double yj = collision_states[jj].pose.pose.position.y; double zj = collision_states[jj].pose.pose.position.z;
        double xij = std::sqrt(std::pow(xi - xj,2) + std::pow(yi - yj,2) + std::pow(zi - zj,2));
        // ROS_INFO("xij, r_safety[jj]: [%lf, %lf]", xij, collision_states[jj].r_safety);
        double gamma_jj;
        if(xij > dc + collision_states[jj].r_safety){
          gamma_jj = 0.0;
        } else if(xij < ds + collision_states[jj].r_safety){
          gamma_jj = 1.0;
        } else {
          gamma_jj = 1.0 - (xij - (ds + collision_states[jj].r_safety)) / std::abs(dc - ds); // Convex interpolation between 1 and 0
        }
        // ROS_INFO("gamma_jj: %lf", gamma_jj);
        // ROS_INFO("mu2: %lf", mu2);
        // Ensure out_cmd.gamma is highest of all gamma values
        if(jj == 0 || gamma_jj > out_cmd.gamma){
          out_cmd.gamma = gamma_jj;
        }
      }
      

      // OLD BELOW
      // Find out the smallest Euclidean distance between this agent and any one of the agents in collision_states
      // double min_distance;
      // double min_distance_r_safety;

      // for (int jj=0; jj < collision_states.size(); jj++){
      //   double xi = current_state.pose.position.x; double yi = current_state.pose.position.y; double zi = current_state.pose.position.z;
      //   double xj = collision_states[jj].pose.pose.position.x; double yj = collision_states[jj].pose.pose.position.y; double zj = collision_states[jj].pose.pose.position.z;
      //   double xij = std::sqrt(std::pow(xi - xj,2) + std::pow(yi - yj,2) + std::pow(zi - zj,2));
      //   ROS_INFO("xij, r_safety[jj]: [%lf, %lf]", xij, collision_states[jj].r_safety);
      //   if(jj == 0 || xij < min_distance){
      //     min_distance = xij;
      //     min_distance_r_safety = collision_states[jj].r_safety;
      //   }
      // }
      // ROS_INFO("min_distance, r0: [%lf, %lf]", min_distance, min_distance_r_safety);
      // ROS_INFO("dc + r0, ds + r0: [%lf, %lf]", dc + min_distance_r_safety, ds + min_distance_r_safety);
      // if (min_distance > dc + min_distance_r_safety){
      //   out_cmd.gamma = 0.0;
      // } else if(min_distance < ds + min_distance_r_safety){
      //   out_cmd.gamma = 1.0;
      // } else{
      //   out_cmd.gamma = 1.0 - (min_distance - (ds + min_distance_r_safety)) / std::abs(dc - ds); // Convex interpolation between 1 and 0
      // }
      //  END OLD
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
    t0_q = t0 + msgs->trajectory[0]; // NOTE: ros::Time::now() is different in gazebo.
    xc_q = msgs->trajectory[1];
    yc_q = msgs->trajectory[2];
    R_q = msgs->trajectory[3];
    wd_q = msgs->trajectory[4];
    phi0_q = msgs->trajectory[5];
    // ROS_INFO("ms_rpa callback worked");
  }

  if(msgs->type.compare("square") == 0){
    type_q = msgs->type;
    t0_q = t0 + msgs->trajectory[0];
    xc_q = msgs->trajectory[1];
    yc_q = msgs->trajectory[2];
    Leng_q = msgs->trajectory[3];
    psi_q = msgs->trajectory[4];
    V_q = msgs->trajectory[5];
    startLIdx_q = msgs->trajectory[6];
    T_q = Leng_q / V_q;
    fillet_q = msgs->trajectory[7];
  }
}


void IO_control_collision::vicon_obstacle(const geometry_msgs::TransformStamped::ConstPtr& msgs, const int ii){
  obstacles[ii].pose.header = msgs->header;
  obstacles[ii].pose.pose.position.x = msgs->transform.translation.x;
  obstacles[ii].pose.pose.position.y = msgs->transform.translation.y;
  obstacles[ii].pose.pose.position.z = msgs->transform.translation.z;
  obstacles[ii].pose.pose.orientation = msgs->transform.rotation;

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
      // ROS_INFO("Parameters switched: \n t0, xc, yc, R, wd, phi0: [%lf,%lf, %lf, %lf, %lf, %lf]", t0, xc, yc, R, wd, phi0);
    }

    if(type_q.compare("square") == 0){
      path_type = type_q;
      t0 = t0_q;
      xc = xc_q;
      yc = yc_q;
      Leng = Leng_q;
      psi = psi_q;
      V = V_q;
      startLIdx = startLIdx_q;
      T = T_q;
      fillet = fillet_q;
    }
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
  // ROS_INFO("dc, distance, close_poses.size(): [%lf, %lf, %d]", dc, distance, close_poses.size());
  return close_poses;
}

// Overloaded for PoseStamped
// Helper function: calculate neighbors within dc of the agent
std::vector<PoseStamped_Radius> IO_control_collision::collision_neighbors(const std::vector<geometry_msgs::PoseStamped> &other_agents, const geometry_msgs::PoseStamped &current_state){
  double distance = 0.0;
  std::vector<PoseStamped_Radius> close_poses;
  PoseStamped_Radius temp_PSR;
  // ROS_INFO("other_agents.size() for overload: %d", other_agents.size());
  for(int ii=0; ii < other_agents.size(); ii++){
    distance = std::sqrt(std::pow(current_state.pose.position.x - other_agents[ii].pose.position.x,2) +\
      std::pow(current_state.pose.position.y - other_agents[ii].pose.position.y,2) + std::pow(current_state.pose.position.z - other_agents[ii].pose.position.z,2));
    // ROS_INFO("other_agents.x, otheragents.y, other agents.z: [%lf, %lf, %lf], \n \
    current_state.x, current_state.y, current_state.z: [%lf, %lf, %lf]",\
    other_agents[ii].pose.position.x, other_agents[ii].pose.position.y, other_agents[ii].pose.position.z,\
    current_state.pose.position.x, current_state.pose.position.y, current_state.pose.position.z);
    // ROS_INFO("distance, ")
    if(distance < dc + ds){ // r_safety for all agents is equal to ds
      // Save the close poses
      temp_PSR.pose = other_agents[ii];
      temp_PSR.r_safety = ds; // All UGVs have same safety radius
      close_poses.push_back(temp_PSR);
    }
  }
  // ROS_INFO("dc, distance, close_poses.size(): [%lf, %lf, %d]", dc, distance, close_poses.size());
  return close_poses;
}

// Overloaded for parsing obstacles
std::vector<PoseStamped_Radius> IO_control_collision::collision_neighbors(const std::vector<PoseStamped_Radius> &obstacle_vector, const geometry_msgs::PoseStamped &current_state){
  double distance = 0.0;
  std::vector<PoseStamped_Radius> close_poses;
  for(int ii=0; ii < obstacle_vector.size(); ii++){
    distance = std::sqrt(std::pow(current_state.pose.position.x - obstacle_vector[ii].pose.pose.position.x,2) +\
      std::pow(current_state.pose.position.y - obstacle_vector[ii].pose.pose.position.y,2) + std::pow(current_state.pose.position.z - obstacle_vector[ii].pose.pose.position.z,2)); 
      // ROS_INFO("distance, r_safety: [%lf, %lf]", distance, obstacle_vector[ii].r_safety);   
    if(distance < dc + obstacle_vector[ii].r_safety){
      // Save the close poses
      close_poses.push_back(obstacle_vector[ii]);
    }
  }
  // ROS_INFO("dc, distance, close_poses.size(): [%lf, %lf, %d]", dc, distance, close_poses.size());
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

// Overloaded to take PoseStamped_Radius arguments
double IO_control_collision::psi_col_helper(const geometry_msgs::Point &m_agent, const PoseStamped_Radius &n_agent){
  geometry_msgs::Vector3 vecij = calc_vec(m_agent,n_agent.pose.pose.position);
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
  if (val <= dc + n_agent.r_safety){
    if (val >= ds + n_agent.r_safety) {
      // ROS_INFO("The if happened for val - dc / garbage for agent %d", agent_index);
      // !!! WATCH OUT FOR IF YOU'RE USING dc OR dc2 !!!
      output =  pow(val - (dc + n_agent.r_safety),2) / (val - (ds + n_agent.r_safety) + ((ds-dc)*(ds-dc))/mu2);
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

// Overloaded for PoseStamped_Radius
geometry_msgs::Vector3 IO_control_collision::psi_col_gradient(const geometry_msgs::PoseStamped &m_agent, const PoseStamped_Radius &n_agent){ //this is supposed to only take the state vector
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
  output.x = -(psi_col_helper(perturb[0],n_agent) - psi_col_helper(perturb[1],n_agent))/(2*h);
  // double temp_var_plusx = (psi_col_helper(perturb[0],n_agent.position));
  // ROS_INFO("temp_var_plusx is %lf", temp_var_plusx);
  // double temp_var_minusx = psi_col_helper(perturb[1],n_agent.position);
  // ROS_INFO("temp_var_minusx is %lf", temp_var_minusx);

  output.y = -(psi_col_helper(perturb[2],n_agent) - psi_col_helper(perturb[3],n_agent))/(2*h);
  // double temp_var_plusy = (psi_col_helper(perturb[2],n_agent.position));
  // ROS_INFO("temp_var_plusy is %lf", temp_var_plusy);
  // double temp_var_minusy = psi_col_helper(perturb[3],n_agent.position);
  // ROS_INFO("temp_var_minusy is %lf", temp_var_minusy);

  output.z = -(psi_col_helper(perturb[4],n_agent) - psi_col_helper(perturb[5],n_agent))/(2*h);
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

double IO_control_collision::difference_norm(const geometry_msgs::PoseStamped &v1, const PoseStamped_Radius &vPSR){
  double out_double = pow(v1.pose.position.x - vPSR.pose.pose.position.x,2) +\
                      pow(v1.pose.position.y - vPSR.pose.pose.position.y,2) +\
                      pow(v1.pose.position.z - vPSR.pose.pose.position.z,2);
  return sqrt(out_double);
}


// main function: create a IO_control_collision class type that handles everything
int main(int argc, char** argv) {
  ros::init(argc, argv, "IO_control_collision_node");

  IO_control_collision IO_control_collision_node;

  ros::spin();

  return 0;
}
