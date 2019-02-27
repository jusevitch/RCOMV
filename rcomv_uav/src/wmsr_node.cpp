

#include "wmsr_node.h"

// Constructor
WMSRNode::WMSRNode()
  :nh_private_("~")
  {
    // Initialize variables assigned in the launch file
    nh_private_.param<int>("n", n, 15);
    nh_private_.param<int>("k", k, 7);

    nh_private_.param<int>("idx", idx, 1);
    nh_private_.param<int>("role", role, 2);

    nh_private_.param<int>("F", F, 0);

    nh_private_.param<double>("x", x0, 0);
    nh_private_.param<double>("y", y0, 0);
    nh_private_.param<double>("z", z0, 0);

    nh_private_.param<int>("demo", demo, 2);

    nh_private_.param<double>("cx", cx, 6);
    nh_private_.param<double>("cy", cy, 6);
    nh_private_.param<double>("cz", cz, 10);

    nh_private_.param<float>("rc", rc, 60);
    nh_private_.param<float>("rp", rp, 20);

    // Initialize msgs
    inform_states.header.stamp = ros::Time::now();
    if (demo == 3 && role == 3) {
      inform_states.point.x = cx; // Change "inform_state" to "info_state"
      inform_states.point.y = cy;
      inform_states.point.z = cz;
    }
    else {
      inform_states.point.x = x0;
      inform_states.point.y = y0;
      inform_states.point.z = z0;
    }

    mali_states.header.stamp = ros::Time::now();
    mali_states.point.x = x0;
    mali_states.point.y = y0;
    mali_states.point.z = z0;

    G.resize(n);

    // Publisher: output
    output_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
              mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
    // Publisher Timer
    // frequency: 10 Hz
    out_pub_timer = nh.createTimer(ros::Duration(0.1),
                &WMSRNode::out_pubCallback, this);


    // Subscriber: subscribe the switch topic
    switch_sub = nh.subscribe("/switch", 10, &WMSRNode::switch_subCallback, this);
    // wait wait and wait!!!!!!!
    while(switch_signal.data == false) {
      // wait
      ros::spinOnce();
      ROS_INFO("Wait until all robots are spawned, stay at initial position");

      const float DEG_2_RAD = M_PI / 180.0;
      Eigen::Vector3d waypoint_position(x0, y0, z0);
      double desired_yaw = 0 * DEG_2_RAD;

      trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
      trajectory_msg.header.stamp = ros::Time::now();
      mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
        waypoint_position, desired_yaw, &trajectory_msg);
      //
       output_pub.publish(trajectory_msg);


      ros::Duration(0.1).sleep();
    }
    //
    ROS_INFO("Simulation is ready, turn on WMSR node...");




    // Publisher: reference
    //std::string pub_topic = "WMSR" + std::to_string(idx) + "/ref";
    std::string pub_topic = "WMSR/ref";     // pub topic is relative to the node namespace
    ref_pub = nh.advertise<ref_msgs>(pub_topic, 10);
    // Publisher Timer
    // frequency: 10 Hz
    ref_pub_timer = nh.createTimer(ros::Duration(0.1),
                &WMSRNode::ref_pubCallback, this);


    // Subscribers: (and msgs list to hold the msgs from subscibed topics)
    for (int i = 1; i <= k; i++) {
      // Initialize msgs list that holds msgs from neighbor agents
      ref_msgs ref_point;
      ref_lists.push_back(ref_point);

      // Initialize subscriber
      int sub_idx = (idx - i) > 0 ? (idx - i) : (n + idx -i);
      // sub topics are resolved in global namespace
      std::string sub_topic = "/uav" + std::to_string(sub_idx) + "/WMSR/ref";


      ref_subs.push_back(nh.subscribe<ref_msgs>(sub_topic, 10,
                         boost::bind(&WMSRNode::ref_subCallback, this, _1, i-1)) );

      ROS_INFO("sub_idx at: [%d] with topic name: ", sub_idx);

      state_msgs obs_state;
      state_lists.push_back(obs_state);

      std::string sub2_topic = "/uav" + std::to_string(sub_idx) + "/odom";
      states_subs.push_back(nh.subscribe<state_msgs>(sub2_topic, 10, boost::bind(&WMSRNode::state_subCallback, this, _1, i-1)) ) ;

      
    }

      ROS_INFO_STREAM("Started uav"<<idx<<" WMSR Node.");
  }


// Destructor
WMSRNode::~WMSRNode()
{
  ros::shutdown();
}

// switch features !!!!!
void WMSRNode::switch_subCallback(const std_msgs::Bool::ConstPtr& msg){
  switch_signal.data = msg->data;
}

// Subscriber Callback Function
void WMSRNode::ref_subCallback(const ref_msgs::ConstPtr& msgs, const int list_idx)
{
  ref_lists[list_idx].header = msgs->header;
  ref_lists[list_idx].point = msgs->point;
}

void WMSRNode::state_subCallback(const state_msgs::ConstPtr& msgs, const int list_idx){
  state_lists[list_idx].pose=msgs->pose;
}

// Reference Publisher Callback
void WMSRNode::ref_pubCallback(const ros::TimerEvent& event)
{
  // Role: 1 = Malicious, 2 wx= Normal, 3 = Leader
  // Leader node
  // Static Formation: node states are static
  if (role == 3) {
      inform_states.header.stamp = ros::Time::now();
      ref_pub.publish(inform_states);    // publish reference states to other WMSR node
  }
  // Normal node
  // implement WMSR algorithm to update node states
  else if (role == 2) {
    inform_states = WMSRNode::WMSRAlgorithm(ref_lists);
    ref_pub.publish(inform_states);    // publish reference states to other WMSR node
  }
  // Malicious node
  // implement WMSR algorithm to calculate a reference states
  // If it's cyber attack, update node states to reference states, but publish a malicious state
  // If it's physical attack, update node states to a malicious states
  else {
    // cyber attack
    //inform_states = WMSRNode::WMSRAlgorithm(ref_lists);

    mali_states.header.stamp = ros::Time::now();
    mali_states.point.x += 0.1;
    mali_states.point.y += 0.1;
    mali_states.point.z = std::max(0.0, mali_states.point.z-1);

    // physical attack
    inform_states = mali_states;

    ref_pub.publish(mali_states);    // publish reference states to other WMSR node
  }

}

// Output Publisher Callback
void WMSRNode::out_pubCallback(const ros::TimerEvent& event)
{
  // calculate the node states in the formation
  WMSRNode::Formation();

  //wx
  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;

  double x, y, z;
  // 1D motion in y direction
  if (demo == 1) {
    x = x0;
    y = inform_formation_states.point.y;
    z = z0;
  }
  // 1D motion in z direction
  else if (demo == 2) {
    x = x0;
    y = y0;
    z = inform_formation_states.point.z;
  }
  // 3D motion
  else {
    x = inform_formation_states.point.x;
    y = inform_formation_states.point.y;
    z = inform_formation_states.point.z;
  }

  const float DEG_2_RAD = M_PI / 180.0;

  Eigen::Vector3d waypoint_position(x, y, z);
  double desired_yaw = 0 * DEG_2_RAD;

  trajectory_msg.header.stamp = ros::Time::now();
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
    waypoint_position, desired_yaw, &trajectory_msg);

  //
   output_pub.publish(trajectory_msg);
}

// Helper Function::
ref_msgs WMSRNode::WMSRAlgorithm(const std::vector<ref_msgs> &list)
{
  ref_msgs ref_states;

  std::vector<double> listx(k,0);
  std::vector<double> listy(k,0);
  std::vector<double> listz(k,0);

  double wx, wy, wz; // uniform weights

  double wav_x = 0, wav_y = 0, wav_z = 0; // weighted averages

  double x, y, z;
  x = inform_states.point.x;
  y = inform_states.point.y;
  z = inform_states.point.z;

  // scan the data
  for (int i = 0; i < k; i++) {
    listx[i] = list[i].point.x;
    listy[i] = list[i].point.y;
    listz[i] = list[i].point.z;
  }

  //remove outliers
  wx = FilterOutlier(listx, k, x, F);
  wy = FilterOutlier(listy, k, y, F);
  wz = FilterOutlier(listz, k, z, F);

  // update node states
  // note: size increases by 1 due to the added node value
  for (int j = 0; j < k+1; j++) {
    wav_x += wx * listx[j];
    wav_y += wy * listy[j];
    wav_z += wz * listz[j];
  }

  ref_states.header.stamp = ros::Time::now();
  ref_states.point.x = wav_x;
  ref_states.point.y = wav_y;
  ref_states.point.z = wav_z;

  return ref_states;
}

// Helper Function: Create formation
void WMSRNode::Formation()
{
  // 1D motion
  if (demo == 1 || demo == 2) {
    inform_formation_states.point = inform_states.point;
    inform_formation_states.header.stamp = ros::Time::now();
  }
  // 3D motion: star shape formation
  else {
    const float DEG_2_RAD = M_PI / 180.0;
    double d_theta = 360 / 5 * DEG_2_RAD;
    double d_radius = 1;
    int group_size = n / 5;

    double theta = (idx-1) / group_size * d_theta;
    double radius = (idx-1) % group_size * d_radius + 2;

    inform_formation_states.point.x = inform_states.point.x + radius * cos(theta);
    inform_formation_states.point.y = inform_states.point.y + radius * sin(theta);
    inform_formation_states.point.z = inform_states.point.z;
    inform_formation_states.header.stamp = ros::Time::now();
  }
}

// Helper Function: remove outlier entries
double FilterOutlier(std::vector<double> &list, const int k, const double inform_state, const int F)
{
  int num_h = 0, num_l = 0;
  int valid_size = k;

  // record the number of values
  // that are greater or less than the current node value
  for (int i = 0; i < k; i++) {
    if (list[i] > inform_state)   num_h++;
    if (list[i] < inform_state)   num_l++;
  }

  // sort the list in ascending order
  std::sort(list.begin(), list.end());

  // if there are F or more values greater (less than) the state value, remove
  // the first F states greater (less than) the state value.
  // Otherwise, remove all states greater (less than) the state value.
  if (num_h > F) {
    valid_size -= F;
    for (int j = k-1; j >= k-F; j--)
      list[j] = 0;
  }
  else {
    valid_size -= num_h;
    for (int j = k-1; j >= k-num_h; j--)
      list[j] = 0;
  }

  if (num_l > F) {
    valid_size -= F;
    for (int j = 0; j <= F-1; j++)
      list[j] = 0;
  }
  else {
    valid_size -= num_l;
    for (int j = 0; j <= num_l-1; j++)
      list[j] = 0;
  }

  // append the current node value to the remaining list
  list.push_back(inform_state);
  valid_size++;

  return 1.0 / double(valid_size);
}
double WMSRNode::calculate_norm(const state_msgs &state1, const state_msgs &state2){
  double val;
  double xs=state1.pose.pose.position.x - state2.pose.pose.position.x;
  double ys=state1.pose.pose.position.y - state2.pose.pose.position.y;
  double zs=state1.pose.pose.position.z - state2.pose.pose.position.z;
  val = sqrt( (xs*xs)+ (ys*ys) + (zs*zs));
  return val;
}
double WMSRNode::self_norm(const tiny_msgs &tiny){
  double val;
  val = sqrt((tiny.x*tiny.x) + (tiny.y*tiny.y) + (tiny.z*tiny.z));
  return val;
}

std::vector<Matrix> WMSRNode::Calc_Adjacency(const std::vector<state_msgs> &state_lists, std::vector<Matrix> &G, float rc, float rp, int n){
  G.resize(2);
  G.at(0).resize(n,std::vector<int>(n));
  G.at(1).resize(n,std::vector<int>(n));

  double val;
  for (int i=0; i<n; i++){
    for (int j=0; j<n; j++){
      val=WMSRNode::calculate_norm(state_lists.at(i),state_lists.at(j));
      if (val<rc)
	G[0][i][j]=1;
      if (val<rp)
	G[1][i][j]=1;
    }
  }
  return G;
}

std::vector<int> WMSRNode::get_in_neighbours(const Matrix &Q, int agent){
  auto agents_no = Q.size();
  std::vector<int> neighbours;
  for (int i=0; i<agents_no; i++){
    if (Q[agent][i] == 1)
	  neighbours.push_back(i);
  }
  return neighbours;
}

tiny_msgs WMSRNode::calc_vec(const tiny_msgs &state1, const tiny_msgs &state2){
  tiny_msgs tiny;
  tiny.x = state1.x - state2.x;
  tiny.y = state1.y - state2.y;
  tiny.z = state1.z - state2.z;
  return tiny;
}

void WMSRNode::populate_state_vector(){
  for(int i=0; i < state_lists.size(); i++){
    swarm_odom[i].x = state_lists[i].pose.pose.position.x - tau[0][i];
    swarm_odom[i].y = state_lists[i].pose.pose.position.y - tau[1][i];
    swarm_odom[i].z = state_lists[i].pose.pose.position.z - tau[2][i];
  }
}

void WMSRNode::save_state_vector(){
  for(int i=0; i < swarm_odom.size(); i++){
    prev_odom[i].x = swarm_odom[i].x - tau[0][i];
    prev_odom[i].y = swarm_odom[i].y - tau[1][i];
    prev_odom[i].z = swarm_odom[i].z - tau[2][i];
  }
}

void WMSRNode::make_tau_vector(){
  tau.resize(3);
  tau.at(0).resize(n);
  tau.at(1).resize(n);
  tau.at(2).resize(n);
  double res = 2*3.14/n;
  for(int i=0; i < n; i++){
    double ang=i*res;
    tau[0][i]=5*std::cos(ang)*std::cos(ang);
    tau[1][i]=5*std::cos(ang)*std::sin(ang);
    tau[2][i]=5*std::sin(ang);
  }
}


float WMSRNode::psi_helper(const tiny_msgs &m_agent, const tiny_msgs &n_agent, const tiny_msgs &tau_ij){
  float mu=1000;
  float output;
  tiny_msgs tiny=WMSRNode::calc_vec(m_agent,n_agent);
  float rshat = rp - WMSRNode::self_norm(tau_ij);
  output = WMSRNode::self_norm(tiny) / (rshat - WMSRNode::self_norm(tiny) + (rshat*rshat)/mu);
  return output;
  
}

tiny_msgs WMSRNode::add_vectors(tiny_msgs &a, tiny_msgs &b){
  tiny_msgs result;
  result.x = a.x + b.x;
  result.y = a.y + b.y;
  result.z = a.z + b.z;
  return result;
}
tiny_msgs WMSRNode::psi_gradient(int m_agent, int n_agent, const tiny_msgs &tau_ij){
  //use rp
  float h=0.001;
  std::vector<tiny_msgs> perturb(6);
  for (int i; i<6; i++){
    perturb.push_back(swarm_odom[m_agent]);
  }
  perturb[0].x+=h;
  perturb[1].x-=h;
  perturb[2].y+=h;
  perturb[2].y-=h;
  perturb[3].z+=h;
  perturb[3].z-=h;
 
  tiny_msgs output;
  output.x = (WMSRNode::psi_helper(perturb[0],swarm_odom[n_agent],tau_ij) - WMSRNode::psi_helper(perturb[1],swarm_odom[n_agent],tau_ij))/(2*h);
  output.y = (WMSRNode::psi_helper(perturb[2],swarm_odom[n_agent],tau_ij) - WMSRNode::psi_helper(perturb[3],swarm_odom[n_agent],tau_ij))/(2*h);
  output.z = (WMSRNode::psi_helper(perturb[4],swarm_odom[n_agent],tau_ij) - WMSRNode::psi_helper(perturb[5],swarm_odom[n_agent],tau_ij))/(2*h);
  return output;
  
}

void WMSRNode::populate_velocity_vector(std::vector<tiny_msgs> &yidot){
  for (int i=0; i<n;i++){
    yidot.push_back(calc_vec(swarm_odom[i],prev_odom[i]));
  }
}

std::vector<Neigh> WMSRNode::multiply_vectors(const std::vector<tiny_msgs> &vec1, const std::vector<tiny_msgs> &vec2, const std::vector<int> neigh){
  std::vector<Neigh> output;
  Neigh sample;
  for (int i=0; i<vec1.size(); i++){
    sample.val=(vec1[i].x*vec2[i].x) + (vec1[i].y*vec2[i].y) + (vec1[i].z*vec2[i].z);
    sample.id=neigh[i];
    output.push_back(sample);
  }
  return output;
}

tiny_msgs WMSRNode::multiply_scalar_vec(const float gain, const std::vector<tiny_msgs> &vec){
   tiny_msgs result;
   result.x = gain*vec.x;
   result.y = gain*vec.y;
   result.z = gain*vec.z;
   return result;
}

NLists WMSRNode::velocity_filter(int i,const std::vector<tiny_msgs> &yidot){
    std::vector<int> neigh_list;
    neigh_list=get_in_neighbours(G.at(0), i);
    std::vector<tiny_msgs> grad_vector;
    std::vector<tiny_msgs> diff_vector;
    std::vector<Neigh> vel_grad;
    NLists nlist;
    if (!neigh_list.empty()){
        for (int j=0; j<neigh_list.size(); j++){
	     tiny_msgs tau_ij = calc_vec(tau[i],tau[neigh_list[j]]); //calculating tauij 
             grad_vector.push_back(psi_gradient(i,neigh_list[j],tau_ij));	
             diff_vector.push_back(calc_vec(yidot[i],yidot[neigh_list[j]]));
         }
        vel_grad = WMSRNode::multiply_vectors(grad_vector,diff_vector,neigh_list);

        //sort in_neighbours with the vel_grad vector
        std::sort(vel_grad.begin(), vel_grad.end(), 
                  [](const Neigh &i, const Neigh &j) { return i.val > j.val; } );
        if (F<vel_grad.size()){
            for(int k=0; k<vel_grad.size();k++){
	        if (k<F)
	        nlist.f_neigh.push_back(vel_grad[k].id);
 	        else
	        nlist.u_neigh.push_back(vel_grad[k].id);
            }
        }
        else{
            for (int k=0; k<vel_grad.size();k++){
	        nlist.f_neigh.push_back(vel_grad[k].id);
            }
        }
    }
}

void WMSRNode::filtered_barrier_function(int iteration){
  if (iteration!=0){
    WMSRNode::save_state_vector();
    WMSRNode::populate_state_vector();
  }
  else {
    WMSRNode::make_tau_vector();
    WMSRNode::save_state_vector();
  }
  auto agents_no = swarm_odom.size();
  barrier_out.resize(agents_no);
  std::vector<tiny_msgs> yidot;
  WMSRNode::populate_velocity_vector(yidot);
  for (int i=0; i<agents_no; i++){// this is where the filter_function starts

    NLists nlist;
    nlist=velocity_filter(i, yidot);
    tiny_msgs psi_gradient_sum;
    psi_gradient_sum.x=0; psi_gradient_sum.y=0; psi_gradient_sum.z=0;
    if (!nlist.u_neigh.empty()){
	for (int j=0; j<nlist.u_neigh.size(); j++){
	  tiny_msgs tau_ij = calc_vec(tau[i],tau[neigh_list[j]]);
	  psi_gradient_sum=add_vectors(psi_gradient_sum,psi_gradient(i,nlist.u_neigh[j],tau_ij));
        }
	
    }


    //Aprox_filtered_i = Aprox(ii,:);
    //Aprox_filtered_i(1,filtered_list) = zeros(1,length(filtered_list));
    //Lprox_filtered_i = -Aprox_filtered_i;
    //Lprox_filtered_i(1,ii) = sum(Aprox_filtered_i);
    
    float gain = -100.0; //% Makes barrier function converge faster.
    barrier_out[i] = multiply_scalar_vec(gain,psi_gradient_sum); 
		    //outvector(ii*2-1:ii*2,1) = -Psi_gradient_sum*gain; % Works for gradient filtering. No Laplacian Term -- converges, but really slow
    
    //if norm(outvector(ii*2-1:ii*2,1),2) >= 50
    //    outvector(ii*2-1:ii*2,1) = outvector(ii*2-1:ii*2,1)/norm(outvector(ii*2-1:ii*2,1),2)*20;
    //end
    if (self_norm(barrier_out[i]) >=50){
      barrier_out[i] = multiply_scalar_vec(20.00f / self_norm(barrier_out[i]), barrier_out[i]);
    }
    
    //if isfield(args,'misbehaving_agents')
    //    misbehaving_agents = args.misbehaving_agents;
    //else
    //    misbehaving_agents = [];
    //end
    
    //for ii=misbehaving_agents    
    //    outvector(ii*2-1:ii*2,1) = [0,80*cos(args.tt/20 + 2)];    

   
  }

  
}

// main function
int main(int argc, char **argv) {

  // Initialize ros
  ros::init(argc, argv, "WMSR_Node");

  // Create an object of WMSRNode that handles everything
  WMSRNode WMSR_Node;

  ros::spin();

  return 0;
}
