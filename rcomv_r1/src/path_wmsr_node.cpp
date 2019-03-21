/*
// ------------------------------------------------------------------
// WMSR node for r1 rover. 2D motions only.
// Reach an agreement on parameters of the refrence trajectory
//------------------------------------------------------------------
*/

#include "path_wmsr_node.h"

// Constructor
WMSRNode::WMSRNode()
  :nh_private_("~")
  {
    // ---------------------------- variables and topic messages ------------------------------
    // Initialize variables assigned in the launch file
    // syntax: ("variable name in the launch file", variable to be assigned, default value)
    //
    nh_private_.param<int>("n", n, 15);
    nh_private_.param<int>("k", k, 7);
    // index and type of agent
    nh_private_.param<int>("idx", idx, 1);
    nh_private_.param<int>("role", role, 2);
    // num of adversaries
    nh_private_.param<int>("F", F, 0);
    // type of attack
    nh_private_.param<int>("attack", attack, 1);
    // inital pose
    nh_private_.param<double>("x", x0, 0);
    nh_private_.param<double>("y", y0, 0);
    nh_private_.param<double>("theta", y0, 0);

    nh_private_.param<float>("rc", rc, 60);
    nh_private_.param<float>("rp", rp, 20);

    nh_private_.param<float>("ds", ds, 2);
    nh_private_.param<float>("dc", dc, 5);

    //------------------------------ More initializations ----------------------------------
    role_list[idx]=role;
    umax=50.0;
    state_lists.resize(n);
    make_tau_vector();


    // Initialize msgs
    // refrence path:
    // 1. for leader agents, they will be assigned with desired values fron the launch file
    // 2. for other agents, they will be assigned with some random offset fron the launch file
    nh_private_.param<std::string>("path_type", inform_center_path.path_type, "cubic");
    // inital location of the reference path
    nh_private_.param<double>("qi_x", inform_center_path.qi_x, 0);
    nh_private_.param<double>("qi_y", inform_center_path.qi_y, 0);
    nh_private_.param<double>("qi_theta", inform_center_path.qi_theta, M_PI*0.5);
    // final location of the reference path
    nh_private_.param<double>("qf_x", inform_center_path.qf_x, 10);
    nh_private_.param<double>("qf_y", inform_center_path.qf_y, 0);
    nh_private_.param<double>("qf_theta", inform_center_path.qf_theta, M_PI*-0.5);
    nh_private_.param<double>("T", inform_center_path.T, 15);
    nh_private_.param<double>("poly_k", inform_center_path.poly_k, 40);
    // 3. malicous path: assign a different final location
    mali_path.qf_x = inform_center_path.qf_x * 2;
    mali_path.qf_y = inform_center_path.qf_y * 2;
    mali_path.poly_k = inform_center_path.poly_k * 2;

    // ------------------------- Subscribers and Publishers -----------------------------
    // Publisher: command trajectory for the controller
    output_pub = nh.advertise<path_msgs>(
                  "ref", 10);
    // Publisher Timer with frequency: 10 Hz
    out_pub_timer = nh.createTimer(ros::Duration(0.1),
              &WMSRNode::out_pubCallback, this);

    // Subscriber: subscribe the switch topic
    switch_sub = nh.subscribe("/switch", 10, &WMSRNode::switch_subCallback, this);
    // wait until receiving the turn on signal
    while(switch_signal.data == false) {

    ros::spinOnce();
    ROS_INFO("Wait until all robots are spawned, stay at initial positions.");

    // make the robot stay at the initial position.
    path_msgs trajectory_msg;
    trajectory_msg.qi_x = x0; trajectory_msg.qi_y = y0; trajectory_msg.qi_theta = theta0;
    trajectory_msg.qf_x = x0; trajectory_msg.qf_y = y0; trajectory_msg.qf_theta = theta0;
    trajectory_msg.poly_k = 0;
    trajectory_msg.T = 1000;
    output_pub.publish(trajectory_msg);

    ros::Duration(0.1).sleep();
    }
    ROS_INFO("Simulation is ready, turn on WMSR node...");

    for (int i=1; i<=n+1; i++){
      std::string sub3_topic = "/ugv" + std::to_string(idx) + "/odom";
      states_subs.push_back(nh.subscribe<state_msgs>(sub3_topic, 10, boost::bind(&WMSRNode::state_subCallback, this, _1, i)) ) ;
    }
    filtered_barrier_collision(iteration,idx);




    //Test barrier functions
    std::string pubtopic = "/ugv" + std::to_string(idx) + "/barrier";

    new_pub=nh.advertise<tiny_msgs>(pubtopic,10);
    new_pub_timer = nh.createTimer(ros::Duration(0.1),
				   &WMSRNode::new_pubCallback,this);



    // Publisher: reference
    // pub topic is relative to the node namespace
    std::string pub_topic = "WMSR/ref";
    ref_pub = nh.advertise<path_msgs>(pub_topic, 10);
    // Publisher Timer with frequency: 10 Hz
    ref_pub_timer = nh.createTimer(ros::Duration(0.1),
                &WMSRNode::ref_pubCallback, this);


    // Subscribers: (msgs list holds the msgs from subscibed topics)
    for (int i = 1; i <= k; i++) {
      // Initialize the msgs list that holds msgs from neighbor agents
      path_msgs ref_point;
      ref_lists.push_back(ref_point);

      // Initialize subscriber
      int sub_idx = (idx - i) > 0 ? (idx - i) : (n + idx -i);
      // sub topics are resolved in global namespace
      std::string sub_topic = "/ugv" + std::to_string(sub_idx) + "/WMSR/ref";

      ref_subs.push_back(nh.subscribe<path_msgs>(sub_topic, 10,
                    boost::bind(&WMSRNode::ref_subCallback, this, _1, i-1)) );

      ROS_INFO("sub_idx at: [%d] with topic name: ", sub_idx);
    }
    ROS_INFO_STREAM("Started ugv"<<idx<<" WMSR Node.");
  }

  // Destructor
WMSRNode::~WMSRNode()
{
  ros::shutdown();
}

// Switch signal Subscriber Callback Function
void WMSRNode::switch_subCallback(const std_msgs::Bool::ConstPtr& msg){
  switch_signal.data = msg->data;
}

void WMSRNode::state_subCallback(const state_msgs::ConstPtr& msgs, const int list_idx){
  state_lists[list_idx].position=msgs->pose.pose.position;
  state_lists[list_idx].orientation=msgs->pose.pose.orientation;
}

void WMSRNode::new_pubCallback(const ros::TimerEvent& event){
  new_pub.publish(barrier_out);
    
}

//  Subscriber Callback Function: subscribes reference paths of other WMSR nodes
void WMSRNode::ref_subCallback(const path_msgs::ConstPtr& msgs, const int list_idx)
{
  ref_lists[list_idx].path_type = msgs->path_type;
  ref_lists[list_idx].qi_x = msgs->qi_x;
  ref_lists[list_idx].qi_y = msgs->qi_y;
  ref_lists[list_idx].qi_theta = msgs->qi_theta;
  ref_lists[list_idx].qf_x = msgs->qf_x;
  ref_lists[list_idx].qf_y = msgs->qf_y;
  ref_lists[list_idx].qf_theta = msgs->qf_theta;
  //ref_lists[list_idx].t0 = msgs->t0;
  ref_lists[list_idx].T = msgs->T;
  ref_lists[list_idx].poly_k = msgs->poly_k;
}

// Publisher Callback Function: publishes the reference path to other WMSR nodes
void WMSRNode::ref_pubCallback(const ros::TimerEvent& event)
{
  // Role: 1 = Malicious, 2 = Normal, 3 = Leader
  // Leader nodes
  // Always publish its own reference path (leader following)
  if (role == 3) {
      ref_pub.publish(inform_center_path);    // publish the reference path to other WMSR node
  }
  // Normal nodes
  // implement WMSR algorithm to update the reference path
  else if (role == 2) {
    inform_center_path = WMSRNode::WMSRAlgorithm(ref_lists);
    ref_pub.publish(inform_center_path);    // publish the reference path to other WMSR node
  }
  // Malicious nodes
  // implement WMSR algorithm to calculate the new inform states
  // If it's a CYBER attack, update the reference path using WMSR, but publish some malicious path
  // If it's a PHYSICAL attack, update the inform states to some malicious path, and publish the malicious path
  else {
    // malicous path
    mali_path.qf_x += 0.2;
    mali_path.qf_y += 0.2;
    // cyber attack
    if (attack == 1) {
      inform_center_path = WMSRNode::WMSRAlgorithm(ref_lists);
    }
    // physical attack
    if (attack == 2) {
      inform_center_path = mali_path;
    }

    ref_pub.publish(mali_path);    // publish malicious reference path to other WMSR node
  }
}


// Output Publisher Callback
void WMSRNode::out_pubCallback(const ros::TimerEvent& event)
{
  output_pub.publish(inform_center_path);
}

// Helper Function:: WMSR Algorithm
path_msgs WMSRNode::WMSRAlgorithm(const std::vector<path_msgs> &list)
{
  path_msgs ref_path;

  std::vector<double> list_qi_x(k,0);
  std::vector<double> list_qi_y(k,0);
  std::vector<double> list_qi_theta(k,0);
  std::vector<double> list_qf_x(k,0);
  std::vector<double> list_qf_y(k,0);
  std::vector<double> list_qf_theta(k,0);
  //std::vector<double> list_t0(k,0);
  std::vector<double> list_T(k,0);
  std::vector<double> list_poly_k(k,0);

  double qi_x, qi_y, qi_theta, qf_x, qf_y, qf_theta, t0, T, poly_k; // uniform weights


  qi_x = inform_center_path.qi_x;
  qi_y = inform_center_path.qi_y;
  qi_theta = inform_center_path.qi_theta;
  qf_x = inform_center_path.qf_x;
  qf_y = inform_center_path.qf_y;
  qf_theta = inform_center_path.qf_theta;
  //t0 = inform_center_path.t0;
  T = inform_center_path.T;
  poly_k = inform_center_path.poly_k;

  // scan the data
  for (int i = 0; i < k; i++) {
    list_qi_x[i] = list[i].qi_x;
    list_qi_y[i] = list[i].qi_y;
    list_qi_theta[i] = list[i].qi_theta;
    list_qf_x[i] = list[i].qf_x;
    list_qf_y[i] = list[i].qf_y;
    list_qf_theta[i] = list[i].qf_theta;
    //list_t0[i] = list[i].t0;
    list_T[i] = list[i].T;
    list_poly_k[i] = list[i].poly_k;
  }

  //remove outliers, and compute the weighted average
  ref_path.qi_x = FilterOutlier(list_qi_x, k, qi_x, F);
  ref_path.qi_y = FilterOutlier(list_qi_y, k, qi_y, F);
  ref_path.qi_theta = FilterOutlier(list_qi_theta, k, qi_theta, F);
  ref_path.qf_x = FilterOutlier(list_qf_x, k, qf_x, F);
  ref_path.qf_y = FilterOutlier(list_qf_y, k, qf_y, F);
  ref_path.qf_theta = FilterOutlier(list_qf_theta, k, qf_theta, F);
  //ref_path.t0 = FilterOutlier(list_t0, k, t0, F);
  ref_path.T = FilterOutlier(list_T, k, T, F);
  ref_path.poly_k = FilterOutlier(list_poly_k, k, poly_k, F);

  ref_path.path_type = inform_center_path.path_type;

  return ref_path;
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

  // if there are F or more values greater (less than) the current node value, remove
  // the first F states greater (less than) the current node value.
  // Otherwise, remove all states greater (less than) the current node value.
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

  // uniform weights
  double wts = 1.0 / double(valid_size);
  double weighted_average = 0;

  // compute the weighted average
  for (int j = 0; j < k+1; j++) {
    weighted_average += wts * list[j];
  }

  // return the filtered value
  return weighted_average;
}

double WMSRNode::calculate_norm(const pose_msgs &state1, const pose_msgs &state2){
  double val;
  double xs=state1.position.x - state2.position.x;
  double ys=state1.position.y - state2.position.y;
  double zs=state1.position.z - state2.position.z;
  val = sqrt( (xs*xs)+ (ys*ys) + (zs*zs));
  return val;
}
double WMSRNode::self_norm(const tiny_msgs &tiny){
  double val;
  val = sqrt((tiny.x*tiny.x) + (tiny.y*tiny.y) + (tiny.z*tiny.z));
  return val;
}

void WMSRNode::Calc_Adjacency(){
  G.resize(2);
  G.at(0).resize(n,std::vector<int>(n));
  G.at(1).resize(n,std::vector<int>(n));

  double val;
  for (int i=0; i<n; i++){
    for (int j=0; j<n; j++){
      val=WMSRNode::calculate_norm(state_lists.at(i),state_lists.at(j));
      if (val<rc) // communication radius
	G[0][i][j]=1;
      if (val<rp) // proximity radius
	G[1][i][j]=1;
    }
  }
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

tiny_msgs WMSRNode::calc_fvec(const std::vector<float> &state1, const std::vector<float> &state2){
  tiny_msgs tiny;
  tiny.x = state1[0] - state2[0];
  tiny.y = state1[1] - state2[1];
  tiny.z = state1[2] - state2[2];
  return tiny;
}

void WMSRNode::populate_state_vector(){
  for(int i=0; i < state_lists.size(); i++){
    swarm_tau[i].x = state_lists[i].position.x - tau[i][0];
    swarm_tau[i].y = state_lists[i].position.y - tau[i][1];
    swarm_tau[i].z = state_lists[i].position.z - tau[i][2];


    swarm_odom[i].x = state_lists[i].position.x;
    swarm_odom[i].y = state_lists[i].position.y;
    swarm_odom[i].z = state_lists[i].position.z;
  }
}

void WMSRNode::save_state_vector(){
  for(int i=0; i < swarm_odom.size(); i++){
    prev_tau[i].x = swarm_odom[i].x - tau[i][0];
    prev_tau[i].y = swarm_odom[i].y - tau[i][1];
    prev_tau[i].z = swarm_odom[i].z - tau[i][2];

    prev_odom[i].x = swarm_odom[i].x;
    prev_odom[i].y = swarm_odom[i].y;
    prev_odom[i].z = swarm_odom[i].z;
  }
}

void WMSRNode::make_tau_vector(){
  tau.resize(n);
  double res = 2*3.14/n;
  for(int i=0; i < n; i++){
    double ang=i*res;
    tau.at(i).resize(3);
    tau[i][0]=5*std::cos(ang);
    tau[i][1]=5*std::sin(ang);
    tau[i][2]=0;
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

tiny_msgs WMSRNode::subtract_vectors(tiny_msgs &a, tiny_msgs &b){
  tiny_msgs result;
  result.x = a.x - b.x;
  result.y = a.y - b.y;
  result.z = a.z - b.z;
  return result;
}
tiny_msgs WMSRNode::psi_gradient(int m_agent, int n_agent, const tiny_msgs &tau_ij){
  //use rp
  float h=0.001;
  std::vector<tiny_msgs> perturb(6);
  for (int i; i<6; i++){
    perturb.push_back(swarm_tau[m_agent]);
  }
  perturb[0].x+=h;
  perturb[1].x-=h;
  perturb[2].y+=h;
  perturb[2].y-=h;
  perturb[3].z+=h;
  perturb[3].z-=h;

  tiny_msgs output;
  output.x = (psi_helper(perturb[0],swarm_tau[n_agent],tau_ij) - psi_helper(perturb[1],swarm_tau[n_agent],tau_ij))/(2*h);
  output.y = (psi_helper(perturb[2],swarm_tau[n_agent],tau_ij) - psi_helper(perturb[3],swarm_tau[n_agent],tau_ij))/(2*h);
  output.z = (psi_helper(perturb[4],swarm_tau[n_agent],tau_ij) - psi_helper(perturb[5],swarm_tau[n_agent],tau_ij))/(2*h);
  return output;

}

float WMSRNode::psi_col_helper(const tiny_msgs &m_agent, const tiny_msgs &n_agent){
  tiny_msgs vecij = calc_vec(m_agent,n_agent);
  double val=self_norm(vecij);
  double mu2=10000;
  float output;
  if (val <= dc){
    output =  ((val - dc)*(val-dc)) / (val - ds + ((ds-dc)*(ds-dc))) / mu2;
  }
  else if (val < ds)
    output = mu2;
  else
    output = 0.0;
}

tiny_msgs WMSRNode::psi_col_gradient(int m_agent, int n_agent){
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
  output.x = (psi_col_helper(perturb[0],swarm_odom[n_agent]) - psi_col_helper(perturb[1],swarm_odom[n_agent]))/(2*h);
  output.y = (psi_col_helper(perturb[2],swarm_odom[n_agent]) - psi_col_helper(perturb[3],swarm_odom[n_agent]))/(2*h);
  output.z = (psi_col_helper(perturb[4],swarm_odom[n_agent]) - psi_col_helper(perturb[5],swarm_odom[n_agent]))/(2*h);
  return output;

}

void WMSRNode::populate_velocity_vector(std::vector<tiny_msgs> &yidot){
  for (int i=0; i<n;i++){
    yidot.push_back(calc_vec(swarm_tau[i],prev_tau[i]));
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

tiny_msgs WMSRNode::multiply_scalar_vec(const float gain, const tiny_msgs &vec){
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
          tiny_msgs tau_ij = calc_fvec(tau[i],tau[neigh_list[j]]); //calculating tauij
          grad_vector.push_back(psi_gradient(i,neigh_list[j],tau_ij));
          diff_vector.push_back(calc_vec(yidot[i],yidot[neigh_list[j]]));
         }
        vel_grad = WMSRNode::multiply_vectors(grad_vector,diff_vector,neigh_list);

        //sort in_neighbours with the vel_grad vector
        std::sort(vel_grad.begin(), vel_grad.end(),
                  [](const Neigh &i, const Neigh &j) { return i.val > j.val; } );
        if (F<vel_grad.size()){//filter out the highest multiples of the velocities and gradient of the velocities
            for(int k=0; k<vel_grad.size();k++){
              if (k<F)
              nlist.f_neigh.push_back(vel_grad[k].id);
              else
              nlist.u_neigh.push_back(vel_grad[k].id);
            }
        }
        else{
            for (int k=0; k<vel_grad.size();k++){//the in-neighbours are even less than F, return a filtered list
              nlist.f_neigh.push_back(vel_grad[k].id);
            }
        }
    }
}

void WMSRNode::filtered_barrier_function(int iteration, int i){
  if (iteration!=0){
    save_state_vector();
    populate_state_vector();
  }
  else {
    save_state_vector();
  }
  if (role_list[i]==1){
    tiny_msgs malic;
    malic.x=0;
    malic.y=80*std::cos(iteration/20 + 2);
    malic.z=0;
    barrier_out=malic;
  }
  else{
    std::vector<tiny_msgs> yidot;
    populate_velocity_vector(yidot);

    NLists nlist;
    nlist=velocity_filter(i, yidot);
    tiny_msgs psi_gradient_sum;
    psi_gradient_sum.x=0; psi_gradient_sum.y=0; psi_gradient_sum.z=0;
    if (!nlist.u_neigh.empty()){
      for (int j=0; j<nlist.u_neigh.size(); j++){
        tiny_msgs tau_ij = calc_fvec(tau[i],tau[nlist.u_neigh[j]]);
        tiny_msgs grad_ij = psi_gradient(i,nlist.u_neigh[j],tau_ij);
        psi_gradient_sum=add_vectors(psi_gradient_sum,grad_ij);
      }
    }
    std::vector<int> Glist;
    Glist=G[1][i];
    for (int j=0; j<nlist.f_neigh.size(); j++){
      Glist[nlist.f_neigh[j]]=0;
    }

    //Get the sum of Glist

    float gain = -100.0; //% Makes barrier function converge faster.
    barrier_out = multiply_scalar_vec(gain,psi_gradient_sum);

    if (self_norm(barrier_out) >=50){
      barrier_out = multiply_scalar_vec(20.00f / self_norm(barrier_out), barrier_out);
    }

      //Just check if i's role is 1 in the role_list and change the barrier_out vector accordingly.
      //for ii=misbehaving_agents
      //    outvector(ii*2-1:ii*2,1) = [0,80*cos(args.tt/20 + 2)];
  }

}

void WMSRNode::filtered_barrier_collision(int iteration, int i){
  if (iteration!=0){
    save_state_vector();
    populate_state_vector();
  }
  else {
    save_state_vector();
  }
  std::vector<tiny_msgs> yidot;
  populate_velocity_vector(yidot);

  if (role_list[i]==1){
    if (iteration==0){
      tiny_msgs rand_num;
      static std::default_random_engine e;
      static std::uniform_real_distribution<> dis(0,1);
      rand_num.x = dis(e) - 0.5f;
      rand_num.y = dis(e) - 0.5f;
      rand_num.z = 0;
      barrier_out = multiply_scalar_vec(umax / self_norm(rand_num), barrier_out);
    }
    else{
      barrier_out=subtract_vectors(swarm_odom[i], prev_odom[i]);
      barrier_out=multiply_scalar_vec(umax / self_norm(barrier_out), barrier_out);
    }
      if (self_norm(barrier_out) >=umax){
      barrier_out = multiply_scalar_vec(umax / self_norm(barrier_out), barrier_out);
    }
  }
  else{
    tiny_msgs psi_gradient_sum, psi_collision_sum;
    psi_gradient_sum.x=0; psi_gradient_sum.y=0; psi_gradient_sum.z=0;
    psi_collision_sum.x=0; psi_collision_sum.y=0; psi_collision_sum.z=0;

    std::vector<int> neigh_list;
    neigh_list=get_in_neighbours(G.at(0), i);
    if (!neigh_list.empty()){
      for (int j=0; j<neigh_list.size(); j++){
        tiny_msgs grad_vector=psi_col_gradient(i,j);
        psi_collision_sum=add_vectors(psi_collision_sum,grad_vector);
      }
    }
    NLists nlist;
    nlist=velocity_filter(i,yidot);

    if (!nlist.u_neigh.empty()){
      for (int j=0; j<nlist.u_neigh.size(); j++){
        tiny_msgs tau_ij = calc_fvec(tau[i],tau[j]);
        tiny_msgs grad_vector=psi_gradient(i,j,tau_ij);
        psi_gradient_sum = add_vectors(psi_gradient_sum, grad_vector);
      }
    }

    float gain=-10.0;
    barrier_out = add_vectors(psi_gradient_sum, psi_collision_sum);
    barrier_out = multiply_scalar_vec(gain,barrier_out);

    if (self_norm(barrier_out) >=umax){
      barrier_out = multiply_scalar_vec(umax / self_norm(barrier_out), barrier_out);
    }
  }
  iteration+=1;
}


// main function: create a WMSRNode class type that handles everything
int main(int argc, char** argv) {
  ros::init(argc, argv, "path_trajectory_WMSRNode");

  WMSRNode path_trajectory_WMSR_node;

  ros::spin();

  return 0;
}
