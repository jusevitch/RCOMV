/*
// ------------------------------------------------------------------
// MSRPA node for r1 rover. 2D motions only.
//
//------------------------------------------------------------------
*/

#include "msrpa.h"

// Constructor
MSRPA::MSRPA()
  :nh_private_("~")
  {
    // Initialize variables assigned in the launch file
    // syntax: ("variable name in the launch file", variable to be assigned, default value)
    nh_private_.param<int>("n", n, 15);
    nh_private_.param<int>("k", k, 7);

    nh_private_.param<int>("idx", idx, 1);
    nh_private_.param<int>("role", role, 2);

    nh_private_.param<int>("F", F, 0);

    nh_private_.param<double>("x", x0, 0);
    nh_private_.param<double>("y", y0, 0);

    nh_private_.param<int>("demo", demo, 2);

    nh_private_.param<double>("cx", cx, 6);
    nh_private_.param<double>("cy", cy, 6);

    nh_private_.param<float>("rc", rc, 1010);
    nh_private_.param<float>("rp", rp, 1000);

    nh_private_.param<float>("ds", ds, 2);
    nh_private_.param<float>("dc", dc, 5);

    nh_private_.param<int>("eta",eta,10);

    ref_msgs NANMSG;
    NANMSG.pose.position.x=std::numeric_limits<double>::quiet_NaN();
    NANMSG.pose.position.y=std::numeric_limits<double>::quiet_NaN();
    NANMSG.pose.position.z=std::numeric_limits<double>::quiet_NaN();


    // Initialize msgs
    if (role == 3){
       reference.pose.position.x = cx;
       reference.pose.position.y = cy;
       reference.pose.position.z = 0;
       reference.pose.orientation.x=0;
       reference.pose.orientation.y=0;
       reference.pose.orientation.z=0;
       inform_states=reference;
       //inform_states=get_leader_reference(iteration);
       
    }
    else if (role == 1){
       inform_states=get_malicious_reference();
       reference.pose.position.x = 0;
       reference.pose.position.y = 0;
       reference.pose.position.z = 0;
       reference.pose.orientation.x=0;
       reference.pose.orientation.y=0;
       reference.pose.orientation.z=0;
    }
    else{
       inform_states=NANMSG;
       reference.pose.position.x = 0;
       reference.pose.position.y = 0;
       reference.pose.position.z = 0;
       reference.pose.orientation.x=0;
       reference.pose.orientation.y=0;
       reference.pose.orientation.z=0;
    }
    control.pose.position.x = 0;
    control.pose.position.y = 0;
    control.pose.position.z = 0;
    control.pose.orientation.x=0;
    control.pose.orientation.y=0;
    control.pose.orientation.z=0;

    umax=50.0;

    state_lists.resize(n);
    make_tau_vector();
    initialize_cvec();
    Calc_Laplacian();

    //we are not publishing the goals to the PID controller anymore so it is 100% irrelevant

    // Subscriber: subscribe the switch topic
    switch_sub = nh.subscribe("/switch", 10, &MSRPA::switch_subCallback, this);
    // wait until receiving the turn on signal
    while(switch_signal.data == false) {

      ros::spinOnce();
      ROS_INFO("Wait until all robots are spawned, stay at initial positions.");

      ros::Duration(0.1).sleep();
    }
    
    ROS_INFO("Simulation is ready, turn on MSRPA node...");

    out_pub = nh.advertise<ref_msgs>("ref",10);
    
    // Publisher: reference
    // pub topic is relative to the node namespace
    std::string pub_topic = "MSRPA/ref";
    ref_pub = nh.advertise<ref_msgs>(pub_topic, 2);
    // Publisher Timer
    // frequency: 10 Hz
    ref_pub_timer = nh.createTimer(ros::Duration(0.1),
                &MSRPA::ref_pubCallback, this);

    states_sub = nh.subscribe<state_graph_builder::posegraph>("/graph",10,&MSRPA::graph_subCallback, this);

    // Subscribers: (msgs list to hold the msgs from subscibed topics)
    // Why does i start from 1? This is wrong.
    for (int i = 1; i <= k; i++) {

      // Initialize subscriber
      int sub_idx = (idx - i) > 0 ? (idx - i) : (n + idx -i);
      // sub topics are resolved in global namespace
      std::string sub_topic = "/ugv" + std::to_string(sub_idx) + "/MSRPA/ref";

      ref_subs.push_back(nh.subscribe<ref_msgs>(sub_topic, 2,
                         boost::bind(&MSRPA::ref_subCallback, this, _1, i-1)) );

      ROS_INFO("sub_idx at: [%d] with topic name: ", sub_idx);

    }


    //new_pub=nh.advertise<tiny_msgs>("barrier",10);
    //new_pub_timer = nh.createTimer(ros::Duration(0.01),
    //           &MSRPA::new_pubCallback,this);

    while(ros::ok){
      ros::spinOnce();
    }


    ROS_INFO_STREAM("Started ugv"<<idx<<" MSRPA Node.");
  }


// Destructor
MSRPA::~MSRPA()
{
  ros::shutdown();
}


void MSRPA::new_pubCallback(const ros::TimerEvent& event){
  Calc_Adjacency();
  // for (int i=0; i<n; i++){
  //   ROS_INFO("Swarm x %lf", swarm_odom[i].x);
  //   ROS_INFO("Swarm y %lf",swarm_odom[i].y);
  // }
  filtered_barrier_collision(idx-1); // TESTING
  double angle=std::atan2(barrier_out.y, barrier_out.x);
  // ROS_INFO("Barrier function agent %i : [%lf, %lf, %lf, %lf]", idx, barrier_out.x, barrier_out.y, angle, state_lists[idx].orientation.z);
  // ROS_INFO("Orientation of agent %i : [%lf, %lf, %lf]", idx, state_lists[idx].orientation.x, state_lists[idx].orientation.y, state_lists[idx].orientation.z);
  new_pub.publish(barrier_out);
}
// Switch signal Subscriber Callback Function
void MSRPA::switch_subCallback(const std_msgs::Bool::ConstPtr& msg){
  switch_signal.data = msg->data;
}


void MSRPA::graph_subCallback(const state_graph_builder::posegraph::ConstPtr& msgs){
  state_lists = msgs->poses;
}
//  Subscriber Callback Function: subscribes reference paths of other WMSR nodes
void MSRPA::ref_subCallback(const ref_msgs::ConstPtr& msgs, const int list_idx)
{
  cvec[list_idx].x = msgs->pose.position.x;
  cvec[list_idx].y = msgs->pose.position.y;
  cvec[list_idx].z = msgs->pose.position.z;

}

// inform states Publisher Callback
void MSRPA::ref_pubCallback(const ros::TimerEvent& event)
{

  Consensus(idx-1);
  inform_states.pose.position=cvec[idx-1];// own index
  if (iteration%eta==0)
    reference=update_reference(reference,control);
  out_pub.publish(reference); // updated reference value
  ref_pub.publish(inform_states); //MSRPA messages
  //ROS_INFO("Iteration: [%ld]", iteration);
  //ROS_INFO("cvec [%lf]", cvec[idx-1].x);
}

void MSRPA::initialize_cvec(){
  cvec.resize(n);
  for (int i=0; i < n; i++){
    cvec[i].x=std::numeric_limits<double>::quiet_NaN();
    cvec[i].y=std::numeric_limits<double>::quiet_NaN();
  }
}
double MSRPA::calculate_norm(const pose_msgs &state1, const pose_msgs &state2){
  double val;
  double xs=state1.position.x - state2.position.x;
  double ys=state1.position.y - state2.position.y;
  double zs=state1.position.z - state2.position.z;
  val = sqrt( (xs*xs)+ (ys*ys) + (zs*zs));
  return val;
}
double MSRPA::self_norm(const tiny_msgs &tiny){
  double val;
  val = sqrt((tiny.x*tiny.x) + (tiny.y*tiny.y) + (tiny.z*tiny.z));
  return val;
}

ref_msgs MSRPA::update_reference(const ref_msgs reference, const ref_msgs control){
  ref_msgs output;

  output.pose.position.x = reference.pose.position.x + control.pose.position.x;
  output.pose.position.y = reference.pose.position.y + control.pose.position.y;
  output.pose.position.z = reference.pose.position.z + control.pose.position.z;
  output.pose.orientation.x = reference.pose.orientation.x + control.pose.orientation.x;
  output.pose.orientation.y = reference.pose.orientation.y + control.pose.orientation.y;
  output.pose.orientation.z = reference.pose.orientation.z + control.pose.orientation.z;
  return output;
  
}


ref_msgs MSRPA::get_leader_reference(uint t){
  ref_msgs leader;
  leader.pose.position.x = radius*cos(t/M_PI);
  leader.pose.position.y = radius*sin(t/M_PI);
  leader.pose.position.z = 0;
  leader.pose.orientation.x=0;
  leader.pose.orientation.y=0;
  leader.pose.orientation.z=0;
  return leader;
}

ref_msgs MSRPA::get_malicious_reference(){
  ref_msgs malicious;
  static std::default_random_engine e;
  static std::uniform_real_distribution<> dis(0,1);
  malicious.pose.position.x = 100*dis(e) - 50;
  malicious.pose.position.y = 100*dis(e) - 50;
  malicious.pose.position.z = 0;
  malicious.pose.orientation.x=0;
  malicious.pose.orientation.y=0;
  malicious.pose.orientation.z=0;
  return malicious;
}
void MSRPA::Calc_Laplacian(){
  bool flag=1;
  if (n%2 == 1){
    if (k > floor(n/2))
      flag=0;
  }
  else if (n%2 == 0){
    if (k>= n/2)
      flag = 0;
  }

  //This part may be completely irrelevant
  L.resize(n,std::vector<int>(n));
  Anan.resize(n,std::vector<int>(n));
  

  if (flag==1){
    for (int i=0; i < n; i++){
      for (int j=0; j < n; j++){
	if (i==j){
	  Anan[i][j]=2*k;
	  L[i][j]=2*k;
	}
	else{
	  Anan[i][j]=0;
	  if (((j+1+k > n) && (((j+1+k)%n) >=i+1)) || ((i+1+k > n) && (((i+1+k)%n) >= j+1)))
	    L[i][j] = -1;
	  else if (abs(i-j) <= k) //indexing doesn't make a difference
	    L[i][j] = -1;
	  else
	    L[i][j]=0;
	    
	}
      }
    }
  }

  for (int i=0; i < n; i++){
    for (int j=0; j < n; j++){
      Anan[i][j]=Anan[i][j]-L[i][j];
      if (Anan[i][j]==0)
	Anan[i][j]=std::numeric_limits<int>::quiet_NaN();
      ROS_INFO("Anan for [%d, %d] for agent %d: %d", i,j, idx, Anan[i][j]);// Verified to be correct
    }
  } 
}

std::vector<FMatrix> MSRPA::BFunc(){
  std::vector<FMatrix> Output;
  Output.resize(2);
  for (int i=0; i < Output.size(); i++){
    Output.at(i).resize(n,std::vector<double>(n));
  }

  // you can return an std::vector of Outputs instead
  for (int d=0; d < n; d++){
    for (int j=0; j < n; j++){
      Output[0][j][d]=Anan[j][d]*cvec[d].x; //Cvec is internally updated
      Output[1][j][d]=Anan[j][d]*cvec[d].y;
    }
  }
  return Output;
}

void MSRPA::Consensus(int i){//are you passing an index of idx-1??
  iteration+=1;
  
  if (iteration==1)
    B = BFunc(); //already creating the B matrix in case

  if (iteration > 1){

    if ((iteration % eta) == (eta-1)){
      if (role==1){
  	
  	control.pose.position.x = cos(exp(iteration/2))/iteration;
        control.pose.position.y = sin(exp(iteration/2))/iteration;
  	control.pose.position.z = 0;
	
      }
	
    }

    if (role==2){
      // sorting has to be done on a higher level of two dimensions orrrr?
      FMatrix follow_ref;
      FMatrix ref2;
      std::vector<double> p;
      //follow_ref.resize(n);
      //populate B matrix entries onto pair vector
      int z=0;
      for (int t=0; t < n; t++){
	if (!std::isnan(B[0][i][t]) && t!=i){
	  follow_ref.push_back(p);
	  follow_ref.at(z).resize(3);
	  follow_ref[z][0]=B[0][i][t];
	  follow_ref[z][1]=B[1][i][t];
	  follow_ref[z][2]=0.0;
	  z+=1;
	}
  	//ROS_INFO("B for [%d, %d] for agent %d: [%lf, %lf] %d", i,t, i+1, B[0][i][t], B[1][i][t], std::isnan(B[0][i][t]));
      }
      if (z>0){
	std::sort(follow_ref.begin(), follow_ref.end());
	std::vector<double> vec = follow_ref[0];
	double xref = vec[0];
	double yref = vec[1];
	std::vector<int> counts;
	int myCount=1;   
	//ref2.push_back(follow_ref[0]);
	//ROS_INFO("ref2 [%lf, %lf]", follow_ref[0][0], follow_ref[0][1]);
      

	if (follow_ref.size()>1){

	  for (int j=1; j < follow_ref.size(); j++){
	    std::vector<double> comp = follow_ref[j];
	    if (((comp[0] - xref) < 0.1) && ((comp[1] - yref) < 0.1)){ //we have another occurrence!
	      myCount+=1;
	    }
	    else{
	 
	      counts.push_back(myCount);
	      myCount=1;
	      vec[0] = xref;
	      vec[1] = yref;
	      xref = comp[0];
	      yref = comp[1];
	      ref2.push_back(vec);
	    }
	     

	  }
	}
	ref2.push_back(follow_ref[follow_ref.size()-1]);
	counts.push_back(myCount);

	if (!follow_ref.empty()){
	  std::vector<int>::iterator maximum;
	  maximum = std::max_element(std::begin(counts),std::end(counts));
	  int max_idx = std::distance(counts.begin(),maximum);
	  int max_counts = *maximum;
	  if (idx==3){
	    //ROS_INFO("MAX_COUNTS: %d, %d, %ld, %lf, %lf", max_counts, max_idx, counts.size(), ref2[max_idx][0], ref2[max_idx][1]);
	  }      
          if (max_counts >= (F + 1)){
     	    auto c = ref2[max_idx][0];
     	    auto d = ref2[max_idx][1];
     	    cvec[i].x=c;
     	    cvec[i].y=d;
     	    cvec[i].z=0;
     	    control=castToPoseAndSubtract(cvec[i],reference);

	      
     	  }
	}
      }

     

     

    }
	  
  }

  //We are only updating x at this time step with u(t-1)
  if (iteration%eta==0 && role==2){
    cvec[i].x=std::numeric_limits<double>::quiet_NaN();
    cvec[i].y=std::numeric_limits<double>::quiet_NaN();
  }
  
  if (role==3){ //leader
    //ref_msgs leadp=get_leader_reference(floor(iteration/eta)+1);
    //cvec[i] = leadp.pose.position;
    cvec[i].x=cx;
    cvec[i].y=cy;
  }
  else if (role==1){ //malicious
    ref_msgs mali = get_malicious_reference();
    cvec[i] = mali.pose.position;
      
 }

  //UPDATE B
  B = BFunc();

    

}
void MSRPA::Calc_Adjacency(){
  G.resize(2);
  G.at(0).resize(n,std::vector<int>(n));
  G.at(1).resize(n,std::vector<int>(n));

  double val;
  for (int i=0; i<n; i++){
    for (int j=0; j<n; j++){
      val=MSRPA::calculate_norm(state_lists.at(i),state_lists.at(j));
      //ROS_INFO("Adjacency val: %lf", val);
      if(i==j){
	G[0][i][j]=0;
	G[1][i][j]=0;
      }
      else{
	if (val<rc)
	  G[0][i][j]=1;// communication radius
	else
	  G[0][i][j]=0;
	if (val<rp)
	  G[1][i][j]=1;// proximity radius
	else
	  G[1][i][j]=0;
      }
    }
  }
}

ref_msgs MSRPA::castToPoseAndSubtract(const tiny_msgs point, const ref_msgs pose){
  ref_msgs output;
  output.pose.position.x = point.x - pose.pose.position.x;
  output.pose.position.y = point.y - pose.pose.position.y;
  output.pose.position.z = point.z - pose.pose.position.z;

  output.pose.orientation = pose.pose.orientation;
  return output;
}

std::vector<int> MSRPA::get_in_neighbours(int rad_type, int agent){
  int agents_no = G[rad_type].size();
  std::vector<int> neighbours;
  for (int i=0; i<agents_no; i++){
    if (G[rad_type][agent][i] == 1)
	  neighbours.push_back(i);
  }
  return neighbours;
}

tiny_msgs MSRPA::calc_vec(const tiny_msgs& state1, const tiny_msgs& state2){
  tiny_msgs tiny;
  tiny.x = state1.x - state2.x;
  tiny.y = state1.y - state2.y;
  tiny.z = state1.z - state2.z;
  return tiny;
}

tiny_msgs MSRPA::calc_fvec(int i, int j){
  tiny_msgs tiny;
  tiny.x = tau[i][0] - tau[j][0];
  tiny.y = tau[i][1] - tau[j][1];
  tiny.z = tau[i][2] - tau[j][2];
  return tiny;
}

void MSRPA::populate_state_vector(){
  swarm_tau.resize(n);
  swarm_odom.resize(n);
  for(int i=0; i < state_lists.size(); i++){
    swarm_tau[i].x = state_lists[i].position.x - tau[i][0];
    swarm_tau[i].y = state_lists[i].position.y - tau[i][1];
    swarm_tau[i].z = state_lists[i].position.z - tau[i][2];


    //ROS_INFO("Swarm odom, [%d, %lf]", i,state_lists[i].position.x);


    swarm_odom[i].x = state_lists[i].position.x;
    swarm_odom[i].y = state_lists[i].position.y;
    swarm_odom[i].z = state_lists[i].position.z;
  }
}

void MSRPA::save_state_vector(){
  prev_tau.resize(n);
  prev_odom.resize(n);
  for(int i=0; i < swarm_odom.size(); i++){
    prev_tau[i].x = swarm_odom[i].x - tau[i][0];
    prev_tau[i].y = swarm_odom[i].y - tau[i][1];
    prev_tau[i].z = swarm_odom[i].z - tau[i][2];

    prev_odom[i].x = swarm_odom[i].x;
    prev_odom[i].y = swarm_odom[i].y;
    prev_odom[i].z = swarm_odom[i].z;
  }
}

void MSRPA::make_tau_vector(){
  tau.resize(n);
  double res = 2*3.14/n;
  for(int i=0; i < n; i++){
    double ang=i*res;
    tau.at(i).resize(3);
    tau[i][0]=10.0*std::cos(ang);
    tau[i][1]=10.0*std::sin(ang);
    tau[i][2]=0.0;
    ROS_INFO("tau for agent %i : [%lf, %lf, %lf]", i, tau[i][0], tau[i][1], tau[i][2]);
  }

  // for(int jj=0; jj < n; jj++){
  //   ROS_INFO("Agent %i tau vector: [%lf, %lf, %lf]", jj, tau[jj][0], tau[jj][1], tau[jj][2]);
  // }
}

tiny_msgs MSRPA::add_vectors(const tiny_msgs &a, const tiny_msgs &b){
  tiny_msgs result;
  result.x = a.x + b.x;
  result.y = a.y + b.y;
  result.z = a.z + b.z;
  return result;
}

tiny_msgs MSRPA::subtract_vectors(const tiny_msgs &a, const tiny_msgs &b){
  tiny_msgs result;
  result.x = a.x - b.x;
  result.y = a.y - b.y;
  result.z = a.z - b.z;
  return result;
}


float MSRPA::psi_helper(const tiny_msgs &m_agent, const tiny_msgs &n_agent, const tiny_msgs &tau_ij){

  //Reference MATLAB code
  // mu1 = 1000; % What should this be? not sure
  // yij = y_i - y_j;
  // rshat = rs - norm(tauij,2);
  // outscalar = norm(yij,2)^2 / (rshat - norm(yij,2) + rshat^2/mu1);

  // ROS_INFO("m_agent")

  double mu1=1000.0;
  double output;
  tiny_msgs tiny=calc_vec(m_agent,n_agent);
  double tiny_norm = self_norm(tiny);
  double rshat = rp - self_norm(tau_ij);
  output = (tiny_norm*tiny_norm) / (rshat - tiny_norm + (rshat*rshat)/mu1);

  // ROS_INFO("\n m_agent vector: [%lf, %lf, %lf] \n n_agent vector: [%lf, %lf, %lf] \n output: %lf \n",\
  // m_agent.x, m_agent.y, m_agent.z, n_agent.x, n_agent.y, n_agent.z, output);

  return output;

}
tiny_msgs MSRPA::psi_gradient(int m_agent, int n_agent, const tiny_msgs &tau_ij){
  //use rp

  float h=0.001;
  std::vector<tiny_msgs> perturb;
  for (int i=0; i<6; i++){
    perturb.push_back(swarm_tau[m_agent]);
  }

  perturb[0].x+=h; //ROS_INFO("perturb[0] for m_agent = %i: [%lf, %lf, %lf]", m_agent, perturb[0].x, perturb[0].y,perturb[0].z);
  perturb[1].x-=h;
  perturb[2].y+=h;
  perturb[3].y-=h;
  perturb[4].z+=h;
  perturb[5].z-=h;

  tiny_msgs output;
  output.x = (psi_helper(perturb[0],swarm_tau[n_agent],tau_ij) - psi_helper(perturb[1],swarm_tau[n_agent],tau_ij))/(2*h);
  output.y = (psi_helper(perturb[2],swarm_tau[n_agent],tau_ij) - psi_helper(perturb[3],swarm_tau[n_agent],tau_ij))/(2*h);
  output.z = (psi_helper(perturb[4],swarm_tau[n_agent],tau_ij) - psi_helper(perturb[5],swarm_tau[n_agent],tau_ij))/(2*h);

  //ROS_INFO("\n Output vector for m_agent %i : [%lf, %lf, %lf] \n", m_agent, output.x, output.y, output.z);

  return output;

}

float MSRPA::psi_col_helper(const tiny_msgs &m_agent, const  tiny_msgs &n_agent){
  tiny_msgs vecij = calc_vec(m_agent,n_agent);
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

tiny_msgs MSRPA::psi_col_gradient(int m_agent, int n_agent){ //this is supposed to only take the state vector
  float h=0.001;
  std::vector<tiny_msgs> perturb;
  for (int i=0; i<6; i++){
    perturb.push_back(swarm_odom[m_agent]);
  }
  perturb[0].x+=h;
  perturb[1].x-=h;
  perturb[2].y+=h;
  perturb[3].y-=h;
  perturb[4].z+=h;
  perturb[5].z-=h;


  tiny_msgs output;
  output.x = (psi_col_helper(perturb[0],swarm_odom[n_agent]) - psi_col_helper(perturb[1],swarm_odom[n_agent]))/(2*h);

  output.y = (psi_col_helper(perturb[2],swarm_odom[n_agent]) - psi_col_helper(perturb[3],swarm_odom[n_agent]))/(2*h);

  output.z = (psi_col_helper(perturb[4],swarm_odom[n_agent]) - psi_col_helper(perturb[5],swarm_odom[n_agent]))/(2*h);
  //std::cout << "Output" << output << std::endl;
  return output;

}

void MSRPA::populate_velocity_vector(){
  yidot.resize(n);
  for (int i=0; i<n;i++){

    // NEED TO DIVIDE BY TIME TO GET ACTUAL DERIVATIVE
    yidot[i]=calc_vec(swarm_tau[i],prev_tau[i]);
    // The denominator should be the publish frequence from
    // new_pub_timer in this file
    yidot[i].x=yidot[i].x/0.01;
    yidot[i].y=yidot[i].y/0.01;
    yidot[i].z=yidot[i].z/0.01;

  }
}

std::vector<Neigh> MSRPA::multiply_vectors(const std::vector<tiny_msgs> &vec1, const std::vector<tiny_msgs> &vec2, const std::vector<int> neigh){
  std::vector<Neigh> output;
  Neigh sample;
  for (int i=0; i<vec1.size(); i++){
    sample.val=(vec1[i].x*vec2[i].x) + (vec1[i].y*vec2[i].y) + (vec1[i].z*vec2[i].z);
    sample.id=neigh[i];
    output.push_back(sample);
  }
  return output;
}

tiny_msgs MSRPA::multiply_scalar_vec(const float gain, const tiny_msgs &vec){
   tiny_msgs result;
   result.x = gain*vec.x;
   result.y = gain*vec.y;
   result.z = gain*vec.z;
   return result;
}

NLists MSRPA::velocity_filter(int i){
    std::vector<int> neigh_list;
    neigh_list=get_in_neighbours(1, i);
    std::vector<tiny_msgs> grad_vector;
    std::vector<tiny_msgs> diff_vector;
    std::vector<Neigh> vel_grad;
    NLists nlist;
    if (!neigh_list.empty()){
        for (int j=0; j<neigh_list.size(); j++){
          tiny_msgs tau_ij = calc_fvec(i,neigh_list[j]); //calculating tauij
          grad_vector.push_back(psi_gradient(i,neigh_list[j],tau_ij));
	        diff_vector.push_back(calc_vec(yidot[i],yidot[neigh_list[j]]));
	  //ROS_INFO("At [%d], grad_vector %lf, diff_vector %lf", j, grad_vector[j].x, diff_vector[j].x);
         }
        vel_grad = MSRPA::multiply_vectors(grad_vector,diff_vector,neigh_list);
    	// for (int k=0; k<vel_grad.size(); k++){
    	//   ROS_INFO("vel_grad: %d, %lf", k, vel_grad[k].val); // noticed vel_grad to be a very tiny value in the order of 10^-4
    	// }

        //sort in_neighbours with the vel_grad vector
        std::sort(vel_grad.begin(), vel_grad.end(),
                  [](const Neigh &i, const Neigh &j) { return i.val > j.val; } );

	// for (int k=0; k<vel_grad.size(); k++){
	//   if (i==0)
    	//   ROS_INFO("vel_grad: %d, %lf %d", k, vel_grad[k].val,F); // just want to check my sorting OK IT WORKS
    	// }
        if (F<vel_grad.size()){//filter out the highest multiples of the velocities and gradient of the velocities
            for(int k=0; k<vel_grad.size();k++){
              if (k<F) // take into account the Fth value as value
              nlist.f_neigh.push_back(vel_grad[k].id);
	      // else if (k>vel_grad.size()-F-1)
	      // nlist.f_neigh.push_back(vel_grad[k].id);
              else
              nlist.u_neigh.push_back(vel_grad[k].id);
            }
	    nlist.filtered_only=0;
        }
        else{
            for (int k=0; k<vel_grad.size();k++){//the in-neighbours are even less than F, return a filtered list
              nlist.f_neigh.push_back(vel_grad[k].id);
            }
	    nlist.filtered_only=1;
        }
    }
    return nlist;
}

NLists MSRPA::norm_filter(int i){

  // use swarm_tau
    std::vector<int> neigh_list;    
    std::vector<tiny_msgs> vector_list;
    std::vector<Neigh> norm_vector;
    NLists nlist;
    
    neigh_list=get_in_neighbours(1, i);
    if (!neigh_list.empty()){
      for(int j=0; j<neigh_list.size(); j++){
	vector_list.push_back(calc_vec(swarm_tau[i],swarm_tau[neigh_list[j]]));		
      }

      norm_vector = multiply_vectors(vector_list, vector_list, neigh_list);

       std::sort(norm_vector.begin(), norm_vector.end(),
                  [](const Neigh &i, const Neigh &j) { return i.val > j.val; } );


       if (F<norm_vector.size()){
	 for(int k=0; k<norm_vector.size();k++){
              if (k<F) // take into account the Fth value as value
              nlist.f_neigh.push_back(norm_vector[k].id);
	      else if (k>norm_vector.size()-F-1)
	       nlist.f_neigh.push_back(norm_vector[k].id);
              else
              nlist.u_neigh.push_back(norm_vector[k].id);
            }
	    nlist.filtered_only=0;
	 
       }
       else{
	 for (int k=0; k<norm_vector.size();k++){//the in-neighbours are even less than F, return a filtered list
              nlist.f_neigh.push_back(norm_vector[k].id);
            }
	    nlist.filtered_only=1;	 
       }

    }
    return nlist;
}

tiny_msgs MSRPA::weighted_sum(const std::vector<int> &weights){
  tiny_msgs gradient;
  gradient.x=0;gradient.y=0; gradient.z=0;
  for (int j=0; j < swarm_tau.size(); j++){
    gradient.x -= weights[j]*swarm_tau[j].x;
    gradient.y -= weights[j]*swarm_tau[j].y;
    gradient.z -= weights[j]*swarm_tau[j].z;
  }
  return gradient;
}

void MSRPA::filtered_barrier_collision(int i){
  // i -= 1;
  if (iteration!=0){
    save_state_vector(); // Saves current state into the previous state for derivative calculation
    populate_state_vector(); // Udpates the current state from subscriber
  }
  else {
    populate_state_vector();
    save_state_vector();
  }
  populate_velocity_vector();

  if (role==1){
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
    psi_gradient_sum.x=0.0; psi_gradient_sum.y=0.0; psi_gradient_sum.z=0.0;
    psi_collision_sum.x=0.0; psi_collision_sum.y=0.0; psi_collision_sum.z=0.0;

    //-----------------------Collision sum-------------------------------//

    std::vector<int> neigh_list;
    neigh_list=get_in_neighbours(1, i);
    if (!neigh_list.empty()){
      for (int j=0; j<neigh_list.size(); j++){
        tiny_msgs grad_vector=psi_col_gradient(i,neigh_list[j]);
        psi_collision_sum=add_vectors(psi_collision_sum,grad_vector);
      }
    }

    //verified to be correct

    //--------------------------Gradient sum----------------------------//


    NLists nlist;
    nlist=norm_filter(i);

    // Testing
    // if(idx == 1){
    //   for (int j=0; j<nlist.u_neigh.size(); j++){
    //     ROS_INFO("Agent %i has filter neighbor %i", i, nlist.u_neigh.at(j));
    //     if(j == nlist.u_neigh.size() - 1){
    //       ROS_INFO("END OF FILTER LOOP");
    //     }
    //   }
    //}
    if (nlist.filtered_only==0){
      for (int j=0; j<nlist.u_neigh.size(); j++){
        tiny_msgs tau_ij = calc_fvec(i,nlist.u_neigh[j]);
        tiny_msgs grad_vector=psi_gradient(i,nlist.u_neigh[j],tau_ij);
    	if (i==0){
    	  ROS_INFO("unfiltered list for %d, %d", i, nlist.u_neigh[j]);
	  //ROS_INFO("Gradient %lf:",grad_vector.x);
	}
	
        psi_gradient_sum = add_vectors(psi_gradient_sum, grad_vector);
      }
    }

    std::vector<int> Glist, Gf;
    int sum_of_G=0;
    Glist=G[1][i];
    for (int j=0; j<nlist.f_neigh.size(); j++){
      Glist[nlist.f_neigh[j]]=0;
    }
    for (int j=0; j<Glist.size(); j++){
      Gf.push_back(-Glist[j]);
      sum_of_G+=-Glist[j];
    }

    Gf[i]=sum_of_G;
    //tiny_msgs extra_gradient = weighted_sum(Gf);

    float gain= -10.0;

    barrier_out = add_vectors(psi_gradient_sum, psi_collision_sum);
    
    
     if(idx == 1)
        ROS_INFO("Before gain addition %i: [%lf, %lf, %lf]", idx, barrier_out.x, barrier_out.y, barrier_out.z);

     barrier_out = multiply_scalar_vec(gain,barrier_out);

     //barrier_out = add_vectors(barrier_out, extra_gradient);

     if(idx == 1)
        ROS_INFO("After adding extra gradient %i: [%lf, %lf, %lf]", idx, barrier_out.x, barrier_out.y, barrier_out.z);
     if (self_norm(barrier_out) >=umax){
       barrier_out = multiply_scalar_vec(umax / self_norm(barrier_out), barrier_out);
     }
        

  }
  iteration+=1;
}


// main function
int main(int argc, char **argv) {

  // Initialize ros
  ros::init(argc, argv, "msrpa_node");

  // Create an object of MSRPA that handles everything
  MSRPA MS_RPA_Node;

  ros::spin();

  return 0;
}
