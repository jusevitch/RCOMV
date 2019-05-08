/*
// ------------------------------------------------------------------
// MSRPA node for r1 rover. 2D motions only.
//
//------------------------------------------------------------------
*/

#include "msrpa.h"

// Constructor
MSRPA::MSRPA()
    : nh_private_("~")
{
  // Initialize variables assigned in the launch file
  // syntax: ("variable name in the launch file", variable to be assigned, default value)
  nh_private_.param<int>("n", n, 15);
  nh_private_.param<int>("k", k, 7);
  nh_private_.param<std::vector<int> >("in_neighbors", in_neighbors, std::vector<int>());

  nh_private_.param<int>("idx", idx, 1);
  nh_private_.param<int>("rover_number", rover_number, 0); // gives Rover number; 0 throws an error.
  // Role 1: Malicious
  // Role 2: Follower
  // Role 3: Leader
  nh_private_.param<int>("role", role, 2); // What do role numbers correspond to again?

  nh_private_.param<int>("F", F, 0);
  nh_private_.param<int>("eta", eta, 10);
  nh_private_.param<double>("Rf", Rf, 10);

  //Trajectory parameters:
  nh_private_.param<double>("t0", t0, 0);
  nh_private_.param<double>("xc", xc, 0);
  nh_private_.param<double>("yc", yc, 0);
  nh_private_.param<double>("Rad", Rad, 100);
  nh_private_.param<double>("wd", wd, 0); // wd in the code
  nh_private_.param<double>("phi0", phi0, 0);
  nh_private_.param<double>("Leng", Leng, 10);
  nh_private_.param<double>("psi", psi, 0);
  nh_private_.param<double>("v", v, 0);
  nh_private_.param<double>("start_L", start_L, 0);
  nh_private_.param<std::string>("trajectory_type", trajectory_type, "None");

  // Common namespace of all nodes. This variable helps the node know what topics to subscribe to 
  //  to receive MS-RPA information from other nodes in the network.
  nh_private_.param<std::string>("common_namespace", common_namespace, "/ugv");
  nh_private_.param<int>("gazebo", gazebo,0);

  NANMSG.type = "NaN";
  double nanmsg = std::numeric_limits<double>::quiet_NaN();
  ;
  for (int i = 0; i < 7; i++)
  {
    NANMSG.trajectory.push_back(nanmsg);
  }
  for (int i = 0; i < 2; i++)
  {
    NANMSG.formation.push_back(nanmsg);
  }

  // Initialize msgs
  ROS_INFO("t0, xc, yc, Rad, wd, phi0, role: [%lf, %lf, %lf, %lf, %lf, %lf], %d", t0, xc, yc, Rad, wd, phi0, role);  
  if (role == 3)
  {
   reset_message.type = trajectory_type;
   reset_message.trajectory = {t0, xc, yc, Rad, wd, phi0};
   reset_message.formation = {Rf, static_cast<double>(n)};
    inform_states =reset_message;
  }
  else if (role == 1)
  {
   reset_message= get_malicious_reference();
    inform_states =reset_message;
  }
  else
  {
    inform_states = NANMSG;
    reset_message= NANMSG;
  }

  // "reference" is the value to be sent to the IO_collision_control nodes
  // What is control??
  internal_state = reset_message; // Initialize the internal_state
  reference_state = reset_message; // Initialize the reference_state
  initialize_cvec();
  Calc_Laplacian();

  //we are not publishing the goals to the PID controller anymore so it is 100% irrelevant

  // Subscriber: subscribe the switch topic
  // switch_sub = nh.subscribe("/switch", 10, &MSRPA::switch_subCallback, this);
  // wait until receiving the turn on signal
  // while (switch_signal.data == false)
  // {

  //   ros::spinOnce();
  //   ROS_INFO("Wait until all robots are spawned, stay at initial positions.");

  //   ros::Duration(0.1).sleep();
  // }

  ROS_INFO("Simulation is ready, turn on MSRPA node...");

  if(role == 3){
    leader_cmd_sub = nh.subscribe<ref_msgs>("/leader_cmd", 1, &MSRPA::leader_subCallback, this);
  }

  // The "ref" topic is the topic which should be streamed to the IO_control_collision node
  out_pub = nh.advertise<ref_msgs>("ref", 10);

  // Publisher:reset_message
  // pub topic is relative to the node namespace
  // The "MSRPA/ref" topic is for MSRPA nodes to send values to their outneighbors
  std::string pub_topic = "MSRPA/ref";
  ref_pub = nh.advertise<ref_msgs>(pub_topic, 2);
  // Publisher Timer
  // frequency: 10 Hz
  ref_pub_timer = nh.createTimer(ros::Duration(0.1),
                                 &MSRPA::ref_pubCallback, this);

  // Subscribers: (msgs list to hold the msgs from subscibed topics)

  // This is specific to k-circulant graphs
  if(gazebo){
    for (int i = 1; i <= k; i++)
    {

      // Initialize subscriber
      int sub_idx = (idx - i) > 0 ? (idx - i) : (n + idx - i);
      // sub topics are resolved in global namespace
      std::string sub_topic = "/" + common_namespace + std::to_string(sub_idx) + "/MSRPA/ref";

      ref_subs.push_back(nh.subscribe<ref_msgs>(sub_topic, 2,
                                                boost::bind(&MSRPA::ref_subCallback, this, _1, i - 1)));

      ROS_INFO("sub_idx at: [%d] with topic name: ", sub_idx);
    }
  } else {
    for(int i = 0; i < in_neighbors.size(); i++) 
    {
      std::string sub_topic = "/" + common_namespace + std::to_string(in_neighbors[i]) + "/MSRPA/ref";
      ref_subs.push_back(nh.subscribe<ref_msgs>(sub_topic, 2,
                                                boost::bind(&MSRPA::ref_subCallback, this, _1, i)));
      ROS_INFO("I subscribed to %s", sub_topic.c_str());
    }
  }

  while (ros::ok)
  {
    ros::spinOnce();
  }

  ROS_INFO_STREAM("Started ugv" << idx << " MSRPA Node.");

  
}

// Destructor
MSRPA::~MSRPA()
{
  ros::shutdown();
}

// switch node to start simulations
void MSRPA::switch_subCallback(const std_msgs::Bool::ConstPtr &msg)
{
  switch_signal.data = msg->data;
}

//  Subscriber Callback Function: subscribesreset_messagepaths of other nodes
void MSRPA::ref_subCallback(const ref_msgs::ConstPtr &msgs, const int list_idx)
{
  ROS_INFO("cvec.size(): %lu", cvec.size());
  ROS_INFO("list_idx: %d", list_idx);
  cvec[list_idx].type = msgs->type;
  cvec[list_idx].trajectory = msgs->trajectory;
  cvec[list_idx].formation = msgs->formation;
}

// inform states Publisher Callback
void MSRPA::ref_pubCallback(const ros::TimerEvent &event)
{

  Consensus(idx - 1); // Update internal_state
  
  // Compares the type strings to see if internal_state is NaN. Only the string needs to be compared;
  // normal messages should NOT have "NaN" as their trajectorytype.
  bool internal_state_is_NANMSG = (internal_state.type.compare(NANMSG.type) == 0);

// ROS_INFO("NANMSG.type: %s", NANMSG.type.c_str());
//   ROS_INFO("internal_state.type: %s \n\
//   internal_state.trajectory = [%lf, %lf, %lf, %lf, %lf, %lf, %lf] \n\
//   internal_state.formation = [%lf, %lf]",
//   internal_state.type.c_str(),\
//   internal_state.trajectory[0], internal_state.trajectory[1], internal_state.trajectory[2], internal_state.trajectory[3], internal_state.trajectory[4], internal_state.trajectory[5], internal_state.trajectory[6],\
//   internal_state.formation[0], internal_state.formation[1]);
  // ROS_INFO("internal_state_is_NANMSG: %d", internal_state_is_NANMSG ? 1 : 0);
  
  // ROS_INFO("Iteration: [%ld]", iteration);
  // ROS_INFO("interation mod eta: %d", iteration % eta);
  if (iteration % eta == 0){
    // If iteration % eta == 0 and internal_state is not NANMSG, set reference_state = internal_state
    // internal_state == NANMSG means that the agent didn't get the same message from at least F+1 other agents
    if (!(internal_state_is_NANMSG)){
      reference_state = internal_state;
    }

    if (reference_state.type.compare(NANMSG.type) != 0) {
      // Publish reference_state to IO_control_collision. 
      // If internal_state is NANMSG, reference_state will be the same as the previous timestep.
      // If reference_stae is NANMSG, nothing will be published.
      out_pub.publish(reference_state);
    }

    internal_state = reset_message; // Resets the internal state. 

    if (role == 3){
      ref_pub.publish(internal_state); // Publish only if agent is a leader (or malicious--add later)
      // ROS_INFO("This published");
    }
  } else {
    if (!(internal_state_is_NANMSG)){
      // If internal state is not the NaN message, publish it to out-neighbors
      ref_pub.publish(internal_state); //MSRPA messages
      // ROS_INFO("This published for agent %d", idx);
    } 
  }

  
  //ROS_INFO("cvec [%lf]", cvec[idx-1].x);
}

void MSRPA::initialize_cvec()
{
  cvec.resize(k); // This only needs to be as big as the number of in-neighbors
  for (int i = 0; i < k; i++)
  {
    cvec[i] = NANMSG;
  }
}

// Why do we have this function???
sref_msgs MSRPA::update_reference(const sref_msgs reset_message, const sref_msgs control)
{
  sref_msgs output;

  output.pose.position.x = reset_message.pose.position.x + control.pose.position.x;
  output.pose.position.y = reset_message.pose.position.y + control.pose.position.y;
  output.pose.position.z = reset_message.pose.position.z + control.pose.position.z;
  output.pose.orientation.x = reset_message.pose.orientation.x + control.pose.orientation.x;
  output.pose.orientation.y = reset_message.pose.orientation.y + control.pose.orientation.y;
  output.pose.orientation.z = reset_message.pose.orientation.z + control.pose.orientation.z;
  return output;
}

sref_msgs MSRPA::get_leader_reference(uint t)
{
  sref_msgs leader;
  leader.pose.position.x = Rf * cos(t / M_PI);
  leader.pose.position.y = Rf * sin(t / M_PI);
  leader.pose.position.z = 0;
  leader.pose.orientation.x = 0;
  leader.pose.orientation.y = 0;
  leader.pose.orientation.z = 0;
  return leader;
}

ref_msgs MSRPA::get_malicious_reference()
{
  ref_msgs malicious;
  static std::default_random_engine e;
  static std::uniform_real_distribution<double> dis(0, 1);

  malicious.type = "circular";
  for (int i = 0; i < 7; i++)
  {
    malicious.trajectory.push_back(100 * dis(e));
  }
  for (int i = 0; i < 2; i++)
  {
    malicious.formation.push_back(100 * dis(e));
  }
  return malicious;
}

void MSRPA::Calc_Laplacian()
{
  bool flag = 1;
  if (n % 2 == 1)
  {
    if (k > floor(n / 2))
      flag = 0;
  }
  else if (n % 2 == 0)
  {
    if (k >= n / 2)
      flag = 0;
  }

  //This part may be completely irrelevant
  L.resize(n, std::vector<int>(n));
  Anan.resize(n, std::vector<int>(n));

  if (flag == 1)
  {
    for (int i = 0; i < n; i++)
    {
      for (int j = 0; j < n; j++)
      {
        if (i == j)
        {
          Anan[i][j] = 2 * k;
          L[i][j] = 2 * k;
        }
        else
        {
          Anan[i][j] = 0;
          if (((j + 1 + k > n) && (((j + 1 + k) % n) >= i + 1)) || ((i + 1 + k > n) && (((i + 1 + k) % n) >= j + 1)))
            L[i][j] = -1;
          else if (abs(i - j) <= k) //indexing doesn't make a difference
            L[i][j] = -1;
          else
            L[i][j] = 0;
        }
      }
    }
  }

  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j < n; j++)
    {
      Anan[i][j] = Anan[i][j] - L[i][j];
      if (Anan[i][j] == 0)
        Anan[i][j] = std::numeric_limits<int>::quiet_NaN();
      //ROS_INFO("Anan for [%d, %d] for agent %d: %d", i,j, idx, Anan[i][j]);// Verified to be correct
    }
  }
}

std::vector<double> MSRPA::multiply_scalar_vec(const double val, const std::vector<double> vec)
{
  std::vector<double> output;
  for (int i = 0; i < vec.size(); i++)
  {
    output.push_back(vec[i] * val);
  }
  return output;
}

std::vector<FMatrix> MSRPA::BFunc() // Updates and returns the B matrix (where B is defined in the original MATLAB code)
{
  std::vector<FMatrix> Output;
  Output.resize(9);

  // Note: the size of 9 above is because MSRPA.msg currently (5/3/19) has at most 9 floats in it. This value will need to be updated later.

  for (int i = 0; i < Output.size(); i++)
  {
    Output.at(i).resize(n, std::vector<double>(n));
  }

  // you can return an std::vector of Outputs instead
  for (int d = 0; d < n; d++)
  {
    for (int j = 0; j < n; j++)
    {
      for (int o = 0; o < 7; o++)
      {
        Output[o][j][d] = Anan[j][d] * cvec[d].trajectory[o]; //Cvec is internally updated
      }
      for (int o = 7; o < 9; o++)
      {
        Output[o][j][d] = Anan[j][d] * cvec[d].formation[o - 7];
      }

      if (cvec[d].type == "circular") // This will also need to be changed to include square and other trajectories
        Output[6][j][d] = 0.0;
    }
  }
  return Output;
}


void MSRPA::Consensus(int i)
{

  // Debugging only
  // print_cvec();

  // REQUIRED. Increases iteration number.
  iteration += 1;
  
  // Only applies if agent is normal (role == 2)
  if (role == 2) {
    // Messages from in-neighbors are all in cvec already
    // Compare all messages, test if this agent has received F+1 of the same message
    // std::vector<std::vector<int> > groups_with_same_messages;
    std::vector<ref_msgs> accepted_messages;
    // Create a temporary vector to store the indices of all agents with messages the same as agent ii's
    std::vector<int> agents_with_same_messages;

    // Loop over all messages from in-neighbors and pick out the groups with the same messages
    // This can be made more efficient, but it's a little tricky to implement.
    std::vector<int> checked_agents;
    std::vector<int>::iterator it;

    for(int ii=0; ii < k; ii++){
      agents_with_same_messages.clear();
      agents_with_same_messages.push_back(ii);
      checked_agents.push_back(ii);

      for(int jj=ii+1; jj < k; jj++){
        // ROS_INFO("test_messages_equal for agent %d: %d", idx, test_messages_equal(cvec[ii], cvec[jj]) ? 1 : 0);

        it = find(checked_agents.begin(), checked_agents.end(), jj);
        if(it == checked_agents.end()){
          // jj has not been compared, so compare it with ii.
          if(test_messages_equal(cvec[ii], cvec[jj])){
            agents_with_same_messages.push_back(jj);
            checked_agents.push_back(jj);
          }
        }
      }

      if (agents_with_same_messages.size() >= F+1){
        accepted_messages.push_back(cvec[agents_with_same_messages[0]]);
        // groups_with_same_messages.push_back(agents_with_same_messages);
      }
    }

    // If F+1 have been receive, update any relevant variables

    if(accepted_messages.size() > 1){
      ROS_INFO("ERROR!! Two values received by agent %d with more than F+1 instances each. Check your algorithm.", idx);
    } else if(accepted_messages.size() == 1) {
      internal_state = accepted_messages[0];
      ROS_INFO("internal_state was updated");
      ROS_INFO("internal_state.type: %s ", internal_state.type.c_str());
    } // else: the internal state remains the same.
  }
}




sref_msgs MSRPA::castToPoseAndSubtract(const tiny_msgs point, const sref_msgs pose)
{
  sref_msgs output;
  output.pose.position.x = point.x - pose.pose.position.x;
  output.pose.position.y = point.y - pose.pose.position.y;
  output.pose.position.z = point.z - pose.pose.position.z;

  output.pose.orientation = pose.pose.orientation;
  return output;
}

// Function allowing you to publish messages to leaders from command line
void MSRPA::leader_subCallback(const ref_msgs::ConstPtr& msgs){
  // Test to see if leader
  // ROS_INFO("role: %d", role);
  if(role == 3) {
    if(((msgs->type.compare("circular") == 0) || (msgs->type.compare("square") == 0)) &&\
    msgs->trajectory.size() >= 7 &&\
    msgs->formation.size() >= 2) {
      reset_message = *msgs;
      reset_message.formation[1] = n;
      internal_state = *msgs;
      internal_state.formation[1] = n; // Note: we don't offer the capability of changing n yet.
      // ROS_INFO("This function callback was called");
    } else {
      ROS_INFO("ERROR: message to leaders was incorrect. Check the length of the vectors.");
    }
  }
}


bool MSRPA::test_messages_equal(const ref_msgs message1, const ref_msgs message2) {
  // Tests if two ref_msgs are equal
  if ((message1.type.compare(message2.type) == 0) && (message1.trajectory == message2.trajectory) && (message1.formation == message2.formation)){
    return true;
  } else {
    return false;
  }
}

// Testing purposes only
void MSRPA::print_cvec(){
  // ROS_INFO("cvec.size(): %d", cvec.size());
  
  if (cvec.size() > 0){
    for(int ii=0; ii < cvec.size(); ii++){
    ROS_INFO("cvec[%d].type: %s \n\
    cvec.trajectory = [%lf, %lf, %lf, %lf, %lf, %lf, %lf] \n\
    cvec.formation = [%lf, %lf]",
    ii, cvec[ii].type.c_str(),\
    cvec[ii].trajectory[0], cvec[ii].trajectory[1], cvec[ii].trajectory[2], cvec[ii].trajectory[3], cvec[ii].trajectory[4], cvec[ii].trajectory[5], cvec[ii].trajectory[6],\
    cvec[ii].formation[0], cvec[ii].formation[1]);
    }
  } else {
    ROS_INFO("Length of cvec is zero");
  }
}

// main function
int main(int argc, char **argv)
{

  // Initialize ros
  ros::init(argc, argv, "msrpa_node");

  // Create an object of MSRPA that handles everything
  MSRPA MS_RPA_Node;

  ros::spin();

  return 0;
}
