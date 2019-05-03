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

  nh_private_.param<int>("idx", idx, 1);
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

  // Common namespace of all nodes. This variable helps the node know what topics to subscribe to 
  //  to receive MS-RPA information from other nodes in the network.
  nh_private_.param<std::string>("common_namespace", common_namespace, "/ugv");

  NANMSG.type = "circular";
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
    reference.type = "circular";
    reference.trajectory = {t0, xc, yc, Rad, wd, phi0};
    reference.formation = {Rf, static_cast<double>(n)};
    inform_states = reference;
  }
  else if (role == 1)
  {
    reference = get_malicious_reference();
    inform_states = reference;
  }
  else
  {
    inform_states = NANMSG;
    reference = NANMSG;
  }

  control = reference;
  initialize_cvec();
  Calc_Laplacian();

  //we are not publishing the goals to the PID controller anymore so it is 100% irrelevant

  // Subscriber: subscribe the switch topic
  switch_sub = nh.subscribe("/switch", 10, &MSRPA::switch_subCallback, this);
  // wait until receiving the turn on signal
  while (switch_signal.data == false)
  {

    ros::spinOnce();
    ROS_INFO("Wait until all robots are spawned, stay at initial positions.");

    ros::Duration(0.1).sleep();
  }

  ROS_INFO("Simulation is ready, turn on MSRPA node...");

  if(role == 3){
    leader_cmd_sub = nh.subscribe<ref_msgs>("/leader_cmd", 1, &MSRPA::leader_subCallback, this);
  }

  // The "ref" topic is the topic which should be streamed to the IO_control_collision node
  out_pub = nh.advertise<ref_msgs>("ref", 10);

  // Publisher: reference
  // pub topic is relative to the node namespace
  // The "MSRPA/ref" topic is for MSRPA nodes to send values to their outneighbors
  std::string pub_topic = "MSRPA/ref";
  ref_pub = nh.advertise<ref_msgs>(pub_topic, 2);
  // Publisher Timer
  // frequency: 10 Hz
  ref_pub_timer = nh.createTimer(ros::Duration(0.1),
                                 &MSRPA::ref_pubCallback, this);

  // Subscribers: (msgs list to hold the msgs from subscibed topics)
  // Why does i start from 1? This is wrong.
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

//  Subscriber Callback Function: subscribes reference paths of other nodes
void MSRPA::ref_subCallback(const ref_msgs::ConstPtr &msgs, const int list_idx)
{
  cvec[list_idx].type = msgs->type;
  cvec[list_idx].trajectory = msgs->trajectory;
  cvec[list_idx].formation = msgs->formation;
}

// inform states Publisher Callback
void MSRPA::ref_pubCallback(const ros::TimerEvent &event)
{

  Consensus(idx - 1);
  inform_states = cvec[idx - 1]; // own index
  if (iteration % eta == 0)
    reference = control;
  out_pub.publish(reference);     // updated reference value
  ref_pub.publish(inform_states); //MSRPA messages
  //ROS_INFO("Iteration: [%ld]", iteration);
  //ROS_INFO("cvec [%lf]", cvec[idx-1].x);
}

void MSRPA::initialize_cvec()
{
  cvec.resize(n);
  for (int i = 0; i < n; i++)
  {
    cvec[i] = NANMSG;
  }
}

sref_msgs MSRPA::update_reference(const sref_msgs reference, const sref_msgs control)
{
  sref_msgs output;

  output.pose.position.x = reference.pose.position.x + control.pose.position.x;
  output.pose.position.y = reference.pose.position.y + control.pose.position.y;
  output.pose.position.z = reference.pose.position.z + control.pose.position.z;
  output.pose.orientation.x = reference.pose.orientation.x + control.pose.orientation.x;
  output.pose.orientation.y = reference.pose.orientation.y + control.pose.orientation.y;
  output.pose.orientation.z = reference.pose.orientation.z + control.pose.orientation.z;
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

std::vector<FMatrix> MSRPA::BFunc()
{
  std::vector<FMatrix> Output;
  Output.resize(9);
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

      if (cvec[d].type == "circular")
        Output[6][j][d] = 0.0;
    }
  }
  return Output;
}

void MSRPA::Consensus(int i)
{ //are you passing an index of idx-1??
  iteration += 1;

  if (iteration == 1)
    B = BFunc(); //already creating the B matrix in case

  if (iteration > 1)
  {

    if ((iteration % eta) == (eta - 1))
    {
      if (role == 1)
      {

        control = get_malicious_reference();
      }
    }

    if (role == 2)
    {
      FMatrix follow_ref;
      FMatrix ref2;
      std::vector<double> p;
      //follow_ref.resize(n);
      //populate B matrix entries onto pair vector
      int z = 0;
      for (int t = 0; t < n; t++)
      {
        if (!std::isnan(B[0][i][t]) && t != i)
        {
          follow_ref.push_back(p);
          follow_ref.at(z).resize(9);
          for (int o = 0; o < 9; o++)
          {
            follow_ref[z][o] = B[o][i][t];
          }
          z += 1;
          //ROS_INFO("B for [%d, %d] for agent %d: [%lf, %lf] %d", i,t, i+1, B[6][i][t], B[7][i][t], std::isnan(B[0][i][t]));
        }
      }
      if (z > 0)
      {
        std::sort(follow_ref.begin(), follow_ref.end());
        std::vector<double> vec = follow_ref[0];
        double xref = vec[0];
        double yref = vec[1];
        std::vector<int> counts;
        int myCount = 1;
        //ref2.push_back(follow_ref[0]);
        //ROS_INFO("ref2 [%lf, %lf]", follow_ref[0][0], follow_ref[0][1]);

        if (follow_ref.size() > 1)
        {

          for (int j = 1; j < follow_ref.size(); j++)
          {
            std::vector<double> comp = follow_ref[j];
            if (((comp[0] - xref) < 0.1) && ((comp[1] - yref) < 0.1))
            { //we have another occurrence!
              myCount += 1;
            }
            else
            {

              counts.push_back(myCount);
              myCount = 1;
              vec[0] = xref;
              vec[1] = yref;
              xref = comp[0];
              yref = comp[1];
              ref2.push_back(vec);
            }
          }
        }
        ref2.push_back(follow_ref[follow_ref.size() - 1]);
        counts.push_back(myCount);

        if (!follow_ref.empty())
        {
          std::vector<int>::iterator maximum;
          maximum = std::max_element(std::begin(counts), std::end(counts));
          int max_idx = std::distance(counts.begin(), maximum);
          int max_counts = *maximum;
          if (idx == 3)
          {
            //ROS_INFO("MAX_COUNTS: %d, %d, %ld, %lf, %lf", max_counts, max_idx, counts.size(), ref2[max_idx][0], ref2[max_idx][1]);
          }

          if (max_counts >= (F + 1))
          {
            auto c = ref2[max_idx];
            cvec[i].trajectory.clear();
            cvec[i].formation.clear();
            for (int o = 0; o < 7; o++)
            {
              cvec[i].trajectory.push_back(c[o]);
            }
            if (cvec[i].type == "circular")
              cvec[i].trajectory[6] = 0.0;
            for (int o = 7; o < 9; o++)
            {
              cvec[i].formation.push_back(c[o]);
            }
            control = cvec[i];
          }
        }
      }
    }
  }

  //We are only updating x at this time step with u(t-1)
  if (iteration % eta == 0 && role == 2)
  {
    cvec[i] = NANMSG;
  }

  if (role == 3)
  { //leader
    //ref_msgs leadp=get_leader_reference(floor(iteration/eta)+1);
    //cvec[i] = leadp.pose.position;
    cvec[i] = reference;
  }
  else if (role == 1)
  { //malicious
    ref_msgs mali = get_malicious_reference();
    cvec[i] = mali;
  }

  //UPDATE B
  B = BFunc();
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

void MSRPA::leader_subCallback(const ref_msgs::ConstPtr& msgs){
  // Test to see if leader
  ROS_INFO("role: %d", role);
  if(role == 3) {
    if(((msgs->type.compare("circular") == 0) || (msgs->type.compare("square") == 0)) &&\
    msgs->trajectory.size() >= 7 &&\
    msgs->formation.size() >= 2) {
      control = *msgs;
      control.formation[1] = n; // Note: we don't offer the capability of changing n yet.
    } else {
      ROS_INFO("ERROR: message to leaders was incorrect. Check the length of the vectors.");
    }
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
