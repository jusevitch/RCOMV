/*
// ------------------------------------------------------------------
// WMSR node for r1 rover. 2D motions only.
//
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
    nh_private_.param<int>("n", n, 15);
    nh_private_.param<int>("k", k, 7);

    nh_private_.param<int>("idx", idx, 1);
    nh_private_.param<int>("role", role, 2);

    nh_private_.param<int>("F", F, 0);

    nh_private_.param<double>("x", x0, 0);
    nh_private_.param<double>("y", y0, 0);

    // Initialize msgs
    // refrence path:
    // 1. for leader agents, they will be assigned with desired values fron the launch file
    // 2. for other agents, they will be assigned with random values fron the launch file
    nh_private_.param<std::string>("path_type", inform_center_path.path_type, "circular");
    nh_private_.param<double>("xc", inform_center_path.xc, 0);
    nh_private_.param<double>("yc", inform_center_path.yc, 0);
    nh_private_.param<double>("R", inform_center_path.R, 4);
    nh_private_.param<double>("wd", inform_center_path.wd, 0.5);
    nh_private_.param<double>("t0", inform_center_path.t0, ros::Time().toSec());
    nh_private_.param<double>("R1", inform_center_path.R1, 4);
    nh_private_.param<double>("R2", inform_center_path.R2, 4);
    // 3. malicous path: assigned to different values from desired path
    mali_path.R = inform_center_path.R + 3;
    mali_path.xc = inform_center_path.cx + 1;
    mali_path.yc = inform_center_path.cy + 1;

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
    trajectory_msg.xc = x0;
    trajectory_msg.yc = y0;
    output_pub.publish(trajectory_msg);

    ros::Duration(0.1).sleep();
    }
    ROS_INFO("Simulation is ready, turn on WMSR node...");


    // Publisher: reference
    // pub topic is relative to the node namespace
    std::string pub_topic = "WMSR/ref";
    ref_pub = nh.advertise<path_msgs>(pub_topic, 10);pose.position.x
    // Publisher Timer with frequency: 10 Hz
    ref_pub_timer = nh.createTimer(ros::Duration(0.1),
                &WMSRNode::ref_pubCallback, this);


    // Subscribers: (msgs list to hold the msgs from subscibed topics)
    for (int i = 1; i <= k; i++) {
      // Initialize the msgs list that holds msgs from neighbor agents
      path_msgs ref_point;
      ref_lists.push_back(ref_point);

      // Initialize subscriber
      int sub_idx = (idx - i) > 0 ? (idx - i) : (n + idx -i);
      // sub topics are resolved in global namespace
      std::string sub_topic = "/ugv" + std::to_string(sub_idx) + "/WMSR/ref";

      ref_subs.push_back(nh.subscribe<ref_msgs>(sub_topic, 10,
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


//  Subscriber Callback Function: subscribes reference paths of other WMSR nodes
void WMSRNode::ref_subCallback(const path_msgs::ConstPtr& msgs, const int list_idx)
{
  ref_lists[list_idx].path_type = msgs->path_type;
  ref_lists[list_idx].xc = msgs->xc;
  ref_lists[list_idx].yc = msgs->yc;
  ref_lists[list_idx].wd = msgs->wd;
  ref_lists[list_idx].t0 = msgs->t0;
  ref_lists[list_idx].R = msgs->R;
  ref_lists[list_idx].R1 = msgs->R1;
  ref_lists[list_idx].R2 = msgs->R2;
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
    mali_path.R += 0.2;
    mali_path.R1 += 0.2;
    mali_path.R2 += 0.2;
    // cyber attack
    if (attack == 1) {
      inform_center_path = WMSRNode::WMSRAlgorithm(ref_lists);
    }
    // physical attack
    if (attack == 2) {
      inform_center_path = mali_path;
    }

    ref_pub.publish(mali_path);    // publish malicious path to other WMSR node
  }
}


// Helper Function::
path_msgs WMSRNode::WMSRAlgorithm(const std::vector<path_msgs> &list)
{
  path_msgs ref_path;

  std::vector<double> list_xc(k,0);
  std::vector<double> list_yc(k,0);
  std::vector<double> list_wd(k,0);
  std::vector<double> list_t0(k,0);
  std::vector<double> list_R(k,0);
  std::vector<double> list_R1(k,0);
  std::vector<double> list_R2(k,0);

  double w_xc, w_yc, w_wd, w_t0, w_R, w_R1, w_R2; // uniform weights

  double wav_xc = 0, wav_yc = 0, wav_wd = 0, wav_t0 = 0, wav_R = 0, wav_R1 = 0, wav_R2 = 0; // weighted averages

  double xc, yc, wd, t0, R, R1, R2;
  xc = inform_center_path.xc;
  yc = inform_center_path.yc;
  wd = inform_center_path.wd;
  t0 = inform_center_path.t0;
  R = inform_center_path.R;
  R1 = inform_center_path.R1;
  R2 = inform_center_path.R2;

  // scan the data
  for (int i = 0; i < k; i++) {
    list_xc[i] = list[i].xc;
    list_yc[i] = list[i].yc;
    list_wd[i] = list[i].wd;
    list_t0[i] = list[i].t0;
    list_R[i] = list[i].R;
    list_R1[i] = list[i].R1;
    list_R2[i] = list[i].R2;
  }

  //remove outliers
  w_xc = FilterOutlier(list_xc, k, xc, F);
  w_yc = FilterOutlier(list_yc, k, yc, F);
  w_wd = FilterOutlier(list_wd, k, wd, F);
  w_t0 = FilterOutlier(list_t0, k, t0, F);
  w_R = FilterOutlier(list_R, k, R, F);
  w_R1 = FilterOutlier(list_R1, k, R1, F);
  w_R2 = FilterOutlier(list_R2, k, R2, F);

  // update reference path of the current WMSR node
  // note: size increases by 1 due to the added states of the current WMSR node
  for (int j = 0; j < k+1; j++) {
    wav_xc += w_xc * list_xc[j];
    wav_yc += w_yc * list_yc[j];
    wav_wd += w_wd * list_wd[j];
    wav_t0 += w_t0 * list_t0[j];
    wav_R += w_R * list_R[j];
    wav_R1 += w_R1 * list_R1[j];
    wav_R2 += w_R2 * list_R2[j];
  }
  ref_path.path_type = inform_center_path.path_type;
  ref_path.xc = wav_xc;
  ref_path.yc = wav_yc;
  ref_path.wd = wav_wd;
  ref_path.t0 = wav_t0;
  ref_path.R = wav_R;
  ref_path.R1 = wav_R1;
  ref_path.R2 = wav_R2;

  return ref_path;
}


// Output Publisher Callback
void WMSRNode::out_pubCallback(const ros::TimerEvent& event)
{
  // calculate the node states in the formation
  WMSRNode::Formation();
  output_pub.publish(inform_formation_path);
}

// Helper Function: Create formation
void WMSRNode::Formation()
{
  const float DEG_2_RAD = M_PI / 180.0;
  double d_theta = 360 / 5 * DEG_2_RAD;
  double d_radius = 1;
  int group_size = n / 5;

  double theta = (idx-1) / group_size * d_theta;
  double radius = (idx-1) % group_size * d_radius + 2;

  inform_formation_path.xc = inform_center_path.xc + radius * cos(theta);
  inform_formation_path.yc = inform_center_path.yc + radius * sin(theta);
  inform_formation_path.path_type = inform_center_path.path_type;
  inform_formation_path.wd = inform_center_path.wd;
  inform_formation_path.t0 = inform_center_path.t0;
  inform_formation_path.R = inform_center_path.R;
  inform_formation_path.R1 = inform_center_path.R1;
  inform_formation_path.R2 = inform_center_path.R2;
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

  // return the weights
  return 1.0 / double(valid_size);
}
