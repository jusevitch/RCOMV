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
    //nh_private_.param<double>("t0", inform_center_path.t0, ros::Time().toSec());
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


// main function: create a WMSRNode class type that handles everything
int main(int argc, char** argv) {
  ros::init(argc, argv, "path_trajectory_WMSRNode");

  WMSRNode path_trajectory_WMSR_node;

  ros::spin();

  return 0;
}
