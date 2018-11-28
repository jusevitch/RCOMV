/*
// ------------------------------------------------------------------
// WMSR node for r1 rover. 2D motions only.
//
//------------------------------------------------------------------
*/

#include "wmsr_node.h"

// Constructor
WMSRNode::WMSRNode()
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

    // Initialize msgs
    inform_states.header.stamp = ros::Time::now();
    if (demo == 3 && role == 3) {
      inform_states.pose.position.x = cx;
      inform_states.pose.position.y = cy;
    }
    else {
      inform_states.pose.position.x = x0;
      inform_states.pose.position.y = y0;
    }

    mali_states.header.stamp = ros::Time::now();
    mali_states.pose.position.x = x0;
    mali_states.pose.position.y = y0;


    // Publisher: command trajectory for the controller
    output_pub = nh.advertise<ref_msgs>(
              "ref", 10);
    // Publisher Timer
    // frequency: 10 Hz
    out_pub_timer = nh.createTimer(ros::Duration(0.1),
                &WMSRNode::out_pubCallback, this);


    // Subscriber: subscribe the switch topic
    switch_sub = nh.subscribe("/switch", 10, &WMSRNode::switch_subCallback, this);
    // wait until receiving the turn on signal
    while(switch_signal.data == false) {

      ros::spinOnce();
      ROS_INFO("Wait until all robots are spawned, stay at initial positions.");


      // make the robot stay at the initial position.
      ref_msgs trajectory_msg;
      trajectory_msg.header.stamp = ros::Time::now();
      trajectory_msg.pose.position.x = x0;
      trajectory_msg.pose.position.y = y0;
       output_pub.publish(trajectory_msg);

      ros::Duration(0.1).sleep();
    }
    //
    ROS_INFO("Simulation is ready, turn on WMSR node...");


    // Publisher: reference
    // pub topic is relative to the node namespace
    std::string pub_topic = "WMSR/ref";
    ref_pub = nh.advertise<ref_msgs>(pub_topic, 10);
    // Publisher Timer
    // frequency: 10 Hz
    ref_pub_timer = nh.createTimer(ros::Duration(0.1),
                &WMSRNode::ref_pubCallback, this);


    // Subscribers: (msgs list to hold the msgs from subscibed topics)
    for (int i = 1; i <= k; i++) {
      // Initialize the msgs list that holds msgs from neighbor agents
      ref_msgs ref_point;
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
void WMSRNode::ref_subCallback(const ref_msgs::ConstPtr& msgs, const int list_idx)
{
  ref_lists[list_idx].header = msgs->header;
  ref_lists[list_idx].pose = msgs->pose;
}

// inform states Publisher Callback
void WMSRNode::ref_pubCallback(const ros::TimerEvent& event)
{
  // Role: 1 = Malicious, 2 = Normal, 3 = Leader
  // Leader nodes
  // Static Formation: inform states are static
  if (role == 3) {
      inform_states.header.stamp = ros::Time::now();
      ref_pub.publish(inform_states);    // publish inform states to other WMSR node
  }
  // Normal nodes
  // implement WMSR algorithm to update inform states
  else if (role == 2) {
    inform_states = WMSRNode::WMSRAlgorithm(ref_lists);
    ref_pub.publish(inform_states);    // publish inform states to other WMSR node
  }
  // Malicious nodes
  // implement WMSR algorithm to calculate the new inform states
  // If it's a CYBER attack, update the inform states to the new inform states, but publish some malicious states
  // If it's a PHYSICAL attack, update the inform states to some malicious states, and publish the malicious states
  else {
    // cyber attack
    //inform_states = WMSRNode::WMSRAlgorithm(ref_lists);

    mali_states.header.stamp = ros::Time::now();
    mali_states.pose.position.x += 0.1;
    mali_states.pose.position.y += 0.1;
    //mali_states.point.z = std::max(0.0, mali_states.point.z-1);

    // physical attack
    inform_states = mali_states;

    ref_pub.publish(mali_states);    // publish malicious states to other WMSR node
  }

}

// Output Publisher Callback
void WMSRNode::out_pubCallback(const ros::TimerEvent& event)
{
  // calculate the node states in the formation
  WMSRNode::Formation();

  // output msg
  ref_msgs trajectory_msg;

  double x, y;
  // 1D motion in x direction
  if (demo == 1) {
    x = inform_formation_states.pose.position.x;
    y = y0;
  }
  // 1D motion in y direction
  else if (demo == 2) {
    x = x0;
    y = inform_formation_states.pose.position.y;
  }
  // 2D motion
  else {
    x = inform_formation_states.pose.position.x;
    y = inform_formation_states.pose.position.y;
  }

  trajectory_msg.header.stamp = ros::Time::now();
  trajectory_msg.pose.position.x = x;
  trajectory_msg.pose.position.y = y;

  output_pub.publish(trajectory_msg);
}

// Helper Function::
ref_msgs WMSRNode::WMSRAlgorithm(const std::vector<ref_msgs> &list)
{
  ref_msgs ref_states;

  std::vector<double> listx(k,0);
  std::vector<double> listy(k,0);

  double wx, wy; // uniform weights

  double wav_x = 0, wav_y = 0; // weighted averages

  double x, y;
  x = inform_states.pose.position.x;
  y = inform_states.pose.position.y;

  // scan the data
  for (int i = 0; i < k; i++) {
    listx[i] = list[i].pose.position.x;
    listy[i] = list[i].pose.position.y;
  }

  //remove outliers
  wx = FilterOutlier(listx, k, x, F);
  wy = FilterOutlier(listy, k, y, F);

  // update inform states of the current WMSR node
  // note: size increases by 1 due to the added states of the current WMSR node
  for (int j = 0; j < k+1; j++) {
    wav_x += wx * listx[j];
    wav_y += wy * listy[j];
  }

  ref_states.header.stamp = ros::Time::now();
  ref_states.pose.position.x = wav_x;
  ref_states.pose.position.y = wav_y;

  return ref_states;
}

// Helper Function: Create formation
void WMSRNode::Formation()
{
  // 1D motion
  if (demo == 1 || demo == 2) {
    inform_formation_states.pose = inform_states.pose;
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

    inform_formation_states.pose.position.x = inform_states.pose.position.x + radius * cos(theta);
    inform_formation_states.pose.position.y = inform_states.pose.position.y + radius * sin(theta);
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


// main function
int main(int argc, char **argv) {

  // Initialize ros
  ros::init(argc, argv, "WMSR_Node");

  // Create an object of WMSRNode that handles everything
  WMSRNode WMSR_Node;

  ros::spin();

  return 0;
}
