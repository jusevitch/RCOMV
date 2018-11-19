

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


// main function
int main(int argc, char **argv) {

  // Initialize ros
  ros::init(argc, argv, "WMSR_Node");

  // Create an object of WMSRNode that handles everything
  WMSRNode WMSR_Node;

  ros::spin();

  return 0;
}
