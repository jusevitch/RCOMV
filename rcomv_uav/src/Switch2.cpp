#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <vector>


bool checkStatus(const std::vector<bool> uavs_status, int n) {
  for (int i=0; i<n; i++) {
      if (!uavs_status[i]) {
        ROS_INFO("at least one uav is not ready...");
        return false;
      }
  }
  return true;
}


class SwitchNode {

public:
  SwitchNode():nh_private_("~"){
    // access data from launch file
    nh_private_.param("n", n, 15);

    // initialize variables
    uavs_status = std::vector<bool>(n,false);
    switch_signal.data = false;

    // initialize Publisher
    ros::Publisher switch_pub = nh.advertise<std_msgs::Bool>("switch", 10);

    // initialize Subscribers
    for (int i=0; i<n; i++) {
      subs[i] = nh.subscribe<std_msgs::Bool>("/uav"+std::to_string(i+1)+"/status", 10,
                  boost::bind(&SwitchNode::subsCallback,this,_1,i));
    }

    //
    bool switch_status = false;
    while (!switch_status && ros::ok()) {
      ROS_INFO("uavs are not ready...");
      switch_status = checkStatus(uavs_status, n);
    }
    ROS_INFO("all uavs are ready, waking up WMSR nodes");
    switch_signal.data = switch_status;
    switch_pub.publish(switch_signal);

  }

  ~SwitchNode() {
      
  }

private:
  // ROS NodeHandle
  ros::NodeHandle nh;
  ros::NodeHandle nh_private_;
  // messages to publish or subscribe on topics
  std_msgs::Bool switch_signal;
  // Subscribers
  std::vector<ros::Subscriber> subs;
  // Publishers
  ros::Publisher switch_pub;
  // callback functions
  void subsCallback(const std_msgs::Bool::ConstPtr& msg, const int idx) {
    uavs_status[idx] = msg->data;
  }
  // private variables
  int n;
  std::vector<bool> uavs_status;

}; // end of class


int main (int argc, char** argv) {
  ros::init(argc, argv, "UpperLevel_Switch_Node");

  SwitchNode switch_node;

  ros::spin();

  return 0;
}
