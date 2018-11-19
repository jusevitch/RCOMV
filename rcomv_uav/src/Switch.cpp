
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <gazebo_msgs/SetPhysicsProperties.h>
#include <gazebo_msgs/GetPhysicsProperties.h>

#include <thread>
#include <chrono>

int main (int argc, char** argv) {
  ros::init(argc, argv, "UpperLevel_Switch_Node");
  ros::NodeHandle nh;

  ROS_INFO("Started UpperLevel_Switch_Node.");

  // Publisher: publish "switch" topic thats turns on the WMSR node
  std_msgs::Bool switch_signal;
  switch_signal.data = false;
  ros::Publisher switch_pub = nh.advertise<std_msgs::Bool>("switch", 10);
  switch_pub.publish(switch_signal);

  // service clients of gazebo set/get physics properties services
  // These services set/return the properties of the physics engine used in simulation
  // which are last steps when starting gazebo.
  ros::ServiceClient physics_reconfigure_set_client_ = nh.serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");
  ros::ServiceClient physics_reconfigure_get_client_ = nh.serviceClient<gazebo_msgs::GetPhysicsProperties>("/gazebo/get_physics_properties");

  // try to unpause gazebo for 10 seconds
  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;
  while (!unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }
  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  }
  else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  // Wait until the services are successfully set up
  physics_reconfigure_set_client_.waitForExistence();
  physics_reconfigure_get_client_.waitForExistence();


  // publish "switch signal" to all WMSR nodes
  switch_signal.data = true;
  switch_pub.publish(switch_signal);
  ROS_INFO("Simulation is ready, publishing switch signal and waking up WMSR nodes...");


  while (ros::ok()) {
    ROS_INFO("Simultion Running...");
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  ros::spin();
  return 0;
}
