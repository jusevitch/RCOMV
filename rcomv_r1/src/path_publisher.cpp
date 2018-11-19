
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <gazebo_msgs/SetPhysicsProperties.h>
#include <gazebo_msgs/GetPhysicsProperties.h>

#include <chrono>
#include <thread>
#include <stdlib.h>
#include <math.h>


int main (int argc, char** argv) {
  ros::init(argc, argv, "path publisher");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private_("~");

  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("ref", 10);

  // service clients of gazebo set/get physics properties services
  // These services set/return the properties of the physics engine used in simulation
  // which are last steps when starting gazebo.
  ros::ServiceClient physics_reconfigure_set_client_ =
    nh.serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");
  ros::ServiceClient physics_reconfigure_get_client_ =
    nh.serviceClient<gazebo_msgs::GetPhysicsProperties>("/gazebo/get_physics_properties");


  // initialize parameters
  double radius;
  double peak;
  double cx,cy;
  geometry_msgs::PoseStamped ref_state;

  nh_private_.param<double>("radius", radius, 2);
  nh_private_.param<double>("cx", cx, 0);
  nh_private_.param<double>("cy", cy, 0);
  nh_private_.param<double>("peak", peak, 0);

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

  ROS_INFO("Start Path Publisher.");

  // frequency: 10Hz
  ros::Rate rate(10);
  double theta = 0;
  while(ros::ok()) {
    ref_state.pose.position.x = cx + radius * cos(theta);
    ref_state.pose.position.y = cy + radius * sin(theta);
    ref_state.header.stamp = ros::Time::now();
    theta += 0.1/radius;
    pub.publish(ref_state);

    ros::spinOnce();
    rate.sleep();
  }


  ros::shutdown();
  return 0;
}
