#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <stdlib.h>
#include <math.h>
#include <vector>

bool sim_running = false;
void callback(const sensor_msgs::ImuPtr& msg) {
  sim_running = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "circular_path_uav");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private_("~");

  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  ROS_INFO("Started Circular_Path_Publisher.");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // initialize circular path parameters
  double cx, cy, cz, radius, shift_angle;
  std::vector<double> input_stream;

  nh_private_.param("path", input_stream, std::vector<double> (4,0));
  nh_private_.param("shift_angle", shift_angle, 0.0);
  cx = input_stream[0];
  cy = input_stream[1];
  cz = input_stream[2];
  radius = input_stream[3];
  ROS_INFO("Receiving data from launch file...");
  ROS_INFO("The circular path is characterized by [center_xyz, radius]: [%f,%f,%f,%f]",cx,cy,cz,radius);

  // Trying to unpause Gazebo for 5 seconds.
  while (i <= 15 && !unpaused) {
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
  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(5.0).sleep();

  // The IMU is used, to determine if the simulator is running or not.
  ros::Subscriber sub = nh.subscribe("imu", 10, &callback);

  ROS_INFO("Wait for simulation to become ready...");
  while (!sim_running && ros::ok()) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("...ok");


  // Start Cirular Path Following
  ROS_INFO("Start publishing waypoints.");

  const float DEG_2_RAD = M_PI / 180.0;

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  //double radius = 5;
  double theta = 0 + shift_angle;
  Eigen::Vector3d waypoint_position(cx + radius * cos(theta), cy + radius * sin(theta), cz);
  double desired_yaw = 0 * DEG_2_RAD;



  // update the waypoint at 5HZ
  ros::Rate rate(5);
  trajectory_msg.header.stamp = ros::Time::now();

  // wait for 30 sec, until it reaches inital condition
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
    waypoint_position, desired_yaw, &trajectory_msg);
  ROS_INFO("Publishing waypoint : [%f, %f, %f]", waypoint_position.x(),
          waypoint_position.y(), waypoint_position.z());
  trajectory_pub.publish(trajectory_msg);
  ros::Duration(10.0).sleep();

  while (sim_running && ros::ok()) {
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      waypoint_position, desired_yaw, &trajectory_msg);

    ROS_INFO("Publishing waypoint : [%f, %f, %f]", waypoint_position.x(),
              waypoint_position.y(), waypoint_position.z());

    trajectory_pub.publish(trajectory_msg);

    //update
    theta = fmod((theta+0.2), (2*M_PI));
    waypoint_position.x() = cx + radius * cos(theta);
    waypoint_position.y() = cy + radius * sin(theta);
    //swaypoint_position.z() += 0.1;

    // wait until next iteration
    rate.sleep();
  }

  ROS_INFO("Simulation Session Ends!");
  ros::shutdown();
  ROS_INFO("...Shutdown");
  return 0;

}
