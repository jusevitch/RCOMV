#include <ros/ros.h>
#include "pidctrl.h"

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "Controller");

    // Create an object of SubscribAndPublish that will take care of everything
    SubscribeAndPublish controller;

    ros::spin();

    return 0;
}
