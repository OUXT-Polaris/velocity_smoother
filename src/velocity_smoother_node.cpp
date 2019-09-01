/**
 * @file velocity_smoother_node.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief Main function for Velocity Smoother Node
 * @version 0.1
 * @date 2019-08-31
 * 
 * @copyright Copyright (c) 2019
 * 
 */

// headers for ros
#include <ros/ros.h>

// headers in this package
#include <velocity_smoother/velocity_smoother.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "velocity_smoother_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    velocity_smoother::VelocitySmoother smoother(nh,pnh);
    ros::spin();
    return 0;
}