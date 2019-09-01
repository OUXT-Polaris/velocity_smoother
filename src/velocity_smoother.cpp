/**
 * @file velocity_smoother.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief Implimentation of Velocity Smoother Class
 * @version 0.1
 * @date 2019-08-31
 * 
 * @copyright Copyright (c) 2019
 * 
 */

// headers in this package
#include <velocity_smoother/velocity_smoother.h>

namespace velocity_smoother
{
    VelocitySmoother::VelocitySmoother(ros::NodeHandle nh,ros::NodeHandle pnh)
    {
        nh_ = nh;
        pnh_ = pnh;
        pnh_.param<std::string>("input_topic", input_topic_, "input");
    }

    VelocitySmoother::~VelocitySmoother()
    {

    }

    void VelocitySmoother::paramCllback(velocity_smoother::VelocitySmootherConfig &config, uint32_t level)
    {
        twist_sub_.shutdown();
        twist_pub_.shutdown();
        k_longitudinal_ = config.k_longitudinal;
        k_lateral_ = config.k_lateral;
        k_vertical_ = config.k_vertical;
        k_roll_ = config.k_roll;
        k_pitch_ = config.k_pitch;
        k_yaw_ = config.k_yaw;
        use_twist_stamped_ = config.use_twist_stamped;
        if(use_twist_stamped_)
        {
            twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(input_topic_,1);
            twist_sub_ = nh_.subscribe(input_topic_,1,&VelocitySmoother::twistStampedCallback,this);
        }
        else
        {
            twist_pub_ = nh_.advertise<geometry_msgs::Twist>(input_topic_,1);
            twist_sub_ = nh_.subscribe(input_topic_,1,&VelocitySmoother::twistCallback,this);
        }
        return;
    }

    void VelocitySmoother::twistStampedCallback(const geometry_msgs::TwistStamped::ConstPtr msg)
    {
        update(*msg);
        return;
    }

    void VelocitySmoother::twistCallback(const geometry_msgs::Twist::ConstPtr msg)
    {
        std_msgs::Header header;
        geometry_msgs::TwistStamped twist;
        twist.header = header;
        twist.twist = *msg;
        update(twist);
        return;
    }

    void VelocitySmoother::update(geometry_msgs::TwistStamped twist)
    {
        if(current_twist_)
        {
            current_twist_ = twist;
        }
        else
        {
            current_twist_->header = twist.header;
            current_twist_->twist.linear.x = k_longitudinal_*current_twist_->twist.linear.x + (1-k_longitudinal_)*twist.twist.linear.x;
            current_twist_->twist.linear.y = k_lateral_*current_twist_->twist.linear.y + (1-k_lateral_)*twist.twist.linear.y;
            current_twist_->twist.linear.z = k_vertical_*current_twist_->twist.linear.z + (1-k_vertical_)*twist.twist.linear.z;
            current_twist_->twist.angular.x = k_roll_*current_twist_->twist.angular.x + (1-k_roll_)*current_twist_->twist.angular.x;
            current_twist_->twist.angular.y = k_pitch_*current_twist_->twist.angular.y + (1-k_pitch_)*current_twist_->twist.angular.y;
            current_twist_->twist.angular.z = k_yaw_*current_twist_->twist.angular.z + (1-k_yaw_)*current_twist_->twist.angular.z;
            if(use_twist_stamped_)
            {
                twist_pub_.publish(*current_twist_);
            }
            else
            {
                twist_pub_.publish(current_twist_->twist);
            }
        }
        return;
    }
}