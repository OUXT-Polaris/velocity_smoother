#ifndef VELOCITY_SMOOTHER_VELOCITY_SMOOTHER_H_INCLUDED
#define VELOCITY_SMOOTHER_VELOCITY_SMOOTHER_H_INCLUDED

// Headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <dynamic_reconfigure/server.h>

// Headers in this package
#include <velocity_smoother/VelocitySmootherConfig.h>

// Headers in Boost
#include <boost/optional.hpp>

namespace velocity_smoother
{
    class VelocitySmoother
    {
    public:
        VelocitySmoother(ros::NodeHandle nh,ros::NodeHandle pnh);
        ~VelocitySmoother();
    private:
        void update(geometry_msgs::TwistStamped twist);
        void twistStampedCallback(const geometry_msgs::TwistStamped::ConstPtr msg);
        void twistCallback(const geometry_msgs::Twist::ConstPtr msg);
        void paramCllback(velocity_smoother::VelocitySmootherConfig &config, uint32_t level);
        double k_longitudinal_;
        double k_lateral_;
        double k_vertical_;
        double k_roll_;
        double k_pitch_;
        double k_yaw_;
        bool use_twist_stamped_;
        boost::optional<geometry_msgs::TwistStamped> current_twist_;
        std::string input_topic_;
        ros::NodeHandle pnh_;
        ros::NodeHandle nh_;
        ros::Subscriber twist_sub_;
        ros::Publisher twist_pub_;
        dynamic_reconfigure::Server<velocity_smoother::VelocitySmootherConfig> server_;
        dynamic_reconfigure::Server<velocity_smoother::VelocitySmootherConfig>::CallbackType params_callback_func_;
    };
}

#endif  //VELOCITY_SMOOTHER_VELOCITY_SMOOTHER_H_INCLUDED