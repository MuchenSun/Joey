#ifndef JOEY_INCLUDE_GUARD_HPP
#define JOEY_INCLUDE_GUARD_HPP

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

namespace Joey
{
    class teleop {
    public:
        teleop(ros::NodeHandle *nh);
        void publish_cmd();

    private:
        ros::NodeHandle nh_;
        ros::Publisher cmd_pub_;
        ros::Subscriber joy_sub_;
        geometry_msgs::Twist cmd_;
    
        double max_lin_vel_, max_ang_vel_, lin_boost_ratio_, ang_boost_ratio_;
        bool shutdown_;

        void joy_cb(const sensor_msgs::Joy::ConstPtr& joy_msg);
    };
}

#endif
