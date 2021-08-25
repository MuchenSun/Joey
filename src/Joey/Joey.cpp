#include "Joey/Joey.hpp"

namespace Joey {
    teleop::teleop(ros::NodeHandle *nh): nh_(*nh) {
        // read in parameters
        std::string joy_topic, cmd_topic;
        nh_.getParam("/joey_node/joy_topic", joy_topic);
        nh_.getParam("/joey_node/cmd_topic", cmd_topic);
        ROS_INFO_STREAM("subscribe to joystick topic: " << joy_topic);
        ROS_INFO_STREAM("publish to cmd topic: " << cmd_topic);

        // double max_lin_vel, max_ang_vel, lin_boost_ratio, ang_boost_ratio;
        nh_.getParam("/joey_node/max_lin_vel", max_lin_vel_);
        nh_.getParam("/joey_node/max_ang_vel", max_ang_vel_);
        nh_.getParam("/joey_node/lin_boost_ratio", lin_boost_ratio_);
        nh_.getParam("/joey_node/ang_boost_ratio", ang_boost_ratio_);
        
        // node config
        joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &teleop::joy_cb, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    }

    void teleop::publish_cmd() {
        cmd_pub_.publish(cmd_);
    }

    void teleop::joy_cb(const sensor_msgs::Joy::ConstPtr& joy_msg) {
        int stop_button = joy_msg->buttons[0];

        if(stop_button != 1) {
            double lin_scale = joy_msg->axes[1];
            double lin_boost = lin_boost_ratio_ + joy_msg->buttons[6] * (1-lin_boost_ratio_);
            double ang_scale = joy_msg->axes[2];
            double ang_boost = ang_boost_ratio_ + joy_msg->buttons[7] * (1-ang_boost_ratio_);

            cmd_.linear.x = max_lin_vel_ * lin_scale * lin_boost;
            cmd_.linear.y = 0.0;
            cmd_.linear.z = 0.0;
            cmd_.angular.x = 0.0;
            cmd_.angular.y = 0.0;
            cmd_.angular.z = max_ang_vel_ * ang_scale * ang_boost;
        } else {
            cmd_.linear.x = 0.0; 
            cmd_.linear.y = 0.0;
            cmd_.linear.z = 0.0;
            cmd_.angular.x = 0.0;
            cmd_.angular.y = 0.0;
            cmd_.angular.z = 0.0;
        }
    }
}
