#include "Joey/Joey.hpp"

namespace Joey {
    teleop::teleop(ros::NodeHandle *nh): nh_(*nh) {
        // read in parameters
        std::string joy_topic, cmd_topic;
        nh_.getParam("/joey_node/joy_topic", joy_topic);
        nh_.getParam("/joey_node/cmd_topic", cmd_topic);
        ROS_INFO_STREAM("subscribe to joystick topic: " << joy_topic);
        ROS_INFO_STREAM("publish to cmd topic: " << cmd_topic);

        nh_.getParam("/joey_node/max_lin_vel", max_lin_vel_);
        nh_.getParam("/joey_node/max_ang_vel", max_ang_vel_);
        nh_.getParam("/joey_node/lin_boost_ratio", lin_boost_ratio_);
        nh_.getParam("/joey_node/ang_boost_ratio", ang_boost_ratio_);
        
        nh_.getParam("/joey_node/fractional_boost", fractional_boost_);
        nh_.getParam("/joey_node/lin_vel_axis", lin_vel_axis_);
        nh_.getParam("/joey_node/lin_boost_axis", lin_boost_axis_);
        nh_.getParam("/joey_node/ang_vel_axis", ang_vel_axis_);
        nh_.getParam("/joey_node/ang_boost_axis", ang_boost_axis_);
        nh_.getParam("/joey_node/shutdown_axis", shutdown_axis_);
        nh_.getParam("/joey_node/recover_axis", recover_axis_);

        // node config
        joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &teleop::joy_cb, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        shutdown_ = false;
    }

    void teleop::publish_cmd() {
        cmd_pub_.publish(cmd_);
    }

    void teleop::joy_cb(const sensor_msgs::Joy::ConstPtr& joy_msg) {
        if(joy_msg->buttons[recover_axis_] == 1) { // recover from shutdown mode
            shutdown_ = false;
            ROS_INFO_STREAM("Joystick is recovered from shutdown mode");
        }
        if(joy_msg->buttons[shutdown_axis_] == 1) { // enable shutdown mode
            shutdown_ = true;
            ROS_INFO_STREAM("Joystick is in shutdown mode");
        }
        
        if(!shutdown_) {
            double lin_scale = joy_msg->axes[lin_vel_axis_];
            double ang_scale = joy_msg->axes[ang_vel_axis_];
            double lin_boost, ang_boost;
            if(fractional_boost_) {
                lin_boost = lin_boost_ratio_ + (1-joy_msg->axes[lin_boost_axis_]) * (1-lin_boost_ratio_) / 2;
                ang_boost = ang_boost_ratio_ + (1-joy_msg->axes[ang_boost_axis_]) * (1-ang_boost_ratio_) / 2;
            } else {
                lin_boost = lin_boost_ratio_ + joy_msg->buttons[lin_boost_axis_] * (1-lin_boost_ratio_);
                ang_boost = ang_boost_ratio_ + joy_msg->buttons[ang_boost_axis_] * (1-ang_boost_ratio_);
            }
            
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
