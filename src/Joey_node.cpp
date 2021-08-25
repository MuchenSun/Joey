#include "Joey/Joey.hpp"
#include <iostream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "joey_node");
    
    ROS_INFO_STREAM("Joey node started.");

    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    Joey::teleop my_teleop(&nh);

    while(ros::ok()) {
        my_teleop.publish_cmd();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
