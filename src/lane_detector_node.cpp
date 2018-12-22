//ROS Relevant Libraries
#include "ros/ros.h"

//Customed classes
#include "laneDetectorNode.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lane_detector_node");
    ros::NodeHandle n;
    laneDetectorNode node = laneDetectorNode(n);
    ros::spin();
    return 0;
}