#ifndef VISION_INFO_SERVER_VISION_INFO_SERVER_H_INCLUDED
#define VISION_INFO_SERVER_VISION_INFO_SERVER_H_INCLUDED

// Headers in ROS
#include <ros/ros.h>

class VisionInfoServer
{
public:
    VisionInfoServer(ros::NodeHandle nh,ros::NodeHandle pnh);
};

#endif  //VISION_INFO_SERVER_VISION_INFO_SERVER_H_INCLUDED