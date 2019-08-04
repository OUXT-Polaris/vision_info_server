// Headers in this package
#include <vision_info_server/vision_info_server.h>

// Headers in ROS
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vision_info_server_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    //JskToVisionMsgs converter(nh,pnh);
    ros::spin();
    return 0;
}