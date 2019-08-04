/**
 * @file vision_info_server.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief implimentation of VisionInfoServer Class
 * @version 0.1
 * @date 2019-08-05
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include <vision_info_server/vision_info_server.h>

/**
 * @brief Constructor of VisionInfoServer
 */
VisionInfoServer::VisionInfoServer(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("xml_path", xml_path_, "xml_path");
    ROS_ASSERT(parser_.parseFromFile(xml_path_));
    vision_info_pub_ = pnh_.advertise<vision_msgs::VisionInfo>("vision_info",1,true);
    publish();
}

/**
 * @brief Destructor
 * 
 */
VisionInfoServer::~VisionInfoServer()
{

}

/**
 * @brief Publish data as ROS message.
 * 
 */
void VisionInfoServer::publish()
{
    pnh_.setParam("class_meta_info", parser_.getClassMetaString());
    vision_msgs::VisionInfo msg;
    msg.database_location = pnh_.getNamespace() + "/class_meta_info";
    msg.header.stamp = ros::Time::now();
    vision_info_pub_.publish(msg);
}