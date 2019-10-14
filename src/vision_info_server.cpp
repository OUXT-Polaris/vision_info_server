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

namespace vision_info_server
{
    /**
    * @brief Constructor of VisionInfoServer
    */
    VisionInfoServer::VisionInfoServer(const rclcpp::NodeOptions& options)
    : rclcpp::Node("vision_info_server", options)
    {
        declare_parameter("xml_path","");
        if(!has_parameter("xml_path"))
        {
            RCLCPP_ERROR(get_logger(),"param xml_path does not set.");
            return;
        }
        std::string xml_path = get_parameter("xml_path").get_value<std::string>();
        bool result = parser_.parseFromFile(xml_path);
        if(!result)
        {
            RCLCPP_ERROR(get_logger(),"Error Message"); 
            return;
        }
        vision_info_pub_ = create_publisher<vision_msgs::msg::VisionInfo>("vision_info", rclcpp::QoS(10).transient_local());
        vision_msgs::msg::VisionInfo msg;
        std::string ns = get_name();
        msg.database_location = ns + "/class_meta_info";
        rclcpp::Clock ros_clock(RCL_ROS_TIME);
        msg.header.stamp = ros_clock.now();
        vision_info_pub_->publish(msg);
        declare_parameter("class_meta_info",parser_.getClassMetaString());
    }

    /**
    * @brief Destructor
    * 
    */
    VisionInfoServer::~VisionInfoServer()
    {

    }
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(vision_info_server::VisionInfoServer)