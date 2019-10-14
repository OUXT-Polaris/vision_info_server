/**
 * @file vision_info_parser.h
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief Definition of the VisionInfo Parser Class
 * @version 0.1
 * @date 2019-08-05
 * 
 * @copyright Copyright (c) 2019
 * 
 */

// Headers in Boost
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

// Headers in STL
#include <map>
#include <chrono>

// Headers in ROS
#include <vision_msgs/msg/vision_info.hpp>
#include <rclcpp/rclcpp.hpp>

namespace vision_info_parser
{
    /**
     * @brief Parser Class for the Vision Info
     * 
     */
    class VisionInfoParser
    {
    public:
        VisionInfoParser(std::shared_ptr<rclcpp::Node> node_ptr);
        bool parseFromString(std::string xml_string);
        bool parseFromFile(std::string xml_path);
        bool parseFromRosMessage(vision_msgs::msg::VisionInfo msg);
        inline std::string getClassMetaString()
        {
            return class_meta_str_;
        }
        inline boost::optional<std::map<int,std::string> > getClasses()
        {
            return classes_;
        }
    private:
        boost::optional<std::map<int,std::string> > classes_;
        std::string class_meta_str_;
        std::shared_ptr<rclcpp::Node> node_ptr_;
        std::shared_ptr<rclcpp::SyncParametersClient> param_client_ptr_;
    };
}