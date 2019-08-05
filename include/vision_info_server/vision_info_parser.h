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

// Headers in ROS
#include <vision_msgs/VisionInfo.h>
#include <ros/ros.h>

namespace vision_info_parser
{
    /**
     * @brief Parser Class for the Vision Info
     * 
     */
    class VisionInfoParser
    {
    public:
        VisionInfoParser();
        VisionInfoParser(ros::NodeHandle nh);
        bool parseFromString(std::string xml_string);
        bool parseFromFile(std::string xml_path);
        bool parseFromRosMessage(vision_msgs::VisionInfo msg);
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
        ros::NodeHandle nh_;
    };
}