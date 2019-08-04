#ifndef VISION_INFO_SERVER_VISION_INFO_SERVER_H_INCLUDED
#define VISION_INFO_SERVER_VISION_INFO_SERVER_H_INCLUDED

// Headers in ROS
#include <ros/ros.h>
#include <vision_msgs/VisionInfo.h>

// Headers in Boost
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

class VisionInfoServer
{
public:
    VisionInfoServer(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~VisionInfoServer();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::string xml_path_;
    ros::Publisher vision_info_pub_;
    bool validate();
    void publish();
    boost::optional<std::map<int,std::string> > classes_;
};

#endif  //VISION_INFO_SERVER_VISION_INFO_SERVER_H_INCLUDED