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
    ROS_ASSERT(validate());
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
    std::ifstream ifs(xml_path_);
    std::string str((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    pnh_.setParam("class_meta_info", str);
    vision_msgs::VisionInfo msg;
    msg.database_location = pnh_.getNamespace() + "/class_meta_info";
    msg.header.stamp = ros::Time::now();
    vision_info_pub_.publish(msg);
}

/**
 * @brief Validate XML file.
 * @return If validate returns true, XML file is validate.
 */
bool VisionInfoServer::validate()
{
    using namespace boost::property_tree;
    ptree pt;
    std::map<int,std::string> classes;
    try
    {
        read_xml(xml_path_, pt);
        BOOST_FOREACH (const ptree::value_type& child, pt.get_child("vision_info"))
        {
            if(child.first == "class")
            {
                boost::optional<int> id = child.second.get_optional<int>("<xmlattr>.id");
                boost::optional<std::string> name = child.second.get_optional<std::string>("<xmlattr>.name");
                if(id && name)
                {
                    classes[*id] = *name;
                }
                else
                {
                    ROS_ERROR_STREAM("failed to read xml string");
                    return false;
                }
            }
        }
        classes_ = classes;
    }
    catch(...)
    {
        return false;
    }
    return true;
}