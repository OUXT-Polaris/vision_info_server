/**
 * @file vision_info_parser.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief implimentation of Parser class
 * @version 0.1
 * @date 2019-08-05
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include <vision_info_server/vision_info_parser.h>

namespace vision_info_parser
{
    /**
     * @brief Constructor
     * 
     */
    VisionInfoParser::VisionInfoParser(std::shared_ptr<rclcpp::Node> node_ptr)
    {
        node_ptr_ = node_ptr;
        param_client_ptr_ = std::make_shared<rclcpp::SyncParametersClient>(node_ptr_);
    }

    /**
    * @brief Validate and parse XML string.
    * @return If validate returns true, XML string is validate.
    */
    bool VisionInfoParser::parseFromString(std::string xml_string)
    {
        class_meta_str_ = xml_string;
        using namespace boost::property_tree;
        ptree pt;
        std::stringstream ss;
        ss << xml_string;
        std::map<int,std::string> classes;
        try
        {
            read_xml(ss, pt);
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

    /**
    * @brief Validate and parse XML file.
    * @return If validate returns true, XML file is validate.
    */
    bool VisionInfoParser::parseFromFile(std::string xml_path)
    {
        std::ifstream ifs(xml_path);
        std::string str((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
        return parseFromString(str);
    }
    
    /**
    * @brief Validate and parse ROS message
    * @return If validate returns true, message is validate.
    */
    bool VisionInfoParser::parseFromRosMessage(vision_msgs::msg::VisionInfo msg)
    {
        node_ptr_->declare_parameter(msg.database_location);
        while(!param_client_ptr_->wait_for_service(std::chrono::seconds(1)))
        {
            if(!rclcpp::ok())
            {
                RCLCPP_ERROR(node_ptr_->get_logger(), "Interrupted while waiting for the service.");
                return false;
            }
            RCLCPP_INFO(node_ptr_->get_logger(), "waiting for parameter service...");
        }
        bool has_param = param_client_ptr_->has_parameter(msg.database_location);
        if(!has_param)
        {
            RCLCPP_ERROR(node_ptr_->get_logger(), "No database xml string for the vision info.");
            return false;
        }
        std::string xml_string;
        auto parameters_get_results = param_client_ptr_->get_parameters({msg.database_location});
        for(auto &param : parameters_get_results)
        {
            if(param.get_name() == msg.database_location)
            {
                xml_string = param.as_string();
                return parseFromString(xml_string);
            }
        }
        return false;
    }
}