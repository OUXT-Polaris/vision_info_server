#include <vision_info_server/vision_info_parser.h>

namespace vision_info_parser
{
    /**
     * @brief Constructor
     * 
     */
    VisionInfoParser::VisionInfoParser()
    {

    }

    /**
     * @brief Constructor
     * 
     */
    VisionInfoParser::VisionInfoParser(ros::NodeHandle nh)
    {
        nh_ = nh;
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
    bool VisionInfoParser::parseFromRosMessage(vision_msgs::VisionInfo msg)
    {
        std::string xml_string;
        nh_.param<std::string>(msg.database_location, xml_string, "");
        return parseFromString(xml_string);
    }
}