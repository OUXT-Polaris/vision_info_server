#ifndef VISION_INFO_SERVER_VISION_INFO_SERVER_H_INCLUDED
#define VISION_INFO_SERVER_VISION_INFO_SERVER_H_INCLUDED

/**
 * @file vision_info_server.h
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief definition of the VisionInfoServer Class
 * @version 0.1
 * @date 2019-08-05
 * 
 * @copyright Copyright (c) 2019
 * 
 */

// Headers in this package
#include <vision_info_server/vision_info_parser.h>

// Headers in ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <vision_msgs/msg/vision_info.hpp>

// Headers in Boost
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define VISION_INFO_SERVER_EXPORT __attribute__ ((dllexport))
    #define VISION_INFO_SERVER_IMPORT __attribute__ ((dllimport))
  #else
    #define VISION_INFO_SERVER_EXPORT __declspec(dllexport)
    #define VISION_INFO_SERVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef VISION_INFO_SERVER_BUILDING_DLL
    #define VISION_INFO_SERVER_PUBLIC VISION_INFO_SERVER_EXPORT
  #else
    #define VISION_INFO_SERVER_PUBLIC VISION_INFO_SERVER_IMPORT
  #endif
  #define VISION_INFO_SERVER_PUBLIC_TYPE VISION_INFO_SERVER_PUBLIC
  #define VISION_INFO_SERVER_LOCAL
#else
  #define VISION_INFO_SERVER_EXPORT __attribute__ ((visibility("default")))
  #define VISION_INFO_SERVER_IMPORT
  #if __GNUC__ >= 4
    #define VISION_INFO_SERVER_PUBLIC __attribute__ ((visibility("default")))
    #define VISION_INFO_SERVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define VISION_INFO_SERVER_PUBLIC
    #define VISION_INFO_SERVER_LOCAL
  #endif
  #define VISION_INFO_SERVER_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

namespace vision_info_server
{
    class VisionInfoServer: public rclcpp::Node
    {
    public:
        VISION_INFO_SERVER_PUBLIC
        explicit VisionInfoServer(const rclcpp::NodeOptions& options);
        ~VisionInfoServer();
        void publish();
    private:
        std::string xml_path_;
        boost::optional<std::map<int,std::string> > classes_;
        vision_info_parser::VisionInfoParser parser_;
        std::shared_ptr<rclcpp::SyncParametersClient> param_client_ptr_;
        std::string xml_string_;
        std::shared_ptr<rclcpp::Publisher<vision_msgs::msg::VisionInfo> > vision_info_pub_;
    };
}

#endif  //VISION_INFO_SERVER_VISION_INFO_SERVER_H_INCLUDED