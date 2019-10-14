/**
 * @file vision_info_server_node.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief Implimentation of main function
 * @version 0.1
 * @date 2019-08-05
 * 
 * @copyright Copyright (c) 2019
 * 
 */

// Headers in this package
#include <vision_info_server/vision_info_server.h>

// Headers in ROS
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    rclcpp::NodeOptions options;
    std::shared_ptr<vision_info_server::VisionInfoServer> vision_info_server_node
        = std::make_shared<vision_info_server::VisionInfoServer>(options);
    exe.add_node(vision_info_server_node->get_node_base_interface());
    exe.spin();
    rclcpp::shutdown();
    return 0;
}