/*********************************************************************
* File Name: helloworld.cpp
* Author: BigTree777
* Creation Date: 2023/07/29
* Version: 1.0.0
* 
* Summary:
*   This file is a helloworld program.
* 
* Update History:
* 
**********************************************************************/

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
    // Initialize process
    rclcpp::init(argc, argv);
    // Initialize node
    auto node = rclcpp::Node::make_shared("hello_world");
    // Set loop rate
    rclcpp::WallRate loop_rate(1);
    // Infinite loop until Ctrl-C
    while(rclcpp::ok())
    {
        // Print "Hello World!"
        RCLCPP_INFO(node->get_logger(), "Hello World!");
        // Sleep
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
