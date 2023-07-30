/***********************
 * File: subscriber_pointcloud.cpp
 * Author: BigTree777
 * Date: 2023/07/29
 * Description:
 *      点群データを受信するノード
 * **********************/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I heard: [%s]", msg->header.frame_id.c_str());
}

int main(int argc, char *argv[])
{
    // ROS2の初期化
    rclcpp::init(argc, argv);
    // PointCloudSubscriberの宣言
    auto node = rclcpp::Node::make_shared("subscriber_pointcloud");
    auto pcs = node->create_subscription<sensor_msgs::msg::PointCloud2>("pointcloud", 10, callback);
    // ROS2のスピン
    rclcpp::spin(node);
    // ROS2のシャットダウン
    rclcpp::shutdown();
    return 0;
}