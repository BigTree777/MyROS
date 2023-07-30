/************************************
 * File: publisher_pointcloud.cpp
 * Author: BigTree777
 * Date: 2023/07/29
 * Description:
 *      This file is used to publish pointcloud data.
 * Version: 1.0.0
 ************************************/

#include <rclcpp/rclcpp.hpp>
#include "../include/viewer_pointcloud/publisher_pointcloud.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    PointCloudPublisher pcp("/data/nuscenes/sweeps/LIDAR_TOP", "pointcloud", "map", 0);
    rclcpp::Rate loop_rate(1.0);
    while(rclcpp::ok()){
        pcp.publish();
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;   
}