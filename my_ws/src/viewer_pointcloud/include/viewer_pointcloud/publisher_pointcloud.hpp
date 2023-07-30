/******************
 * File: publisher_pointcloud.hpp
 * Author: BigTree777
 * Date: 2023/07/29
 * Description:
 *    This file is used to publish pointcloud data.
 * 
 ****************/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>

// 点群をファイルから読み込んでトピックを送信するクラス
class PointCloudPublisher{
    public:
        PointCloudPublisher(std::string path, std::string topic_name, std::string frame_id, double rate);
        void publish();
        pcl::PointCloud<pcl::PointXYZI>::Ptr readPointCloud(std::string filename);
        std::vector<std::string> makeFileList(std::string path);
        bool readPCDBinMain(const std::string& fname, pcl::PointCloud<pcl::PointXYZI>& cloud);
        bool readPCDBin(const std::string& fname, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    private:
        std::string path_;
        std::string topic_name_;
        std::string frame_id_;
        double rate_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
        std::vector<std::string> filelist_;
        std::vector<std::string>::iterator it_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
        pcl::VoxelGrid<pcl::PointXYZI> vg_;
};

// コンストラクタ
PointCloudPublisher::PointCloudPublisher(std::string path, std::string topic_name, std::string frame_id, double rate){
    path_ = path;
    topic_name_ = topic_name;
    frame_id_ = frame_id;
    rate_ = rate;
    node_ = rclcpp::Node::make_shared("publisher_pointcloud");
    pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_, 1);
    filelist_ = makeFileList(path_);
    it_ = filelist_.begin();
    cloud_ = readPointCloud(*it_);
    vg_.setLeafSize(0.01f, 0.01f, 0.01f);
    vg_.setInputCloud(cloud_);
    vg_.filter(*cloud_);
}

bool PointCloudPublisher::readPCDBinMain(const std::string& fname, pcl::PointCloud<pcl::PointXYZI>& cloud) {
    if (fname.empty()) return false;
    std::ifstream file(fname, std::ios::in | std::ios::binary);
    if (!file.good()) {
        PCL_ERROR("Couldn't read file");
        return false;
    }

    float max_i = 0;
    while (file) {
        float x,y,z,i,r;
        file.read(reinterpret_cast<char*>(&x), sizeof(float));
        file.read(reinterpret_cast<char*>(&y), sizeof(float));
        file.read(reinterpret_cast<char*>(&z), sizeof(float));
        file.read(reinterpret_cast<char*>(&i), sizeof(float));
        file.read(reinterpret_cast<char*>(&r), sizeof(float));
        max_i = std::max(max_i, i);
        pcl::PointXYZI point{};
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = i; //std::sqrt(x*x + y*y + z*z) < 10 ? 3 : 250 ;
        cloud.push_back(point);
    }
    file.close();
    return true;
}

bool PointCloudPublisher::readPCDBin(const std::string& fname, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    std::ifstream file(fname, std::ios::in | std::ios::binary);
    if (!file.good()) {
        PCL_ERROR("Couldn't read file");
        return false;
    }
    return readPCDBinMain(fname, *cloud);
}

// 点群をトピックに送信する関数
void PointCloudPublisher::publish(){
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_, output);
    output.header.frame_id = frame_id_;
    pub_->publish(output);
    ++it_;
    if(it_ == filelist_.end()){
        it_ = filelist_.begin();
    }
    cloud_ = readPointCloud(*it_);
    vg_.setInputCloud(cloud_);
    vg_.filter(*cloud_);
}

// 点群をファイルから読み込んで点群を返す関数
pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudPublisher::readPointCloud(std::string filename){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if(readPCDBin(filename, cloud) == false){
        PCL_ERROR("Couldn't read file");
    }
    return cloud;
}

// 点群の保存されているファイルのパスのリストを作成する関数
std::vector<std::string> PointCloudPublisher::makeFileList(std::string path){
    std::vector<std::string> filelist;
    std::string filename;
    DIR *dp;
    struct dirent *dirp;
    if((dp = opendir(path.c_str())) == NULL){
        std::cout << "Error(" << errno << ") opening " << path << std::endl;
        return filelist;
    }
    while((dirp = readdir(dp)) != NULL){
        filename = dirp->d_name;
        if(filename == "." || filename == ".."){
            continue;
        }
        filelist.push_back(path + "/" + filename);
    }
    closedir(dp);
    return filelist;
}

