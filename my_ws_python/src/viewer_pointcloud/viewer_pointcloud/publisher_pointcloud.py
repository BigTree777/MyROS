'''
File: publisher_pointcloud.py
Author: BigTree777
Date: 2023/07/29
Description:
    This file is a node that publishes pointcloud data from local file.
'''
import os
import sys
import copy
import glob
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from include.convert_pointcloud2 import ConvertPointCloud2

class LiDAR(Node):
    def __init__(self):
        '''
        ROSノードを初期化する
        '''
        # Initialize ROS node.
        super().__init__("lidar")

        # 点群の保存されているファイルのパスを取得する
        # TODO: PointPillarsなどの予測結果から読み込むパターンも作成する
        # nuscenes
        self.pc_file_path_list = sorted(glob.glob("/data/nuscenes/sweeps/LIDAR_TOP/*.bin"))
        self.is_nuscenes = True
        # kitti
        # self.pc_file_path_list = sorted(glob.glob("/data/KITTI/kitti_data/train_lidar/PointCloud/*.bin"))
        # self.is_nuscenes = False

        # 点群をROSのPointCloud2型に変換するクラスのインスタンスを作成する
        self.converter = ConvertPointCloud2()

        # 点群をトピックとしてパブリッシュする
        self.pc_publisher = self.create_publisher(PointCloud2, "pointcloud", 10)

        # 3Dボックスをトピックとしてパブリッシュする
        self.box3d_publisher = self.create_publisher(Marker, "box3d", 10)

        # 100msごとに点群をパブリッシュする
        self.timer = self.create_timer(0.1, self.timer_callback)

    def create_pointcloud_msg(self):
        '''
        点群をROSのPointCloud2型に変換する
        '''
        # 点群を読み込む
        self.pc_file_path = self.pc_file_path_list.pop(0)
        pointcloud = self.load_pointcloud(self.pc_file_path)

        # 生点群をROSのPointCloud2型に変換する
        return self.converter.array_to_point_cloud2(pointcloud)

    def add_points_to_marker(self, marker, pts):
        '''
        3Dボックスの頂点を追加する
        Reference: https://github.com/charlesq34/frustum-pointnets/blob/master/mayavi/viz_util.py
        '''
        point = Point()
        for k in range(0,4):
            #http://docs.enthought.com/mayavi/mayavi/auto/mlab_helper_functions.html
            i,j=k,(k+1)%4
            point.x = pts[i][0]
            point.y = pts[i][1]
            point.z = pts[i][2]
            marker.points.append(copy.deepcopy(point))
            point.x = pts[j][0]
            point.y = pts[j][1]
            point.z = pts[j][2]
            marker.points.append(copy.deepcopy(point))

            i,j=k+4,(k+1)%4 + 4
            point.x = pts[i][0]
            point.y = pts[i][1]
            point.z = pts[i][2]
            marker.points.append(copy.deepcopy(point))
            point.x = pts[j][0]
            point.y = pts[j][1]
            point.z = pts[j][2]
            marker.points.append(copy.deepcopy(point))

            i,j=k,k+4
            point.x = pts[i][0]
            point.y = pts[i][1]
            point.z = pts[i][2]
            marker.points.append(copy.deepcopy(point))

            point.x = pts[j][0]
            point.y = pts[j][1]
            point.z = pts[j][2]
            marker.points.append(copy.deepcopy(point))
        return marker

    def create_box3d_msg(self):

        marker = Marker()
        marker.header.frame_id = "map"  # 3Dボックスの座標系を指定
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        x_min = -1.0
        x_max = 1.0
        y_min = -3.0
        y_max = 3.0
        z_min = -1.0
        z_max = 0.0
        pts = [
            [x_min, y_min, z_min],
            [x_max, y_min, z_min],
            [x_max, y_max, z_min],
            [x_min, y_max, z_min],
            [x_min, y_min, z_max],
            [x_max, y_min, z_max],
            [x_max, y_max, z_max],
            [x_min, y_max, z_max],
        ]
        marker = self.add_points_to_marker(marker, pts)
        return marker
        

    def timer_callback(self):
        '''
        点群と3Dボックスをパブリッシュする
        '''
        # 点群のメッセージを作成する
        pc_msg = self.create_pointcloud_msg()

        # 3Dボックスのメッセージを作成する
        box3d_msg = self.create_box3d_msg()

        # メッセージをパブリッシュする
        self.pc_publisher.publish(pc_msg)
        self.box3d_publisher.publish(box3d_msg)
    
    def load_pointcloud(self, path):
        '''
        Load pointcloud data from local file.
        '''
        # Get file path
        # Read pointcloud data from local file.
        pointcloud = np.fromfile(path, dtype=np.float32)
        # Reshape pointcloud data.
        if self.is_nuscenes:
            pointcloud = pointcloud.reshape(-1, 5)
            pointcloud = pointcloud[:, :4]
        else:
            pointcloud = pointcloud.reshape(-1, 4)
        
        return pointcloud

def main():
    # Initialize ROS node.
    rclpy.init()
    # Create node.
    node = LiDAR()
    # Spin node.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nCtrl-C detected.")
    # Shutdown ROS node.
    rclpy.try_shutdown()

if __name__=="__main__":
    main()