import sys
import time
import array
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField

class ConvertPointCloud2(object):
    '''
    numpy配列からPointCloud2型のメッセージを作成する
    TODO: 別ファイルに移動する
    '''
    def __init__(self, frame_id="map", is_xyzi=True):
        '''
        すべての点群データに共通する情報を設定する
        Args:
            frame_id (str): 座標系の名前
            is_xyzi (bool): intensityを持つかどうか, default=True
        '''
        self.msg = PointCloud2()
        self.msg.header.frame_id = frame_id
        self.msg.height = 1 # 点群は1次元配列であるため固定

        if is_xyzi:
            self.msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
            ]
        else:
            self.msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]

        # バイトオーダー (ビッグエンディアンかリトルエンディアンか)
        self.msg.is_bigendian = sys.byteorder != 'little'
        
        # 一点のデータが占めるバイト数
        self.msg.point_step = 16 if is_xyzi else 12

    def array_to_point_cloud2(self, np_array):
        """
        numpyの配列をPointCloud2に変換する
        Args:
            np_array (np.array): 4次元のnumpy配列([x, y, z, intensity]), dtype=float32, shape=(N, 4),
        Returns:
            PointCloud2: ROSのPointCloud2型                    
        """
        assert(np_array.dtype == np.float32)
        assert(len(np_array.shape) == 2)
        assert(np_array.shape[1] == 4)

        # Create the PointCloud2 message
        current_time = time.time()
        self.msg.header.stamp.sec = int(current_time)
        self.msg.header.stamp.nanosec = int((current_time - self.msg.header.stamp.sec) * 1e9)
        self.msg.width = np_array.shape[0]

        # Check if message is dense
        self.msg.is_dense = not np.isnan(np_array).any()
        self.msg.row_step = self.msg.point_step * self.msg.width

        # Reference: https://github.com/Box-Robotics/ros2_numpy/blob/humble/ros2_numpy/point_cloud2.py
        byte_array = np_array.tobytes()
        as_array = array.array("B", byte_array)
        self.msg.data = as_array

        return self.msg