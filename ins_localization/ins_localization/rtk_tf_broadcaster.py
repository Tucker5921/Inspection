# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix, Imu
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped
# import tf2_ros
# import numpy as np
# from pyproj import Transformer

# class RTKTFBroadcaster(Node):
#     def __init__(self):
#         super().__init__('rtk_tf_broadcaster')

#         # 1. 座標轉換器 (WGS84 -> UTM) - 根據你的地點調整 EPSG
#         # EPSG:32651 對應台灣/西太平洋區域的 UTM Zone 51N
#         self.transformer = Transformer.from_crs("epsg:4326", "epsg:32651", always_xy=True)

#         # 2. 訂閱 RTK 和 Spot Odom
#         self.sub_gps = self.create_subscription(NavSatFix, '/rtk', self.gps_callback, 10)
#         self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

#         # 3. TF 廣播器
#         self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

#         # 狀態變量
#         self.latest_odom = None
#         self.map_origin = None # 存儲第一個點作為局部原點

#     def odom_callback(self, msg):
#         self.latest_odom = msg
#     def gps_callback(self, msg):
#         # 定義狀態對照表
#         status_map = {
#             -1: "No Fix",
#             0: "SPS (Standard GPS)",
#             1: "DGPS (Differential)",
#             2: "RTK Fixed (Perfect!)"
#         }
        
#         current_status = msg.status.status
        
#         # 顯示目前狀態 (建議用 info，這樣你才知道有在跑)
#         self.get_logger().info(f"RTK Status: {status_map.get(current_status, 'Unknown')} ({current_status})")

#         # if current_status < 2: 
#         #     # 如果是 0 或 1，精度可能只有公尺級，會導致撿拾失敗
#         #     self.get_logger().warn(f"Accuracy insufficient for picking! Current: {current_status}")
#         #     return

#         # 執行座標轉換 (UTM)
#         utm_x, utm_y = self.transformer.transform(msg.longitude, msg.latitude)
#         self.get_logger().info(f"UTM Coord: X={utm_x:.3f}, Y={utm_y:.3f}")
    
#         # 計算 map -> odom 的轉換
#         # 這裡簡化為 2D 平移。
#         # 核心邏輯：RTK_pos = T_map_odom * Odom_pos
#         # 因此 T_map_odom = RTK_pos - Odom_pos (假設方向已對齊)
        
#         t = TransformStamped()
#         t.header.stamp = self.latest_odom.header.stamp
#         t.header.frame_id = 'map'
#         t.child_frame_id = 'odom'

#         # 計算平移補償
#         t.transform.translation.x = utm_x - self.latest_odom.pose.pose.position.x
#         t.transform.translation.y = utm_y - self.latest_odom.pose.pose.position.y
#         t.transform.translation.z = 0.0 # 野外巡檢通常先關注 2D

#         # 航向對齊 (Yaw) - 這裡暫設為 0，實務上需透過 IMU 獲取偏差
#         t.transform.rotation = self.latest_odom.pose.pose.orientation 

#         # 發布 TF
#         self.tf_broadcaster.sendTransform(t)

# def main():
#     rclpy.init()
#     node = RTKTFBroadcaster()
#     rclpy.spin(node)
#     rclpy.shutdown()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
import math
from pyproj import Transformer
import tf_transformations

class RTKTFBroadcaster(Node):
    def __init__(self):
        super().__init__('rtk_tf_broadcaster')

        # 1. 座標轉換器 (WGS84 -> UTM)
        self.transformer = Transformer.from_crs("epsg:4326", "epsg:32651", always_xy=True)

        # 2. 訂閱
        self.sub_gps = self.create_subscription(NavSatFix, '/rtk', self.gps_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # 3. TF 廣播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 狀態變量與標定參數
        self.latest_odom = None
        self.is_calibrated = False
        self.yaw_offset = 0.0
        
        # 用於計算 Yaw 的起始點
        self.start_utm = None
        self.start_odom = None
        self.min_calib_dist = 2.0  # 標定所需行走距離 (公尺)

    def odom_callback(self, msg):
        self.latest_odom = msg

    def gps_callback(self, msg):
        if self.latest_odom is None:
            return

        # A. 經緯度轉平面 UTM
        utm_x, utm_y = self.transformer.transform(msg.longitude, msg.latitude)
        odom_x = self.latest_odom.pose.pose.position.x
        odom_y = self.latest_odom.pose.pose.position.y

        # B. 航向角 (Yaw) 自動標定邏輯
        if not self.is_calibrated:
            if self.start_utm is None:
                self.start_utm = (utm_x, utm_y)
                self.start_odom = (odom_x, odom_y)
                self.get_logger().info("Calibration: Step 1/2 - Starting position recorded. Please move Spot straight for 2m.")
            else:
                # 計算移動距離
                dist = math.sqrt((utm_x - self.start_utm[0])**2 + (utm_y - self.start_utm[1])**2)
                if dist > self.min_calib_dist:
                    # 計算 RTK 軌跡在 Map 上的角度
                    angle_rtk = math.atan2(utm_y - self.start_utm[1], utm_x - self.start_utm[0])
                    # 計算 Odom 軌跡角度
                    angle_odom = math.atan2(odom_y - self.start_odom[1], odom_x - self.start_odom[0])
                    
                    self.yaw_offset = angle_rtk - angle_odom
                    self.is_calibrated = True
                    self.get_logger().info(f"Calibration: Step 2/2 - SUCCESS! Yaw Offset: {math.degrees(self.yaw_offset):.2f}°")
        
        # C. 構造並發布 TF (map -> odom)
        t = TransformStamped()
        t.header.stamp = self.latest_odom.header.stamp # 重要：使用 odom 的時間戳記對齊 Bag 數據
        t.header.frame_id = 'map'
        t.child_frame_id = self.latest_odom.header.frame_id # 通常是 'odom'

        # 計算考慮旋轉後的平移 (Rotate Odom to Map Frame then Offset)
        cos_y = math.cos(self.yaw_offset)
        sin_y = math.sin(self.yaw_offset)
        
        t.transform.translation.x = utm_x - (odom_x * cos_y - odom_y * sin_y)
        t.transform.translation.y = utm_y - (odom_x * sin_y + odom_y * cos_y)
        t.transform.translation.z = 0.0

        # 將 Yaw Offset 轉換為四元數發布
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.yaw_offset)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = RTKTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()