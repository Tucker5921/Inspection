# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import TransformStamped
# import tf2_ros
# import yaml
# import os

# class ZoneRecorder(Node):
#     def __init__(self):
#         super().__init__('zone_recorder')
        
#         # TF 監聽器：獲取機器人在 map 座標系下的位置
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
#         self.recorded_points = []
#         self.get_logger().info("\n" + "="*40 + 
#                                "\n[Zone Recorder Ready]" +
#                                "\n1. Move Spot to a corner." +
#                                "\n2. Run: 'ros2 service call /record_point std_srvs/srv/Empty'" +
#                                "\n3. Once done, 'ros2 service call /save_zones std_srvs/srv/Empty'" +
#                                "\n" + "="*40)

#         # 建立服務來觸發記錄與存檔
#         from std_srvs.srv import Empty
#         self.srv_record = self.create_service(Empty, 'record_point', self.record_callback)
#         self.srv_save = self.create_service(Empty, 'save_zones', self.save_callback)

#     def record_callback(self, request, response):
#         try:
#             # 獲取最新位置
#             now = rclpy.time.Time()
#             trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
#             x = trans.transform.translation.x
#             y = trans.transform.translation.y
            
#             self.recorded_points.append([float(x), float(y)])
#             self.get_logger().info(f"Point Added: [{x:.3f}, {y:.3f}] - Total: {len(self.recorded_points)}")
#         except Exception as e:
#             self.get_logger().error(f"Could not record point: {e}")
#         return response

#     def save_callback(self, request, response):
#         data = {
#             'geofence': {
#                 'inclusion_coords': self.recorded_points,
#                 'exclusion_zones': [] # 禁區可以分次記錄或手動補入
#             }
#         }
        
#         file_path = os.path.expanduser('~/ins_ws/src/ins_geofencing/config/zones.yaml')
#         os.makedirs(os.path.dirname(file_path), exist_ok=True)
        
#         with open(file_path, 'w') as f:
#             yaml.dump(data, f, default_flow_style=False)
            
#         self.get_logger().info(f"Success! Zones saved to: {file_path}")
#         return response

# def main():
#     rclpy.init()
#     node = ZoneRecorder()
#     rclpy.spin(node)
#     rclpy.shutdown()
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger
import tf2_ros
import yaml
import os

class ZoneRecorderPro(Node):
    def __init__(self):
        super().__init__('zone_recorder_pro')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 數據結構
        self.inclusion_coords = []
        self.exclusion_zones = [] # 這是 list of lists
        self.current_exclusion_points = []
        
        # 服務建立
        self.create_service(Trigger, 'record_inclusion', self.record_inclusion_cb)
        self.create_service(Trigger, 'start_new_exclusion', self.start_exclusion_cb)
        self.create_service(Trigger, 'record_exclusion_point', self.record_ex_point_cb)
        self.create_service(Trigger, 'save_zones', self.save_callback)

        self.get_logger().info("\n" + "="*50 + 
                               "\n[標定工具進階版]" +
                               "\n1. 記錄邊界點: ros2 service call /record_inclusion std_srvs/srv/Trigger" +
                               "\n2. 開始新禁區: ros2 service call /start_new_exclusion std_srvs/srv/Trigger" +
                               "\n3. 記錄禁區點: ros2 service call /record_exclusion_point std_srvs/srv/Trigger" +
                               "\n4. 存檔導出:   ros2 service call /save_zones std_srvs/srv/Trigger" +
                               "\n" + "="*50)

    def get_current_pose(self):
        now = rclpy.time.Time()
        trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
        return [float(trans.transform.translation.x), float(trans.transform.translation.y)]

    def record_inclusion_cb(self, _, response):
        pos = self.get_current_pose()
        self.inclusion_coords.append(pos)
        response.success = True
        response.message = f"邊界點已增加: {pos}"
        return response

    def start_exclusion_cb(self, _, response):
        if self.current_exclusion_points:
            self.exclusion_zones.append(self.current_exclusion_points)
        self.current_exclusion_points = []
        response.success = True
        response.message = "開始記錄一個新的禁區多邊形"
        return response

    def record_ex_point_cb(self, _, response):
        pos = self.get_current_pose()
        self.current_exclusion_points.append(pos)
        response.success = True
        response.message = f"禁區點已增加: {pos}"
        return response

    def save_callback(self, _, response):
        # 存檔前把最後一個沒結尾的禁區放進去
        if self.current_exclusion_points:
            self.exclusion_zones.append(self.current_exclusion_points)
            self.current_exclusion_points = []

        data = {
            'geofence': {
                'inclusion_coords': self.inclusion_coords,
                'exclusion_zones': self.exclusion_zones
            }
        }
        
        path = os.path.expanduser('~/ins_ws/src/ins_geofencing/config/zones.yaml')
        with open(path, 'w') as f:
            yaml.dump(data, f)
        
        response.success = True
        response.message = f"所有區域已儲存至 {path}"
        return response

def main():
    rclpy.init()
    node = ZoneRecorderPro()
    rclpy.spin(node)
    rclpy.shutdown()