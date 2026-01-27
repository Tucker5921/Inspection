import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
import tf2_ros
from shapely.geometry import Point, Polygon
import yaml
import os

class GeofenceMonitor(Node):
    def __init__(self):
        super().__init__('geofence_monitor')

        # 1. 讀取 YAML 配置文件
        self.yaml_path = os.path.expanduser('~/ins_ws/src/ins_geofencing/config/zones.yaml')
        self.inclusion_poly = None
        self.exclusion_polys = []
        self.load_zones()

        # 2. 安全狀態發布
        self.safe_pub = self.create_publisher(Bool, '/safety/is_allowed', 10)

        # 3. TF 監聽器 (監控 map -> base_link)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 4. 定時檢查 (10Hz)
        self.timer = self.create_timer(0.1, self.check_safety)
        self.get_logger().info("Geofence Monitor is ACTIVE. Shielding Spot...")

    def load_zones(self):
        if not os.path.exists(self.yaml_path):
            self.get_logger().error(f"YAML file not found at {self.yaml_path}")
            return

        with open(self.yaml_path, 'r') as f:
            data = yaml.safe_load(f)
            zones = data.get('geofence', {})
            
            # 建立邊界多邊形
            inc_pts = zones.get('inclusion_coords', [])
            if len(inc_pts) >= 3:
                self.inclusion_poly = Polygon(inc_pts)
                self.get_logger().info("Inclusion zone loaded.")
            
            # 建立多個禁區多邊形
            ex_zones = zones.get('exclusion_zones', [])
            self.exclusion_polys = [Polygon(pts) for pts in ex_zones if len(pts) >= 3]
            self.get_logger().info(f"Loaded {len(self.exclusion_polys)} exclusion zones.")

    def check_safety(self):
        if self.inclusion_poly is None:
            return

        try:
            # 獲取機器人在地圖上的位置 (map -> base_link)
            now = rclpy.time.Time()
            # 我們搜尋最近 0.1 秒內的變換
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now, 
                                                    timeout=rclpy.duration.Duration(seconds=0.05))
            
            robot_x = trans.transform.translation.x
            robot_y = trans.transform.translation.y
            robot_pt = Point(robot_x, robot_y)

            # 邏輯判定
            # 1. 必須在綠框內
            is_inside_inclusion = self.inclusion_poly.contains(robot_pt)
            # 2. 不能在任何紅框內
            is_inside_exclusion = any(poly.contains(robot_pt) for poly in self.exclusion_polys)

            is_safe = is_inside_inclusion and not is_inside_exclusion

            # 發布安全信號
            msg = Bool()
            msg.data = is_safe
            self.safe_pub.publish(msg)

            if not is_safe:
                self.get_logger().warn(f"!!! OUT OF BOUNDS !!! Position: ({robot_x:.2f}, {robot_y:.2f})", 
                                       throttle_duration_sec=1.0)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # 如果 TF 暫時斷開，基於安全考量，可以不發布 True
            pass

def main():
    rclpy.init()
    node = GeofenceMonitor()
    rclpy.spin(node)
    rclpy.shutdown()