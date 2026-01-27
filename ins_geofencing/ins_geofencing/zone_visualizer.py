import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import yaml
import os

class ZoneVisualizer(Node):
    def __init__(self):
        super().__init__('zone_visualizer')
        
        # 1. 讀取 YAML 檔案
        self.yaml_path = os.path.expanduser('~/ins_ws/src/ins_geofencing/config/zones.yaml')
        self.marker_pub = self.create_publisher(Marker, 'geofence_markers', 10)
        
        # 定時發布 (2Hz 即可，不需要太快)
        self.timer = self.create_timer(0.5, self.publish_markers)
        self.get_logger().info(f"Visualizing zones from: {self.yaml_path}")

    def create_marker(self, points, marker_id, color, label):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = label
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP # 使用線段連接
        marker.action = Marker.ADD
        
        # 設定線條寬度
        marker.scale.x = 0.1 
        
        # 設定顏色 (RGBA)
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.8 # 透明度

        # 將座標點加入 Marker (需將最後一點連回第一點)
        points_to_draw = points + [points[0]]
        for pt in points_to_draw:
            p = Point()
            p.x = float(pt[0])
            p.y = float(pt[1])
            p.z = 0.0
            marker.points.append(p)
            
        return marker

    def publish_markers(self):
        if not os.path.exists(self.yaml_path):
            return

        with open(self.yaml_path, 'r') as f:
            try:
                data = yaml.safe_load(f)
                zones = data['geofence']
                
                # 1. 畫邊界 (綠色)
                inclusion_pts = zones.get('inclusion_coords', [])
                if inclusion_pts:
                    self.marker_pub.publish(self.create_marker(inclusion_pts, 0, [0.0, 1.0, 0.0], "inclusion"))

                # 2. 畫禁區 (紅色)
                exclusion_zones = zones.get('exclusion_zones', [])
                for i, zone_pts in enumerate(exclusion_zones):
                    if zone_pts:
                        self.marker_pub.publish(self.create_marker(zone_pts, i + 1, [1.0, 0.0, 0.0], "exclusion"))
            except Exception as e:
                self.get_logger().error(f"Error reading YAML: {e}")

def main():
    rclpy.init()
    node = ZoneVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()