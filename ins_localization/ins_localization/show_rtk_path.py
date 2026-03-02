import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
from pyproj import Transformer

class PureRTKVisualizer(Node):
    def __init__(self):
        super().__init__('pure_rtk_visualizer')

        # 座標轉換器 (WGS84 -> UTM)
        self.transformer = Transformer.from_crs("epsg:4326", "epsg:32651", always_xy=True)

        # 訂閱純 RTK 數據
        self.sub_gps = self.create_subscription(NavSatFix, '/rtk', self.gps_callback, 10)

        # 發布路徑與 TF
        self.path_pub = self.create_publisher(Path, '/rtk_path', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.rtk_path = Path()
        self.rtk_path.header.frame_id = 'map'
        self.origin_utm = None # 用來把座標平移到原點，RViz 才不會因為座標太大(百萬級)而抖動

    def gps_callback(self, msg):
        # 1. 經緯度轉 UTM
        utm_x, utm_y = self.transformer.transform(msg.longitude, msg.latitude)

        # 2. 設定第一個點為地圖原點 (重要：解決 RViz 渲染抖動問題)
        if self.origin_utm is None:
            self.origin_utm = (utm_x, utm_y)
            self.get_logger().info(f"Origin Set: {self.origin_utm}")

        rel_x = utm_x - self.origin_utm[0]
        rel_y = utm_y - self.origin_utm[1]

        # 3. 發布 TF (map -> base_link) 讓 RViz 知道車在哪
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = rel_x
        t.transform.translation.y = rel_y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0 # 純 RTK 沒方向，先給 1
        self.tf_broadcaster.sendTransform(t)

        # 4. 更新並發布軌跡
        pose = PoseStamped()
        pose.header = t.header
        pose.pose.position.x = rel_x
        pose.pose.position.y = rel_y
        self.rtk_path.poses.append(pose)
        self.rtk_path.header.stamp = msg.header.stamp
        self.path_pub.publish(self.rtk_path)

def main():
    rclpy.init()
    node = PureRTKVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()