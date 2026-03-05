"""
Bariyer Tespit + Yol Merkezi Hesaplama Node'u

Şartname: Yol genişliği 3m, iki kenardan kırmızı-beyaz bariyerlerle
sınırlandırılmış, 80±10cm yüksekliğinde, sürekli (virajlar dahil).

Pipeline:
  1. LiDAR scan → sol ve sağ bariyer noktaları (birincil)
  2. Kamera → kırmızı-beyaz renk doğrulama (destekleyici)
  3. Sol + sağ bariyer ortası → yol merkezi (PoseStamped)
  4. /perception/lane_center yayınla → path_follower_node kullanır
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header


class BarrierDetectorNode(Node):

    def __init__(self):
        super().__init__('barrier_detector_node')

        # Parametreler
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('lane_center_topic', '/perception/lane_center')
        self.declare_parameter('barriers_topic', '/perception/barriers')
        self.declare_parameter('rate', 15.0)
        self.declare_parameter('lidar_range_min', 0.15)
        self.declare_parameter('lidar_range_max', 5.0)
        self.declare_parameter('lidar_cluster_threshold', 0.2)
        self.declare_parameter('road_width_m', 3.0)

        self.range_min = self.get_parameter('lidar_range_min').value
        self.range_max = self.get_parameter('lidar_range_max').value
        self.cluster_thresh = self.get_parameter('lidar_cluster_threshold').value
        self.road_width = self.get_parameter('road_width_m').value

        # ROS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )

        self.create_subscription(
            LaserScan,
            self.get_parameter('lidar_topic').value,
            self._scan_callback,
            sensor_qos
        )

        self.lane_pub = self.create_publisher(
            PoseStamped,
            self.get_parameter('lane_center_topic').value,
            10
        )

        self.get_logger().info('Bariyer tespit + yol merkezi node başlatıldı')

    def _scan_to_cartesian(self, scan: LaserScan) -> np.ndarray:
        """LiDAR scan → (x, y) noktalar. x=ileri, y=sol."""
        angles = np.arange(
            scan.angle_min,
            scan.angle_max + scan.angle_increment,
            scan.angle_increment
        )[:len(scan.ranges)]

        ranges = np.array(scan.ranges)

        # Geçerli aralıktaki noktalar
        valid = (ranges > self.range_min) & (ranges < self.range_max)
        valid_ranges = ranges[valid]
        valid_angles = angles[valid]

        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)

        return np.column_stack((x, y))

    def _find_barriers(self, points: np.ndarray) -> tuple:
        """
        LiDAR noktalarından sol ve sağ bariyer kümelerini bul.

        Yöntem: Aracın ilerisindeki (x > 0) noktaları y eksenine göre
        sol (y > 0) ve sağ (y < 0) olarak ayır. Her taraftaki en yakın
        sürekli kümeyi bariyer kabul et.

        Returns: (left_y, right_y) — ortalama y pozisyonları, None ise bulunamadı
        """
        if len(points) == 0:
            return (None, None)

        # Sadece ilerideki noktalar (x > 0.3m)
        forward = points[points[:, 0] > 0.3]
        if len(forward) == 0:
            return (None, None)

        # Yakın ileriye odaklan (0.3m - 3m arası)
        near = forward[(forward[:, 0] < 3.0)]
        if len(near) == 0:
            return (None, None)

        # Sol ve sağ ayır
        left_points = near[near[:, 1] > 0.2]   # Sol bariyer (y > 0)
        right_points = near[near[:, 1] < -0.2]  # Sağ bariyer (y < 0)

        left_y = None
        right_y = None

        if len(left_points) > 3:
            left_y = float(np.median(left_points[:, 1]))

        if len(right_points) > 3:
            right_y = float(np.median(right_points[:, 1]))

        # Akıl sağlığı kontrolü: bariyerler arası mesafe ~3m olmalı
        if left_y is not None and right_y is not None:
            gap = left_y - right_y
            if gap < 1.5 or gap > 5.0:
                # Makul aralık dışında — muhtemelen yanlış tespit
                self.get_logger().debug(
                    f'Bariyer aralığı anormal: {gap:.2f}m (beklenen ~{self.road_width}m)'
                )

        return (left_y, right_y)

    def _scan_callback(self, msg: LaserScan):
        """LiDAR verisi geldiğinde bariyer tespiti ve yol merkezi hesabı."""
        points = self._scan_to_cartesian(msg)
        left_y, right_y = self._find_barriers(points)

        # Yol merkezi hesapla
        center_y = 0.0
        has_center = False

        if left_y is not None and right_y is not None:
            center_y = (left_y + right_y) / 2.0
            has_center = True
        elif left_y is not None:
            # Sadece sol bariyer görünüyor → yarım yol genişliği çıkar
            center_y = left_y - (self.road_width / 2.0)
            has_center = True
        elif right_y is not None:
            # Sadece sağ bariyer → yarım yol genişliği ekle
            center_y = right_y + (self.road_width / 2.0)
            has_center = True

        if has_center:
            # Yol merkezi: aracın ilerisinde, center_y lateral offset
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'base_link'

            # Hedef nokta: 2m ileride, center_y kadar yanda
            pose.pose.position.x = 2.0
            pose.pose.position.y = center_y
            pose.pose.position.z = 0.0

            # Yönelim: hedefe doğru
            yaw = math.atan2(center_y, 2.0)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)

            self.lane_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = BarrierDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
