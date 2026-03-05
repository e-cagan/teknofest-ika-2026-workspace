"""
Koni Slalom / Engelden Kaçınma Node'u

Algı: cone_detector_node → /perception/cones (ConeArray)
Çıkış: /cmd_vel_nav (Twist)

Şartname: Trafik konilerine dokunmadan ilerle. Her temas -5 puan.
Otonom koşuda PAS GEÇİLEMEZ. 50 puan.

Yöntem: Gap-based navigation.
  1. Tespit edilen konilerin pozisyonlarını al
  2. Koniler arası boşlukları (gap) bul
  3. En geniş ve en uygun gap'i seç
  4. Gap merkezine doğru ilerle
  5. Gap yoksa dur ve bekle
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from ika_msgs.msg import ConeArray


class ConeAvoidanceNode(Node):

    def __init__(self):
        super().__init__('cone_avoidance_node')

        # Parametreler
        self.declare_parameter('cones_topic', '/perception/cones')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('rate', 15.0)
        self.declare_parameter('avoidance_distance_m', 2.0)
        self.declare_parameter('safety_margin_m', 0.4)
        self.declare_parameter('approach_speed', 0.4)
        self.declare_parameter('pass_speed', 0.3)
        self.declare_parameter('min_gap_width_m', 1.2)
        self.declare_parameter('vehicle_width_m', 1.0)

        self.avoid_dist = self.get_parameter('avoidance_distance_m').value
        self.safety_margin = self.get_parameter('safety_margin_m').value
        self.approach_speed = self.get_parameter('approach_speed').value
        self.pass_speed = self.get_parameter('pass_speed').value
        self.min_gap = self.get_parameter('min_gap_width_m').value
        self.vehicle_width = self.get_parameter('vehicle_width_m').value

        # Durum
        self.latest_cones = None

        # ROS
        self.create_subscription(ConeArray, self.get_parameter('cones_topic').value,
                                 self._cones_callback, 10)
        self.cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10
        )

        rate = self.get_parameter('rate').value
        self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info('Koni kaçınma node başlatıldı')

    def _cones_callback(self, msg: ConeArray):
        self.latest_cones = msg

    def _find_best_gap(self, cones: list) -> tuple:
        """
        Konilerin Y pozisyonlarından boşlukları bul.
        Bariyerler de sınır olarak eklenir (±1.5m yol kenarı).

        Returns: (gap_center_y, gap_width) veya (0.0, road_width)
        """
        if not cones:
            return (0.0, 3.0)  # Koni yok → düz git

        # Tüm konilerin Y pozisyonları + bariyer sınırları
        y_positions = sorted([c.y for c in cones])

        # Sol ve sağ bariyer sınırı ekle
        boundaries = [-1.5] + y_positions + [1.5]

        # Ardışık noktalar arası boşlukları hesapla
        best_center = 0.0
        best_width = 0.0

        for i in range(len(boundaries) - 1):
            gap_width = boundaries[i + 1] - boundaries[i]
            gap_center = (boundaries[i] + boundaries[i + 1]) / 2.0

            if gap_width > best_width:
                best_width = gap_width
                best_center = gap_center

        return (best_center, best_width)

    def _control_loop(self):
        cmd = Twist()

        if self.latest_cones is None or self.latest_cones.total_count == 0:
            # Koni yok — path_follower devralır (bu node sadece koni varken aktif)
            return

        cones = self.latest_cones.cones

        # En yakın koniyi bul
        nearest_dist = float('inf')
        for cone in cones:
            if cone.distance < nearest_dist:
                nearest_dist = cone.distance

        # Koni uzaktaysa — yavaş yaklaş
        if nearest_dist > self.avoid_dist:
            cmd.linear.x = self.approach_speed
            self.cmd_vel_pub.publish(cmd)
            return

        # Gap bul
        # Sadece yakındaki konileri değerlendir (avoidance_distance içindekiler)
        nearby_cones = [c for c in cones if c.distance < self.avoid_dist * 1.5]
        gap_center, gap_width = self._find_best_gap(nearby_cones)

        if gap_width < self.min_gap:
            # Geçecek yer yok — dur
            self.get_logger().warn(f'Koni gap yetersiz: {gap_width:.2f}m < {self.min_gap}m')
            self.cmd_vel_pub.publish(cmd)
            return

        # Gap merkezine doğru yönlen
        # gap_center = hedef Y pozisyonu (base_link frame)
        angular_z = math.atan2(gap_center, nearest_dist) * 2.0
        angular_z = max(-1.5, min(1.5, angular_z))

        cmd.linear.x = self.pass_speed
        cmd.angular.z = angular_z

        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ConeAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
