"""
Kayar Engel Geçiş Planlama Node'u

Algı: sliding_obstacle_detector_node → /perception/sliding_obstacle
Çıkış: /cmd_vel_nav (Twist)

Strateji: Dur → Engelin pozisyonunu ve hızını izle → Parkur dışına
çıktığı veya yeterli boşluk oluştuğu anda hızla geç.

Şartname: Engel 1m genişlik, 20cm/s, tamamen parkur dışına çıkabiliyor.
Temassız geçilirse 50 puan.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from ika_msgs.msg import SlidingObstacle

# Planner durumları
STATE_APPROACH = 0      # Engel bölgesine yaklaş
STATE_WAIT = 1          # Dur ve engeli izle
STATE_GO = 2            # Boşluk var, geç
STATE_PASSED = 3        # Engel geçildi


class SlidingObstaclePlannerNode(Node):

    def __init__(self):
        super().__init__('sliding_obstacle_planner_node')

        self.declare_parameter('obstacle_topic', '/perception/sliding_obstacle')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('wait_distance_m', 3.0)
        self.declare_parameter('go_gap_threshold_m', 1.5)
        self.declare_parameter('go_speed', 0.6)
        self.declare_parameter('vehicle_width_m', 1.0)
        self.declare_parameter('safety_margin_m', 0.3)

        self.wait_dist = self.get_parameter('wait_distance_m').value
        self.go_gap = self.get_parameter('go_gap_threshold_m').value
        self.go_speed = self.get_parameter('go_speed').value
        self.vehicle_width = self.get_parameter('vehicle_width_m').value
        self.safety_margin = self.get_parameter('safety_margin_m').value

        self.state = STATE_APPROACH
        self.latest_obstacle = None
        self.required_gap = self.vehicle_width + self.safety_margin

        self.create_subscription(
            SlidingObstacle,
            self.get_parameter('obstacle_topic').value,
            self._obstacle_callback, 10
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10
        )

        rate = self.get_parameter('rate').value
        self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info('Kayar engel planner başlatıldı')

    def _obstacle_callback(self, msg: SlidingObstacle):
        self.latest_obstacle = msg

    def _control_loop(self):
        cmd = Twist()

        if self.latest_obstacle is None:
            # Henüz engel verisi yok — yavaş yaklaş
            cmd.linear.x = 0.3
            self.cmd_vel_pub.publish(cmd)
            return

        obs = self.latest_obstacle

        if self.state == STATE_APPROACH:
            # Engel bölgesine yaklaş, engeli görünce dur
            if obs.detected:
                self.state = STATE_WAIT
                self.get_logger().info('Kayar engel tespit edildi — bekleme moduna geçildi')
            else:
                cmd.linear.x = 0.3

        elif self.state == STATE_WAIT:
            # Dur ve engeli izle — boşluk oluşmasını bekle
            if obs.passage_clear and obs.gap_width_m >= self.required_gap:
                self.state = STATE_GO
                self.get_logger().info(
                    f'Boşluk yeterli ({obs.gap_width_m:.2f}m) — geçiş başlatılıyor'
                )
            elif not obs.detected:
                # Engel görünmüyor — muhtemelen parkur dışına çıktı
                self.state = STATE_GO
                self.get_logger().info('Engel parkur dışına çıktı — geçiş başlatılıyor')
            # else: dur, bekle (cmd zaten sıfır)

        elif self.state == STATE_GO:
            # Hızla geç
            cmd.linear.x = self.go_speed

            # Engel artık arkamızda mı kontrol et
            if not obs.detected:
                self.state = STATE_PASSED
                self.get_logger().info('Kayar engel geçildi')

        elif self.state == STATE_PASSED:
            # Engel geçildi — normal navigasyona dön
            pass

        self.cmd_vel_pub.publish(cmd)

    def reset(self):
        """Stage manager tarafından yeni koşu başında çağrılır."""
        self.state = STATE_APPROACH
        self.latest_obstacle = None


def main(args=None):
    rclpy.init(args=args)
    node = SlidingObstaclePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
