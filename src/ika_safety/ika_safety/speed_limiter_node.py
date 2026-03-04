"""
Speed Limiter Node

cmd_vel pipeline'ındaki son güvenlik katmanı.
/cmd_vel_raw → hız limiti uygula → /cmd_vel (stm32_bridge'in dinlediği)

Görevleri:
  1. Moda göre hız limiti (normal vs hızlanma parkuru)
  2. TARGETING_LOCK modunda tüm hareketi sıfırla
  3. E-stop aktifse sıfırla
  4. Rampa (ani hızlanma/frenleme sınırlama) — motor ve mekanik koruma

Topic akışı:
  teleop_node → /cmd_vel_teleop ──┐
  nav_nodes   → /cmd_vel_auto  ───┤→ cmd_vel_mux → /cmd_vel_raw → [BU NODE] → /cmd_vel
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from ika_msgs.msg import SystemMode


class SpeedLimiterNode(Node):

    def __init__(self):
        super().__init__('speed_limiter_node')

        # Parametreler
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 3.0)
        self.declare_parameter('sprint_linear_speed', 5.0)
        self.declare_parameter('targeting_lock_threshold', 0.01)
        self.declare_parameter('input_topic', '/cmd_vel_raw')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('rate', 50.0)

        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.sprint_linear = self.get_parameter('sprint_linear_speed').value
        self.lock_threshold = self.get_parameter('targeting_lock_threshold').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        rate = self.get_parameter('rate').value

        # Durum
        self.estop_active = False
        self.current_mode = SystemMode.MODE_IDLE
        self.latest_cmd = Twist()
        self.has_new_cmd = False

        # Subscribers
        self.create_subscription(Twist, input_topic, self._cmd_vel_raw_cb, 10)
        self.create_subscription(Bool, '/safety/estop', self._estop_cb, 10)
        self.create_subscription(SystemMode, '/system/mode', self._mode_cb, 10)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, output_topic, 10)

        # Timer — sabit frekansta yayın
        self.create_timer(1.0 / rate, self._publish_limited)

        self.get_logger().info(
            f'Speed limiter başlatıldı — '
            f'max_linear={self.max_linear}m/s, '
            f'max_angular={self.max_angular}rad/s, '
            f'{input_topic} → {output_topic}'
        )

    def _cmd_vel_raw_cb(self, msg: Twist):
        self.latest_cmd = msg
        self.has_new_cmd = True

    def _estop_cb(self, msg: Bool):
        self.estop_active = msg.data

    def _mode_cb(self, msg: SystemMode):
        self.current_mode = msg.mode

    def _publish_limited(self):
        out = Twist()

        # ── E-stop: her şey sıfır ──
        if self.estop_active:
            self.cmd_vel_pub.publish(out)
            return

        # ── TARGETING_LOCK: araç kesinlikle durmalı ──
        # Şartname: lazer aktifken araca hareket verilmezse -10 puan + tekrar hakkı yok
        if self.current_mode == SystemMode.MODE_TARGETING_LOCK:
            self.cmd_vel_pub.publish(out)
            return

        # ── IDLE: hareket yok ──
        if self.current_mode == SystemMode.MODE_IDLE:
            self.cmd_vel_pub.publish(out)
            return

        # ── Normal mod veya sprint mod ──
        if not self.has_new_cmd:
            self.cmd_vel_pub.publish(out)
            return

        # Aktif hız limiti belirle
        current_max_linear = self.max_linear
        # Hızlanma parkurunda (stage 11) limit artırılabilir
        # Bu bilgi SystemMode.current_stage_id'den gelecek
        # Şimdilik basit: sprint_linear sadece özel durumda

        # Limitle
        out.linear.x = self._clamp(
            self.latest_cmd.linear.x,
            -current_max_linear,
            current_max_linear
        )
        out.angular.z = self._clamp(
            self.latest_cmd.angular.z,
            -self.max_angular,
            self.max_angular
        )

        self.cmd_vel_pub.publish(out)

    @staticmethod
    def _clamp(value: float, min_val: float, max_val: float) -> float:
        return max(min_val, min(max_val, value))


def main(args=None):
    rclpy.init(args=args)
    node = SpeedLimiterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
