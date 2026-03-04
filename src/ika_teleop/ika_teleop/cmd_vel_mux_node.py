"""
cmd_vel Multiplexer — TEKNOFEST 2026 İKA

Birden fazla cmd_vel kaynağını (teleop, navigation) tek /cmd_vel çıkışına yönlendirir.
Aktif moda göre doğru kaynağı seçer.

Kurallar:
  MODE_IDLE           → sıfır çıkış
  MODE_MANUAL         → /cmd_vel_teleop → /cmd_vel
  MODE_AUTONOMOUS     → /cmd_vel_nav → /cmd_vel
  MODE_ESTOP          → sıfır çıkış (kesinlikle)
  MODE_TARGETING_LOCK → sıfır çıkış (atış sırasında hareket yasak)

Input topics:
  /cmd_vel_teleop   — teleop node'larından (joystick veya keyboard)
  /cmd_vel_nav      — navigation node'larından (otonom sürüş)
  /system/mode      — aktif mod bilgisi

Output:
  /cmd_vel          — STM32 bridge'e giden nihai komut
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ika_msgs.msg import SystemMode


class CmdVelMuxNode(Node):

    def __init__(self):
        super().__init__('cmd_vel_mux_node')

        # ── Parametreler ──
        self.declare_parameter('teleop_timeout', 0.5)
        self.declare_parameter('nav_timeout', 0.5)
        self.declare_parameter('publish_rate', 20.0)

        self.teleop_timeout = self.get_parameter('teleop_timeout').value
        self.nav_timeout = self.get_parameter('nav_timeout').value
        publish_rate = self.get_parameter('publish_rate').value

        # ── Durum ──
        self.current_mode = SystemMode.MODE_IDLE

        self.last_teleop_cmd = Twist()
        self.last_nav_cmd = Twist()
        self.last_teleop_time = self.get_clock().now()
        self.last_nav_time = self.get_clock().now()

        # ── Subscribers ──
        self.teleop_sub = self.create_subscription(
            Twist, '/cmd_vel_teleop', self._teleop_callback, 10
        )
        self.nav_sub = self.create_subscription(
            Twist, '/cmd_vel_nav', self._nav_callback, 10
        )
        self.mode_sub = self.create_subscription(
            SystemMode, '/system/mode', self._mode_callback, 10
        )

        # ── Publisher ──
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Timer ──
        self.timer = self.create_timer(1.0 / publish_rate, self._mux_callback)

        self.get_logger().info('cmd_vel mux başlatıldı')

    def _teleop_callback(self, msg: Twist):
        self.last_teleop_cmd = msg
        self.last_teleop_time = self.get_clock().now()

    def _nav_callback(self, msg: Twist):
        self.last_nav_cmd = msg
        self.last_nav_time = self.get_clock().now()

    def _mode_callback(self, msg: SystemMode):
        old_mode = self.current_mode
        self.current_mode = msg.mode
        if old_mode != self.current_mode:
            mode_names = {
                SystemMode.MODE_IDLE: 'IDLE',
                SystemMode.MODE_MANUAL: 'MANUAL',
                SystemMode.MODE_AUTONOMOUS: 'AUTONOMOUS',
                SystemMode.MODE_ESTOP: 'ESTOP',
                SystemMode.MODE_TARGETING_LOCK: 'TARGETING_LOCK',
            }
            self.get_logger().info(
                f'Mod değişti: {mode_names.get(old_mode, "?")} → '
                f'{mode_names.get(self.current_mode, "?")}'
            )

    def _is_timed_out(self, last_time, timeout: float) -> bool:
        """Kaynak timeout oldu mu?"""
        dt = (self.get_clock().now() - last_time).nanoseconds / 1e9
        return dt > timeout

    def _mux_callback(self):
        """Mod'a göre doğru cmd_vel kaynağını seç ve yayınla."""
        output = Twist()  # Varsayılan: sıfır (dur)

        if self.current_mode == SystemMode.MODE_MANUAL:
            if not self._is_timed_out(self.last_teleop_time, self.teleop_timeout):
                output = self.last_teleop_cmd

        elif self.current_mode == SystemMode.MODE_AUTONOMOUS:
            if not self._is_timed_out(self.last_nav_time, self.nav_timeout):
                output = self.last_nav_cmd

        elif self.current_mode == SystemMode.MODE_ESTOP:
            # Kesinlikle sıfır — output zaten Twist() default
            pass

        elif self.current_mode == SystemMode.MODE_TARGETING_LOCK:
            # Atış sırasında hareket yasak — sıfır
            pass

        elif self.current_mode == SystemMode.MODE_IDLE:
            # Boşta — sıfır
            pass

        self.cmd_vel_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
