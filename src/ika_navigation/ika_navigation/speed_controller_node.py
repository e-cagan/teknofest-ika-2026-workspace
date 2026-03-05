"""
Hızlanma Parkuru Node'u

Şartname: 30m boyunca sıfırdan başlayarak ivmeli hızlanma.
Bitimde 10m güvenli durma mesafesi. 10m'de duramayan araçlara cezai puan.
Diğer takımlarla eş zamanlı yarış.

Davranış:
  IDLE       → bekleme
  ACCELERATE → maks hıza kadar ivmelen (30m boyunca)
  BRAKE      → mesafe sonunda güvenli frenle (10m içinde)
  STOPPED    → durdu, tamamlandı
"""

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

STATE_IDLE = 0
STATE_ACCELERATE = 1
STATE_BRAKE = 2
STATE_STOPPED = 3


class SpeedControllerNode(Node):

    def __init__(self):
        super().__init__('speed_controller_node')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('sprint_speed', 4.0)
        self.declare_parameter('acceleration_mps2', 2.0)
        self.declare_parameter('braking_distance_m', 8.0)
        self.declare_parameter('braking_deceleration_mps2', 3.0)
        self.declare_parameter('sprint_distance_m', 30.0)

        self.sprint_speed = self.get_parameter('sprint_speed').value
        self.accel = self.get_parameter('acceleration_mps2').value
        self.brake_dist = self.get_parameter('braking_distance_m').value
        self.brake_decel = self.get_parameter('braking_deceleration_mps2').value
        self.sprint_dist = self.get_parameter('sprint_distance_m').value

        # Durum
        self.state = STATE_IDLE
        self.current_speed = 0.0
        self.distance_traveled = 0.0
        self.last_time = time.time()
        self.start_odom_x = None
        self.current_odom_x = 0.0

        # ROS
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10
        )

        rate = self.get_parameter('rate').value
        self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info('Hızlanma parkuru node başlatıldı')

    def _odom_callback(self, msg: Odometry):
        self.current_odom_x = msg.pose.pose.position.x
        self.current_speed = abs(msg.twist.twist.linear.x)

    def start_sprint(self):
        """Stage manager tarafından çağrılır — sprint başlat."""
        self.state = STATE_ACCELERATE
        self.start_odom_x = self.current_odom_x
        self.distance_traveled = 0.0
        self.get_logger().info('Hızlanma parkuru başladı!')

    def _control_loop(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        cmd = Twist()

        if self.state == STATE_IDLE:
            self.cmd_vel_pub.publish(cmd)
            return

        # Mesafe hesabı
        if self.start_odom_x is not None:
            self.distance_traveled = abs(self.current_odom_x - self.start_odom_x)

        remaining = self.sprint_dist - self.distance_traveled

        if self.state == STATE_ACCELERATE:
            # İvmelen
            target_speed = min(
                self.current_speed + self.accel * dt,
                self.sprint_speed
            )
            cmd.linear.x = target_speed

            # Frenleme mesafesine gelince frenle
            # v² = 2*a*d → frenleme mesafesi = v²/(2*a)
            brake_needed = (self.current_speed ** 2) / (2.0 * self.brake_decel)
            if remaining <= brake_needed + 2.0:  # 2m güvenlik payı
                self.state = STATE_BRAKE
                self.get_logger().info(
                    f'Frenleme başladı — mesafe={self.distance_traveled:.1f}m, '
                    f'kalan={remaining:.1f}m, hız={self.current_speed:.1f}m/s'
                )

        elif self.state == STATE_BRAKE:
            # Frenle
            target_speed = max(
                self.current_speed - self.brake_decel * dt,
                0.0
            )
            cmd.linear.x = target_speed

            if self.current_speed < 0.05:
                self.state = STATE_STOPPED
                self.get_logger().info(
                    f'Hızlanma parkuru tamamlandı — '
                    f'toplam mesafe={self.distance_traveled:.1f}m'
                )

        elif self.state == STATE_STOPPED:
            cmd.linear.x = 0.0

        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SpeedControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
