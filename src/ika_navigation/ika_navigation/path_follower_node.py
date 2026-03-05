"""
Bariyer Takibi / Yol Merkezi Takip Node'u

Algı: barrier_detector_node → /perception/lane_center (PoseStamped)
Çıkış: /cmd_vel_nav (Twist)

Yöntem: PID kontrol ile yol merkezinde kalma.
  - Lateral PID: y offset → angular_z düzeltmesi
  - Heading PID: yaw hata → angular_z düzeltmesi
  - Linear hız: sabit veya bariyere yakınlığa göre adaptif
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped


class PIDController:
    """Basit PID kontrol."""

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit

        self.integral = 0.0
        self.prev_error = 0.0
        self.first_call = True

    def compute(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0

        # Proportional
        p = self.kp * error

        # Integral (anti-windup ile)
        self.integral += error * dt
        if self.output_limit:
            self.integral = max(-self.output_limit, min(self.output_limit, self.integral))
        i = self.ki * self.integral

        # Derivative
        if self.first_call:
            d_term = 0.0
            self.first_call = False
        else:
            d_term = self.kd * (error - self.prev_error) / dt

        self.prev_error = error

        output = p + i + d_term
        if self.output_limit:
            output = max(-self.output_limit, min(self.output_limit, output))

        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.first_call = True


class PathFollowerNode(Node):

    def __init__(self):
        super().__init__('path_follower_node')

        # Parametreler
        self.declare_parameter('lane_center_topic', '/perception/lane_center')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('lateral_kp', 1.2)
        self.declare_parameter('lateral_ki', 0.0)
        self.declare_parameter('lateral_kd', 0.3)
        self.declare_parameter('heading_kp', 1.5)
        self.declare_parameter('heading_ki', 0.0)
        self.declare_parameter('heading_kd', 0.2)
        self.declare_parameter('default_linear_speed', 0.5)
        self.declare_parameter('slow_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('min_barrier_distance_m', 0.5)
        self.declare_parameter('stop_distance_m', 0.3)

        self.default_speed = self.get_parameter('default_linear_speed').value
        self.slow_speed = self.get_parameter('slow_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.min_barrier_dist = self.get_parameter('min_barrier_distance_m').value
        self.stop_dist = self.get_parameter('stop_distance_m').value

        # PID
        self.lateral_pid = PIDController(
            kp=self.get_parameter('lateral_kp').value,
            ki=self.get_parameter('lateral_ki').value,
            kd=self.get_parameter('lateral_kd').value,
            output_limit=self.max_angular,
        )
        self.heading_pid = PIDController(
            kp=self.get_parameter('heading_kp').value,
            ki=self.get_parameter('heading_ki').value,
            kd=self.get_parameter('heading_kd').value,
            output_limit=self.max_angular,
        )

        # Durum
        self.latest_lane_center = None
        self.last_time = self.get_clock().now()
        self.active = False  # stage_manager tarafından aktif edilecek

        # ROS
        self.create_subscription(
            PoseStamped,
            self.get_parameter('lane_center_topic').value,
            self._lane_center_callback, 10
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            10
        )

        rate = self.get_parameter('rate').value
        self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info('Path follower başlatıldı')

    def _lane_center_callback(self, msg: PoseStamped):
        self.latest_lane_center = msg

    def _control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        cmd = Twist()

        if self.latest_lane_center is None:
            # Lane center verisi yok — yavaş düz git
            cmd.linear.x = self.slow_speed * 0.5
            self.cmd_vel_pub.publish(cmd)
            return

        pose = self.latest_lane_center

        # Lateral hata: hedef noktanın y offset'i (0 = tam merkezde)
        lateral_error = pose.pose.position.y

        # Heading hata: hedef noktaya doğru yaw
        heading_error = math.atan2(
            pose.pose.position.y, pose.pose.position.x
        )

        # PID hesapla
        lateral_correction = self.lateral_pid.compute(lateral_error, dt)
        heading_correction = self.heading_pid.compute(heading_error, dt)

        # Angular: iki PID'nin ağırlıklı toplamı
        angular_z = 0.6 * lateral_correction + 0.4 * heading_correction
        angular_z = max(-self.max_angular, min(self.max_angular, angular_z))

        # Linear hız: lateral hataya göre adaptif
        if abs(lateral_error) > self.min_barrier_dist:
            linear_x = self.slow_speed
        else:
            linear_x = self.default_speed

        cmd.linear.x = linear_x
        cmd.angular.z = angular_z

        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
