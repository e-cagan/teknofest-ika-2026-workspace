"""
Fake STM32 Node — UART Olmadan Hareket Simülasyonu

/cmd_vel alır → basit kinematik model → /odom + TF yayınlar
Gerçek stm32_bridge_node yerine kullanılır.
Ayrıca sahte batarya, safety, heartbeat yayınlar.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from ika_msgs.msg import VehicleState, SafetyStatus


class FakeSTM32Node(Node):

    def __init__(self):
        super().__init__('fake_stm32_node')

        self.declare_parameter('wheel_base', 0.5)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('battery_voltage', 24.0)

        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_speed = self.get_parameter('max_speed').value
        self.bat_voltage = self.get_parameter('battery_voltage').value

        # Simülasyon durumu
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vth = 0.0
        self.last_time = self.get_clock().now()

        # E-stop
        self.estop = False

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(Bool, '/safety/estop', self._estop_cb, 10)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', sensor_qos)
        self.vehicle_pub = self.create_publisher(VehicleState, '/vehicle_state', sensor_qos)
        self.heartbeat_pub = self.create_publisher(Bool, '/safety/stm32_heartbeat', 10)
        self.safety_pub = self.create_publisher(SafetyStatus, '/safety/status', 10)

        # TF
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timerlar
        rate = self.get_parameter('update_rate').value
        self.create_timer(1.0 / rate, self._update)
        self.create_timer(0.5, self._publish_heartbeat)
        self.create_timer(1.0, self._publish_vehicle_state)
        self.create_timer(0.2, self._publish_safety)

        self.get_logger().info('Fake STM32 başlatıldı — UART simülasyonu aktif')

    def _cmd_vel_cb(self, msg: Twist):
        if self.estop:
            self.vx = 0.0
            self.vth = 0.0
            return
        self.vx = max(-self.max_speed, min(self.max_speed, msg.linear.x))
        self.vth = msg.angular.z

    def _estop_cb(self, msg: Bool):
        self.estop = msg.data
        if self.estop:
            self.vx = 0.0
            self.vth = 0.0

    def _update(self):
        """Basit kinematik model ile pozisyon güncelle."""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0 or dt > 1.0:
            return

        # Kinematik güncelleme
        delta_x = self.vx * math.cos(self.theta) * dt
        delta_y = self.vx * math.sin(self.theta) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_th
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Quaternion
        q = Quaternion()
        q.z = math.sin(self.theta / 2.0)
        q.w = math.cos(self.theta / 2.0)

        # Odom mesajı
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth
        self.odom_pub.publish(odom)

        # TF: odom → base_link
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

    def _publish_heartbeat(self):
        msg = Bool()
        msg.data = True
        self.heartbeat_pub.publish(msg)

    def _publish_vehicle_state(self):
        msg = VehicleState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.speed_mps = abs(self.vx)
        msg.heading_rad = self.theta
        msg.battery_voltage = self.bat_voltage
        msg.battery_current = 2.5
        msg.headlights_on = False
        self.vehicle_pub.publish(msg)

    def _publish_safety(self):
        msg = SafetyStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.estop_physical = False
        msg.estop_active = self.estop
        msg.heartbeat_jetson_ok = True
        msg.heartbeat_operator_ok = True
        msg.battery_voltage = self.bat_voltage
        msg.system_health = SafetyStatus.HEALTH_OK
        self.safety_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeSTM32Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
