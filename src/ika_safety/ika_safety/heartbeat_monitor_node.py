"""
Heartbeat Monitor Node

İki heartbeat kaynağını izler:
  1. STM32 ↔ Jetson (stm32_bridge_node'un yayınladığı /safety/stm32_heartbeat)
  2. Operatör ↔ Araç (Foxglove'dan periyodik gelen /safety/operator_heartbeat)

Herhangi biri timeout olursa /safety/heartbeat_lost yayınlar.
estop_relay_node bunu dinleyerek e-stop tetikler.
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class HeartbeatMonitorNode(Node):

    def __init__(self):
        super().__init__('heartbeat_monitor_node')

        # Parametreler
        self.declare_parameter('stm32_heartbeat_topic', '/safety/stm32_heartbeat')
        self.declare_parameter('operator_heartbeat_topic', '/safety/operator_heartbeat')
        self.declare_parameter('stm32_timeout_sec', 2.0)
        self.declare_parameter('operator_timeout_sec', 3.0)
        self.declare_parameter('check_rate', 5.0)

        stm32_topic = self.get_parameter('stm32_heartbeat_topic').value
        operator_topic = self.get_parameter('operator_heartbeat_topic').value
        self.stm32_timeout = self.get_parameter('stm32_timeout_sec').value
        self.operator_timeout = self.get_parameter('operator_timeout_sec').value
        check_rate = self.get_parameter('check_rate').value

        # Son heartbeat zamanları
        self.last_stm32_hb = time.time()
        self.last_operator_hb = time.time()

        # Durum
        self.stm32_ok = True
        self.operator_ok = True

        # Subscribers
        self.create_subscription(Bool, stm32_topic, self._stm32_hb_cb, 10)
        self.create_subscription(Bool, operator_topic, self._operator_hb_cb, 10)

        # Publishers
        self.heartbeat_lost_pub = self.create_publisher(
            String, '/safety/heartbeat_lost', 10
        )
        self.stm32_status_pub = self.create_publisher(
            Bool, '/safety/stm32_alive', 10
        )
        self.operator_status_pub = self.create_publisher(
            Bool, '/safety/operator_alive', 10
        )

        # Timer
        self.create_timer(1.0 / check_rate, self._check_heartbeats)

        self.get_logger().info(
            f'Heartbeat monitor başlatıldı — '
            f'STM32 timeout={self.stm32_timeout}s, '
            f'Operator timeout={self.operator_timeout}s'
        )

    def _stm32_hb_cb(self, msg: Bool):
        if msg.data:
            self.last_stm32_hb = time.time()

    def _operator_hb_cb(self, msg: Bool):
        self.last_operator_hb = time.time()

    def _check_heartbeats(self):
        now = time.time()

        # STM32 kontrolü
        stm32_elapsed = now - self.last_stm32_hb
        stm32_was_ok = self.stm32_ok
        self.stm32_ok = stm32_elapsed < self.stm32_timeout

        if not self.stm32_ok and stm32_was_ok:
            self.get_logger().error(
                f'STM32 heartbeat KAYIP! ({stm32_elapsed:.1f}s > {self.stm32_timeout}s)'
            )
            lost_msg = String()
            lost_msg.data = 'stm32'
            self.heartbeat_lost_pub.publish(lost_msg)

        if self.stm32_ok and not stm32_was_ok:
            self.get_logger().info('STM32 heartbeat geri geldi')

        # Operatör kontrolü
        operator_elapsed = now - self.last_operator_hb
        operator_was_ok = self.operator_ok
        self.operator_ok = operator_elapsed < self.operator_timeout

        if not self.operator_ok and operator_was_ok:
            self.get_logger().error(
                f'Operatör heartbeat KAYIP! ({operator_elapsed:.1f}s > {self.operator_timeout}s)'
            )
            lost_msg = String()
            lost_msg.data = 'operator'
            self.heartbeat_lost_pub.publish(lost_msg)

        if self.operator_ok and not operator_was_ok:
            self.get_logger().info('Operatör heartbeat geri geldi')

        # Durum yayınla
        stm32_msg = Bool()
        stm32_msg.data = self.stm32_ok
        self.stm32_status_pub.publish(stm32_msg)

        operator_msg = Bool()
        operator_msg.data = self.operator_ok
        self.operator_status_pub.publish(operator_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
