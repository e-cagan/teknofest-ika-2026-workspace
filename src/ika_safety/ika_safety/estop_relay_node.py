"""
E-Stop Relay Node

Tüm e-stop kaynaklarını birleştirir ve tek bir /safety/estop (Bool) yayınlar.
stm32_bridge_node bu topic'i dinleyerek motorları durdurur.

Kaynaklar:
  1. Fiziksel buton (STM32'den → /safety/status içindeki estop_physical)
  2. Operatör yazılımsal buton (Foxglove'dan → /safety/estop_request)
  3. Heartbeat kaybı (heartbeat_monitor_node'dan → /safety/heartbeat_lost)

Güvenlik politikası:
  - Herhangi bir kaynak e-stop tetiklerse → /safety/estop = true
  - Serbest bırakma: require_manual_reset=true ise operatörün /safety/estop_reset
    servisi çağırması gerekir. Otomatik serbest bırakma yok.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from ika_msgs.msg import SafetyStatus


class EstopRelayNode(Node):

    def __init__(self):
        super().__init__('estop_relay_node')

        # Parametreler
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('require_manual_reset', True)

        publish_rate = self.get_parameter('publish_rate').value
        self.require_manual_reset = self.get_parameter('require_manual_reset').value

        # E-stop kaynakları durumu
        self.estop_physical = False       # Fiziksel buton
        self.estop_software = False       # Foxglove butonu
        self.estop_heartbeat_loss = False  # Heartbeat kaybı
        self.estop_latched = False         # Manuel reset gerekli mi

        # Subscribers
        self.create_subscription(
            SafetyStatus, '/safety/status', self._safety_status_cb, 10
        )
        self.create_subscription(
            Bool, '/safety/estop_request', self._estop_request_cb, 10
        )
        self.create_subscription(
            String, '/safety/heartbeat_lost', self._heartbeat_lost_cb, 10
        )
        self.create_subscription(
            Bool, '/safety/stm32_alive', self._stm32_alive_cb, 10
        )
        self.create_subscription(
            Bool, '/safety/operator_alive', self._operator_alive_cb, 10
        )

        # Publisher — ana e-stop çıkışı
        self.estop_pub = self.create_publisher(Bool, '/safety/estop', 10)

        # Reset servisi
        self.reset_srv = self.create_service(
            Trigger, '/safety/estop_reset', self._reset_callback
        )

        # Timer
        self.create_timer(1.0 / publish_rate, self._publish_estop)

        self.get_logger().info(
            f'E-Stop relay başlatıldı — manual_reset={self.require_manual_reset}'
        )

    def _safety_status_cb(self, msg: SafetyStatus):
        """stm32_bridge_node'dan gelen fiziksel buton durumu."""
        self.estop_physical = msg.estop_physical

    def _estop_request_cb(self, msg: Bool):
        """Foxglove'dan gelen yazılımsal e-stop."""
        self.estop_software = msg.data

    def _heartbeat_lost_cb(self, msg: String):
        """Heartbeat kaybı bildirimi."""
        self.estop_heartbeat_loss = True
        self.get_logger().warn(f'Heartbeat kaybı: {msg.data} → E-STOP tetiklendi')

    def _stm32_alive_cb(self, msg: Bool):
        """STM32 heartbeat durumu geri geldiyse kaynağı temizle."""
        if msg.data:
            # Sadece heartbeat kaynağını temizle, latch durumunu değil
            pass  # Reset servisi ile temizlenmeli

    def _operator_alive_cb(self, msg: Bool):
        """Operatör heartbeat durumu geri geldiyse kaynağı temizle."""
        if msg.data:
            pass  # Reset servisi ile temizlenmeli

    def _publish_estop(self):
        """E-stop durumunu değerlendir ve yayınla."""
        # Herhangi bir kaynak aktifse e-stop
        any_active = (
            self.estop_physical
            or self.estop_software
            or self.estop_heartbeat_loss
        )

        if any_active:
            self.estop_latched = True

        # Latch mekanizması: bir kez tetiklenince reset gerekir
        if self.require_manual_reset:
            estop_output = self.estop_latched
        else:
            estop_output = any_active

        msg = Bool()
        msg.data = estop_output
        self.estop_pub.publish(msg)

    def _reset_callback(self, request, response):
        """
        E-stop serbest bırakma servisi.
        Tüm kaynaklar temiz olmalı, ancak o zaman reset yapılabilir.
        """
        # Hala aktif bir kaynak var mı kontrol et
        if self.estop_physical:
            response.success = False
            response.message = 'Fiziksel e-stop butonu hala aktif — önce butonu serbest bırakın'
            self.get_logger().warn(response.message)
            return response

        if self.estop_heartbeat_loss:
            # Heartbeat geri geldiyse temizlenebilir
            response.success = False
            response.message = 'Heartbeat kaybı henüz çözülmedi'
            self.get_logger().warn(response.message)
            return response

        # Tüm kaynaklar temiz, reset yap
        self.estop_software = False
        self.estop_heartbeat_loss = False
        self.estop_latched = False

        response.success = True
        response.message = 'E-STOP serbest bırakıldı'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = EstopRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
