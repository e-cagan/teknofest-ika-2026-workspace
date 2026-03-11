"""
Behavior Executor Node — Aşama Bazlı Davranış Yönetimi

Stage manager aşama geçişi bildirdiğinde, ilgili navigation stratejisini aktif eder.
Birden fazla nav node'un aynı anda /cmd_vel_nav'a yazmasını engeller.

Aşama → Davranış eşlemesi config'den gelir:
  1:  "default"      → path_follower (yavaş, su geçişi)
  2:  "default"      → path_follower (yavaş, çakıllı)
  3:  "default"      → path_follower + slope_controller yan eğim
  4:  "default"      → path_follower (yavaş, dik engel)
  5:  "cone_avoid"   → cone_avoidance_node aktif
  6:  "slide_pass"   → sliding_obstacle_planner aktif
  7:  "default"      → path_follower (yavaş, engebeli)
  8:  "slope_climb"  → slope_controller tırmanma modu
  9:  "targeting"    → dur + atış sekansı (ika_targeting devralır)
  10: "slope_desc"   → slope_controller iniş modu
  11: "sprint"       → speed_controller hızlanma

Yöntem: /autonomy/active_behavior topic'i yayınlar.
Nav node'lar bu topic'i dinleyip kendi behavior'ları aktif olduğunda cmd_vel yazar,
değilse sessiz kalır.
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from ika_msgs.msg import SystemMode


# Davranış sabitleri
BEHAVIOR_IDLE = 'idle'
BEHAVIOR_DEFAULT = 'default'
BEHAVIOR_CONE_AVOID = 'cone_avoid'
BEHAVIOR_SLIDE_PASS = 'slide_pass'
BEHAVIOR_SLOPE_CLIMB = 'slope_climb'
BEHAVIOR_SLOPE_DESC = 'slope_desc'
BEHAVIOR_TARGETING = 'targeting'
BEHAVIOR_SPRINT = 'sprint'


class BehaviorExecutorNode(Node):

    def __init__(self):
        super().__init__('behavior_executor_node')

        # Parametreler
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('default_speed', 0.5)
        self.declare_parameter('slow_speed', 0.3)
        self.declare_parameter('water_speed', 0.3)
        self.declare_parameter('gravel_speed', 0.3)
        self.declare_parameter('bump_speed', 0.25)
        self.declare_parameter('obstacle_approach_speed', 0.3)

        # Stage → behavior eşlemesi
        self.declare_parameter('stage_behaviors', '{}')
        raw_behaviors = self.get_parameter('stage_behaviors').value

        self.stage_behavior_map = {}
        try:
            parsed_behaviors = json.loads(raw_behaviors)
            for k, v in parsed_behaviors.items():
                self.stage_behavior_map[int(k)] = str(v)
        except json.JSONDecodeError:
            self.get_logger().error("stage_behaviors geçerli bir JSON değil!")

        # Aşama bazlı hız ayarları
        self.stage_speeds = {
            1: self.get_parameter('water_speed').value,
            2: self.get_parameter('gravel_speed').value,
            3: self.get_parameter('slow_speed').value,
            4: self.get_parameter('obstacle_approach_speed').value,
            7: self.get_parameter('bump_speed').value,
        }
        self.default_speed = self.get_parameter('default_speed').value

        # ── Durum ──
        self.current_behavior = BEHAVIOR_IDLE
        self.current_stage_id = 0
        self.current_mode = SystemMode.MODE_IDLE

        # ── ROS ──
        self.create_subscription(
            String, '/autonomy/stage_transition',
            self._transition_callback, 10
        )
        self.create_subscription(
            SystemMode, '/system/mode', self._mode_callback, 10
        )

        # Aktif davranış yayını — nav node'lar bunu dinler
        self.behavior_pub = self.create_publisher(
            String, '/autonomy/active_behavior', 10
        )
        # Varsayılan cmd_vel (default behavior'da kullanılır)
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel_nav', 10
        )

        rate = self.get_parameter('rate').value
        self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info(
            f'Behavior executor başlatıldı — '
            f'stage_behaviors={self.stage_behavior_map}'
        )

    def _mode_callback(self, msg: SystemMode):
        self.current_mode = msg.mode
        self.current_stage_id = msg.current_stage_id

        if self.current_mode != SystemMode.MODE_AUTONOMOUS:
            self.current_behavior = BEHAVIOR_IDLE

    def _transition_callback(self, msg: String):
        """Aşama geçişi: 'old_stage:new_stage' formatında."""
        try:
            parts = msg.data.split(':')
            new_stage = int(parts[1])
        except (IndexError, ValueError):
            return

        # Yeni aşamaya uygun davranışı belirle
        new_behavior = self.stage_behavior_map.get(new_stage, BEHAVIOR_DEFAULT)
        old_behavior = self.current_behavior
        self.current_behavior = new_behavior
        self.current_stage_id = new_stage

        self.get_logger().info(
            f'Davranış geçişi: {old_behavior} → {new_behavior} '
            f'(aşama {new_stage})'
        )

    def _control_loop(self):
        """Aktif davranışı yayınla ve default durumda cmd_vel gönder."""
        # Aktif davranışı yayınla
        behavior_msg = String()
        behavior_msg.data = self.current_behavior
        self.behavior_pub.publish(behavior_msg)

        # Otonom modda değilsek veya idle'daysa bir şey yapma
        if self.current_mode != SystemMode.MODE_AUTONOMOUS:
            return

        if self.current_behavior == BEHAVIOR_IDLE:
            return

        # "default" behavior: path_follower çalışır ama ek olarak
        # bu node aşamaya özel hız limiti set eder
        if self.current_behavior == BEHAVIOR_DEFAULT:
            # Path follower zaten /cmd_vel_nav'a yazıyor
            # Burada ek bir şey yapmamıza gerek yok
            # Path follower'ın hızını aşama bazlı ayarlamak
            # ileride parametre güncelleme ile yapılabilir
            pass

        # "targeting" behavior: araç durmalı, atış sistemi devralır
        elif self.current_behavior == BEHAVIOR_TARGETING:
            cmd = Twist()  # Sıfır — dur
            self.cmd_vel_pub.publish(cmd)

        # Diğer davranışlar kendi node'ları tarafından yönetilir
        # (cone_avoid, slide_pass, slope_climb/desc, sprint)
        # Bu node'lar /autonomy/active_behavior'ı dinleyip
        # kendi sıraları geldiğinde /cmd_vel_nav'a yazarlar


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()