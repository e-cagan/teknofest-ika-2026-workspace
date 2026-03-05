"""
Stage Manager Node — Aşama Geçiş Yönetimi

Perception'dan gelen tabela tespitine göre hangi aşamadayız belirler.
Her aşama geçişinde behavior_executor_node'a yeni davranış tetikler.

Pipeline:
  /perception/stage_info → tabela tespiti
  → Ardışık onay (noise filtreleme)
  → Aşama geçişi
  → /autonomy/current_stage yayını
  → /autonomy/stage_transition olayı

Aşama sırası sabit: 1 → 2 → 3 → ... → 11
Geri dönüş yok (parkur tek yönlü).
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ika_msgs.msg import StageInfo, SystemMode


class StageManagerNode(Node):

    def __init__(self):
        super().__init__('stage_manager_node')

        # Parametreler
        self.declare_parameter('stage_info_topic', '/perception/stage_info')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('stage_confirm_count', 3)
        self.declare_parameter('stage_confirm_timeout_sec', 2.0)
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('stage_order', [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11])

        self.confirm_count = self.get_parameter('stage_confirm_count').value
        self.confirm_timeout = self.get_parameter('stage_confirm_timeout_sec').value
        self.min_conf = self.get_parameter('min_confidence').value
        self.stage_order = self.get_parameter('stage_order').value

        # ── Durum ──
        self.current_stage_index = -1  # Henüz başlamadı
        self.current_stage_id = 0
        self.candidate_stage = 0
        self.candidate_count = 0
        self.candidate_first_seen = 0.0

        self.stage_completed = set()
        self.run_active = False

        # ── ROS ──
        self.create_subscription(
            StageInfo,
            self.get_parameter('stage_info_topic').value,
            self._stage_info_callback, 10
        )
        self.create_subscription(
            SystemMode, '/system/mode', self._mode_callback, 10
        )

        self.stage_pub = self.create_publisher(StageInfo, '/autonomy/current_stage', 10)
        self.transition_pub = self.create_publisher(String, '/autonomy/stage_transition', 10)

        rate = self.get_parameter('rate').value
        self.create_timer(1.0 / rate, self._publish_current_stage)

        self.get_logger().info(
            f'Stage manager başlatıldı — '
            f'sıra={self.stage_order}, '
            f'onay={self.confirm_count} tespit'
        )

    def _mode_callback(self, msg: SystemMode):
        was_active = self.run_active
        self.run_active = msg.run_active

        if self.run_active and not was_active:
            self._reset_for_new_run()

    def _reset_for_new_run(self):
        """Yeni koşu başlangıcı."""
        self.current_stage_index = -1
        self.current_stage_id = 0
        self.candidate_stage = 0
        self.candidate_count = 0
        self.stage_completed = set()
        self.get_logger().info('Yeni koşu — stage manager sıfırlandı')

    def _stage_info_callback(self, msg: StageInfo):
        """Tabela tespiti geldiğinde."""
        if not self.run_active:
            return

        if msg.stage_id == 0 or msg.confidence < self.min_conf:
            return

        detected_id = msg.stage_id

        # Zaten tamamlanmış aşama → yoksay
        if detected_id in self.stage_completed:
            return

        # Geçerli aşama mı — sıradaki veya sonraki olmalı
        if detected_id in self.stage_order:
            detected_index = self.stage_order.index(detected_id)

            # Geriye gidemeyiz
            if detected_index <= self.current_stage_index:
                return

        # Aday onaylama
        now = time.time()

        if detected_id == self.candidate_stage:
            # Aynı aday — sayacı artır
            self.candidate_count += 1

            # Timeout kontrolü
            if now - self.candidate_first_seen > self.confirm_timeout:
                self.candidate_count = 1
                self.candidate_first_seen = now

        else:
            # Yeni aday
            self.candidate_stage = detected_id
            self.candidate_count = 1
            self.candidate_first_seen = now

        # Yeterli onay → aşama geçişi
        if self.candidate_count >= self.confirm_count:
            self._transition_to_stage(detected_id)
            self.candidate_stage = 0
            self.candidate_count = 0

    def _transition_to_stage(self, new_stage_id: int):
        """Yeni aşamaya geçiş."""
        old_stage = self.current_stage_id

        # Eski aşamayı tamamlanmış olarak işaretle
        if self.current_stage_id > 0:
            self.stage_completed.add(self.current_stage_id)

        self.current_stage_id = new_stage_id

        if new_stage_id in self.stage_order:
            self.current_stage_index = self.stage_order.index(new_stage_id)

        self.get_logger().info(
            f'AŞAMA GEÇİŞİ: {old_stage} → {new_stage_id}'
        )

        # Transition event yayınla
        transition_msg = String()
        transition_msg.data = f'{old_stage}:{new_stage_id}'
        self.transition_pub.publish(transition_msg)

    def _publish_current_stage(self):
        """Mevcut aşamayı periyodik yayınla."""
        msg = StageInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.stage_id = self.current_stage_id
        msg.confidence = 1.0 if self.current_stage_id > 0 else 0.0
        msg.distance_m = 0.0
        msg.bearing_rad = 0.0
        self.stage_pub.publish(msg)

    def skip_to_next_stage(self):
        """Pas geçme durumunda sonraki aşamaya atla."""
        if self.current_stage_id > 0:
            self.stage_completed.add(self.current_stage_id)

        if self.current_stage_index + 1 < len(self.stage_order):
            next_stage = self.stage_order[self.current_stage_index + 1]
            self._transition_to_stage(next_stage)
        else:
            self.get_logger().info('Tüm aşamalar tamamlandı / pas geçildi')
            self.current_stage_id = 0


def main(args=None):
    rclpy.init(args=args)
    node = StageManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
