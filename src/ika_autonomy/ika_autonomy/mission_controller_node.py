"""
Mission Controller Node — Üst Seviye Görev Yönetimi

Sorumluluklar:
  - Sistem modu yönetimi (IDLE / MANUAL / AUTONOMOUS / ESTOP / TARGETING_LOCK)
  - 15 dakika koşu süresi takibi
  - Pas hakkı yönetimi (koşu başına 1)
  - Koşu başlatma/durdurma
  - SystemMode mesajı yayını

Servisler:
  /system/set_mode   → SetMode — mod geçişi
  /system/skip_stage → SkipStage — pas hakkı kullanımı

Publish:
  /system/mode → SystemMode — tüm node'lar bunu dinler
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ika_msgs.msg import SystemMode, StageInfo
from ika_msgs.srv import SetMode, SkipStage


class MissionControllerNode(Node):

    def __init__(self):
        super().__init__('mission_controller_node')

        # Parametreler
        self.declare_parameter('run_time_limit_sec', 900.0)
        self.declare_parameter('warning_time_sec', 60.0)
        self.declare_parameter('skips_per_run', 1)
        self.declare_parameter('unskippable_stages_manual', [11])
        self.declare_parameter('unskippable_stages_auto', [5, 11])
        self.declare_parameter('stage_max_points', {})

        self.time_limit = self.get_parameter('run_time_limit_sec').value
        self.warning_time = self.get_parameter('warning_time_sec').value
        self.max_skips = self.get_parameter('skips_per_run').value
        self.unskippable_manual = self.get_parameter('unskippable_stages_manual').value
        self.unskippable_auto = self.get_parameter('unskippable_stages_auto').value

        # Stage max points — pas geçme cezası hesabı
        self.stage_points = {}
        raw = self.get_parameter('stage_max_points').value
        if isinstance(raw, dict):
            self.stage_points = {int(k): int(v) for k, v in raw.items()}

        # ── Durum ──
        self.current_mode = SystemMode.MODE_IDLE
        self.current_stage_id = 0
        self.run_active = False
        self.run_start_time = None
        self.skips_remaining = self.max_skips
        self.skipped_stages = []
        self.warning_issued = False

        # ── Publishers ──
        self.mode_pub = self.create_publisher(SystemMode, '/system/mode', 10)
        self.event_pub = self.create_publisher(String, '/system/event', 10)

        # ── Subscribers ──
        self.create_subscription(
            StageInfo, '/autonomy/current_stage', self._stage_callback, 10
        )

        # ── Servisler ──
        self.set_mode_srv = self.create_service(
            SetMode, '/system/set_mode', self._set_mode_callback
        )
        self.skip_stage_srv = self.create_service(
            SkipStage, '/system/skip_stage', self._skip_stage_callback
        )

        # ── Timer ──
        self.create_timer(0.1, self._publish_mode)  # 10 Hz mode yayını
        self.create_timer(1.0, self._check_time)     # 1 Hz süre kontrolü

        self.get_logger().info(
            f'Mission controller başlatıldı — '
            f'süre limiti={self.time_limit}s, '
            f'pas hakkı={self.max_skips}'
        )

    def _stage_callback(self, msg: StageInfo):
        """Stage manager'dan aktif aşama bilgisi."""
        self.current_stage_id = msg.stage_id

    def _publish_mode(self):
        """Sistem modunu periyodik yayınla."""
        msg = SystemMode()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = self.current_mode
        msg.current_stage_id = self.current_stage_id
        msg.run_active = self.run_active

        if self.run_active and self.run_start_time is not None:
            msg.elapsed_time_sec = time.time() - self.run_start_time
        else:
            msg.elapsed_time_sec = 0.0

        self.mode_pub.publish(msg)

    def _check_time(self):
        """Koşu süresi kontrolü."""
        if not self.run_active or self.run_start_time is None:
            return

        elapsed = time.time() - self.run_start_time
        remaining = self.time_limit - elapsed

        # Son dakika uyarısı
        if remaining <= self.warning_time and not self.warning_issued:
            self.warning_issued = True
            self.get_logger().warn(f'UYARI: Koşu süresinin bitmesine {remaining:.0f} saniye kaldı!')
            self._publish_event(f'TIME_WARNING:{remaining:.0f}s')

        # Süre doldu
        if remaining <= 0:
            self.get_logger().error('SÜRE DOLDU — Koşu sonlandırılıyor!')
            self._publish_event('TIME_EXPIRED')
            self._end_run()

    def _publish_event(self, event: str):
        """Sistem olayı yayınla (Foxglove'da görüntüleme için)."""
        msg = String()
        msg.data = event
        self.event_pub.publish(msg)

    def _start_run(self, mode: int):
        """Koşu başlat."""
        self.run_active = True
        self.run_start_time = time.time()
        self.skips_remaining = self.max_skips
        self.skipped_stages = []
        self.warning_issued = False
        self.current_stage_id = 0

        mode_name = 'MANUEL' if mode == SystemMode.MODE_MANUAL else 'OTONOM'
        self.get_logger().info(f'Koşu başladı — mod: {mode_name}')
        self._publish_event(f'RUN_START:{mode_name}')

    def _end_run(self):
        """Koşu sonlandır."""
        self.run_active = False
        elapsed = time.time() - self.run_start_time if self.run_start_time else 0
        self.current_mode = SystemMode.MODE_IDLE

        self.get_logger().info(f'Koşu tamamlandı — süre: {elapsed:.1f}s')
        self._publish_event(f'RUN_END:{elapsed:.1f}s')

    def _set_mode_callback(self, request, response):
        """Mod değiştirme servisi."""
        requested = request.requested_mode
        prev_mode = self.current_mode

        # Geçiş kuralları
        if requested == SystemMode.MODE_ESTOP:
            # E-stop her zaman kabul edilir
            self.current_mode = SystemMode.MODE_ESTOP
            response.success = True
            response.message = 'E-STOP aktif'

        elif self.current_mode == SystemMode.MODE_ESTOP:
            if requested == SystemMode.MODE_IDLE:
                self.current_mode = SystemMode.MODE_IDLE
                response.success = True
                response.message = 'E-STOP serbest → IDLE'
            else:
                response.success = False
                response.message = 'E-STOP aktifken sadece IDLE moduna geçilebilir'

        elif requested == SystemMode.MODE_MANUAL:
            self.current_mode = SystemMode.MODE_MANUAL
            if not self.run_active:
                self._start_run(SystemMode.MODE_MANUAL)
            response.success = True
            response.message = 'MANUAL mod aktif'

        elif requested == SystemMode.MODE_AUTONOMOUS:
            self.current_mode = SystemMode.MODE_AUTONOMOUS
            if not self.run_active:
                self._start_run(SystemMode.MODE_AUTONOMOUS)
            response.success = True
            response.message = 'AUTONOMOUS mod aktif'

        elif requested == SystemMode.MODE_TARGETING_LOCK:
            if self.current_mode in [SystemMode.MODE_MANUAL, SystemMode.MODE_AUTONOMOUS]:
                self.current_mode = SystemMode.MODE_TARGETING_LOCK
                response.success = True
                response.message = 'TARGETING_LOCK aktif — araç hareketsiz'
            else:
                response.success = False
                response.message = 'TARGETING_LOCK sadece MANUAL veya AUTONOMOUS moddan'

        elif requested == SystemMode.MODE_IDLE:
            if self.run_active:
                self._end_run()
            self.current_mode = SystemMode.MODE_IDLE
            response.success = True
            response.message = 'IDLE — sistem boşta'

        else:
            response.success = False
            response.message = f'Bilinmeyen mod: {requested}'

        response.previous_mode = prev_mode
        response.current_mode = self.current_mode

        if response.success and prev_mode != self.current_mode:
            self.get_logger().info(response.message)

        return response

    def _skip_stage_callback(self, request, response):
        """Pas geçme servisi."""
        stage_id = request.stage_id

        # Pas hakkı var mı
        if self.skips_remaining <= 0:
            response.success = False
            response.message = 'Pas hakkı kalmadı'
            response.skips_remaining = 0
            response.penalty_points = 0
            return response

        # Pas geçilebilir mi
        unskippable = self.unskippable_auto if \
            self.current_mode == SystemMode.MODE_AUTONOMOUS else \
            self.unskippable_manual

        if stage_id in unskippable:
            response.success = False
            response.message = f'Aşama {stage_id} pas geçilemez'
            response.skips_remaining = self.skips_remaining
            response.penalty_points = 0
            return response

        # Pas geç
        self.skips_remaining -= 1
        self.skipped_stages.append(stage_id)

        penalty = -self.stage_points.get(stage_id, 0)

        response.success = True
        response.message = f'Aşama {stage_id} pas geçildi — ceza: {penalty} puan'
        response.skips_remaining = self.skips_remaining
        response.penalty_points = penalty

        self.get_logger().info(response.message)
        self._publish_event(f'STAGE_SKIP:{stage_id}:{penalty}')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = MissionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
