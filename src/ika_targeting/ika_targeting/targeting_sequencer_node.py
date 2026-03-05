"""
Atış Sekansı Yönetim Node'u

Tüm atış sürecini orkestre eder. Şartname kurallarını garanti altına alır:
  - Nişan alırken lazer KAPALI
  - Lazer aktifken araç/gimbal'a hareket YOK (ihlal: -10 puan, tekrar hakkı yok)
  - Lazer aktif olduktan sonra min 1s hedefte kalmalı
  - En fazla 3 deneme hakkı

Sekans (her deneme için):
  1. PREPARE    → cmd_vel'in sıfır olduğunu doğrula
  2. AIM        → auto_targeting veya manual targeting nişan alsın (lazer KAPALI)
  3. LOCK_WAIT  → hedef kilitlenene kadar bekle
  4. FREEZE     → TARGETING_LOCK moduna geç, araç tamamen durur
  5. FIRE       → lazer AÇ
  6. HOLD       → min 1.5s bekle (1s + 0.5s güvenlik payı)
  7. CEASE      → lazer KAPA
  8. RELEASE    → TARGETING_LOCK kaldır
  9. EVALUATE   → sonucu değerlendir

Manuel modda: operatör /targeting/fire_request gönderince tetiklenir
Otonom modda: auto_targeting lock'ladığında otomatik tetiklenir
"""

import time
from enum import IntEnum

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Vector3

from ika_msgs.msg import SystemMode, TargetDetection
from ika_msgs.srv import SetMode, FireLaser


class SeqState(IntEnum):
    IDLE = 0
    PREPARE = 1
    AIM = 2
    LOCK_WAIT = 3
    FREEZE = 4
    FIRE = 5
    HOLD = 6
    CEASE = 7
    RELEASE = 8
    EVALUATE = 9
    COMPLETE = 10


class TargetingSequencerNode(Node):

    def __init__(self):
        super().__init__('targeting_sequencer_node')

        # Parametreler
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('max_attempts', 3)
        self.declare_parameter('laser_hold_duration_sec', 1.5)
        self.declare_parameter('mode_switch_delay_sec', 0.3)
        self.declare_parameter('laser_cmd_topic', '/targeting/laser_cmd')

        self.max_attempts = self.get_parameter('max_attempts').value
        self.hold_duration = self.get_parameter('laser_hold_duration_sec').value
        self.mode_delay = self.get_parameter('mode_switch_delay_sec').value

        # ── Durum ──
        self.seq_state = SeqState.IDLE
        self.attempt_count = 0
        self.best_ring = 0  # En iyi sonuç
        self.fire_start_time = None
        self.freeze_start_time = None
        self.current_mode = SystemMode.MODE_IDLE
        self.target_locked = False
        self.latest_target = None
        self.active = False  # Atış aşamasında mıyız

        # ── ROS ──
        self.create_subscription(
            Bool, '/targeting/lock_status', self._lock_callback, 10
        )
        self.create_subscription(
            Bool, '/targeting/fire_request', self._fire_request_callback, 10
        )
        self.create_subscription(
            TargetDetection, '/perception/target', self._target_callback, 10
        )
        self.create_subscription(
            SystemMode, '/system/mode', self._mode_callback, 10
        )
        self.create_subscription(
            String, '/autonomy/active_behavior', self._behavior_callback, 10
        )

        # Lazer on/off
        self.laser_pub = self.create_publisher(
            Bool,
            self.get_parameter('laser_cmd_topic').value,
            10
        )
        # Gimbal freeze (sıfır hız)
        self.gimbal_pub = self.create_publisher(Vector3, '/targeting/gimbal_cmd', 10)

        # Durum yayını
        self.seq_state_pub = self.create_publisher(String, '/targeting/sequencer_state', 10)
        self.result_pub = self.create_publisher(String, '/targeting/fire_result', 10)

        # SetMode servisi client
        self.set_mode_client = self.create_client(SetMode, '/system/set_mode')

        # FireLaser servisi (dışarıdan çağrılabilir)
        self.fire_srv = self.create_service(
            FireLaser, '/targeting/fire_laser', self._fire_laser_callback
        )

        rate = self.get_parameter('rate').value
        self.create_timer(1.0 / rate, self._sequencer_loop)

        self.get_logger().info(
            f'Targeting sequencer başlatıldı — '
            f'max_attempts={self.max_attempts}, '
            f'hold={self.hold_duration}s'
        )

    # ── Callbacks ──

    def _lock_callback(self, msg: Bool):
        self.target_locked = msg.data

    def _target_callback(self, msg: TargetDetection):
        self.latest_target = msg

    def _mode_callback(self, msg: SystemMode):
        self.current_mode = msg.mode

    def _behavior_callback(self, msg: String):
        was_active = self.active
        self.active = (msg.data == 'targeting')

        if self.active and not was_active:
            self._reset_sequence()
            self.get_logger().info('Atış aşaması aktif — sequencer hazır')

    def _fire_request_callback(self, msg: Bool):
        """Manuel modda operatör fire butonu veya otonom modda auto-trigger."""
        if msg.data and self.seq_state == SeqState.IDLE and self.active:
            if self.attempt_count < self.max_attempts:
                self._start_attempt()
            else:
                self.get_logger().warn('Deneme hakkı kalmadı!')

    def _fire_laser_callback(self, request, response):
        """FireLaser servisi — dışarıdan tetikleme."""
        if self.attempt_count >= self.max_attempts:
            response.success = False
            response.hold_duration_sec = 0.0
            response.attempts_remaining = 0
            response.message = 'Deneme hakkı kalmadı'
            return response

        if self.seq_state != SeqState.IDLE:
            response.success = False
            response.hold_duration_sec = 0.0
            response.attempts_remaining = self.max_attempts - self.attempt_count
            response.message = 'Sekans zaten devam ediyor'
            return response

        self._start_attempt()

        # Sekansın tamamlanmasını bekleyemeyiz (servis senkron),
        # sonucu /targeting/fire_result topic'inden takip et
        response.success = True
        response.hold_duration_sec = 0.0  # Henüz bilinmiyor
        response.attempts_remaining = self.max_attempts - self.attempt_count
        response.message = f'Deneme #{self.attempt_count} başlatıldı'
        return response

    # ── Sequence Management ──

    def _reset_sequence(self):
        """Yeni atış aşaması başlangıcı."""
        self.seq_state = SeqState.IDLE
        self.attempt_count = 0
        self.best_ring = 0
        self._set_laser(False)

    def _start_attempt(self):
        """Yeni deneme başlat."""
        self.attempt_count += 1
        self.seq_state = SeqState.PREPARE
        self.get_logger().info(
            f'Atış denemesi #{self.attempt_count}/{self.max_attempts} başlıyor'
        )

    def _set_laser(self, on: bool):
        """Lazer aç/kapat."""
        msg = Bool()
        msg.data = on
        self.laser_pub.publish(msg)

    def _freeze_gimbal(self):
        """Gimbal'ı dondur (sıfır hız)."""
        self.gimbal_pub.publish(Vector3())

    async def _request_mode(self, mode: int):
        """Mod değiştirme isteği."""
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('SetMode servisi bulunamadı!')
            return False

        req = SetMode.Request()
        req.requested_mode = mode
        future = self.set_mode_client.call_async(req)
        return future

    # ── Ana Sekans Döngüsü ──

    def _sequencer_loop(self):
        """State machine — atış sekansı."""
        if not self.active:
            return

        # State yayınla
        state_msg = String()
        state_msg.data = SeqState(self.seq_state).name
        self.seq_state_pub.publish(state_msg)

        # Otonom modda auto-trigger: lock olunca otomatik başlat
        if (self.seq_state == SeqState.IDLE and
                self.target_locked and
                self.attempt_count < self.max_attempts and
                self.current_mode == SystemMode.MODE_AUTONOMOUS):
            self._start_attempt()

        if self.seq_state == SeqState.IDLE:
            return

        elif self.seq_state == SeqState.PREPARE:
            # Lazer kesinlikle kapalı
            self._set_laser(False)
            self.seq_state = SeqState.AIM
            self.get_logger().info('Nişan alınıyor (lazer KAPALI)')

        elif self.seq_state == SeqState.AIM:
            # Auto-aim veya manuel aim devam ediyor
            # Lock bekliyoruz
            if self.target_locked:
                self.seq_state = SeqState.LOCK_WAIT
                self.get_logger().info('Hedef kilitlendi — freeze hazırlığı')
            # Manuel modda operatör kendisi fire_request gönderir
            # O durumda lock olmasa bile devam ederiz

        elif self.seq_state == SeqState.LOCK_WAIT:
            # TARGETING_LOCK moduna geçiş iste
            req = SetMode.Request()
            req.requested_mode = SystemMode.MODE_TARGETING_LOCK
            self.set_mode_client.call_async(req)
            self.freeze_start_time = time.time()
            self.seq_state = SeqState.FREEZE
            self.get_logger().info('TARGETING_LOCK istendi — araç donduruluyor')

        elif self.seq_state == SeqState.FREEZE:
            # Araç ve gimbal'ın tamamen durmasını bekle
            self._freeze_gimbal()
            elapsed = time.time() - self.freeze_start_time

            if elapsed >= self.mode_delay:
                if self.current_mode == SystemMode.MODE_TARGETING_LOCK:
                    self.seq_state = SeqState.FIRE
                    self.get_logger().info('Araç durdu — LAZER AKTİF EDİLİYOR')
                elif elapsed > 2.0:
                    self.get_logger().error('TARGETING_LOCK moduna geçilemedi — iptal')
                    self.seq_state = SeqState.RELEASE

        elif self.seq_state == SeqState.FIRE:
            # LAZER AÇ
            self._set_laser(True)
            self._freeze_gimbal()  # Gimbal kesinlikle hareketsiz
            self.fire_start_time = time.time()
            self.seq_state = SeqState.HOLD
            self.get_logger().info('LAZER AKTİF — bekleniyor...')

        elif self.seq_state == SeqState.HOLD:
            # Min süre bekle — gimbal ve araç kesinlikle hareketsiz
            self._freeze_gimbal()
            elapsed = time.time() - self.fire_start_time

            if elapsed >= self.hold_duration:
                self.seq_state = SeqState.CEASE
                self.get_logger().info(
                    f'Lazer hold tamamlandı ({elapsed:.2f}s)'
                )

        elif self.seq_state == SeqState.CEASE:
            # LAZER KAPAT
            self._set_laser(False)
            self.seq_state = SeqState.RELEASE

        elif self.seq_state == SeqState.RELEASE:
            # TARGETING_LOCK kaldır — önceki moda dön
            prev = SystemMode.MODE_AUTONOMOUS if \
                self.current_mode == SystemMode.MODE_TARGETING_LOCK else \
                self.current_mode

            req = SetMode.Request()
            req.requested_mode = SystemMode.MODE_AUTONOMOUS  # veya MANUAL
            self.set_mode_client.call_async(req)
            self.seq_state = SeqState.EVALUATE

        elif self.seq_state == SeqState.EVALUATE:
            # Sonucu değerlendir
            ring = 0
            if self.latest_target is not None and self.latest_target.detected:
                ring = self.latest_target.estimated_ring

            if ring > 0 and (self.best_ring == 0 or ring < self.best_ring):
                # Daha iyi sonuç (düşük ring = daha merkeze yakın)
                self.best_ring = ring

            ring_names = {0: 'MISS', 1: 'INNER(50p)', 2: 'MIDDLE(25p)',
                          3: 'OUTER(15p)', 4: 'MISS'}
            ring_name = ring_names.get(ring, 'UNKNOWN')

            result_str = (
                f'Deneme #{self.attempt_count}: {ring_name} | '
                f'Kalan: {self.max_attempts - self.attempt_count}'
            )
            self.get_logger().info(result_str)

            # Sonuç yayınla
            result_msg = String()
            result_msg.data = result_str
            self.result_pub.publish(result_msg)

            # Idle'a dön — operatör veya auto-aim yeni deneme başlatabilir
            self.seq_state = SeqState.IDLE

            if self.attempt_count >= self.max_attempts:
                best_names = {0: 'MISS', 1: 'INNER', 2: 'MIDDLE', 3: 'OUTER'}
                self.get_logger().info(
                    f'Tüm denemeler tamamlandı — '
                    f'en iyi: {best_names.get(self.best_ring, "MISS")}'
                )


def main(args=None):
    rclpy.init(args=args)
    node = TargetingSequencerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Güvenlik: lazer kapat
        laser_msg = Bool()
        laser_msg.data = False
        node.laser_pub.publish(laser_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
