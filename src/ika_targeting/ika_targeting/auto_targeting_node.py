"""
Otonom Nişan Alma Node'u (Auto-Aim)

Algı: target_detector_node → /perception/target (TargetDetection)
Çıkış: /targeting/gimbal_cmd (Vector3) — gimbal pan/tilt hız komutu

Yöntem:
  1. Hedef görünüyor mu? → Hayır: arama modu (sweep)
  2. Hedef görünüyor → PID ile gimbal'ı hedefe yönlendir
  3. error_x ve error_y threshold altına düşünce → "kilitli" say
  4. Ardışık N frame kilitli → /targeting/lock_status = True yayınla
  5. Sequencer lock'u görünce atış sekansını başlatır

Durum makinesi:
  SEARCHING  → hedef yok, gimbal sweep
  TRACKING   → hedef var, PID aktif, henüz kilitlenmedi
  LOCKED     → hedef kilitli, atışa hazır
  LOST       → hedef kayboldu (tracking'den → aramaya dön)
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, String

from ika_msgs.msg import TargetDetection

STATE_SEARCHING = 'SEARCHING'
STATE_TRACKING = 'TRACKING'
STATE_LOCKED = 'LOCKED'
STATE_LOST = 'LOST'
STATE_IDLE = 'IDLE'


class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.first = True

    def compute(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0

        p = self.kp * error
        self.integral += error * dt
        if self.output_limit:
            self.integral = max(-self.output_limit, min(self.output_limit, self.integral))
        i = self.ki * self.integral

        if self.first:
            d = 0.0
            self.first = False
        else:
            d = self.kd * (error - self.prev_error) / dt
        self.prev_error = error

        output = p + i + d
        if self.output_limit:
            output = max(-self.output_limit, min(self.output_limit, output))
        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.first = True


class AutoTargetingNode(Node):

    def __init__(self):
        super().__init__('auto_targeting_node')

        # Parametreler
        self.declare_parameter('target_topic', '/perception/target')
        self.declare_parameter('gimbal_cmd_topic', '/targeting/gimbal_cmd')
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('pan_kp', 0.8)
        self.declare_parameter('pan_ki', 0.05)
        self.declare_parameter('pan_kd', 0.15)
        self.declare_parameter('tilt_kp', 0.8)
        self.declare_parameter('tilt_ki', 0.05)
        self.declare_parameter('tilt_kd', 0.15)
        self.declare_parameter('lock_error_threshold', 0.03)
        self.declare_parameter('lock_stable_count', 10)
        self.declare_parameter('lock_timeout_sec', 15.0)
        self.declare_parameter('gimbal_pan_min_rad', -0.785)
        self.declare_parameter('gimbal_pan_max_rad', 0.785)
        self.declare_parameter('gimbal_tilt_min_rad', -0.524)
        self.declare_parameter('gimbal_tilt_max_rad', 0.524)
        self.declare_parameter('search_enabled', True)
        self.declare_parameter('search_pan_speed', 0.2)
        self.declare_parameter('search_tilt_speed', 0.1)

        self.lock_thresh = self.get_parameter('lock_error_threshold').value
        self.lock_count_req = self.get_parameter('lock_stable_count').value
        self.lock_timeout = self.get_parameter('lock_timeout_sec').value
        self.pan_min = self.get_parameter('gimbal_pan_min_rad').value
        self.pan_max = self.get_parameter('gimbal_pan_max_rad').value
        self.tilt_min = self.get_parameter('gimbal_tilt_min_rad').value
        self.tilt_max = self.get_parameter('gimbal_tilt_max_rad').value
        self.search_enabled = self.get_parameter('search_enabled').value
        self.search_pan_speed = self.get_parameter('search_pan_speed').value
        self.search_tilt_speed = self.get_parameter('search_tilt_speed').value

        # PID
        self.pan_pid = PIDController(
            kp=self.get_parameter('pan_kp').value,
            ki=self.get_parameter('pan_ki').value,
            kd=self.get_parameter('pan_kd').value,
            output_limit=1.0,
        )
        self.tilt_pid = PIDController(
            kp=self.get_parameter('tilt_kp').value,
            ki=self.get_parameter('tilt_ki').value,
            kd=self.get_parameter('tilt_kd').value,
            output_limit=1.0,
        )

        # ── Durum ──
        self.state = STATE_IDLE
        self.enabled = False
        self.latest_target = None
        self.lock_count = 0
        self.last_time = time.time()
        self.tracking_start_time = None

        # Arama modu
        self.search_direction = 1.0  # Sweep yönü
        self.current_search_pan = 0.0

        # ── ROS ──
        self.create_subscription(
            TargetDetection,
            self.get_parameter('target_topic').value,
            self._target_callback, 10
        )
        self.create_subscription(
            String, '/autonomy/active_behavior',
            self._behavior_callback, 10
        )

        self.gimbal_pub = self.create_publisher(
            Vector3,
            self.get_parameter('gimbal_cmd_topic').value,
            10
        )
        self.lock_pub = self.create_publisher(Bool, '/targeting/lock_status', 10)
        self.state_pub = self.create_publisher(String, '/targeting/aim_state', 10)

        rate = self.get_parameter('rate').value
        self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info('Auto targeting node başlatıldı')

    def _behavior_callback(self, msg: String):
        """targeting davranışı aktif mi?"""
        was_enabled = self.enabled
        self.enabled = (msg.data == 'targeting')

        if self.enabled and not was_enabled:
            self._start_aiming()
        elif not self.enabled and was_enabled:
            self._stop_aiming()

    def _start_aiming(self):
        """Nişan almaya başla."""
        self.state = STATE_SEARCHING
        self.lock_count = 0
        self.tracking_start_time = time.time()
        self.pan_pid.reset()
        self.tilt_pid.reset()
        self.current_search_pan = 0.0
        self.get_logger().info('Otonom nişan başladı — hedef aranıyor')

    def _stop_aiming(self):
        """Nişan almayı durdur."""
        self.state = STATE_IDLE
        self.lock_count = 0
        # Gimbal'ı durdur
        self.gimbal_pub.publish(Vector3())

    def _target_callback(self, msg: TargetDetection):
        self.latest_target = msg

    def _control_loop(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        if not self.enabled or self.state == STATE_IDLE:
            return

        gimbal_cmd = Vector3()
        target = self.latest_target

        has_target = target is not None and target.detected

        # ── State geçişleri ──
        if self.state == STATE_SEARCHING:
            if has_target:
                self.state = STATE_TRACKING
                self.tracking_start_time = time.time()
                self.get_logger().info('Hedef bulundu — takibe geçiliyor')
            else:
                # Arama sweep
                gimbal_cmd = self._search_sweep(dt)

        elif self.state == STATE_TRACKING:
            if not has_target:
                self.state = STATE_LOST
                self.lock_count = 0
                self.get_logger().warn('Hedef kayboldu')
            else:
                # PID ile takip
                gimbal_cmd = self._track_target(target, dt)

                # Kilitlenme kontrolü
                error_mag = math.sqrt(target.error_x**2 + target.error_y**2)
                if error_mag < self.lock_thresh:
                    self.lock_count += 1
                else:
                    self.lock_count = max(0, self.lock_count - 2)

                if self.lock_count >= self.lock_count_req:
                    self.state = STATE_LOCKED
                    self.get_logger().info(
                        f'HEDEF KİLİTLİ — error={error_mag:.4f}, '
                        f'ring={target.estimated_ring}'
                    )

            # Timeout
            if time.time() - self.tracking_start_time > self.lock_timeout:
                self.get_logger().warn('Nişan timeout — kilitlenemedi')
                self.state = STATE_LOST

        elif self.state == STATE_LOCKED:
            if has_target:
                # Kilitli pozisyonu koru — çok düşük gain PID
                gimbal_cmd = self._maintain_lock(target, dt)

                # Lock kaybı kontrolü
                error_mag = math.sqrt(target.error_x**2 + target.error_y**2)
                if error_mag > self.lock_thresh * 3:
                    self.lock_count = 0
                    self.state = STATE_TRACKING
                    self.get_logger().warn('Lock kaybedildi — takibe dönülüyor')
            else:
                self.lock_count = 0
                self.state = STATE_LOST

        elif self.state == STATE_LOST:
            if has_target:
                self.state = STATE_TRACKING
                self.tracking_start_time = time.time()
            elif self.search_enabled:
                self.state = STATE_SEARCHING

        # Gimbal komutu gönder
        self.gimbal_pub.publish(gimbal_cmd)

        # Lock durumu yayınla
        lock_msg = Bool()
        lock_msg.data = (self.state == STATE_LOCKED)
        self.lock_pub.publish(lock_msg)

        # State yayınla
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

    def _search_sweep(self, dt: float) -> Vector3:
        """Hedef aranırken gimbal sweep hareketi."""
        cmd = Vector3()

        self.current_search_pan += self.search_direction * self.search_pan_speed * dt

        if self.current_search_pan > self.pan_max * 0.8:
            self.search_direction = -1.0
        elif self.current_search_pan < self.pan_min * 0.8:
            self.search_direction = 1.0

        cmd.x = self.search_direction * self.search_pan_speed
        cmd.y = 0.0
        return cmd

    def _track_target(self, target: TargetDetection, dt: float) -> Vector3:
        """PID ile hedefi takip et."""
        cmd = Vector3()

        # error_x: pozitif = hedef sağda → gimbal sağa dön (negatif pan)
        # error_y: pozitif = hedef aşağıda → gimbal aşağı (negatif tilt)
        pan_output = -self.pan_pid.compute(target.error_x, dt)
        tilt_output = -self.tilt_pid.compute(target.error_y, dt)

        cmd.x = pan_output
        cmd.y = tilt_output
        return cmd

    def _maintain_lock(self, target: TargetDetection, dt: float) -> Vector3:
        """Kilitli pozisyonu koru — düşük gain."""
        cmd = Vector3()
        cmd.x = -target.error_x * 0.3  # Düşük gain — salınım olmasın
        cmd.y = -target.error_y * 0.3
        return cmd


def main(args=None):
    rclpy.init(args=args)
    node = AutoTargetingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
