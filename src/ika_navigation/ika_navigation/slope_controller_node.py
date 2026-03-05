"""
Eğim Kontrolü Node'u

Algı: IMU → /imu/data (sensor_msgs/Imu)
Çıkış: Eğim durumu + stop noktası yönetimi

Şartname:
  - %45 dik eğim (çıkış + iniş), stop noktalarında en az 2s dur
  - Durmazsan dik eğim puanı 0
  - %20 yan eğim — stabil geçiş

Davranış:
  FLAT         → normal sürüş
  CLIMBING     → tırmanma (yavaş)
  STOP_HOLD    → stop noktasında durma (2s+)
  DESCENDING   → iniş (çok yavaş, frenli)
  LATERAL      → yan eğim (yavaş, dikkatli)
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import String

STATE_FLAT = 'FLAT'
STATE_CLIMBING = 'CLIMBING'
STATE_STOP_HOLD = 'STOP_HOLD'
STATE_DESCENDING = 'DESCENDING'
STATE_LATERAL = 'LATERAL'


class SlopeControllerNode(Node):

    def __init__(self):
        super().__init__('slope_controller_node')

        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('steep_pitch_threshold_deg', 20.0)
        self.declare_parameter('steep_pitch_max_deg', 30.0)
        self.declare_parameter('lateral_roll_threshold_deg', 8.0)
        self.declare_parameter('lateral_roll_max_deg', 15.0)
        self.declare_parameter('stop_hold_duration_sec', 2.5)
        self.declare_parameter('climb_speed', 0.3)
        self.declare_parameter('descent_speed', 0.2)
        self.declare_parameter('lateral_slope_speed', 0.3)

        self.steep_thresh = math.radians(self.get_parameter('steep_pitch_threshold_deg').value)
        self.steep_max = math.radians(self.get_parameter('steep_pitch_max_deg').value)
        self.lateral_thresh = math.radians(self.get_parameter('lateral_roll_threshold_deg').value)
        self.lateral_max = math.radians(self.get_parameter('lateral_roll_max_deg').value)
        self.stop_hold_dur = self.get_parameter('stop_hold_duration_sec').value
        self.climb_speed = self.get_parameter('climb_speed').value
        self.descent_speed = self.get_parameter('descent_speed').value
        self.lateral_speed = self.get_parameter('lateral_slope_speed').value

        # Durum
        self.state = STATE_FLAT
        self.current_pitch = 0.0
        self.current_roll = 0.0
        self.stop_start_time = None
        self.stop_completed = False
        self.climb_stop_done = False
        self.descent_stop_done = False

        # ROS
        self.create_subscription(Imu, self.get_parameter('imu_topic').value,
                                 self._imu_callback, 10)
        self.cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10
        )
        self.state_pub = self.create_publisher(String, '/navigation/slope_state', 10)

        rate = self.get_parameter('rate').value
        self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info('Eğim kontrol node başlatıldı')

    def _quaternion_to_euler(self, q) -> tuple:
        """Quaternion → (roll, pitch, yaw) Euler açıları."""
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return (roll, pitch, yaw)

    def _imu_callback(self, msg: Imu):
        roll, pitch, _ = self._quaternion_to_euler(msg.orientation)
        self.current_pitch = pitch
        self.current_roll = roll

    def _control_loop(self):
        cmd = Twist()
        abs_pitch = abs(self.current_pitch)
        abs_roll = abs(self.current_roll)

        prev_state = self.state

        # ── State geçişleri ──
        if self.state == STATE_FLAT:
            if abs_pitch > self.steep_thresh:
                if self.current_pitch > 0:
                    self.state = STATE_CLIMBING
                else:
                    self.state = STATE_DESCENDING
            elif abs_roll > self.lateral_thresh:
                self.state = STATE_LATERAL

        elif self.state == STATE_CLIMBING:
            if abs_pitch < self.steep_thresh * 0.5:
                # Rampa tepe noktası / düzlük — stop noktası
                if not self.climb_stop_done:
                    self.state = STATE_STOP_HOLD
                    self.stop_start_time = time.time()
                    self.get_logger().info('Tırmanma stop noktası — 2s bekleme başladı')
                else:
                    self.state = STATE_FLAT

        elif self.state == STATE_DESCENDING:
            if abs_pitch < self.steep_thresh * 0.5:
                # İniş bitişi — stop noktası
                if not self.descent_stop_done:
                    self.state = STATE_STOP_HOLD
                    self.stop_start_time = time.time()
                    self.get_logger().info('İniş stop noktası — 2s bekleme başladı')
                else:
                    self.state = STATE_FLAT

        elif self.state == STATE_STOP_HOLD:
            elapsed = time.time() - self.stop_start_time
            if elapsed >= self.stop_hold_dur:
                self.stop_completed = True
                # Hangi stop'u tamamladık?
                if not self.climb_stop_done:
                    self.climb_stop_done = True
                    self.get_logger().info(f'Tırmanma stop tamamlandı ({elapsed:.1f}s)')
                else:
                    self.descent_stop_done = True
                    self.get_logger().info(f'İniş stop tamamlandı ({elapsed:.1f}s)')
                self.state = STATE_FLAT

        elif self.state == STATE_LATERAL:
            if abs_roll < self.lateral_thresh * 0.5:
                self.state = STATE_FLAT

        # ── Hız kararı ──
        if self.state == STATE_FLAT:
            cmd.linear.x = 0.0  # Path follower devralır

        elif self.state == STATE_CLIMBING:
            cmd.linear.x = self.climb_speed

        elif self.state == STATE_DESCENDING:
            cmd.linear.x = self.descent_speed

        elif self.state == STATE_STOP_HOLD:
            cmd.linear.x = 0.0  # Tamamen dur
            cmd.angular.z = 0.0

        elif self.state == STATE_LATERAL:
            cmd.linear.x = self.lateral_speed

        # Güvenlik — aşırı eğim
        if abs_pitch > self.steep_max or abs_roll > self.lateral_max:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().warn(
                f'Aşırı eğim! pitch={math.degrees(self.current_pitch):.1f}° '
                f'roll={math.degrees(self.current_roll):.1f}° — DURDURULDU'
            )

        # State yayınla
        if self.state != prev_state:
            state_msg = String()
            state_msg.data = self.state
            self.state_pub.publish(state_msg)

        # cmd_vel yayınla (sadece eğim aktifken)
        if self.state != STATE_FLAT:
            self.cmd_vel_pub.publish(cmd)

    def reset(self):
        """Yeni koşu başında çağrılır."""
        self.state = STATE_FLAT
        self.climb_stop_done = False
        self.descent_stop_done = False
        self.stop_completed = False


def main(args=None):
    rclpy.init(args=args)
    node = SlopeControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
