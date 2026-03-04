"""
STM32 UART Bridge Node — TEKNOFEST 2026 İKA

Sorumluluklar:
  - /cmd_vel → diferansiyel kinematik → UART motor komutu
  - UART enkoder verisi → odometri hesabı → /odom yayını
  - UART batarya/safety → /vehicle_state, /safety/stm32_heartbeat yayını
  - Heartbeat mekanizması (bağlantı kopma tespiti)
  - E-stop komutu gönderme

UART Protokolü (ASCII):
  TX (Jetson → STM32):
    M:<left_rpm>,<right_rpm>\n    Motor hız komutu
    E:<0|1>\n                     E-Stop (1=durdur, 0=serbest)
    B:<0|1>\n                     Fren (1=kilitle, 0=serbest)
    L:<0|1>\n                     Far (1=aç, 0=kapat)
    H\n                           Heartbeat ping

  RX (STM32 → Jetson):
    ENC:<left_ticks>,<right_ticks>\n    Enkoder verileri
    BAT:<voltage>,<current>\n           Batarya durumu
    SAF:<estop_state>,<comms_state>\n   Güvenlik durumu
    ACK:H\n                             Heartbeat ACK
    ERR:<code>\n                        Hata kodu
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Header
from sensor_msgs.msg import JointState

from ika_msgs.msg import VehicleState, SafetyStatus

import serial
from tf2_ros import TransformBroadcaster


def quaternion_from_yaw(yaw: float) -> Quaternion:
    """Yaw açısından quaternion oluştur (2D odometri için yeterli)."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class STM32BridgeNode(Node):

    def __init__(self):
        super().__init__('stm32_bridge_node')

        # ── Parametreler ──
        self._declare_params()
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.serial_timeout = self.get_parameter('serial_timeout').value

        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.ticks_per_rev = self.get_parameter('ticks_per_revolution').value

        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.max_rpm = self.get_parameter('max_rpm').value

        self.cmd_rate = self.get_parameter('cmd_rate').value
        self.hb_rate = self.get_parameter('heartbeat_rate').value
        self.hb_timeout = self.get_parameter('heartbeat_timeout').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value

        # ── Hesaplanan sabitler ──
        self.meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev

        # ── Durum değişkenleri ──
        self.target_left_rpm = 0.0
        self.target_right_rpm = 0.0

        # Odometri
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.last_odom_time = self.get_clock().now()

        # Zamanlama
        self.last_cmd_vel_time = self.get_clock().now()
        self.last_heartbeat_ack = time.time()
        self.stm32_connected = False

        # E-stop
        self.estop_active = False

        # ── Serial bağlantı ──
        self.ser = None
        self._connect_serial()

        # ── QoS ──
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # ── Subscribers ──
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10
        )
        self.estop_sub = self.create_subscription(
            Bool, '/safety/estop', self._estop_callback, 10
        )
        self.headlight_sub = self.create_subscription(
            Bool, '/vehicle/headlights', self._headlight_callback, 10
        )

        # ── Publishers ──
        self.odom_pub = self.create_publisher(Odometry, '/odom', sensor_qos)
        self.vehicle_state_pub = self.create_publisher(VehicleState, '/vehicle_state', sensor_qos)
        self.safety_pub = self.create_publisher(SafetyStatus, '/safety/status', 10)
        self.stm32_heartbeat_pub = self.create_publisher(Bool, '/safety/stm32_heartbeat', 10)

        # ── TF Broadcaster ──
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── Timerlar ──
        self.cmd_timer = self.create_timer(1.0 / self.cmd_rate, self._cmd_timer_callback)
        self.hb_timer = self.create_timer(1.0 / self.hb_rate, self._heartbeat_timer_callback)

        # ── Serial okuma thread'i ──
        self.read_thread_active = True
        self.read_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
        self.read_thread.start()

        self.get_logger().info(
            f'STM32 Bridge başlatıldı — port={self.serial_port}, baud={self.baud_rate}, '
            f'wheel_base={self.wheel_base}m, wheel_radius={self.wheel_radius}m'
        )

    # ══════════════════════════════════════════
    #  PARAMETRE TANIMLARI
    # ══════════════════════════════════════════

    def _declare_params(self):
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('serial_timeout', 0.01)
        self.declare_parameter('wheel_base', 0.5)
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('ticks_per_revolution', 1440)
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 3.0)
        self.declare_parameter('max_rpm', 300)
        self.declare_parameter('cmd_rate', 20.0)
        self.declare_parameter('heartbeat_rate', 2.0)
        self.declare_parameter('heartbeat_timeout', 2.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)

    # ══════════════════════════════════════════
    #  SERIAL BAĞLANTI
    # ══════════════════════════════════════════

    def _connect_serial(self):
        """Serial port bağlantısı kur."""
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.serial_timeout,
                write_timeout=self.serial_timeout
            )
            self.stm32_connected = True
            self.last_heartbeat_ack = time.time()
            self.get_logger().info(f'Serial bağlantı kuruldu: {self.serial_port}')
        except serial.SerialException as e:
            self.ser = None
            self.stm32_connected = False
            self.get_logger().error(f'Serial bağlantı hatası: {e}')

    def _send(self, message: str):
        """STM32'ye ASCII mesaj gönder."""
        if self.ser is None or not self.ser.is_open:
            return
        try:
            self.ser.write(f'{message}\n'.encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial yazma hatası: {e}')
            self.stm32_connected = False

    # ══════════════════════════════════════════
    #  DİFERANSİYEL KİNEMATİK
    # ══════════════════════════════════════════

    def _cmd_vel_to_wheel_speeds(self, linear_x: float, angular_z: float):
        """
        cmd_vel (linear.x, angular.z) → sol/sağ teker hızı (RPM).

        v_left  = linear_x - (angular_z * wheel_base / 2)
        v_right = linear_x + (angular_z * wheel_base / 2)

        RPM = (speed_mps / (2π * wheel_radius)) * 60
        """
        # Hız limitlerini uygula
        linear_x = max(-self.max_linear, min(self.max_linear, linear_x))
        angular_z = max(-self.max_angular, min(self.max_angular, angular_z))

        # m/s cinsinden teker hızları
        v_left = linear_x - (angular_z * self.wheel_base / 2.0)
        v_right = linear_x + (angular_z * self.wheel_base / 2.0)

        # m/s → RPM
        rpm_left = (v_left / (2.0 * math.pi * self.wheel_radius)) * 60.0
        rpm_right = (v_right / (2.0 * math.pi * self.wheel_radius)) * 60.0

        # RPM limiti
        rpm_left = max(-self.max_rpm, min(self.max_rpm, rpm_left))
        rpm_right = max(-self.max_rpm, min(self.max_rpm, rpm_right))

        return rpm_left, rpm_right

    # ══════════════════════════════════════════
    #  ODOMETRİ HESABI
    # ══════════════════════════════════════════

    def _update_odometry(self, left_ticks: int, right_ticks: int):
        """
        Enkoder tick'lerinden odometri hesapla ve /odom + TF yayınla.

        delta_s     = (delta_left + delta_right) / 2
        delta_theta = (delta_right - delta_left) / wheel_base
        x += delta_s * cos(theta + delta_theta/2)
        y += delta_s * sin(theta + delta_theta/2)
        theta += delta_theta
        """
        now = self.get_clock().now()

        # İlk okumada sadece kaydet
        if self.prev_left_ticks is None:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.last_odom_time = now
            return

        # Delta tick → delta metre
        dl = (left_ticks - self.prev_left_ticks) * self.meters_per_tick
        dr = (right_ticks - self.prev_right_ticks) * self.meters_per_tick

        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks

        # Kinematik
        delta_s = (dl + dr) / 2.0
        delta_theta = (dr - dl) / self.wheel_base

        # Mid-point integration (daha doğru)
        self.odom_x += delta_s * math.cos(self.odom_theta + delta_theta / 2.0)
        self.odom_y += delta_s * math.sin(self.odom_theta + delta_theta / 2.0)
        self.odom_theta += delta_theta

        # Theta'yı [-π, π] aralığında tut
        self.odom_theta = math.atan2(
            math.sin(self.odom_theta), math.cos(self.odom_theta)
        )

        # Hız hesabı
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now

        if dt > 0:
            linear_vel = delta_s / dt
            angular_vel = delta_theta / dt
        else:
            linear_vel = 0.0
            angular_vel = 0.0

        # ── Odometry mesajı ──
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.odom_x
        odom_msg.pose.pose.position.y = self.odom_y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = quaternion_from_yaw(self.odom_theta)

        odom_msg.twist.twist.linear.x = linear_vel
        odom_msg.twist.twist.angular.z = angular_vel

        self.odom_pub.publish(odom_msg)

        # ── TF: odom → base_link ──
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.odom_x
        t.transform.translation.y = self.odom_y
        t.transform.translation.z = 0.0
        t.transform.rotation = quaternion_from_yaw(self.odom_theta)

        self.tf_broadcaster.sendTransform(t)

    # ══════════════════════════════════════════
    #  CALLBACKS
    # ══════════════════════════════════════════

    def _cmd_vel_callback(self, msg: Twist):
        """cmd_vel mesajı geldiğinde teker hızlarını hesapla."""
        self.last_cmd_vel_time = self.get_clock().now()

        if self.estop_active:
            self.target_left_rpm = 0.0
            self.target_right_rpm = 0.0
            return

        self.target_left_rpm, self.target_right_rpm = \
            self._cmd_vel_to_wheel_speeds(msg.linear.x, msg.angular.z)

    def _estop_callback(self, msg: Bool):
        """E-stop komutu."""
        self.estop_active = msg.data
        if self.estop_active:
            self.target_left_rpm = 0.0
            self.target_right_rpm = 0.0
            self._send('E:1')
            self.get_logger().warn('E-STOP AKTİF — motorlar durduruldu')
        else:
            self._send('E:0')
            self.get_logger().info('E-STOP serbest bırakıldı')

    def _headlight_callback(self, msg: Bool):
        """Far kontrolü."""
        self._send(f'L:{1 if msg.data else 0}')

    # ══════════════════════════════════════════
    #  TIMER CALLBACKS
    # ══════════════════════════════════════════

    def _cmd_timer_callback(self):
        """
        Periyodik motor komutu gönder (cmd_rate Hz).
        cmd_vel timeout kontrolü de burada.
        """
        now = self.get_clock().now()
        dt = (now - self.last_cmd_vel_time).nanoseconds / 1e9

        # cmd_vel timeout — uzun süredir komut gelmediyse durdur
        if dt > self.cmd_vel_timeout:
            self.target_left_rpm = 0.0
            self.target_right_rpm = 0.0

        # E-stop aktifse kesinlikle sıfır
        if self.estop_active:
            self.target_left_rpm = 0.0
            self.target_right_rpm = 0.0

        # Motor komutunu gönder
        left = int(round(self.target_left_rpm))
        right = int(round(self.target_right_rpm))
        self._send(f'M:{left},{right}')

    def _heartbeat_timer_callback(self):
        """Heartbeat ping gönder ve timeout kontrol et."""
        self._send('H')

        # Timeout kontrolü
        elapsed = time.time() - self.last_heartbeat_ack
        was_connected = self.stm32_connected

        if elapsed > self.hb_timeout:
            self.stm32_connected = False
            if was_connected:
                self.get_logger().error(
                    f'STM32 heartbeat timeout ({self.hb_timeout}s) — bağlantı koptu!'
                )
                # Motorları durdur
                self.target_left_rpm = 0.0
                self.target_right_rpm = 0.0
        else:
            self.stm32_connected = True

        # Heartbeat durumu yayınla
        hb_msg = Bool()
        hb_msg.data = self.stm32_connected
        self.stm32_heartbeat_pub.publish(hb_msg)

    # ══════════════════════════════════════════
    #  SERIAL OKUMA (AYRI THREAD)
    # ══════════════════════════════════════════

    def _serial_read_loop(self):
        """
        Ayrı thread'de serial port'u sürekli oku ve parse et.

        Beklenen mesajlar:
          ENC:<left>,<right>\n
          BAT:<voltage>,<current>\n
          SAF:<estop>,<comms>\n
          ACK:H\n
          ERR:<code>\n
        """
        while self.read_thread_active:
            if self.ser is None or not self.ser.is_open:
                time.sleep(1.0)
                self._connect_serial()
                continue

            try:
                raw = self.ser.readline()
                if not raw:
                    continue

                line = raw.decode('ascii', errors='ignore').strip()
                if not line:
                    continue

                self._parse_message(line)

            except serial.SerialException as e:
                self.get_logger().warn(f'Serial okuma hatası: {e}')
                self.stm32_connected = False
                time.sleep(1.0)
            except Exception as e:
                self.get_logger().warn(f'Parse hatası: {e} — raw: {raw}')

    def _parse_message(self, line: str):
        """Gelen ASCII satırını parse et."""
        try:
            if line.startswith('ENC:'):
                parts = line[4:].split(',')
                left_ticks = int(parts[0])
                right_ticks = int(parts[1])
                self._update_odometry(left_ticks, right_ticks)

            elif line.startswith('BAT:'):
                parts = line[4:].split(',')
                voltage = float(parts[0])
                current = float(parts[1])
                self._publish_vehicle_state(voltage, current)

            elif line.startswith('SAF:'):
                parts = line[4:].split(',')
                estop_hw = int(parts[0])
                comms_state = int(parts[1])
                self._publish_safety_status(estop_hw, comms_state)

            elif line.startswith('ACK:H'):
                self.last_heartbeat_ack = time.time()
                if not self.stm32_connected:
                    self.get_logger().info('STM32 heartbeat geri geldi — bağlantı sağlandı')
                self.stm32_connected = True

            elif line.startswith('ERR:'):
                error_code = line[4:]
                self.get_logger().error(f'STM32 hata kodu: {error_code}')

            else:
                self.get_logger().debug(f'Bilinmeyen mesaj: {line}')

        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Parse hatası: {e} — line: {line}')

    # ══════════════════════════════════════════
    #  PUBLISH HELPERS
    # ══════════════════════════════════════════

    def _publish_vehicle_state(self, voltage: float, current: float):
        """VehicleState mesajı yayınla."""
        msg = VehicleState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.speed_mps = abs(self.target_left_rpm + self.target_right_rpm) / 2.0 * \
            (2.0 * math.pi * self.wheel_radius) / 60.0
        msg.heading_rad = self.odom_theta
        msg.encoder_left_ticks = self.prev_left_ticks or 0
        msg.encoder_right_ticks = self.prev_right_ticks or 0
        msg.battery_voltage = voltage
        msg.battery_current = current
        msg.headlights_on = False  # TODO: durumu takip et

        self.vehicle_state_pub.publish(msg)

    def _publish_safety_status(self, estop_hw: int, comms_state: int):
        """SafetyStatus mesajı yayınla."""
        msg = SafetyStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.estop_physical = bool(estop_hw)
        msg.estop_active = self.estop_active or bool(estop_hw)
        msg.heartbeat_jetson_ok = self.stm32_connected
        msg.heartbeat_operator_ok = True  # TODO: operator heartbeat ayrı node
        msg.battery_voltage = 0.0  # BAT mesajından gelecek
        msg.system_health = SafetyStatus.HEALTH_OK

        if not self.stm32_connected:
            msg.system_health = SafetyStatus.HEALTH_CRITICAL
        elif msg.estop_active:
            msg.system_health = SafetyStatus.HEALTH_WARNING

        self.safety_pub.publish(msg)

        # Fiziksel e-stop aktifse yazılımı da tetikle
        if estop_hw and not self.estop_active:
            self.estop_active = True
            self.target_left_rpm = 0.0
            self.target_right_rpm = 0.0
            self.get_logger().warn('Fiziksel E-STOP algılandı!')

    # ══════════════════════════════════════════
    #  CLEANUP
    # ══════════════════════════════════════════

    def destroy_node(self):
        """Node kapanırken motorları durdur ve serial'i kapat."""
        self.read_thread_active = False
        self.get_logger().info('Kapatılıyor — motorlar durduruluyor...')
        self._send('M:0,0')
        self._send('E:1')
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = STM32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()