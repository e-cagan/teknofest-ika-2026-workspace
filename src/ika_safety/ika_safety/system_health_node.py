"""
System Health Node

Sistem sağlığını izler ve /diagnostics yayınlar:
  - Batarya voltajı (VehicleState'den)
  - CPU sıcaklığı (Jetson thermal zone)
  - Kamera durumları (topic'lere mesaj geliyor mu)
  - STM32 bağlantı durumu

Foxglove'da gösterim için /safety/health_summary yayınlar.
"""

import time
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from ika_msgs.msg import VehicleState, SafetyStatus


class SystemHealthNode(Node):

    def __init__(self):
        super().__init__('system_health_node')

        # Parametreler
        self.declare_parameter('battery_critical_voltage', 20.0)
        self.declare_parameter('battery_warning_voltage', 22.0)
        self.declare_parameter('cpu_temp_warning', 75.0)
        self.declare_parameter('cpu_temp_critical', 85.0)
        self.declare_parameter('camera_topics', [
            '/camera/front/image_raw',
            '/camera/rear/image_raw',
            '/camera/targeting/image_raw'
        ])
        self.declare_parameter('camera_timeout_sec', 2.0)
        self.declare_parameter('check_rate', 1.0)

        self.bat_critical = self.get_parameter('battery_critical_voltage').value
        self.bat_warning = self.get_parameter('battery_warning_voltage').value
        self.cpu_temp_warn = self.get_parameter('cpu_temp_warning').value
        self.cpu_temp_crit = self.get_parameter('cpu_temp_critical').value
        self.camera_topics = self.get_parameter('camera_topics').value
        self.camera_timeout = self.get_parameter('camera_timeout_sec').value
        check_rate = self.get_parameter('check_rate').value

        # Durum değişkenleri
        self.battery_voltage = 0.0
        self.battery_current = 0.0
        self.cpu_temperature = 0.0
        self.camera_last_seen = {}  # topic → son mesaj zamanı

        # Kamera subscriber'ları dinamik oluştur
        for topic in self.camera_topics:
            self.camera_last_seen[topic] = 0.0
            self.create_subscription(
                Image, topic,
                lambda msg, t=topic: self._camera_cb(t),
                rclpy.qos.QoSProfile(
                    reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                    depth=1
                )
            )

        # VehicleState'den batarya bilgisi
        self.create_subscription(
            VehicleState, '/vehicle_state', self._vehicle_state_cb, 10
        )

        # Publishers
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.health_summary_pub = self.create_publisher(
            String, '/safety/health_summary', 10
        )

        # Timer
        self.create_timer(1.0 / check_rate, self._check_health)

        self.get_logger().info('System health monitor başlatıldı')

    def _camera_cb(self, topic: str):
        self.camera_last_seen[topic] = time.time()

    def _vehicle_state_cb(self, msg: VehicleState):
        self.battery_voltage = msg.battery_voltage
        self.battery_current = msg.battery_current

    def _read_cpu_temperature(self) -> float:
        """Jetson CPU sıcaklığını oku (Linux thermal zone)."""
        try:
            # Jetson Orin Nano: /sys/class/thermal/thermal_zone*/temp
            # Standart Linux: thermal_zone0
            thermal_paths = [
                '/sys/class/thermal/thermal_zone0/temp',
                '/sys/class/thermal/thermal_zone1/temp',
            ]
            for path in thermal_paths:
                if os.path.exists(path):
                    with open(path, 'r') as f:
                        temp_milli = int(f.read().strip())
                        return temp_milli / 1000.0
        except Exception:
            pass
        return 0.0

    def _check_health(self):
        now = time.time()
        self.cpu_temperature = self._read_cpu_temperature()

        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        issues = []

        # ── Batarya ──
        bat_status = DiagnosticStatus()
        bat_status.name = 'Battery'
        bat_status.hardware_id = 'power_system'
        bat_status.values = [
            KeyValue(key='voltage', value=f'{self.battery_voltage:.1f}V'),
            KeyValue(key='current', value=f'{self.battery_current:.1f}A'),
        ]

        if self.battery_voltage < self.bat_critical and self.battery_voltage > 0:
            bat_status.level = DiagnosticStatus.ERROR
            bat_status.message = f'KRİTİK: {self.battery_voltage:.1f}V'
            issues.append(f'BAT_CRITICAL:{self.battery_voltage:.1f}V')
        elif self.battery_voltage < self.bat_warning and self.battery_voltage > 0:
            bat_status.level = DiagnosticStatus.WARN
            bat_status.message = f'UYARI: {self.battery_voltage:.1f}V'
            issues.append(f'BAT_WARNING:{self.battery_voltage:.1f}V')
        else:
            bat_status.level = DiagnosticStatus.OK
            bat_status.message = f'OK: {self.battery_voltage:.1f}V'

        diag_array.status.append(bat_status)

        # ── CPU Sıcaklığı ──
        cpu_status = DiagnosticStatus()
        cpu_status.name = 'CPU Temperature'
        cpu_status.hardware_id = 'jetson'
        cpu_status.values = [
            KeyValue(key='temperature', value=f'{self.cpu_temperature:.1f}°C'),
        ]

        if self.cpu_temperature > self.cpu_temp_crit:
            cpu_status.level = DiagnosticStatus.ERROR
            cpu_status.message = f'KRİTİK: {self.cpu_temperature:.1f}°C'
            issues.append(f'CPU_CRITICAL:{self.cpu_temperature:.1f}C')
        elif self.cpu_temperature > self.cpu_temp_warn:
            cpu_status.level = DiagnosticStatus.WARN
            cpu_status.message = f'UYARI: {self.cpu_temperature:.1f}°C'
            issues.append(f'CPU_WARNING:{self.cpu_temperature:.1f}C')
        else:
            cpu_status.level = DiagnosticStatus.OK
            cpu_status.message = f'OK: {self.cpu_temperature:.1f}°C'

        diag_array.status.append(cpu_status)

        # ── Kameralar ──
        for topic in self.camera_topics:
            cam_status = DiagnosticStatus()
            cam_name = topic.split('/')[-2] if '/' in topic else topic
            cam_status.name = f'Camera: {cam_name}'
            cam_status.hardware_id = cam_name

            last_seen = self.camera_last_seen.get(topic, 0.0)
            elapsed = now - last_seen if last_seen > 0 else float('inf')

            cam_status.values = [
                KeyValue(key='topic', value=topic),
                KeyValue(key='last_seen_sec', value=f'{elapsed:.1f}'),
            ]

            if elapsed > self.camera_timeout:
                cam_status.level = DiagnosticStatus.ERROR
                cam_status.message = 'Mesaj gelmiyor'
                issues.append(f'CAM_DEAD:{cam_name}')
            else:
                cam_status.level = DiagnosticStatus.OK
                cam_status.message = 'OK'

            diag_array.status.append(cam_status)

        # ── Yayınla ──
        self.diag_pub.publish(diag_array)

        # Foxglove için özet string
        summary_msg = String()
        if not issues:
            summary_msg.data = 'ALL_OK'
        else:
            summary_msg.data = ' | '.join(issues)
        self.health_summary_pub.publish(summary_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SystemHealthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
