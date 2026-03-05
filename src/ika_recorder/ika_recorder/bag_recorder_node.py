"""
ROS2 Bag Recorder Node

Tüm önemli topic'leri rosbag2 ile kaydeder.
Video recorder'dan farkı: rosbag hem replay hem analiz için kullanılır.
Foxglove'da playback yapılabilir. Hakem heyetine video teslim edilirken
rosbag da debug/analiz için çok değerli.

Kontrol:
  /recorder/bag/start (Bool) → kaydı başlat
  /recorder/bag/stop  (Bool) → kaydı durdur
  SystemMode run_active → otomatik başlat/durdur

Not: Bu node subprocess ile `ros2 bag record` çalıştırır.
rosbag2_py ile programatik kayıt da yapılabilir ama subprocess
daha stabil ve tüm serialization'ı handle eder.
"""

import os
import signal
import subprocess
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from ika_msgs.msg import SystemMode


class BagRecorderNode(Node):

    def __init__(self):
        super().__init__('bag_recorder_node')

        # Parametreler
        self.declare_parameter('output_dir', '/home/cagan/ika_recordings')
        self.declare_parameter('topics', [])
        self.declare_parameter('auto_start', False)
        self.declare_parameter('start_on_run', True)
        self.declare_parameter('storage_id', 'sqlite3')

        self.output_dir = self.get_parameter('output_dir').value
        self.topics = self.get_parameter('topics').value
        self.auto_start = self.get_parameter('auto_start').value
        self.start_on_run = self.get_parameter('start_on_run').value
        self.storage_id = self.get_parameter('storage_id').value

        # Durum
        self.recording = False
        self.bag_process = None

        # Dizin oluştur
        os.makedirs(self.output_dir, exist_ok=True)

        # Kontrol
        self.create_subscription(
            Bool, '/recorder/bag/start', self._start_callback, 10
        )
        self.create_subscription(
            Bool, '/recorder/bag/stop', self._stop_callback, 10
        )
        self.create_subscription(
            SystemMode, '/system/mode', self._mode_callback, 10
        )

        self.status_pub = self.create_publisher(
            String, '/recorder/bag/status', 10
        )
        self.create_timer(2.0, self._publish_status)

        if self.auto_start:
            self._start_recording()

        self.get_logger().info(
            f'Bag recorder başlatıldı — '
            f'{len(self.topics)} topic, '
            f'çıktı: {self.output_dir}'
        )

    def _start_recording(self):
        """rosbag2 kaydını subprocess olarak başlat."""
        if self.recording:
            self.get_logger().warn('Bag kaydı zaten devam ediyor')
            return

        if not self.topics:
            self.get_logger().error('Kaydedilecek topic listesi boş!')
            return

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_name = f'ika_run_{timestamp}'
        bag_path = os.path.join(self.output_dir, bag_name)

        # ros2 bag record komutu
        cmd = [
            'ros2', 'bag', 'record',
            '-o', bag_path,
            '-s', self.storage_id,
        ] + self.topics

        try:
            self.bag_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,  # Process group oluştur (temiz kill için)
            )
            self.recording = True
            self.get_logger().info(f'Bag kaydı başladı: {bag_path}')
        except Exception as e:
            self.get_logger().error(f'Bag kaydı başlatılamadı: {e}')

    def _stop_recording(self):
        """rosbag2 kaydını durdur."""
        if not self.recording or self.bag_process is None:
            return

        try:
            # Process group'a SIGINT gönder (rosbag graceful shutdown)
            os.killpg(os.getpgid(self.bag_process.pid), signal.SIGINT)
            self.bag_process.wait(timeout=10)
            self.get_logger().info('Bag kaydı durduruldu')
        except subprocess.TimeoutExpired:
            self.get_logger().warn('Bag process timeout — zorla kapatılıyor')
            os.killpg(os.getpgid(self.bag_process.pid), signal.SIGKILL)
        except Exception as e:
            self.get_logger().error(f'Bag durdurma hatası: {e}')
        finally:
            self.bag_process = None
            self.recording = False

    def _start_callback(self, msg: Bool):
        if msg.data:
            self._start_recording()

    def _stop_callback(self, msg: Bool):
        if msg.data:
            self._stop_recording()

    def _mode_callback(self, msg: SystemMode):
        if self.start_on_run:
            if msg.run_active and not self.recording:
                self._start_recording()
            elif not msg.run_active and self.recording:
                self._stop_recording()

    def _publish_status(self):
        status = String()
        if self.recording and self.bag_process is not None:
            # Process hala çalışıyor mu kontrol et
            if self.bag_process.poll() is not None:
                # Process bitti — beklenmedik kapanma
                stderr = self.bag_process.stderr.read().decode() if self.bag_process.stderr else ''
                self.get_logger().error(f'Bag process beklenmedik kapandı: {stderr}')
                self.recording = False
                self.bag_process = None
                status.data = 'ERROR'
            else:
                status.data = 'RECORDING'
        else:
            status.data = 'IDLE'
        self.status_pub.publish(status)

    def destroy_node(self):
        self._stop_recording()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BagRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
