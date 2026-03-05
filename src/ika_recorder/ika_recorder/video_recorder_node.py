"""
Video Recorder Node — 3 Kamera MP4/AVI Kaydı

Şartname: Yarışmacılar parkur boyunca araç sürüş (ileri ve geri)
kameralarından ve nişan kamerasından aldıkları görüntüleri kaydedecek
ve parkur tamamlandıktan sonra hakem heyetine iletecektir.

Her kamera için ayrı video dosyası oluşturur.
Dosya adı: {timestamp}_{camera_name}.avi

Kontrol:
  /recorder/video/start (Bool) → kaydı başlat
  /recorder/video/stop  (Bool) → kaydı durdur
  SystemMode run_active → otomatik başlat/durdur
"""

import os
import time
from datetime import datetime

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge

from ika_msgs.msg import SystemMode


class CameraRecorder:
    """Tek bir kamera için video yazıcı."""

    def __init__(self, topic: str, output_path: str, fps: float, codec: str, logger):
        self.topic = topic
        self.output_path = output_path
        self.fps = fps
        self.codec = codec
        self.logger = logger

        self.writer = None
        self.frame_count = 0
        self.initialized = False

    def write_frame(self, frame: np.ndarray):
        """Frame yaz. İlk frame'de video boyutu belirlenir."""
        if not self.initialized:
            h, w = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*self.codec)
            self.writer = cv2.VideoWriter(
                self.output_path, fourcc, self.fps, (w, h)
            )
            if not self.writer.isOpened():
                self.logger.error(f'VideoWriter açılamadı: {self.output_path}')
                return
            self.initialized = True
            self.logger.info(
                f'Video kaydı başladı: {self.output_path} ({w}x{h} @ {self.fps}fps)'
            )

        self.writer.write(frame)
        self.frame_count += 1

    def release(self):
        """Video dosyasını kapat."""
        if self.writer is not None and self.writer.isOpened():
            self.writer.release()
            self.logger.info(
                f'Video kaydı tamamlandı: {self.output_path} '
                f'({self.frame_count} frame)'
            )
        self.writer = None
        self.initialized = False
        self.frame_count = 0


class VideoRecorderNode(Node):

    def __init__(self):
        super().__init__('video_recorder_node')

        # Parametreler
        self.declare_parameter('camera_topics', [
            '/camera/front/image_raw',
            '/camera/rear/image_raw',
            '/camera/targeting/image_raw',
        ])
        self.declare_parameter('output_dir', '/home/cagan/ika_recordings')
        self.declare_parameter('fps', 20.0)
        self.declare_parameter('codec', 'XVID')
        self.declare_parameter('file_extension', '.avi')
        self.declare_parameter('auto_start', False)
        self.declare_parameter('start_on_run', True)

        self.camera_topics = self.get_parameter('camera_topics').value
        self.output_dir = self.get_parameter('output_dir').value
        self.fps = self.get_parameter('fps').value
        self.codec = self.get_parameter('codec').value
        self.file_ext = self.get_parameter('file_extension').value
        self.auto_start = self.get_parameter('auto_start').value
        self.start_on_run = self.get_parameter('start_on_run').value

        # Durum
        self.recording = False
        self.bridge = CvBridge()
        self.recorders = {}  # topic → CameraRecorder

        # Dizin oluştur
        os.makedirs(self.output_dir, exist_ok=True)

        # Kamera subscriber'ları
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )

        for topic in self.camera_topics:
            self.create_subscription(
                Image, topic,
                lambda msg, t=topic: self._image_callback(t, msg),
                sensor_qos
            )

        # Kontrol subscriber'ları
        self.create_subscription(
            Bool, '/recorder/video/start', self._start_callback, 10
        )
        self.create_subscription(
            Bool, '/recorder/video/stop', self._stop_callback, 10
        )
        self.create_subscription(
            SystemMode, '/system/mode', self._mode_callback, 10
        )

        # Durum yayını
        self.status_pub = self.create_publisher(
            String, '/recorder/video/status', 10
        )
        self.create_timer(1.0, self._publish_status)

        # Otomatik başlatma
        if self.auto_start:
            self._start_recording()

        self.get_logger().info(
            f'Video recorder başlatıldı — '
            f'{len(self.camera_topics)} kamera, '
            f'çıktı: {self.output_dir}'
        )

    def _topic_to_name(self, topic: str) -> str:
        """Topic adından dosya ismi oluştur."""
        # "/camera/front/image_raw" → "front"
        parts = topic.strip('/').split('/')
        if len(parts) >= 2:
            return parts[-2]  # "front", "rear", "targeting"
        return topic.replace('/', '_').strip('_')

    def _start_recording(self):
        """Kaydı başlat — her kamera için yeni dosya oluştur."""
        if self.recording:
            self.get_logger().warn('Kayıt zaten devam ediyor')
            return

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        for topic in self.camera_topics:
            cam_name = self._topic_to_name(topic)
            filename = f'{timestamp}_{cam_name}{self.file_ext}'
            filepath = os.path.join(self.output_dir, filename)

            self.recorders[topic] = CameraRecorder(
                topic=topic,
                output_path=filepath,
                fps=self.fps,
                codec=self.codec,
                logger=self.get_logger(),
            )

        self.recording = True
        self.get_logger().info(f'Video kaydı başlatıldı — {timestamp}')

    def _stop_recording(self):
        """Kaydı durdur — tüm dosyaları kapat."""
        if not self.recording:
            return

        for topic, recorder in self.recorders.items():
            recorder.release()

        self.recorders.clear()
        self.recording = False
        self.get_logger().info('Video kaydı durduruldu')

    def _image_callback(self, topic: str, msg: Image):
        """Kamera frame'i geldiğinde kaydet."""
        if not self.recording:
            return

        if topic not in self.recorders:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.recorders[topic].write_frame(frame)
        except Exception as e:
            self.get_logger().warn(f'Frame kayıt hatası ({topic}): {e}')

    def _start_callback(self, msg: Bool):
        if msg.data:
            self._start_recording()

    def _stop_callback(self, msg: Bool):
        if msg.data:
            self._stop_recording()

    def _mode_callback(self, msg: SystemMode):
        """Koşu başladığında otomatik kayıt."""
        if self.start_on_run:
            if msg.run_active and not self.recording:
                self._start_recording()
            elif not msg.run_active and self.recording:
                self._stop_recording()

    def _publish_status(self):
        """Kayıt durumu yayınla."""
        status = String()
        if self.recording:
            total_frames = sum(r.frame_count for r in self.recorders.values())
            status.data = f'RECORDING|frames={total_frames}'
        else:
            status.data = 'IDLE'
        self.status_pub.publish(status)

    def destroy_node(self):
        self._stop_recording()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
