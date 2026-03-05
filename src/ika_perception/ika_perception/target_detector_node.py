"""
Atış Hedefi Tespit Node'u

Şartname: A3 kağıt boyutunda, konsantrik daireler (6/12/18cm çap),
minimum 10m mesafe, çerçeveye oturtulmuş.
Nişan kamerası ile tespit.

Pipeline:
  1. Nişan kamerası frame
  2. Siyah-kırmızı renk segmentasyonu
  3. HoughCircles → konsantrik daire tespiti
  4. En iç dairenin merkez pikseli → hata hesabı
  5. Tahmini isabet bölgesi (inner/middle/outer)
  6. /perception/target yayınla → auto_targeting_node gimbal PID için kullanır
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ika_msgs.msg import TargetDetection


class TargetDetectorNode(Node):

    def __init__(self):
        super().__init__('target_detector_node')

        # Parametreler
        self.declare_parameter('camera_topic', '/camera/targeting/image_raw')
        self.declare_parameter('publish_topic', '/perception/target')
        self.declare_parameter('rate', 15.0)

        # Siyah alan HSV
        self.declare_parameter('black_lower_h', 0)
        self.declare_parameter('black_lower_s', 0)
        self.declare_parameter('black_lower_v', 0)
        self.declare_parameter('black_upper_h', 180)
        self.declare_parameter('black_upper_s', 80)
        self.declare_parameter('black_upper_v', 60)

        # Kırmızı alan HSV
        for prefix in ['red_lower1', 'red_upper1', 'red_lower2', 'red_upper2']:
            for suffix in ['h', 's', 'v']:
                self.declare_parameter(f'{prefix}_{suffix}', 0)

        # Hough
        self.declare_parameter('hough_dp', 1.5)
        self.declare_parameter('hough_min_dist', 20)
        self.declare_parameter('hough_param1', 80)
        self.declare_parameter('hough_param2', 30)
        self.declare_parameter('hough_min_radius', 5)
        self.declare_parameter('hough_max_radius', 300)

        # Hedef çapları
        self.declare_parameter('inner_diameter_cm', 6.0)
        self.declare_parameter('middle_diameter_cm', 12.0)
        self.declare_parameter('outer_diameter_cm', 18.0)

        # HSV aralıkları yükle
        self.black_lower = np.array([
            self.get_parameter('black_lower_h').value,
            self.get_parameter('black_lower_s').value,
            self.get_parameter('black_lower_v').value,
        ])
        self.black_upper = np.array([
            self.get_parameter('black_upper_h').value,
            self.get_parameter('black_upper_s').value,
            self.get_parameter('black_upper_v').value,
        ])

        self.red_lower1 = np.array([
            self.get_parameter('red_lower1_h').value,
            self.get_parameter('red_lower1_s').value,
            self.get_parameter('red_lower1_v').value,
        ])
        self.red_upper1 = np.array([
            self.get_parameter('red_upper1_h').value,
            self.get_parameter('red_upper1_s').value,
            self.get_parameter('red_upper1_v').value,
        ])
        self.red_lower2 = np.array([
            self.get_parameter('red_lower2_h').value,
            self.get_parameter('red_lower2_s').value,
            self.get_parameter('red_lower2_v').value,
        ])
        self.red_upper2 = np.array([
            self.get_parameter('red_upper2_h').value,
            self.get_parameter('red_upper2_s').value,
            self.get_parameter('red_upper2_v').value,
        ])

        self.inner_d = self.get_parameter('inner_diameter_cm').value
        self.middle_d = self.get_parameter('middle_diameter_cm').value
        self.outer_d = self.get_parameter('outer_diameter_cm').value

        # ROS
        self.bridge = CvBridge()
        self.create_subscription(
            Image,
            self.get_parameter('camera_topic').value,
            self._image_callback, 10
        )
        self.pub = self.create_publisher(
            TargetDetection,
            self.get_parameter('publish_topic').value,
            10
        )

        self.get_logger().info('Hedef tespit node başlatıldı')

    def _detect_target_region(self, frame: np.ndarray) -> np.ndarray:
        """Hedefin olduğu bölgeyi siyah+kırmızı renk birleşimiyle bul."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Siyah maske
        black_mask = cv2.inRange(hsv, self.black_lower, self.black_upper)

        # Kırmızı maske
        red_mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        red_mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # Hedef bölge: siyah + kırmızı birlikte
        combined = cv2.bitwise_or(black_mask, red_mask)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)

        return combined

    def _find_concentric_circles(self, mask: np.ndarray) -> list:
        """Konsantrik daireleri tespit et."""
        blurred = cv2.GaussianBlur(mask, (9, 9), 2)

        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=self.get_parameter('hough_dp').value,
            minDist=self.get_parameter('hough_min_dist').value,
            param1=self.get_parameter('hough_param1').value,
            param2=self.get_parameter('hough_param2').value,
            minRadius=self.get_parameter('hough_min_radius').value,
            maxRadius=self.get_parameter('hough_max_radius').value,
        )

        if circles is None:
            return []

        circles = np.uint16(np.around(circles))
        detected = [(int(c[0]), int(c[1]), int(c[2])) for c in circles[0]]

        # Konsantrik daire doğrulama: merkezleri birbirine yakın olan daireleri grupla
        if len(detected) < 1:
            return []

        # En büyük daireyi referans al — muhtemelen dış halka
        detected.sort(key=lambda c: c[2], reverse=True)

        ref_cx, ref_cy, _ = detected[0]
        concentric = []
        for cx, cy, r in detected:
            dist = math.sqrt((cx - ref_cx)**2 + (cy - ref_cy)**2)
            if dist < detected[0][2] * 0.3:  # Merkezler yakın
                concentric.append((cx, cy, r))

        return concentric

    def _estimate_ring(self, circles: list, laser_cx: float, laser_cy: float) -> int:
        """
        Lazer noktasının (görüntü merkezi) hangi halkaya denk geldiğini tahmin et.
        Daireler büyükten küçüğe sıralı varsayılır.
        """
        if not circles:
            return TargetDetection.RING_UNKNOWN

        # Daireleri radius'a göre sırala (küçükten büyüğe)
        sorted_circles = sorted(circles, key=lambda c: c[2])

        # Lazer noktasının (görüntü merkezi) her daire içinde olup olmadığını kontrol et
        for i, (cx, cy, r) in enumerate(sorted_circles):
            dist = math.sqrt((laser_cx - cx)**2 + (laser_cy - cy)**2)
            if dist <= r:
                if i == 0:
                    return TargetDetection.RING_INNER
                elif i == 1:
                    return TargetDetection.RING_MIDDLE
                else:
                    return TargetDetection.RING_OUTER

        return TargetDetection.RING_MISS

    def _image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'Görüntü dönüşüm hatası: {e}')
            return

        h, w = frame.shape[:2]
        img_center_x = w / 2.0
        img_center_y = h / 2.0

        # Hedef bölge maskesi
        mask = self._detect_target_region(frame)

        # Konsantrik daireler
        circles = self._find_concentric_circles(mask)

        det_msg = TargetDetection()
        det_msg.header.stamp = self.get_clock().now().to_msg()
        det_msg.image_width_px = float(w)
        det_msg.image_height_px = float(h)

        if circles:
            # Hedef merkezi: tüm konsantrik dairelerin ortalama merkezi
            avg_cx = np.mean([c[0] for c in circles])
            avg_cy = np.mean([c[1] for c in circles])

            det_msg.detected = True
            det_msg.center_x_px = float(avg_cx)
            det_msg.center_y_px = float(avg_cy)

            # Normalized error (-1 to 1) — gimbal PID için
            det_msg.error_x = (avg_cx - img_center_x) / (w / 2.0)
            det_msg.error_y = (avg_cy - img_center_y) / (h / 2.0)

            # Tahmini halka (görüntü merkezinin — yani lazerin — nereye denk geldiği)
            det_msg.estimated_ring = self._estimate_ring(
                circles, img_center_x, img_center_y
            )
        else:
            det_msg.detected = False
            det_msg.center_x_px = 0.0
            det_msg.center_y_px = 0.0
            det_msg.error_x = 0.0
            det_msg.error_y = 0.0
            det_msg.estimated_ring = TargetDetection.RING_UNKNOWN

        self.pub.publish(det_msg)


# math import — _find_concentric_circles'de kullanılıyor
import math


def main(args=None):
    rclpy.init(args=args)
    node = TargetDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
