"""
Parkur Tabelası Tespit Node'u

Şartname: Parkur kenarında her aşamayı belirtmek üzere, yazı tipi Arial Black,
dış çapı 60cm olan kırmızı daireli tabelalar bulunur. (1-11 arası numara)
Otonom görevlerde yarışmacılar bu tabelaları tanıyarak ilgili parkura
geldiklerini algılayacaktır.

Pipeline:
  1. Kamera görüntüsü al
  2. HSV → kırmızı renk maskesi
  3. HoughCircles → kırmızı daire adayları
  4. ROI crop → her daire içi
  5. Template matching → numara tanıma (1-11)
  6. Mesafe tahmini (piksel çapından)
  7. /perception/stage_info yayınla
"""

import os
import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ika_msgs.msg import StageInfo


class StageSignDetectorNode(Node):

    def __init__(self):
        super().__init__('stage_sign_detector_node')

        # Parametreler
        self._declare_params()

        self.camera_topic = self.get_parameter('camera_topic').value
        self.publish_topic = self.get_parameter('publish_topic').value

        # HSV aralıkları — kırmızı iki aralıkta (wrap-around)
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

        # Hough parametreleri
        self.hough_dp = self.get_parameter('hough_dp').value
        self.hough_min_dist = self.get_parameter('hough_min_dist').value
        self.hough_param1 = self.get_parameter('hough_param1').value
        self.hough_param2 = self.get_parameter('hough_param2').value
        self.hough_min_r = self.get_parameter('hough_min_radius').value
        self.hough_max_r = self.get_parameter('hough_max_radius').value

        # Template matching
        self.match_threshold = self.get_parameter('template_match_threshold').value
        self.sign_real_diam = self.get_parameter('sign_real_diameter_m').value
        self.focal_length = self.get_parameter('camera_focal_length_px').value

        # Templateları yükle
        self.templates = {}
        self._load_templates()

        # ROS
        self.bridge = CvBridge()
        self.create_subscription(Image, self.camera_topic, self._image_callback, 10)
        self.pub = self.create_publisher(StageInfo, self.publish_topic, 10)

        self.get_logger().info(
            f'Tabela tanıma başlatıldı — {len(self.templates)} template yüklendi'
        )

    def _declare_params(self):
        self.declare_parameter('camera_topic', '/camera/front/image_raw')
        self.declare_parameter('publish_topic', '/perception/stage_info')
        self.declare_parameter('rate', 10.0)

        for prefix in ['red_lower1', 'red_upper1', 'red_lower2', 'red_upper2']:
            for suffix in ['h', 's', 'v']:
                self.declare_parameter(f'{prefix}_{suffix}', 0)

        self.declare_parameter('hough_dp', 1.2)
        self.declare_parameter('hough_min_dist', 50)
        self.declare_parameter('hough_param1', 100)
        self.declare_parameter('hough_param2', 40)
        self.declare_parameter('hough_min_radius', 20)
        self.declare_parameter('hough_max_radius', 200)
        self.declare_parameter('template_dir', '')
        self.declare_parameter('template_match_threshold', 0.6)
        self.declare_parameter('sign_real_diameter_m', 0.6)
        self.declare_parameter('camera_focal_length_px', 600.0)

    def _load_templates(self):
        """1-11 arası numara templatelarını yükle."""
        template_dir = self.get_parameter('template_dir').value
        if not template_dir:
            # Paket içi templates/ klasörü
            template_dir = os.path.join(
                os.path.dirname(__file__), 'templates'
            )

        for i in range(1, 12):
            path = os.path.join(template_dir, f'{i}.png')
            if os.path.exists(path):
                tmpl = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
                if tmpl is not None:
                    self.templates[i] = tmpl
                    self.get_logger().debug(f'Template yüklendi: {i}')
            else:
                self.get_logger().debug(f'Template bulunamadı: {path}')

        if not self.templates:
            self.get_logger().warn(
                'Hiç template yüklenemedi! Template matching devre dışı, '
                'sadece kırmızı daire tespiti yapılacak.'
            )

    def _detect_red_mask(self, hsv: np.ndarray) -> np.ndarray:
        """Kırmızı renk maskesi oluştur (iki HSV aralığı birleşimi)."""
        mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Morfolojik temizlik
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        return mask

    def _estimate_distance(self, pixel_diameter: float) -> float:
        """Piksel çapından mesafe tahmini (pinhole camera modeli)."""
        if pixel_diameter <= 0:
            return 0.0
        return (self.sign_real_diam * self.focal_length) / pixel_diameter

    def _estimate_bearing(self, center_x: float, image_width: float) -> float:
        """Tabelanın araca göre açısı (rad, 0 = düz ileri)."""
        offset = center_x - (image_width / 2.0)
        return math.atan2(offset, self.focal_length)

    def _match_number(self, roi_gray: np.ndarray) -> tuple:
        """
        ROI içindeki numarayı template matching ile tanı.
        Returns: (stage_id, confidence) veya (0, 0.0)
        """
        if not self.templates:
            return (0, 0.0)

        best_id = 0
        best_score = 0.0

        # ROI'yi birkaç boyutta dene (ölçek değişkenliği için)
        for scale in [0.8, 1.0, 1.2]:
            h, w = roi_gray.shape[:2]
            new_w = max(20, int(w * scale))
            new_h = max(20, int(h * scale))
            scaled = cv2.resize(roi_gray, (new_w, new_h))

            for stage_id, tmpl in self.templates.items():
                # Template'ı ROI boyutuna resize et
                tmpl_resized = cv2.resize(tmpl, (new_w, new_h))

                result = cv2.matchTemplate(
                    scaled, tmpl_resized, cv2.TM_CCOEFF_NORMED
                )
                _, max_val, _, _ = cv2.minMaxLoc(result)

                if max_val > best_score:
                    best_score = max_val
                    best_id = stage_id

        if best_score >= self.match_threshold:
            return (best_id, best_score)

        return (0, 0.0)

    def _image_callback(self, msg: Image):
        """Ana algılama pipeline'ı."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'Görüntü dönüşüm hatası: {e}')
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w = frame.shape[:2]

        # 1. Kırmızı maske
        red_mask = self._detect_red_mask(hsv)

        # 2. HoughCircles — kırmızı maske üzerinde daire tespiti
        blurred = cv2.GaussianBlur(red_mask, (9, 9), 2)
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=self.hough_dp,
            minDist=self.hough_min_dist,
            param1=self.hough_param1,
            param2=self.hough_param2,
            minRadius=self.hough_min_r,
            maxRadius=self.hough_max_r,
        )

        if circles is None:
            return

        circles = np.uint16(np.around(circles))

        best_stage = 0
        best_conf = 0.0
        best_dist = 0.0
        best_bearing = 0.0

        for cx, cy, r in circles[0]:
            cx, cy, r = int(cx), int(cy), int(r)

            # ROI çıkar (daire içindeki kare bölge)
            x1 = max(0, cx - r)
            y1 = max(0, cy - r)
            x2 = min(w, cx + r)
            y2 = min(h, cy + r)

            if x2 - x1 < 20 or y2 - y1 < 20:
                continue

            roi = frame[y1:y2, x1:x2]
            roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

            # 3. Template matching
            stage_id, confidence = self._match_number(roi_gray)

            if confidence > best_conf:
                best_stage = stage_id
                best_conf = confidence
                best_dist = self._estimate_distance(float(r * 2))
                best_bearing = self._estimate_bearing(float(cx), float(w))

        # 4. En iyi sonucu yayınla
        if best_stage > 0:
            stage_msg = StageInfo()
            stage_msg.header.stamp = self.get_clock().now().to_msg()
            stage_msg.stage_id = best_stage
            stage_msg.confidence = best_conf
            stage_msg.distance_m = best_dist
            stage_msg.bearing_rad = best_bearing
            self.pub.publish(stage_msg)

            self.get_logger().info(
                f'Tabela tespit: #{best_stage} | '
                f'conf={best_conf:.2f} | '
                f'dist={best_dist:.1f}m | '
                f'bearing={math.degrees(best_bearing):.1f}°'
            )


def main(args=None):
    rclpy.init(args=args)
    node = StageSignDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
