"""
Trafik Konisi Tespit Node'u — YOLOv8

Şartname: Trafik konileri 40±10cm x 40±10cm kare tabanlı, 75±5cm yükseklikte,
kırmızı-beyaz ya da turuncu-beyaz. Dokunmadan ilerlemeli.
Yerleşim hakem heyeti tarafından yarışma sırasında belirlenir.

50 puan — her temasta -5 puan. Otonom koşuda PAS GEÇİLEMEZ.

Pipeline:
  1. Kamera frame → YOLOv8 inference
  2. Bounding box → koni merkez pikseli
  3. Kamera pikseli → base_link açısı (bearing)
  4. LiDAR scan ile mesafe eşleme (opsiyonel)
  5. /perception/cones (ConeArray) yayınla
"""

import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge

from ika_msgs.msg import Cone, ConeArray

# Lazy import — YOLO yüklenmezse fallback
YOLO_AVAILABLE = False
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    pass


class ConeDetectorNode(Node):

    def __init__(self):
        super().__init__('cone_detector_node')

        # Parametreler
        self.declare_parameter('camera_topic', '/camera/front/image_raw')
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('publish_topic', '/perception/cones')
        self.declare_parameter('rate', 15.0)
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('device', '0')
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('enable_lidar_fusion', True)
        self.declare_parameter('lidar_angle_tolerance_deg', 5.0)
        self.declare_parameter('max_detection_distance_m', 8.0)
        self.declare_parameter('cone_real_height_m', 0.75)
        self.declare_parameter('camera_focal_length_px', 600.0)

        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.iou_thresh = self.get_parameter('iou_threshold').value
        self.enable_lidar = self.get_parameter('enable_lidar_fusion').value
        self.lidar_angle_tol = math.radians(
            self.get_parameter('lidar_angle_tolerance_deg').value
        )
        self.max_dist = self.get_parameter('max_detection_distance_m').value
        self.cone_height = self.get_parameter('cone_real_height_m').value
        self.focal_length = self.get_parameter('camera_focal_length_px').value

        # YOLO model yükle
        self.model = None
        if YOLO_AVAILABLE:
            model_path = self.get_parameter('model_path').value
            if not model_path:
                model_path = 'yolov8n.pt'  # Default — fine-tune edilmiş model ile değiştirilecek
            try:
                self.model = YOLO(model_path)
                device = self.get_parameter('device').value
                self.get_logger().info(f'YOLOv8 model yüklendi: {model_path}, device={device}')
            except Exception as e:
                self.get_logger().error(f'YOLO model yükleme hatası: {e}')
        else:
            self.get_logger().warn(
                'ultralytics kurulu değil! HSV fallback kullanılacak. '
                'pip install ultralytics ile yükleyin.'
            )

        # ROS
        self.bridge = CvBridge()
        self.latest_scan = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )

        self.create_subscription(
            Image,
            self.get_parameter('camera_topic').value,
            self._image_callback, 10
        )
        if self.enable_lidar:
            self.create_subscription(
                LaserScan,
                self.get_parameter('lidar_topic').value,
                self._scan_callback,
                sensor_qos
            )

        self.pub = self.create_publisher(
            ConeArray,
            self.get_parameter('publish_topic').value,
            10
        )

        self.get_logger().info('Koni tespit node başlatıldı')

    def _scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def _bearing_from_pixel(self, center_x: float, image_width: float) -> float:
        """Piksel X → bearing açısı (rad)."""
        offset = center_x - (image_width / 2.0)
        return math.atan2(offset, self.focal_length)

    def _distance_from_height(self, bbox_height_px: float) -> float:
        """Bounding box yüksekliğinden mesafe tahmini."""
        if bbox_height_px <= 0:
            return 0.0
        return (self.cone_height * self.focal_length) / bbox_height_px

    def _lidar_distance_at_bearing(self, bearing: float) -> float:
        """LiDAR scan'den belirli açıdaki mesafeyi al."""
        if self.latest_scan is None:
            return -1.0

        scan = self.latest_scan
        # Bearing → LiDAR index
        angle = bearing
        if angle < scan.angle_min or angle > scan.angle_max:
            return -1.0

        index = int((angle - scan.angle_min) / scan.angle_increment)
        if index < 0 or index >= len(scan.ranges):
            return -1.0

        # Tolerans aralığında en yakın geçerli mesafe
        half_tol = int(self.lidar_angle_tol / scan.angle_increment)
        start = max(0, index - half_tol)
        end = min(len(scan.ranges), index + half_tol + 1)

        valid_ranges = [
            r for r in scan.ranges[start:end]
            if scan.range_min < r < scan.range_max
        ]

        if valid_ranges:
            return min(valid_ranges)
        return -1.0

    def _detect_yolo(self, frame: np.ndarray) -> list:
        """YOLOv8 ile koni tespiti."""
        imgsz = self.get_parameter('imgsz').value
        results = self.model.predict(
            frame,
            conf=self.conf_thresh,
            iou=self.iou_thresh,
            imgsz=imgsz,
            verbose=False,
        )

        detections = []
        if results and len(results) > 0:
            for box in results[0].boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])

                # COCO'da traffic cone class'ı yok, custom model gerekecek
                # Geçici: tüm tespitleri al, sonra fine-tuned model ile filtrelenir
                # Fine-tuned modelde cls_id=0 → cone olacak
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                bh = y2 - y1

                detections.append({
                    'cx': cx, 'cy': cy,
                    'bbox_height': bh,
                    'conf': conf,
                    'cls': cls_id,
                })

        return detections

    def _detect_hsv_fallback(self, frame: np.ndarray) -> list:
        """YOLO yoksa HSV tabanlı koni tespiti (fallback)."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Turuncu
        orange_lower = np.array([5, 100, 100])
        orange_upper = np.array([25, 255, 255])
        mask_orange = cv2.inRange(hsv, orange_lower, orange_upper)

        # Kırmızı
        red_lower1 = np.array([0, 100, 80])
        red_upper1 = np.array([10, 255, 255])
        red_lower2 = np.array([170, 100, 80])
        red_upper2 = np.array([180, 255, 255])
        mask_red = cv2.bitwise_or(
            cv2.inRange(hsv, red_lower1, red_upper1),
            cv2.inRange(hsv, red_lower2, red_upper2)
        )

        mask = cv2.bitwise_or(mask_orange, mask_red)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 200:
                continue

            x, y, w, h = cv2.boundingRect(cnt)

            # Koni şekli: yüksekliğe göre dar, aspect ratio kontrolü
            aspect = h / max(w, 1)
            if aspect < 1.2:  # Koni yukarı doğru daralan bir şekil
                continue

            cx = x + w / 2.0
            cy = y + h / 2.0

            detections.append({
                'cx': cx, 'cy': cy,
                'bbox_height': float(h),
                'conf': 0.5,
                'cls': 0,
            })

        return detections

    def _image_callback(self, msg: Image):
        """Ana algılama pipeline'ı."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'Görüntü dönüşüm hatası: {e}')
            return

        h, w = frame.shape[:2]

        # Tespit
        if self.model is not None:
            detections = self._detect_yolo(frame)
        else:
            detections = self._detect_hsv_fallback(frame)

        # ConeArray oluştur
        cone_array = ConeArray()
        cone_array.header.stamp = self.get_clock().now().to_msg()

        for det in detections:
            bearing = self._bearing_from_pixel(det['cx'], float(w))

            # Mesafe: LiDAR varsa onu kullan, yoksa kameradan tahmin
            distance = -1.0
            if self.enable_lidar and self.latest_scan is not None:
                distance = self._lidar_distance_at_bearing(bearing)

            if distance <= 0:
                distance = self._distance_from_height(det['bbox_height'])

            if distance > self.max_dist or distance <= 0:
                continue

            # Koni pozisyonu (base_link frame, x=ileri, y=sol)
            cone = Cone()
            cone.x = distance * math.cos(bearing)
            cone.y = distance * math.sin(bearing)
            cone.distance = distance
            cone.angle_rad = bearing
            cone.color = Cone.COLOR_ORANGE_WHITE  # Default, renk ayrımı sonra

            cone_array.cones.append(cone)

        cone_array.total_count = len(cone_array.cones)

        if cone_array.total_count > 0:
            self.pub.publish(cone_array)


def main(args=None):
    rclpy.init(args=args)
    node = ConeDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
