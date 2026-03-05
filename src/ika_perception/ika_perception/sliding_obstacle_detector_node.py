"""
Kayar Engel Tespit Node'u

Şartname: 1m genişlik, 20cm/s hız, sağa-sola git-gel, parkur dışına
tamamen çıkabiliyor. Temassız geçilirse 50 puan.

Pipeline:
  1. LiDAR scan → kartezyen noktalar
  2. İlerideki (engel bölgesindeki) noktalardan cluster çıkar
  3. Frame-to-frame fark → hareketli cluster tespiti
  4. Kalman filter → pozisyon ve hız tahmini
  5. Boşluk hesabı → araç geçebilir mi
  6. /perception/sliding_obstacle yayınla
"""

import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from ika_msgs.msg import SlidingObstacle


class SimpleKalmanFilter:
    """1D Kalman filter — pozisyon ve hız tahmini."""

    def __init__(self, process_noise=0.01, measurement_noise=0.05):
        # State: [position, velocity]
        self.x = np.array([0.0, 0.0])
        # Covariance
        self.P = np.eye(2) * 1.0
        # Process noise
        self.Q = np.eye(2) * process_noise
        # Measurement noise
        self.R = np.array([[measurement_noise]])
        # Initialized
        self.initialized = False

    def predict(self, dt: float):
        """Tahmin adımı."""
        F = np.array([[1, dt], [0, 1]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, measurement: float):
        """Güncelleme adımı."""
        if not self.initialized:
            self.x[0] = measurement
            self.x[1] = 0.0
            self.initialized = True
            return

        H = np.array([[1, 0]])
        y = measurement - H @ self.x
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ y).flatten()
        self.P = (np.eye(2) - K @ H) @ self.P

    @property
    def position(self) -> float:
        return self.x[0]

    @property
    def velocity(self) -> float:
        return self.x[1]


class SlidingObstacleDetectorNode(Node):

    def __init__(self):
        super().__init__('sliding_obstacle_detector_node')

        # Parametreler
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('publish_topic', '/perception/sliding_obstacle')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('obstacle_width_m', 1.0)
        self.declare_parameter('obstacle_speed_mps', 0.2)
        self.declare_parameter('road_width_m', 3.0)
        self.declare_parameter('motion_threshold_m', 0.05)
        self.declare_parameter('kalman_process_noise', 0.01)
        self.declare_parameter('kalman_measurement_noise', 0.05)
        self.declare_parameter('vehicle_width_m', 1.0)
        self.declare_parameter('safety_margin_m', 0.3)

        self.obstacle_width = self.get_parameter('obstacle_width_m').value
        self.road_width = self.get_parameter('road_width_m').value
        self.motion_thresh = self.get_parameter('motion_threshold_m').value
        self.vehicle_width = self.get_parameter('vehicle_width_m').value
        self.safety_margin = self.get_parameter('safety_margin_m').value

        # Kalman filter
        self.kf = SimpleKalmanFilter(
            process_noise=self.get_parameter('kalman_process_noise').value,
            measurement_noise=self.get_parameter('kalman_measurement_noise').value,
        )
        self.last_time = time.time()
        self.prev_scan_points = None
        self.obstacle_detected = False

        # ROS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )

        self.create_subscription(
            LaserScan,
            self.get_parameter('lidar_topic').value,
            self._scan_callback,
            sensor_qos
        )

        self.pub = self.create_publisher(
            SlidingObstacle,
            self.get_parameter('publish_topic').value,
            10
        )

        self.get_logger().info('Kayar engel tespit node başlatıldı')

    def _extract_forward_points(self, scan: LaserScan) -> np.ndarray:
        """Aracın ilerisindeki LiDAR noktaları → (x, y)."""
        angles = np.arange(
            scan.angle_min,
            scan.angle_max + scan.angle_increment,
            scan.angle_increment
        )[:len(scan.ranges)]

        ranges = np.array(scan.ranges)
        valid = (ranges > 0.15) & (ranges < 5.0)

        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        points = np.column_stack((x, y))

        # İleri bölge: x > 1m, |y| < road_width/2 + margin
        half_road = self.road_width / 2.0 + 0.5
        forward = points[
            (points[:, 0] > 1.0) &
            (points[:, 0] < 6.0) &
            (np.abs(points[:, 1]) < half_road)
        ]

        return forward

    def _find_obstacle_cluster(self, points: np.ndarray) -> float:
        """
        İleri bölgedeki noktalardan engel kümesini bul.
        Returns: Engelin Y pozisyonu (merkez), veya None.

        Kayar engel: ~1m genişlikte, dar bir Y aralığında yoğun nokta kümesi.
        """
        if len(points) < 3:
            return None

        # Y eksenine göre histogram — engel dar bir Y bandında yoğunlaşır
        y_values = points[:, 1]

        # 10cm bin'lerle histogram
        bins = np.arange(-self.road_width, self.road_width, 0.1)
        hist, edges = np.histogram(y_values, bins=bins)

        # En yoğun bölgeyi bul
        if hist.max() < 3:
            return None

        peak_idx = np.argmax(hist)
        peak_center = (edges[peak_idx] + edges[peak_idx + 1]) / 2.0

        # Peak etrafındaki noktaları al (±obstacle_width/2)
        half_w = self.obstacle_width / 2.0 + 0.1
        cluster_mask = np.abs(y_values - peak_center) < half_w
        cluster = points[cluster_mask]

        if len(cluster) < 3:
            return None

        return float(np.median(cluster[:, 1]))

    def _scan_callback(self, msg: LaserScan):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        points = self._extract_forward_points(msg)
        obstacle_y = self._find_obstacle_cluster(points)

        obs_msg = SlidingObstacle()
        obs_msg.header.stamp = self.get_clock().now().to_msg()

        if obstacle_y is not None:
            self.obstacle_detected = True

            # Kalman predict + update
            self.kf.predict(dt)
            self.kf.update(obstacle_y)

            pos_y = self.kf.position
            vel_y = self.kf.velocity

            # Boşluk hesabı
            # Engel Y pozisyonuna göre araç geçebilecek boşluk
            half_obs = self.obstacle_width / 2.0
            obs_left_edge = pos_y + half_obs
            obs_right_edge = pos_y - half_obs

            half_road = self.road_width / 2.0
            gap_left = half_road - obs_left_edge
            gap_right = obs_right_edge + half_road

            max_gap = max(gap_left, gap_right)
            required = self.vehicle_width + self.safety_margin

            # Engelin parkur dışına çıkmasına kalan süre
            if abs(vel_y) > 0.01:
                if vel_y > 0:
                    dist_to_exit = half_road - pos_y + half_obs
                else:
                    dist_to_exit = pos_y + half_road + half_obs
                time_to_clear = dist_to_exit / abs(vel_y)
            else:
                time_to_clear = 999.0

            obs_msg.detected = True
            obs_msg.position_y = pos_y
            obs_msg.velocity_mps = vel_y
            obs_msg.width_m = self.obstacle_width
            obs_msg.gap_width_m = max_gap
            obs_msg.passage_clear = max_gap >= required
            obs_msg.time_to_clear_sec = time_to_clear

        else:
            # Engel görünmüyor — belki parkur dışına çıktı
            if self.obstacle_detected:
                self.kf.predict(dt)

            obs_msg.detected = False
            obs_msg.position_y = self.kf.position if self.kf.initialized else 0.0
            obs_msg.velocity_mps = self.kf.velocity if self.kf.initialized else 0.0
            obs_msg.width_m = self.obstacle_width
            obs_msg.gap_width_m = self.road_width
            obs_msg.passage_clear = True
            obs_msg.time_to_clear_sec = 0.0

        self.pub.publish(obs_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SlidingObstacleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
