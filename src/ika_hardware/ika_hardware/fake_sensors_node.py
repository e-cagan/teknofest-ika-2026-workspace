"""
Fake Sensors Node — Donanım Olmadan Test İçin

Yayınlar:
  /camera/front/image_raw      — Renkli test pattern (koniler + bariyer çizgileri)
  /camera/rear/image_raw       — Gri test pattern
  /camera/targeting/image_raw  — Hedef pattern (konsantrik daireler)
  /imu/data                    — Düz zemin (configurable pitch/roll)
  /scan                        — 3m koridorlu sahte LiDAR scan
"""

import math
import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu, LaserScan
from geometry_msgs.msg import Quaternion
from cv_bridge import CvBridge


class FakeSensorsNode(Node):

    def __init__(self):
        super().__init__('fake_sensors_node')

        # Parametreler
        self.declare_parameter('camera_fps', 15.0)
        self.declare_parameter('lidar_hz', 10.0)
        self.declare_parameter('imu_hz', 50.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)

        # Simülasyon parametreleri
        self.declare_parameter('sim_pitch_deg', 0.0)     # Eğim simülasyonu
        self.declare_parameter('sim_roll_deg', 0.0)      # Yan eğim simülasyonu
        self.declare_parameter('sim_corridor_width', 3.0) # Koridor genişliği (metre)
        self.declare_parameter('sim_cone_count', 5)       # Sahte koni sayısı
        self.declare_parameter('sim_sliding_obstacle', False)  # Kayar engel sim

        self.img_w = self.get_parameter('image_width').value
        self.img_h = self.get_parameter('image_height').value
        self.corridor_w = self.get_parameter('sim_corridor_width').value
        self.cone_count = self.get_parameter('sim_cone_count').value
        self.sim_pitch = math.radians(self.get_parameter('sim_pitch_deg').value)
        self.sim_roll = math.radians(self.get_parameter('sim_roll_deg').value)
        self.sim_sliding = self.get_parameter('sim_sliding_obstacle').value

        self.bridge = CvBridge()
        self.frame_count = 0
        self.start_time = time.time()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )

        # Publishers
        self.front_cam_pub = self.create_publisher(Image, '/camera/front/image_raw', 10)
        self.rear_cam_pub = self.create_publisher(Image, '/camera/rear/image_raw', 10)
        self.target_cam_pub = self.create_publisher(Image, '/camera/targeting/image_raw', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', sensor_qos)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', sensor_qos)

        # Timerlar
        cam_period = 1.0 / self.get_parameter('camera_fps').value
        lidar_period = 1.0 / self.get_parameter('lidar_hz').value
        imu_period = 1.0 / self.get_parameter('imu_hz').value

        self.create_timer(cam_period, self._publish_cameras)
        self.create_timer(lidar_period, self._publish_lidar)
        self.create_timer(imu_period, self._publish_imu)

        self.get_logger().info(
            f'Fake sensors başlatıldı — '
            f'{self.img_w}x{self.img_h} @ {self.get_parameter("camera_fps").value}fps, '
            f'pitch={self.get_parameter("sim_pitch_deg").value}°, '
            f'roll={self.get_parameter("sim_roll_deg").value}°'
        )

    # ══════════════════════════════════════════
    #  FAKE KAMERA
    # ══════════════════════════════════════════

    def _generate_front_image(self) -> np.ndarray:
        """Ön kamera: yol + bariyerler + koniler + tabela."""
        img = np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)

        # Zemin (gri asfalt)
        img[self.img_h // 2:, :] = (80, 80, 80)

        # Gökyüzü (açık mavi)
        img[:self.img_h // 2, :] = (200, 180, 140)

        # Bariyerler (kırmızı-beyaz çizgili, iki yanda)
        stripe_w = 20
        for y in range(self.img_h // 2, self.img_h):
            # Sol bariyer
            stripe_idx = (y // stripe_w) % 2
            color = (0, 0, 200) if stripe_idx == 0 else (255, 255, 255)
            cv2.line(img, (30, y), (50, y), color, 2)

            # Sağ bariyer
            color = (0, 0, 200) if stripe_idx == 1 else (255, 255, 255)
            cv2.line(img, (self.img_w - 50, y), (self.img_w - 30, y), color, 2)

        # Koniler (turuncu üçgenler)
        t = time.time() - self.start_time
        for i in range(self.cone_count):
            cx = 100 + i * 100 + int(20 * math.sin(t + i))
            cy = self.img_h // 2 + 50 + i * 30
            if cx < self.img_w - 50 and cy < self.img_h - 20:
                pts = np.array([
                    [cx, cy - 40],
                    [cx - 15, cy],
                    [cx + 15, cy]
                ], np.int32)
                cv2.fillPoly(img, [pts], (0, 140, 255))  # Turuncu
                cv2.polylines(img, [pts], True, (255, 255, 255), 1)

        # Tabela (kırmızı daire + numara)
        stage_num = ((int(t) // 5) % 11) + 1
        cx_sign = self.img_w - 100
        cy_sign = self.img_h // 2 - 30
        cv2.circle(img, (cx_sign, cy_sign), 35, (0, 0, 220), -1)
        cv2.circle(img, (cx_sign, cy_sign), 35, (255, 255, 255), 2)
        cv2.putText(img, str(stage_num), (cx_sign - 12, cy_sign + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        # Frame bilgisi
        cv2.putText(img, f'FAKE FRONT CAM | frame={self.frame_count}',
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        return img

    def _generate_rear_image(self) -> np.ndarray:
        """Arka kamera: basit gri görüntü."""
        img = np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)
        img[:] = (60, 60, 60)

        cv2.putText(img, 'FAKE REAR CAM', (self.img_w // 2 - 80, self.img_h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Basit bariyer çizgileri
        for y in range(self.img_h // 2, self.img_h):
            stripe_idx = (y // 20) % 2
            color = (0, 0, 200) if stripe_idx == 0 else (255, 255, 255)
            cv2.line(img, (30, y), (50, y), color, 2)
            cv2.line(img, (self.img_w - 50, y), (self.img_w - 30, y), color, 2)

        return img

    def _generate_targeting_image(self) -> np.ndarray:
        """Nişan kamerası: konsantrik daireli hedef."""
        img = np.ones((self.img_h, self.img_w, 3), dtype=np.uint8) * 200

        cx = self.img_w // 2
        cy = self.img_h // 2

        # Hedef — siyah-kırmızı konsantrik daireler (şartnameye uygun)
        # Dış halka (18cm → piksel olarak 120)
        cv2.circle(img, (cx, cy), 120, (0, 0, 0), -1)
        # Kırmızı halka
        cv2.circle(img, (cx, cy), 100, (0, 0, 200), -1)
        # Orta halka (12cm → 80px)
        cv2.circle(img, (cx, cy), 80, (0, 0, 0), -1)
        # Kırmızı halka
        cv2.circle(img, (cx, cy), 60, (0, 0, 200), -1)
        # İç daire (6cm → 40px)
        cv2.circle(img, (cx, cy), 40, (0, 0, 0), -1)

        # Nişan çizgileri (crosshair)
        cv2.line(img, (cx - 150, cy), (cx + 150, cy), (0, 255, 0), 1)
        cv2.line(img, (cx, cy - 150), (cx, cy + 150), (0, 255, 0), 1)

        cv2.putText(img, 'FAKE TARGET CAM', (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        return img

    def _publish_cameras(self):
        """Sahte kamera görüntüleri yayınla."""
        now = self.get_clock().now().to_msg()
        self.frame_count += 1

        # Ön kamera
        front_img = self._generate_front_image()
        front_msg = self.bridge.cv2_to_imgmsg(front_img, 'bgr8')
        front_msg.header.stamp = now
        front_msg.header.frame_id = 'front_camera_optical_link'
        self.front_cam_pub.publish(front_msg)

        # Arka kamera
        rear_img = self._generate_rear_image()
        rear_msg = self.bridge.cv2_to_imgmsg(rear_img, 'bgr8')
        rear_msg.header.stamp = now
        rear_msg.header.frame_id = 'rear_camera_optical_link'
        self.rear_cam_pub.publish(rear_msg)

        # Nişan kamerası
        target_img = self._generate_targeting_image()
        target_msg = self.bridge.cv2_to_imgmsg(target_img, 'bgr8')
        target_msg.header.stamp = now
        target_msg.header.frame_id = 'targeting_camera_optical_link'
        self.target_cam_pub.publish(target_msg)

    # ══════════════════════════════════════════
    #  FAKE LiDAR
    # ══════════════════════════════════════════

    def _publish_lidar(self):
        """Sahte LiDAR: 3m koridorlu iki bariyer + opsiyonel kayar engel."""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'lidar_link'

        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.radians(1.0)  # 1 derece çözünürlük
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.15
        scan.range_max = 12.0

        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        ranges = []

        half_corridor = self.corridor_w / 2.0
        t = time.time() - self.start_time

        for i in range(num_readings):
            angle = scan.angle_min + i * scan.angle_increment

            # Varsayılan: uzak (boş alan)
            r = scan.range_max

            # Sol bariyer (y = +half_corridor)
            if abs(math.sin(angle)) > 0.01:
                r_left = half_corridor / abs(math.sin(angle))
                if math.sin(angle) > 0 and 0.15 < r_left < r:
                    r = r_left

            # Sağ bariyer (y = -half_corridor)
            if abs(math.sin(angle)) > 0.01:
                r_right = half_corridor / abs(math.sin(angle))
                if math.sin(angle) < 0 and 0.15 < r_right < r:
                    r = r_right

            # Kayar engel simülasyonu
            if self.sim_sliding:
                obs_y = 0.8 * math.sin(t * 0.4)  # 20cm/s git-gel benzeri
                obs_x = 4.0  # 4m ileride
                obs_w = 0.5  # yarım genişlik

                # Engelin LiDAR'da görünüp görünmediği
                px = obs_x
                py = obs_y
                dist = math.sqrt(px * px + py * py)
                obs_angle = math.atan2(py, px)

                if abs(angle - obs_angle) < math.atan2(obs_w, obs_x):
                    if dist < r:
                        r = dist + np.random.normal(0, 0.02)

            # Gürültü ekle
            r += np.random.normal(0, 0.01)
            r = max(scan.range_min, min(scan.range_max, r))
            ranges.append(float(r))

        scan.ranges = ranges
        self.scan_pub.publish(scan)

    # ══════════════════════════════════════════
    #  FAKE IMU
    # ══════════════════════════════════════════

    def _publish_imu(self):
        """Sahte IMU: konfigüre edilebilir pitch/roll."""
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu_link'

        # Euler → Quaternion
        roll = self.sim_roll
        pitch = self.sim_pitch
        yaw = 0.0

        cr = math.cos(roll / 2)
        sr = math.sin(roll / 2)
        cp = math.cos(pitch / 2)
        sp = math.sin(pitch / 2)
        cy = math.cos(yaw / 2)
        sy = math.sin(yaw / 2)

        imu.orientation.w = cr * cp * cy + sr * sp * sy
        imu.orientation.x = sr * cp * cy - cr * sp * sy
        imu.orientation.y = cr * sp * cy + sr * cp * sy
        imu.orientation.z = cr * cp * sy - sr * sp * cy

        # Hafif gürültü
        imu.angular_velocity.x = np.random.normal(0, 0.01)
        imu.angular_velocity.y = np.random.normal(0, 0.01)
        imu.angular_velocity.z = np.random.normal(0, 0.01)

        imu.linear_acceleration.x = np.random.normal(0, 0.05)
        imu.linear_acceleration.y = np.random.normal(0, 0.05)
        imu.linear_acceleration.z = 9.81 + np.random.normal(0, 0.05)

        self.imu_pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = FakeSensorsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
