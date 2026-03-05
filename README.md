# TEKNOFEST 2026 — İnsansız Kara Aracı (İKA) ROS2 Workspace

**Takım:** Istanbul Okan University  
**Yarışma:** TEKNOFEST 2026 İnsansız Kara Araçları Yarışması — Şanlıurfa  
**Platform:** ROS2 Humble · Ubuntu 22.04 · Jetson Orin Nano · STM32F407  
**Tarihler:** 30 Eylül — 4 Ekim 2026

---

## İçindekiler

- [Proje Hakkında](#proje-hakkında)
- [Sistem Mimarisi](#sistem-mimarisi)
- [Donanım](#donanım)
- [Yazılım Paketleri](#yazılım-paketleri)
- [Hızlı Başlangıç](#hızlı-başlangıç)
- [Detaylı Kurulum](#detaylı-kurulum)
- [Launch Dosyaları](#launch-dosyaları)
- [UART Protokolü](#uart-protokolü)
- [Algılama Pipeline'ı](#algılama-pipelineı)
- [Otonom Sürüş](#otonom-sürüş)
- [Lazer Nişanlama Sistemi](#lazer-nişanlama-sistemi)
- [Güvenlik Sistemi](#güvenlik-sistemi)
- [YOLO Koni Modeli](#yolo-koni-modeli)
- [Docker ile Dağıtım](#docker-ile-dağıtım)
- [Foxglove Arayüzü](#foxglove-arayüzü)
- [Parkur Aşamaları ve Puanlama](#parkur-aşamaları-ve-puanlama)
- [Proje Yapısı](#proje-yapısı)
- [Katkıda Bulunma](#katkıda-bulunma)
- [Lisans](#lisans)

---

## Proje Hakkında

Bu repository, TEKNOFEST 2026 İnsansız Kara Araçları yarışması için geliştirilen tam otonom ve uzaktan kontrollü kara aracının ROS2 yazılım yığınını içerir.

Yarışma iki koşudan oluşur:

**1. Koşu — Uzaktan Kontrollü (Manuel):** Araç, operatör tarafından kamera görüntüleri üzerinden joystick ile kontrol edilir. Operatör parkur boyunca araç yanında yürüyemez — tüm sürüş ve atış kamera feed'i üzerinden yapılır.

**2. Koşu — Tam Otonom:** Araç hareketi, lazer nişanlama ve atış görevleri tamamen otonom algoritmalara, sensör verilerine ve onboard yazılıma bağlı şekilde gerçekleştirilir. İnsan müdahalesi yoktur.

Her iki koşuda parkur 11 aşamadan oluşur: su geçişi, çakıllı yol, yan eğim, dik engel, trafik konileri, kayar engel, engebeli arazi, dik eğim çıkış/iniş (2 saniye durma zorunlu), lazer atış ve hızlanma parkuru. Koşu başına 15 dakika süre limiti ve 1 pas hakkı mevcuttur.

---

## Sistem Mimarisi

### End-to-End Veri Akışı

```
┌──────────────────────────────────────────────────────────────────────┐
│                        FOXGLOVE STUDIO (UI)                         │
│                     rosbridge_websocket :9090                       │
└────────────────────────────────┬─────────────────────────────────────┘
                                 │ WebSocket
┌────────────────────────────────┼─────────────────────────────────────┐
│                          JETSON ORIN NANO                            │
│                                                                      │
│  ┌──────────┐  ┌───────────┐  ┌──────────┐  ┌────────────────────┐  │
│  │PERCEPTION│  │NAVIGATION │  │AUTONOMY  │  │    TARGETING       │  │
│  │          │  │           │  │          │  │                    │  │
│  │ tabela   │─▶│ path_     │  │ mission_ │  │ auto_targeting    │  │
│  │ koni     │  │ follower  │  │ controller│  │ (PID gimbal)     │  │
│  │ bariyer  │─▶│ cone_     │◀─│ stage_   │─▶│                  │  │
│  │ kayar    │  │ avoidance │  │ manager  │  │ targeting_        │  │
│  │ engel    │─▶│ slide_    │  │ behavior_│  │ sequencer         │  │
│  │ hedef    │  │ planner   │  │ executor │  │ (atış sekansı)   │  │
│  └──────────┘  │ slope_    │  └──────────┘  └────────────────────┘  │
│                │ controller│                                         │
│                │ speed_    │                                         │
│                │ controller│                                         │
│                └─────┬─────┘                                         │
│                      │ /cmd_vel_nav                                   │
│  ┌───────────┐  ┌────┴──────┐  ┌───────────┐  ┌──────────────────┐  │
│  │  TELEOP   │─▶│ CMD_VEL   │─▶│  SPEED    │─▶│    STM32         │  │
│  │ joystick  │  │   MUX     │  │  LIMITER  │  │    BRIDGE        │  │
│  │ keyboard  │  │           │  │           │  │                  │  │
│  └───────────┘  └───────────┘  └───────────┘  └────────┬─────────┘  │
│                                                         │ UART       │
│  ┌───────────┐  ┌───────────┐  ┌───────────────────────┘            │
│  │  SAFETY   │  │ RECORDER  │  │                                     │
│  │ heartbeat │  │ video mp4 │  │                                     │
│  │ estop     │  │ rosbag2   │  │                                     │
│  │ health    │  └───────────┘  │                                     │
│  └───────────┘                 │                                     │
└────────────────────────────────┼─────────────────────────────────────┘
                                 │ USB-UART (/dev/ttyUSB0)
┌────────────────────────────────┼─────────────────────────────────────┐
│                          STM32F407                                    │
│                                                                      │
│   Motor PWM ──▶ Motor Sürücüler ──▶ 4x DC Motor                    │
│   Enkoder ◀── Teker Enkoderleri                                      │
│   ADC ◀────── Batarya Voltaj/Akım                                   │
│   GPIO ◀───── Acil Durdurma Butonu (Fiziksel)                       │
│   GPIO ──────▶ Far Sistemi                                           │
│   PWM ───────▶ Fren Aktüatörü                                       │
│                                                                      │
│   Bağımsız Güvenlik: Heartbeat timeout → motorları kes              │
└──────────────────────────────────────────────────────────────────────┘
```

### cmd_vel Pipeline'ı

Hareket komutunun operatörden/otonomiden motorlara ulaşma yolu:

```
Joystick ──▶ teleop_joy_node ──▶ /cmd_vel_teleop ──┐
                                                     ├──▶ cmd_vel_mux_node ──▶ /cmd_vel_raw
Nav nodes ──▶ path_follower vb. ──▶ /cmd_vel_nav ──┘         │
                                                              │ (mod'a göre seçim)
                                                              ▼
                                                    speed_limiter_node
                                                      │ hız limiti
                                                      │ e-stop kontrolü
                                                      │ targeting_lock → sıfır
                                                      ▼
                                                    /cmd_vel
                                                      │
                                                      ▼
                                                stm32_bridge_node
                                                  │ diferansiyel kinematik
                                                  │ cmd_vel → left/right RPM
                                                  ▼
                                                UART TX: "M:40,55\n"
                                                  │
                                                  ▼
                                                STM32 → Motor PWM
```

### TF Ağacı

```
odom (stm32_bridge_node tarafından yayınlanır)
└── base_link
    ├── base_footprint
    └── chassis_link
        ├── front_left_wheel_link    (continuous joint)
        ├── front_right_wheel_link   (continuous joint)
        ├── rear_left_wheel_link     (continuous joint)
        ├── rear_right_wheel_link    (continuous joint)
        ├── front_camera_link
        │   └── front_camera_optical_link
        ├── rear_camera_link
        │   └── rear_camera_optical_link
        ├── lidar_link
        ├── imu_link
        └── gimbal_base_link
            └── gimbal_pan_link      (revolute: ±45°)
                └── gimbal_tilt_link (revolute: ±30°)
                    ├── targeting_camera_link
                    │   └── targeting_camera_optical_link
                    └── laser_link
```

---

## Donanım

| Bileşen | Model | Görev |
|---|---|---|
| Görev Bilgisayarı | NVIDIA Jetson Orin Nano | Tüm ROS2 node'ları, algılama, karar verme |
| Mikrodenetleyici | STM32F407 Discovery | Motor kontrol, enkoder okuma, batarya, güvenlik |
| Kamera (ileri) | USB Kamera | Sürüş, tabela tanıma, koni tespiti, bariyer takibi |
| Kamera (geri) | USB Kamera | Geri sürüş görüntüsü |
| Kamera (nişan) | USB Kamera | Gimbal üzerinde, hedef tespiti ve nişanlama |
| LiDAR | RPLidar A1/A2 | 2D tarama — bariyer, koni mesafesi, kayar engel |
| IMU | BNO055 / MPU6050 | Eğim tespiti (pitch/roll), stabilite monitoring |
| Lazer | Kolime lazer modül | Atış simülasyonu (10m mesafede ≤1.5cm nokta çapı) |
| Gimbal | 2-eksen pan-tilt servo | Lazer ve nişan kamerası yönlendirme |
| Motorlar | 4x DC Motor | Diferansiyel sürüş (skid-steer) |
| Batarya | LiPo + BMS | Güç kaynağı (BMS zorunlu — şartname) |
| Acil Durdurma | Mantar tip NC buton | Fiziksel e-stop (araç üzeri + kumanda) |
| Far | LED far sistemi | Aydınlatma (şartname zorunlu) |
| Kumanda | Joystick (Logitech F710) | Manuel koşu operatör kontrolü |

### Araç Boyut Limitleri (Şartname)

| Ölçü | Min | Max |
|---|---|---|
| Boy | 1.2 m | 2.0 m |
| Genişlik | 0.75 m | 1.2 m |
| Yükseklik | 0.4 m | Genişlik × 1.25 |

---

## Yazılım Paketleri

### Paket Özeti

| Paket | Build Tipi | Node Sayısı | Açıklama |
|---|---|---|---|
| `ika_msgs` | ament_cmake | — | 8 mesaj, 3 servis, 2 action tanımı |
| `ika_description` | ament_cmake | 2 | URDF xacro model, TF ağacı (19 segment) |
| `ika_hardware` | ament_python | 1 | STM32 UART bridge, diferansiyel kinematik, odometri |
| `ika_safety` | ament_python | 4 | Heartbeat, e-stop relay, hız limiter, sistem sağlığı |
| `ika_teleop` | ament_python | 3 | Joystick/klavye kontrol, cmd_vel multiplexer |
| `ika_perception` | ament_python | 5 | Tabela, koni (YOLOv8), bariyer, kayar engel, hedef tespiti |
| `ika_navigation` | ament_python | 5 | Bariyer takibi, koni kaçınma, eğim kontrol, hızlanma |
| `ika_targeting` | ament_python | 2 | Otonom nişan (PID gimbal), atış sekansı yönetimi |
| `ika_autonomy` | ament_python | 3 | Mission controller, stage manager, behavior executor |
| `ika_recorder` | ament_python | 2 | 3 kamera MP4 kayıt, rosbag2 kayıt |
| `ika_bringup` | ament_python | — | 6 launch dosyası, konfigürasyon |

**Toplam: 11 paket, 27+ node**

### ika_msgs — Mesaj Tanımları

Tüm paketlerin ortak veri sözleşmesi. Standart ROS2 mesajlarıyla karşılanamayan ihtiyaçlar için custom tanımlar.

**Mesajlar:**

| Mesaj | Üretici | Tüketici | Açıklama |
|---|---|---|---|
| `VehicleState` | stm32_bridge | mission_controller, health, Foxglove | Hız, enkoder, batarya |
| `SafetyStatus` | estop_relay | cmd_vel_mux, mission_controller | E-stop durumu, heartbeat |
| `SystemMode` | mission_controller | cmd_vel_mux, behavior_executor, tüm node'lar | Aktif mod, aşama, süre |
| `StageInfo` | stage_sign_detector | stage_manager | Tespit edilen tabela numarası |
| `Cone` | — | — | Tekil koni verisi (ConeArray içinde) |
| `ConeArray` | cone_detector | cone_avoidance | Tüm tespit edilen koniler |
| `TargetDetection` | target_detector | auto_targeting, sequencer | Hedef pozisyonu ve hata |
| `SlidingObstacle` | sliding_obstacle_detector | sliding_obstacle_planner | Engel pozisyon/hız |

**Servisler:**

| Servis | Server | Client | Açıklama |
|---|---|---|---|
| `SetMode` | mission_controller | Foxglove, targeting_sequencer | Mod geçişi |
| `FireLaser` | targeting_sequencer | teleop, auto_targeting | Atış sekansı tetikle |
| `SkipStage` | mission_controller | Foxglove | Pas hakkı kullanımı |

**Actions:**

| Action | Server | Client | Açıklama |
|---|---|---|---|
| `ExecuteStage` | behavior_executor | stage_manager | Aşamayı otonom yürüt |
| `AutoAim` | auto_targeting | targeting_sequencer | Otomatik nişan al |

### ika_hardware — Donanım Soyutlama

`stm32_bridge_node` merkezi node — Jetson ↔ STM32 UART haberleşmesini yönetir.

Sorumlulukları:
- `/cmd_vel` → diferansiyel kinematik → UART motor komutu
- UART enkoder verisi → odometri hesabı → `/odom` + TF yayını
- UART batarya/safety → `/vehicle_state`, `/safety/status` yayını
- Heartbeat mekanizması ile bağlantı kopma tespiti
- cmd_vel timeout — komut gelmezse motorları durdurma

### ika_safety — Güvenlik Katmanı

Şartname, güvenlik alt sisteminin ana kontrolden bağımsız olmasını zorunlu kılar. Bu paket yazılımsal güvenliği sağlar; donanımsal güvenlik STM32 firmware'inde bağımsız çalışır.

| Node | Görev |
|---|---|
| `heartbeat_monitor_node` | STM32 ↔ Jetson ve Operatör ↔ Araç heartbeat izleme |
| `estop_relay_node` | Tüm e-stop kaynaklarını birleştirme, latch mekanizması, reset servisi |
| `speed_limiter_node` | Hız limiti, TARGETING_LOCK'ta hareket bloklama, e-stop'ta sıfırlama |
| `system_health_node` | Batarya, CPU sıcaklığı, kamera durumu monitoring |

### ika_teleop — Manuel Kontrol

| Node | Görev |
|---|---|
| `teleop_joy_node` | Joystick → sürüş + gimbal + e-stop + far + lazer fire |
| `teleop_keyboard_node` | Terminal klavye kontrolü (test/debug için) |
| `cmd_vel_mux_node` | Mod bazlı cmd_vel kaynağı seçimi (MANUAL→teleop, AUTO→nav) |

Joystick layout (Logitech F710 / Xbox):

| Giriş | Fonksiyon |
|---|---|
| Sol analog Y | İleri / Geri |
| Sol analog X | Sola / Sağa dönüş |
| Sağ analog X | Gimbal pan |
| Sağ analog Y | Gimbal tilt |
| RB | Turbo modu (basılı tutarken) |
| LB | E-stop toggle |
| A | Lazer fire |
| Y | Far toggle |

### ika_perception — Algılama

| Node | Yöntem | Girdi | Çıktı |
|---|---|---|---|
| `stage_sign_detector_node` | HSV + HoughCircles + Template Matching | Ön kamera | `/perception/stage_info` |
| `cone_detector_node` | YOLOv8n + LiDAR füzyon (HSV fallback) | Ön kamera + LiDAR | `/perception/cones` |
| `barrier_detector_node` | LiDAR nokta kümeleme | LiDAR | `/perception/lane_center` |
| `sliding_obstacle_detector_node` | LiDAR frame fark + Kalman filter | LiDAR | `/perception/sliding_obstacle` |
| `target_detector_node` | HSV + HoughCircles konsantrik daire | Nişan kamerası | `/perception/target` |

### ika_navigation — Otonom Sürüş

| Node | Aktif Aşama | Yöntem |
|---|---|---|
| `path_follower_node` | 1,2,3,4,7 | PID ile bariyer ortasında kalma |
| `cone_avoidance_node` | 5 | Gap-based slalom navigasyon |
| `sliding_obstacle_planner_node` | 6 | Dur-izle-geç state machine |
| `slope_controller_node` | 3,8,10 | IMU eğim tespiti + 2s stop hold |
| `speed_controller_node` | 11 | 30m sprint + 10m güvenli frenleme |

### ika_targeting — Lazer Nişanlama

| Node | Görev |
|---|---|
| `auto_targeting_node` | Hedef arama (sweep) → PID takip → kilitlenme |
| `targeting_sequencer_node` | Atış sekansı: TARGETING_LOCK → lazer AÇ → 1.5s hold → lazer KAPA |

### ika_autonomy — Görev Yönetimi

| Node | Görev |
|---|---|
| `mission_controller_node` | Mod yönetimi, 15dk timer, pas hakkı, SystemMode yayını |
| `stage_manager_node` | Tabela tespitinden aşama geçişi (ardışık onay ile gürültü filtresi) |
| `behavior_executor_node` | Aşama → navigation stratejisi eşleme, active_behavior yayını |

### ika_recorder — Veri Kayıt

| Node | Görev |
|---|---|
| `video_recorder_node` | 3 kameranın ayrı ayrı MP4/AVI kaydı (hakem heyetine teslim) |
| `bag_recorder_node` | Tüm topic'lerin rosbag2 kaydı (debug/analiz/replay) |

---

## Hızlı Başlangıç

### Gereksinimler

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+

### Build

```bash
# Repository klonla
git clone https://github.com/REPO_URL/ika_ws.git
cd ika_ws

# ROS2 bağımlılıkları
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-rosbridge-suite \
  ros-humble-joy \
  ros-humble-tf2-ros \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro \
  ros-humble-urdf

# Python bağımlılıkları
pip install pyserial ultralytics --break-system-packages

# Build
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Çalıştır

```bash
# Donanım testi (donanım bağlı değilken)
ros2 launch ika_bringup test_hardware.launch.py

# Manuel koşu (donanım bağlıyken)
ros2 launch ika_bringup manual_run.launch.py use_cameras:=true use_lidar:=true use_stm32:=true

# Otonom koşu
ros2 launch ika_bringup autonomous_run.launch.py use_cameras:=true use_lidar:=true use_stm32:=true

# Tam sistem (runtime'da mod geçişi)
ros2 launch ika_bringup full_system.launch.py use_cameras:=true use_lidar:=true use_stm32:=true
```

---

## Detaylı Kurulum

### Laptop Geliştirme Ortamı

Donanım olmadan yazılım geliştirme ve test:

```bash
# Workspace build
cd ~/ika_ws
colcon build --symlink-install
source install/setup.bash

# Test — donanım node'ları devre dışı
ros2 launch ika_bringup test_hardware.launch.py
# use_cameras, use_lidar, use_stm32 default false — sadece yazılım node'ları çalışır
```

### Jetson Orin Nano Dağıtımı

```bash
# Jetson'a özgü paketler
sudo apt install -y \
  ros-humble-rplidar-ros \
  ros-humble-v4l2-camera \
  ros-humble-robot-localization

# Udev kuralları (USB cihaz sabit isimleri)
sudo cp docker/99-ika.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# YOLO TensorRT export (ilk kez, 5-15 dk)
python3 -c "from ultralytics import YOLO; \
  YOLO('weights/cone_detector.pt').export(format='engine', imgsz=640, half=True)"
```

---

## Launch Dosyaları

| Launch | Açıklama | Donanım Gerekli |
|---|---|---|
| `test_hardware.launch.py` | Donanım testi — teleop + safety + Foxglove | Hayır |
| `hardware.launch.py` | Tüm sensör ve aktüatör node'ları | Evet (opsiyonel) |
| `safety.launch.py` | Güvenlik katmanı | Hayır |
| `manual_run.launch.py` | 1. koşu — manuel kontrol | Evet |
| `autonomous_run.launch.py` | 2. koşu — tam otonom | Evet |
| `full_system.launch.py` | Tüm node'lar — runtime mod geçişi | Evet |

### Launch Arguments

```bash
# Donanım node'larını aç/kapat
ros2 launch ika_bringup full_system.launch.py \
  use_cameras:=true \      # Kamera node'ları
  use_lidar:=true \        # LiDAR node
  use_stm32:=true          # STM32 UART bridge
```

### Runtime Mod Geçişi

```bash
# Manuel moda geç
ros2 service call /system/set_mode ika_msgs/srv/SetMode "{requested_mode: 1}"

# Otonom moda geç
ros2 service call /system/set_mode ika_msgs/srv/SetMode "{requested_mode: 2}"

# E-stop
ros2 service call /system/set_mode ika_msgs/srv/SetMode "{requested_mode: 3}"

# E-stop serbest bırak → IDLE
ros2 service call /system/set_mode ika_msgs/srv/SetMode "{requested_mode: 0}"

# Pas geçme (aşama 4 = dik engel)
ros2 service call /system/skip_stage ika_msgs/srv/SkipStage "{stage_id: 4}"
```

---

## UART Protokolü

Jetson ↔ STM32 haberleşmesi ASCII tabanlı, `\n` ile sonlandırılmış, 115200 baud, 8N1.

### Jetson → STM32 (TX)

| Komut | Format | Açıklama |
|---|---|---|
| Motor hız | `M:<left_rpm>,<right_rpm>\n` | Sol/sağ teker RPM (int) |
| E-Stop | `E:<0\|1>\n` | 1=durdur, 0=serbest |
| Fren | `B:<0\|1>\n` | 1=kilitle, 0=serbest |
| Far | `L:<0\|1>\n` | 1=aç, 0=kapat |
| Heartbeat | `H\n` | Bağlantı kontrolü |

### STM32 → Jetson (RX)

| Mesaj | Format | Frekans |
|---|---|---|
| Enkoder | `ENC:<left_ticks>,<right_ticks>\n` | 20 Hz |
| Batarya | `BAT:<voltage>,<current>\n` | 1 Hz |
| Güvenlik | `SAF:<estop_state>,<comms_state>\n` | 5 Hz |
| Heartbeat ACK | `ACK:H\n` | Her H alındığında |
| Hata | `ERR:<code>\n` | Gerektiğinde |

### Diferansiyel Kinematik

```
cmd_vel (linear.x, angular.z) alındığında:

v_left  = linear.x - (angular.z × wheel_base / 2)
v_right = linear.x + (angular.z × wheel_base / 2)

RPM = (v_mps / (2π × wheel_radius)) × 60
```

### Odometri Hesabı

```
Enkoder tick'leri alındığında:

delta_left  = (left_ticks - prev_left)  × meters_per_tick
delta_right = (right_ticks - prev_right) × meters_per_tick

delta_s     = (delta_left + delta_right) / 2
delta_theta = (delta_right - delta_left) / wheel_base

x     += delta_s × cos(theta + delta_theta / 2)
y     += delta_s × sin(theta + delta_theta / 2)
theta += delta_theta
```

---

## Algılama Pipeline'ı

### Tabela Tanıma

Parkur tabelaları: kırmızı daire, Arial Black font, 60cm çap, 1-11 numara.

```
Kamera frame → HSV kırmızı maske (çift aralık: 0-10° ve 170-180°)
→ Morfolojik temizlik (close + open)
→ GaussianBlur → HoughCircles
→ Her daire için ROI crop
→ Grayscale → Template matching (11 şablon)
→ En yüksek confidence → StageInfo publish
→ Pinhole model ile mesafe tahmini
```

### Koni Tespiti (YOLOv8n)

Trafik konileri: 75cm yükseklik, kırmızı-beyaz veya turuncu-beyaz.

```
Kamera frame → YOLOv8n inference (TensorRT engine, FP16)
→ Bounding box → merkez piksel → bearing açısı
→ LiDAR scan ile mesafe eşleme (bearing toleransı ±5°)
→ (x, y) pozisyon base_link frame'inde → ConeArray publish

Fallback (YOLO yoksa): HSV turuncu/kırmızı filtre → kontur analizi → aspect ratio kontrolü
```

### Bariyer Tespiti + Yol Merkezi

```
LiDAR scan → kartezyen (x, y) noktalar
→ İleri bölge filtresi (0.3m < x < 3.0m)
→ Sol (y > 0.2) ve sağ (y < -0.2) ayırma
→ Her taraftaki median → sol/sağ bariyer pozisyonu
→ Orta nokta = yol merkezi → PoseStamped publish
```

### Kayar Engel Tespiti

```
LiDAR scan → ileri bölge noktaları
→ Y ekseni histogramı (10cm bin) → yoğunluk pik tespiti
→ Pik etrafındaki küme = engel
→ Frame-to-frame → Kalman filter (pozisyon + hız tahmini)
→ Boşluk hesabı → araç geçebilir mi? → SlidingObstacle publish
```

### Hedef Tespiti

```
Nişan kamerası → HSV siyah + kırmızı birleşim maskesi
→ Morfolojik temizlik → HoughCircles
→ Konsantrik daire doğrulama (merkezleri yakın daireler)
→ Ortalama merkez → normalized hata (-1 to 1)
→ Tahmini halka (inner/middle/outer) → TargetDetection publish
```

---

## Otonom Sürüş

### State Machine

```
                   ┌──────────────────────┐
                   │  mission_controller  │
                   │  /system/set_mode    │
                   └──────────┬───────────┘
                              │
              ┌───────────────┼───────────────┐
              ▼               ▼               ▼
          MODE_IDLE     MODE_MANUAL    MODE_AUTONOMOUS
              │               │               │
              │        teleop aktif    ┌───────┴───────┐
              │               │        │ stage_manager │
              │               │        └───────┬───────┘
              │               │                │
              │               │    ┌───────────┴───────────┐
              │               │    │ behavior_executor     │
              │               │    │ active_behavior yayını│
              │               │    └───────────┬───────────┘
              │               │                │
              │               │    ┌───────────┴───────────────────┐
              │               │    │         │         │           │
              │               │ path_    cone_    slide_    slope_
              │               │ follower avoid    planner  controller
              │               │    │         │         │           │
              │               │    └─────────┴─────────┴───────────┘
              │               │                │
              ▼               ▼                ▼
          Herhangi biri → MODE_ESTOP → tüm hareket durur
          Atış anında  → MODE_TARGETING_LOCK → araç hareketsiz
```

### Aşama-Davranış Eşlemesi

| Aşama | Davranış | Navigation Node |
|---|---|---|
| 1 — Su geçişi | `default` | path_follower (yavaş) |
| 2 — Çakıllı yol | `default` | path_follower (yavaş) |
| 3 — Yan eğim | `default` | path_follower + slope_controller |
| 4 — Dik engel | `default` | path_follower (yavaş) |
| 5 — Trafik konileri | `cone_avoid` | cone_avoidance_node |
| 6 — Kayar engel | `slide_pass` | sliding_obstacle_planner_node |
| 7 — Engebeli arazi | `default` | path_follower (yavaş) |
| 8 — Dik eğim çıkış | `slope_climb` | slope_controller_node |
| 9 — Atış bölgesi | `targeting` | dur + targeting sistemi devralır |
| 10 — Dik eğim iniş | `slope_desc` | slope_controller_node |
| 11 — Hızlanma | `sprint` | speed_controller_node |

---

## Lazer Nişanlama Sistemi

### Atış Sekansı (her iki koşu için aynı)

```
1. PREPARE     → Lazer kesinlikle KAPALI
2. AIM         → Nişan al (manuel: joystick, otonom: PID auto-aim)
3. LOCK_WAIT   → Hedef kilitlenene kadar bekle
4. FREEZE      → TARGETING_LOCK moduna geç → araç tamamen durur
5. FIRE        → Lazer AÇ
6. HOLD        → 1.5 saniye bekle (şartname: min 1s + güvenlik payı)
7. CEASE       → Lazer KAPA
8. RELEASE     → TARGETING_LOCK kaldır → normal moda dön
9. EVALUATE    → Sonucu değerlendir (hangi halka?)
```

Şartname kuralları:
- Nişan alırken lazer **KAPALI** olmalı
- Lazer aktifken araç/gimbal'a hareket verilmesi **YASAK** (ihlal: -10 puan, tekrar hakkı yok)
- Lazer aktif olduktan sonra **minimum 1 saniye** hedefte kalmalı
- Koşu başına **en fazla 3 deneme** hakkı, en iyi sonuç geçerli

### Hedef (Şartname)

Konsantrik daireler, A3 kağıt, minimum 10m mesafe:
- İç daire (6cm çap): **50 puan**
- Orta halka (12cm çap): **25 puan**
- Dış halka (18cm çap): **15 puan**

---

## Güvenlik Sistemi

### Çok Katmanlı Güvenlik

**Katman 1 — Donanımsal (STM32):**
- Bağımsız watchdog: Jetson'dan heartbeat gelmezse motorları kes
- Fiziksel e-stop butonu: NC (normally-closed), basıldığında direkt motor kesme
- Haberleşme kesilince otomatik durma

**Katman 2 — Yazılımsal (Jetson):**
- `heartbeat_monitor_node`: STM32 ve operatör heartbeat izleme
- `estop_relay_node`: Tüm kaynakları birleştirme, latch mekanizması
- `speed_limiter_node`: Hız limiti, mod bazlı bloklama
- `system_health_node`: Batarya, CPU, kamera durumu

**Katman 3 — Operatör:**
- Joystick LB butonu: yazılımsal e-stop
- Foxglove arayüzünden e-stop butonu
- E-stop reset servisi (tüm kaynaklar temiz olmalı)

### E-Stop Akışı

```
Fiziksel buton ──────────┐
Operatör (Foxglove) ─────┤
Operatör (Joystick LB) ──┤──▶ estop_relay_node ──▶ /safety/estop
Heartbeat kaybı ──────────┘                              │
                                                         ├──▶ stm32_bridge → "E:1\n" → motorlar durur
                                                         └──▶ speed_limiter → /cmd_vel = sıfır
```

---

## YOLO Koni Modeli

### Model Bilgileri

- Mimari: YOLOv8n (nano — Jetson optimized)
- Class sayısı: 1 (traffic_cone)
- Eğitim verisi: ~7500 görüntü (Roboflow Universe)
- Input boyutu: 640×640
- Inference: TensorRT FP16 (Jetson'da)

### Eğitim Metrikleri

| Metrik | Değer |
|---|---|
| mAP50 | 0.909 |
| mAP50-95 | 0.636 |
| Precision | 0.857 |
| Recall | 0.829 |

### Inference Ayarları

```yaml
# config/perception_params.yaml
cone_detector_node:
  ros__parameters:
    model_path: "/ros2_ws/weights/cone_detector.pt"  # veya .engine
    confidence_threshold: 0.35    # Recall optimize (koni kaçırmamak önemli)
    iou_threshold: 0.45
    device: "0"
    imgsz: 640
```

### TensorRT Export (Jetson'da)

```bash
python3 -c "from ultralytics import YOLO; \
  YOLO('weights/cone_detector.pt').export(format='engine', imgsz=640, half=True)"
```

### YOLO Eğitim (Ayrı Proje)

Eğitim workspace'ten bağımsız olarak `ika_yolo_training/` dizininde yapılır. Detaylar için o dizindeki README'ye bakın.

---

## Docker ile Dağıtım

### Base Image

`dustynv/ros:humble-pytorch-l4t-r36.2.0` — NVIDIA'nın Jetson için özel build'i. İçerir: ROS2 Humble, PyTorch (CUDA), OpenCV (CUDA), TensorRT.

### Build ve Çalıştır

```bash
# Build (Jetson üzerinde, ~20-30 dk)
cd ~/ika_ws
docker compose build

# Tam sistem
docker compose up

# Manuel koşu
docker compose --profile manual up ika_manual

# Otonom koşu
docker compose --profile autonomous up ika_autonomous

# Debug shell
docker compose run --rm ika_robot bash
```

### Docker Compose Servisleri

| Servis | Profil | Komut |
|---|---|---|
| `ika_robot` | (default) | `full_system.launch.py` |
| `ika_manual` | `manual` | `manual_run.launch.py` |
| `ika_autonomous` | `autonomous` | `autonomous_run.launch.py` |

### Volume Mount'lar

| Host | Container | Açıklama |
|---|---|---|
| `/dev` | `/dev` | USB cihaz erişimi |
| `./recordings` | `/ros2_ws/recordings` | Koşu kayıtları (kalıcı) |
| `./weights` | `/ros2_ws/weights` | YOLO model ağırlıkları |

---

## Foxglove Arayüzü

Foxglove Studio, ROS2 topic'lerini görselleştirmek ve operatör kontrolü için kullanılır.

### Bağlantı

1. Foxglove Studio'yu aç (web veya desktop)
2. "Open connection" → WebSocket → `ws://JETSON_IP:9090`

### Önerilen Panel Düzeni

| Panel | Kaynak | Açıklama |
|---|---|---|
| Büyük görüntü | `/camera/front/compressed` | İleri kamera (sürüş) |
| Küçük görüntü | `/camera/rear/compressed` | Geri kamera |
| Sağ üst görüntü | `/camera/targeting/compressed` | Nişan kamerası |
| Gösterge | `/vehicle_state` | Hız, batarya voltajı |
| Gösterge | `/imu/data` | Pitch/Roll eğim |
| Log | `/system/event` | Koşu olayları |
| Durum | `/system/mode` | Aktif mod, aşama, süre |
| Durum | `/safety/health_summary` | Sistem sağlığı |
| 3D | Robot model + `/scan` | LiDAR + araç modeli |
| Buton | `/system/set_mode` | Mod geçişi |
| Buton | `/safety/estop_request` | Yazılımsal e-stop |

---

## Parkur Aşamaları ve Puanlama

### Parkur Haritası

```
                    ┌──────────────────────────────┐
         (11)       │       hızlanma parkuru        │       (11)
         başlangıç  │    30m sprint + 10m fren      │       bitiş
                    └──────────────────────────────┘

         (10)       ┌──────┐  (9)   ┌──────┐  (8)          (7)
         dik eğim   │ atış │        │ dik  │        ┌──────────────┐
         iniş       │bölge │        │eğim  │        │ engebeli     │
                    └──────┘        │çıkış │        │ arazi        │
                                    └──────┘        └──────────────┘

         (4)        (5)                      (6)
         dik engel  trafik konileri           kayar engel

         (3)        (2)              (1)
         yan eğim   çakıllı yol      su geçişi + yağmur     başlangıç
```

### Puanlama Tablosu

| Aşama | Puan | Not |
|---|---|---|
| Su geçişi + yağmur | 10 | — |
| Çakıllı yol | 5 | — |
| Yan eğim (%20) | 5 | — |
| Dik engel (15cm) | 5 | — |
| Trafik konileri | 50 | Her temasta -5 puan |
| Kayar engel | 50 | Temassız geçilirse |
| Engebeli arazi | 5 | Hatalı çıkma: -5 puan |
| Dik eğim parkuru | 15 | Çıkış + inişte 2s durma şartı |
| Atış (İç daire) | 50 | 3 deneme, en iyi geçerli |
| Atış (Orta halka) | 25 | — |
| Atış (Dış halka) | 15 | — |
| Koşu tamamlama süresi | 100 max | En hızlı=100, her sonraki -5 |
| Hızlanma | 25 max | En hızlı=25, her sonraki -5 |
| Bariyer teması | -5 / temas | — |
| Lazer aktifken hareket | -10 | Tekrar hakkı yok |

### Nihai Değerlendirme Ağırlıkları

| Aşama | Ağırlık |
|---|---|
| Kritik Tasarım Raporu | %20 |
| Teknik Sunum | %10 |
| Final Yarışması | %70 |

---

## Proje Yapısı

```
ika_ws/
├── .dockerignore
├── Dockerfile
├── docker-compose.yml
├── requirements.txt
├── README.md
│
├── docker/
│   ├── entrypoint.sh
│   └── 99-ika.rules              # Udev kuralları
│
├── weights/
│   ├── README.md
│   ├── cone_detector.pt           # YOLOv8n eğitilmiş model
│   └── cone_detector.engine       # TensorRT export (Jetson'da oluşur)
│
├── recordings/                     # Koşu kayıtları (video + rosbag)
│
└── src/
    ├── ika_msgs/                   # Custom mesaj/servis/action
    │   ├── msg/
    │   │   ├── VehicleState.msg
    │   │   ├── SafetyStatus.msg
    │   │   ├── SystemMode.msg
    │   │   ├── StageInfo.msg
    │   │   ├── Cone.msg
    │   │   ├── ConeArray.msg
    │   │   ├── TargetDetection.msg
    │   │   └── SlidingObstacle.msg
    │   ├── srv/
    │   │   ├── SetMode.srv
    │   │   ├── FireLaser.srv
    │   │   └── SkipStage.srv
    │   ├── action/
    │   │   ├── ExecuteStage.action
    │   │   └── AutoAim.action
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── ika_description/            # URDF, TF tree
    │   ├── urdf/
    │   │   └── ika_robot.urdf.xacro
    │   ├── config/
    │   ├── launch/
    │   │   └── description.launch.py
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── ika_hardware/               # STM32 UART bridge
    │   ├── ika_hardware/
    │   │   └── stm32_bridge_node.py
    │   ├── config/
    │   │   └── stm32_params.yaml
    │   ├── setup.py
    │   └── package.xml
    │
    ├── ika_safety/                 # Güvenlik katmanı
    │   ├── ika_safety/
    │   │   ├── heartbeat_monitor_node.py
    │   │   ├── estop_relay_node.py
    │   │   ├── speed_limiter_node.py
    │   │   └── system_health_node.py
    │   ├── config/
    │   │   └── safety_params.yaml
    │   ├── setup.py
    │   └── package.xml
    │
    ├── ika_teleop/                 # Manuel kontrol
    │   ├── ika_teleop/
    │   │   ├── teleop_joy_node.py
    │   │   ├── teleop_keyboard_node.py
    │   │   └── cmd_vel_mux_node.py
    │   ├── config/
    │   │   └── teleop_params.yaml
    │   ├── setup.py
    │   └── package.xml
    │
    ├── ika_perception/             # Algılama
    │   ├── ika_perception/
    │   │   ├── stage_sign_detector_node.py
    │   │   ├── cone_detector_node.py
    │   │   ├── barrier_detector_node.py
    │   │   ├── sliding_obstacle_detector_node.py
    │   │   ├── target_detector_node.py
    │   │   └── templates/          # Tabela numara şablonları (1-11)
    │   ├── config/
    │   │   └── perception_params.yaml
    │   ├── setup.py
    │   └── package.xml
    │
    ├── ika_navigation/             # Otonom sürüş
    │   ├── ika_navigation/
    │   │   ├── path_follower_node.py
    │   │   ├── cone_avoidance_node.py
    │   │   ├── sliding_obstacle_planner_node.py
    │   │   ├── slope_controller_node.py
    │   │   └── speed_controller_node.py
    │   ├── config/
    │   │   └── navigation_params.yaml
    │   ├── setup.py
    │   └── package.xml
    │
    ├── ika_targeting/              # Lazer nişanlama
    │   ├── ika_targeting/
    │   │   ├── auto_targeting_node.py
    │   │   └── targeting_sequencer_node.py
    │   ├── config/
    │   │   └── targeting_params.yaml
    │   ├── setup.py
    │   └── package.xml
    │
    ├── ika_autonomy/               # Görev yönetimi
    │   ├── ika_autonomy/
    │   │   ├── mission_controller_node.py
    │   │   ├── stage_manager_node.py
    │   │   └── behavior_executor_node.py
    │   ├── config/
    │   │   └── autonomy_params.yaml
    │   ├── setup.py
    │   └── package.xml
    │
    ├── ika_recorder/               # Veri kayıt
    │   ├── ika_recorder/
    │   │   ├── video_recorder_node.py
    │   │   └── bag_recorder_node.py
    │   ├── config/
    │   │   └── recorder_params.yaml
    │   ├── setup.py
    │   └── package.xml
    │
    └── ika_bringup/                # Launch ve konfigürasyon
        ├── config/
        │   └── foxglove_bridge.yaml
        ├── launch/
        │   ├── hardware.launch.py
        │   ├── safety.launch.py
        │   ├── manual_run.launch.py
        │   ├── autonomous_run.launch.py
        │   ├── full_system.launch.py
        │   └── test_hardware.launch.py
        ├── setup.py
        └── package.xml
```

---

## Önemli Tarihler

| Tarih | Olay |
|---|---|
| 28 Şubat 2026 | Yarışma son başvuru ✓ |
| 31 Mart 2026 | Teknik Yeterlik Formu (TYF) teslimi |
| 14 Nisan 2026 | TYF sonuçları |
| 1 Haziran 2026 | Kritik Tasarım Raporu (KTR) teslimi — max 25 sayfa |
| 22 Haziran 2026 | KTR sonuçları |
| 20 Temmuz 2026 | Araç Kanıt Videosu (AKV) — 2-5 dk, 720p+, YouTube |
| 31 Temmuz 2026 | Finalistlerin açıklanması |
| Ağustos-Eylül 2026 | Teknik Sunum ve Final Yarışmaları |
| 30 Eylül - 4 Ekim 2026 | TEKNOFEST Şanlıurfa |

---

## Katkıda Bulunma

1. Feature branch oluştur: `git checkout -b feature/yeni-ozellik`
2. Değişiklikleri commit et: `git commit -m "feat: yeni özellik açıklaması"`
3. Build test et: `colcon build --symlink-install && colcon test`
4. Push et: `git push origin feature/yeni-ozellik`
5. Pull Request aç

### Commit Mesajı Formatı

```
feat:  Yeni özellik
fix:   Hata düzeltme
docs:  Dokümantasyon
refac: Kod yeniden düzenleme
test:  Test ekleme/düzeltme
cfg:   Konfigürasyon değişikliği
```

---

## Lisans

Apache License 2.0

---

**TEKNOFEST 2026 — Şanlıurfa'da Görüşmek Üzere**