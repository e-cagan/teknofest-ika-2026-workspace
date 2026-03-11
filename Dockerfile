# ══════════════════════════════════════════════════════════════
# TEKNOFEST 2026 İKA — Jetson Orin Nano Docker Image
# ══════════════════════════════════════════════════════════════
#
# Base: dustynv/ros:humble-pytorch-l4t-r36.2.0
# İçerir: ROS2 Humble, PyTorch (CUDA), OpenCV (CUDA), TensorRT
#
# Build:
#   docker build -t ika_robot:latest .
#
# Run:
#   docker compose up
#   veya
#   docker run --runtime nvidia --privileged --network host \
#     -v /dev:/dev -v /tmp/.X11-unix:/tmp/.X11-unix \
#     -e DISPLAY=$DISPLAY \
#     ika_robot:latest \
#     ros2 launch ika_bringup full_system.launch.py \
#       use_cameras:=true use_lidar:=true use_stm32:=true
# ══════════════════════════════════════════════════════════════

# ── Stage 1: Base ──
# ── Stage 1: Base (Resmi ROS Humble Perception İmajı) ──
# Bu imajın içinde ROS 2, OpenCV ve temel görüntü işleme paketleri hazırdır.
FROM ros:humble-perception AS base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=42
ENV SHELL=/bin/bash

WORKDIR /ros2_ws

# ── Stage 2: Sistem Bağımlılıkları ve Jetson PyTorch ──
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    libopenblas-dev \
    libopenmpi-dev \
    libomp-dev \
    nano \
    curl \
    && rm -rf /var/lib/apt/lists/*

# ZIRH 1: Temel paketleri önden kur
RUN pip3 install --no-cache-dir typing_extensions sympy networkx jinja2 fsspec mpmath

# ZIRH 2: NVIDIA JetPack 6 uyumlu PyTorch kurulumu
# --index-url yerine --extra-index-url kullanıyoruz ki mpmath vb. paketleri bulabilsin.
RUN pip3 install --no-cache-dir torch torchvision \
    --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v60/dp/pt

RUN apt-get update && apt-get install -y --no-install-recommends \
    # ROS2 paketleri
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-rosbridge-suite \
    ros-humble-foxglove-bridge \
    ros-humble-joy \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-v4l2-camera \
    ros-humble-rplidar-ros \
    ros-humble-robot-localization \
    ros-humble-xacro \
    ros-humble-urdf \
    ros-humble-diagnostic-msgs \
    # Sistem araçları
    python3-pip \
    udev \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# ── Stage 3: Python bağımlılıkları ──
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt \
    && rm /tmp/requirements.txt

# ZIRH 3: YOLO ve thop paketlerini SADECE EN SON, torch'u ezmesi yasaklanmış şekilde (--no-deps) kuruyoruz
RUN pip3 install --no-cache-dir --no-deps ultralytics>=8.0.0 thop>=0.1.1

# ── Stage 4: YOLO model ağırlıkları ──
# Eğitilmiş model ağırlıkları — container boyutunu artırır ama
# runtime'da indirmeye gerek kalmaz
COPY weights/ /ros2_ws/weights/

# ── Stage 5: ROS2 workspace kopyala ve build et ──
COPY src/ /ros2_ws/src/

# rosdep ile eksik bağımlılıkları kontrol et
RUN . /opt/ros/humble/setup.sh && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true && \
    rm -rf /var/lib/apt/lists/*

# Build
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
      --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
      --parallel-workers $(nproc)

# ── Stage 6: TensorRT model export ──
# İlk çalıştırmada YOLO .pt → .engine dönüşümü yapılır
# Bu adım opsiyonel — build sırasında veya runtime'da yapılabilir
# RUN . /opt/ros/humble/setup.sh && \
#     python3 -c "from ultralytics import YOLO; \
#       model = YOLO('/ros2_ws/weights/cone_detector.pt'); \
#       model.export(format='engine', imgsz=640, half=True)"

# ── Stage 7: Entrypoint ──
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Kayıt dizini
RUN mkdir -p /ros2_ws/recordings

# Port (Foxglove WebSocket)
EXPOSE 8765

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "ika_bringup", "full_system.launch.py", \
     "use_cameras:=true", "use_lidar:=true", "use_stm32:=true"]
