# ══════════════════════════════════════════════════════════════
# TEKNOFEST 2026 İKA — Jetson Orin Nano Docker Image
# ══════════════════════════════════════════════════════════════

# ── Stage 1: Base (NVIDIA'nın Hazır PyTorch + CUDA İmajı) ──
FROM dustynv/l4t-pytorch:r36.2.0 AS base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=42
ENV SHELL=/bin/bash

WORKDIR /ros2_ws

# ── Stage 2: Sistem Araçları ve ROS 2 Repo Kurulumu ──
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    libopenblas-dev \
    libopenmpi-dev \
    libomp-dev \
    nano \
    curl \
    software-properties-common \
    gnupg2 \
    lsb-release \
    && add-apt-repository universe

# ROS 2 Key ve Repo ekle
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ── Stage 3: ROS 2 Paketleri ve Sistem Bağımlılıkları ──
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
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
    python3-colcon-common-extensions \
    python3-rosdep \
    udev \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# ── Stage 4: Python Zırhlama ve Bağımlılıklar ──

# ZIRH 1 & 2: Temel paketleri ve Torch güncellemelerini RESMİ index üzerinden yapıyoruz.
# --index-url eklenerek hatalı yönlendirmeler baypas edildi.
RUN pip3 install --no-cache-dir --index-url https://pypi.org/simple \
    typing_extensions sympy networkx jinja2 fsspec mpmath

# requirements.txt içindeki paketleri kur (yine resmi index üzerinden)
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir --index-url https://pypi.org/simple -r /tmp/requirements.txt \
    && rm /tmp/requirements.txt

# ZIRH 3: YOLO ve thop paketlerini EN SON, torch'a dokunmadan (--no-deps) kur
RUN pip3 install --no-cache-dir --index-url https://pypi.org/simple --no-deps ultralytics>=8.0.0 thop>=0.1.1

# ── Stage 5: Ağırlıklar, Kaynak Kod ve Build ──
COPY weights/ /ros2_ws/weights/
COPY src/ /ros2_ws/src/

RUN . /opt/ros/humble/setup.sh && \
    rosdep init || echo "already init" && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true

RUN . /opt/ros/humble/setup.sh && \
    colcon build \
      --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
      --parallel-workers 2

# ── Stage 6: Entrypoint ve Ayarlar ──
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN mkdir -p /ros2_ws/recordings
EXPOSE 8765

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "ika_bringup", "full_system.launch.py", \
     "use_cameras:=true", "use_lidar:=true", "use_stm32:=true"]