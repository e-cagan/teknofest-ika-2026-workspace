#!/bin/bash
set -e

# ROS2 ortamını yükle
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# YOLO model yolunu environment variable olarak set et
export YOLO_MODEL_PATH="/ros2_ws/weights"

# TensorRT engine varsa onu kullan, yoksa .pt kullan
if [ -f "/ros2_ws/weights/cone_detector.engine" ]; then
    export CONE_MODEL="/ros2_ws/weights/cone_detector.engine"
    echo "[IKA] TensorRT engine kullanılıyor: $CONE_MODEL"
elif [ -f "/ros2_ws/weights/cone_detector.pt" ]; then
    export CONE_MODEL="/ros2_ws/weights/cone_detector.pt"
    echo "[IKA] PyTorch model kullanılıyor: $CONE_MODEL"
    echo "[IKA] Performans için TensorRT export önerilir:"
    echo "       python3 -c \"from ultralytics import YOLO; YOLO('$CONE_MODEL').export(format='engine', imgsz=640, half=True)\""
else
    echo "[IKA] UYARI: Koni modeli bulunamadı! HSV fallback kullanılacak."
    export CONE_MODEL=""
fi

# Udev kurallarını yeniden yükle (USB cihazlar için)
if [ -f /etc/udev/rules.d/99-ika.rules ]; then
    udevadm control --reload-rules 2>/dev/null || true
    udevadm trigger 2>/dev/null || true
fi

echo "══════════════════════════════════════════"
echo "  TEKNOFEST 2026 İKA — Sistem Başlatılıyor"
echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "  Koni model: ${CONE_MODEL:-HSV_FALLBACK}"
echo "══════════════════════════════════════════"

# Komutu çalıştır
exec "$@"
