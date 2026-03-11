#!/bin/bash
# Hızlı deploy — sadece src ve weights transfer et, rebuild yap
# Tam deploy'dan çok daha hızlı (1-2 dk)

JETSON_USER="cagan"
JETSON_IP="192.168.1.100"
JETSON_DIR="/home/${JETSON_USER}/ika_ws"

echo "Hızlı deploy başlıyor..."

# Sadece src, weights, config dosyalarını gönder
rsync -avz --progress \
    --exclude='__pycache__/' \
    --exclude='*.pyc' \
    ~/ika_ws/src/ ${JETSON_USER}@${JETSON_IP}:${JETSON_DIR}/src/

rsync -avz --progress \
    ~/ika_ws/weights/ ${JETSON_USER}@${JETSON_IP}:${JETSON_DIR}/weights/

# Docker rebuild (cache kullanır, hızlı)
ssh ${JETSON_USER}@${JETSON_IP} "cd ${JETSON_DIR} && docker build -t ika_robot:latest ."

echo "Hızlı deploy tamamlandı!"
echo "Çalıştır: ssh ${JETSON_USER}@${JETSON_IP} 'cd ${JETSON_DIR} && docker compose up'"
