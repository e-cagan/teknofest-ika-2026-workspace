#!/bin/bash
# ══════════════════════════════════════════════
# TEKNOFEST 2026 İKA — Jetson Deploy Script
# Laptoptan çalıştır: ./deploy.sh
# ══════════════════════════════════════════════

set -e

# ── Ayarlar (kendi değerlerinle güncelle) ──
JETSON_USER="sebura"
JETSON_IP="192.168.1.21"          # Jetson'un IP adresi
JETSON_DIR="/home/${JETSON_USER}/ika_ws"
WORKSPACE_DIR="$HOME/ika_ws"

# Renk
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}══════════════════════════════════════════${NC}"
echo -e "${GREEN}  İKA Deploy — Jetson Orin Nano${NC}"
echo -e "${GREEN}  Hedef: ${JETSON_USER}@${JETSON_IP}${NC}"
echo -e "${GREEN}══════════════════════════════════════════${NC}"

# ── 1. SSH bağlantı testi ──
echo -e "\n${YELLOW}[1/6] SSH bağlantı testi...${NC}"
if ssh -o ConnectTimeout=5 ${JETSON_USER}@${JETSON_IP} "echo 'SSH OK'" 2>/dev/null; then
    echo -e "${GREEN}  SSH bağlantısı başarılı${NC}"
else
    echo -e "${RED}  SSH bağlantısı başarısız!${NC}"
    echo "  Kontrol et: ssh ${JETSON_USER}@${JETSON_IP}"
    exit 1
fi

# ── 2. Jetson ortam kontrolü ──
echo -e "\n${YELLOW}[2/6] Jetson ortam kontrolü...${NC}"
ssh ${JETSON_USER}@${JETSON_IP} << 'REMOTE_CHECK'
echo "  Hostname: $(hostname)"
echo "  L4T Version: $(head -1 /etc/nv_tegra_release 2>/dev/null || echo 'N/A')"
echo "  CUDA: $(nvcc --version 2>/dev/null | grep 'release' || echo 'N/A')"
echo "  Docker: $(docker --version 2>/dev/null || echo 'YOK')"
echo "  nvidia-docker: $(nvidia-docker --version 2>/dev/null || echo 'N/A')"
echo "  Disk: $(df -h /home | tail -1 | awk '{print $4 " boş / " $2 " toplam"}')"
echo "  RAM: $(free -h | grep Mem | awk '{print $7 " boş / " $2 " toplam"}')"
REMOTE_CHECK

# ── 3. Jetson'da Docker kurulumu kontrolü ──
echo -e "\n${YELLOW}[3/6] Docker kurulum kontrolü...${NC}"
ssh ${JETSON_USER}@${JETSON_IP} << 'REMOTE_DOCKER'
if ! command -v docker &> /dev/null; then
    echo "  Docker bulunamadı — kuruluyor..."
    sudo apt-get update
    sudo apt-get install -y docker.io docker-compose
    sudo usermod -aG docker $USER
    echo "  Docker kuruldu. Yeniden SSH bağlantısı gerekebilir."
else
    echo "  Docker mevcut: $(docker --version)"
fi

# nvidia-container-runtime kontrolü
if ! dpkg -l | grep -q nvidia-container; then
    echo "  nvidia-container-toolkit kuruluyor..."
    sudo apt-get install -y nvidia-container-toolkit
    sudo systemctl restart docker
else
    echo "  nvidia-container-toolkit mevcut"
fi
REMOTE_DOCKER

# ── 4. Workspace transfer ──
echo -e "\n${YELLOW}[4/6] Workspace transfer ediliyor...${NC}"

# Hedef dizin oluştur
ssh ${JETSON_USER}@${JETSON_IP} "mkdir -p ${JETSON_DIR}"

# rsync ile transfer (hızlı, sadece değişenleri gönderir)
rsync -avz --progress \
    --exclude='build/' \
    --exclude='install/' \
    --exclude='log/' \
    --exclude='.git/' \
    --exclude='__pycache__/' \
    --exclude='*.pyc' \
    --exclude='recordings/' \
    --exclude='ika_yolo_training/venv/' \
    --exclude='ika_yolo_training/datasets/' \
    --exclude='ika_yolo_training/runs/' \
    ${WORKSPACE_DIR}/ ${JETSON_USER}@${JETSON_IP}:${JETSON_DIR}/

echo -e "${GREEN}  Transfer tamamlandı${NC}"

# ── 5. Docker image build ──
echo -e "\n${YELLOW}[5/6] Docker image build ediliyor (bu 20-30 dk sürebilir)...${NC}"
ssh ${JETSON_USER}@${JETSON_IP} << REMOTE_BUILD
cd ${JETSON_DIR}
docker build -t ika_robot:latest .
REMOTE_BUILD

echo -e "${GREEN}  Docker image build tamamlandı${NC}"

# ── 6. Udev kuralları ──
echo -e "\n${YELLOW}[6/6] Udev kuralları kuruluyor...${NC}"
ssh ${JETSON_USER}@${JETSON_IP} << REMOTE_UDEV
if [ -f ${JETSON_DIR}/docker/99-ika.rules ]; then
    sudo cp ${JETSON_DIR}/docker/99-ika.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "  Udev kuralları kuruldu"
fi
REMOTE_UDEV

echo -e "\n${GREEN}══════════════════════════════════════════${NC}"
echo -e "${GREEN}  Deploy tamamlandı!${NC}"
echo -e "${GREEN}══════════════════════════════════════════${NC}"
echo ""
echo "Çalıştırmak için:"
echo "  ssh ${JETSON_USER}@${JETSON_IP}"
echo "  cd ${JETSON_DIR}"
echo "  docker compose up                    # Tam sistem"
echo "  docker compose --profile manual up   # Manuel koşu"
echo ""
echo "Foxglove bağlantısı:"
echo "  ws://${JETSON_IP}:8765"
