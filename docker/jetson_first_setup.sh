#!/bin/bash
# ══════════════════════════════════════════════
# Jetson Orin Nano — İlk Kurulum
# Jetson'da çalıştır: bash jetson_first_setup.sh
# ══════════════════════════════════════════════

set -e

echo "═══ Jetson İlk Kurulum Başlıyor ═══"

# 1. Sistem güncelle
echo "[1/7] Sistem güncelleniyor..."
sudo apt-get update && sudo apt-get upgrade -y

# 2. Docker
echo "[2/7] Docker kontrol..."
if ! command -v docker &> /dev/null; then
    sudo apt-get install -y docker.io docker-compose
fi
sudo usermod -aG docker $USER

# 3. nvidia-container-toolkit
echo "[3/7] NVIDIA Container Toolkit..."
if ! dpkg -l | grep -q nvidia-container; then
    sudo apt-get install -y nvidia-container-toolkit
    sudo systemctl restart docker
fi

# 4. Docker daemon nvidia runtime default yap
echo "[4/7] Docker nvidia runtime ayarı..."
sudo tee /etc/docker/daemon.json << 'DAEMON_EOF'
{
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },
    "default-runtime": "nvidia"
}
DAEMON_EOF
sudo systemctl restart docker

# 5. Jetson power mode — MAX performans
echo "[5/7] Power mode: MAXN..."
sudo nvpmodel -m 0  # MAXN mode
sudo jetson_clocks   # Max clock

# 6. Swap alanı artır (Docker build için)
echo "[6/7] Swap kontrol..."
SWAP_SIZE=$(free -m | grep Swap | awk '{print $2}')
if [ "$SWAP_SIZE" -lt 8000 ]; then
    echo "  Swap artırılıyor (8GB)..."
    sudo fallocate -l 8G /swapfile
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
    sudo swapon /swapfile
    echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
fi

# 7. Gerekli dizinler
echo "[7/7] Dizinler oluşturuluyor..."
mkdir -p ~/ika_ws/recordings
mkdir -p ~/ika_ws/weights

echo ""
echo "═══ İlk Kurulum Tamamlandı ═══"
echo ""
echo "ÖNEMLİ: Docker group değişikliği için yeniden giriş yap:"
echo "  exit"
echo "  ssh cagan@$(hostname -I | awk '{print $1}')"
echo ""
echo "Sonra Docker test:"
echo "  docker run --rm --runtime nvidia nvidia/cuda:11.4.3-base-ubuntu20.04 nvidia-smi"
