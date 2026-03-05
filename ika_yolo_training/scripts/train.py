"""
Koni Tespiti — YOLOv8n Eğitim Script'i

Kullanım:
  cd ~/ika_yolo_training
  source venv/bin/activate
  python scripts/train.py
"""

from ultralytics import YOLO


def main():
    model = YOLO('yolov8n.pt')  # Pretrained COCO ağırlıklarından başla

    results = model.train(
        data='datasets/Traffic-cone-1/data.yaml',
        
        # Temel
        epochs=150,                # 100-200 arası, early stopping halleder
        patience=30,               # 30 epoch iyileşme yoksa dur
        batch=16,                  # GPU belleğine göre ayarla (8 veya 16)
        imgsz=640,                 # 640 standart, Jetson'da inference da 640
        
        # Optimizer
        optimizer='AdamW',         # SGD'den daha stabil yakınsar tek class için
        lr0=0.001,                 # Başlangıç learning rate
        lrf=0.01,                  # Final LR = lr0 * lrf = 0.00001
        weight_decay=0.0005,
        warmup_epochs=5,           # İlk 5 epoch LR yavaş artır
        
        # Augmentation
        hsv_h=0.015,               # Hue — renk kaydırma (kırmızı↔turuncu toleransı)
        hsv_s=0.7,                 # Saturation — doygunluk varyasyonu
        hsv_v=0.4,                 # Value — parlaklık varyasyonu
        degrees=15.0,              # Rotasyon ±15°
        translate=0.1,             # Öteleme %10
        scale=0.5,                 # Ölçek 0.5-1.5x
        flipud=0.0,                # Dikey flip KAPALI (koni ters durmaz)
        fliplr=0.5,                # Yatay flip %50
        mosaic=1.0,                # Mosaic augmentation açık
        mixup=0.1,                 # Hafif mixup
        
        # Kayıt
        project='ika_training',
        name='cone_v1',
        save=True,
        save_period=25,            # Her 25 epoch checkpoint
        plots=True,
        
        # Cihaz
        device=0,                  # GPU
        workers=4,
        
        # Tek class optimizasyonu
        single_cls=False,          # Eğer SADECE koni varsa True yapılabilir
                                # Ama ileride tabela class'ı ekleyebilirsin, False bırak
    )

    # Sonuçları yazdır
    print('\n' + '=' * 60)
    print('EĞİTİM TAMAMLANDI')
    print(f'Best model: runs/detect/cone_v1/weights/best.pt')
    print(f'Metrikler: runs/detect/cone_v1/')
    print('=' * 60)

    return results


if __name__ == '__main__':
    main()