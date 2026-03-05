"""
YOLOv8 → TensorRT Engine Export

DİKKAT: Bu script JETSON üzerinde çalıştırılmalı!
TensorRT engine donanıma özgüdür — laptop'ta oluşturulan engine Jetson'da çalışmaz.

Kullanım (Jetson'da):
  python3 export_tensorrt.py --model /ros2_ws/weights/cone_detector.pt

Çıktı:
  /ros2_ws/weights/cone_detector.engine
"""

import argparse
import os
from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default='/ika_ws/weights/best.pt')
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--half', action='store_true', default=True,
                        help='FP16 quantization (Jetson icin onerilen)')
    args = parser.parse_args()

    if not os.path.exists(args.model):
        print(f'HATA: Model bulunamadı: {args.model}')
        return

    model = YOLO(args.model)

    print(f'Model: {args.model}')
    print(f'imgsz: {args.imgsz}')
    print(f'FP16: {args.half}')
    print('TensorRT export başlıyor... (ilk sefer 5-15 dk sürebilir)')

    engine_path = model.export(
        format='engine',
        imgsz=args.imgsz,
        half=args.half,
        device=0,
    )

    print(f'\nExport tamamlandı: {engine_path}')
    print('Bu engine dosyasını cone_detector_node config\'inde model_path olarak ayarlayın.')


if __name__ == '__main__':
    main()