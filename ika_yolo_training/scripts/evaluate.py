"""
Eğitilmiş modeli test set'inde değerlendir.

Kullanım:
  python scripts/evaluate.py
  python scripts/evaluate.py --model runs/detect/cone_v1/weights/best.pt
"""

import argparse
from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default='runs/detect/ika_training/cone_v1/weights/best.pt')
    parser.add_argument('--data', default='datasets/Traffic-cone-1/data.yaml')
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--conf', type=float, default=0.35)
    args = parser.parse_args()

    model = YOLO(args.model)

    # Val set'te değerlendir
    metrics = model.val(
        data=args.data,
        imgsz=args.imgsz,
        conf=args.conf,
        plots=True,
        save_json=True,
    )

    print('\n' + '=' * 60)
    print('DEĞERLENDİRME SONUÇLARI')
    print(f'  mAP50:     {metrics.box.map50:.4f}')
    print(f'  mAP50-95:  {metrics.box.map:.4f}')
    print(f'  Precision: {metrics.box.mp:.4f}')
    print(f'  Recall:    {metrics.box.mr:.4f}')
    print()

    # Hedef kontrol
    targets = {
        'mAP50': (metrics.box.map50, 0.90),
        'mAP50-95': (metrics.box.map, 0.65),
        'Precision': (metrics.box.mp, 0.85),
        'Recall': (metrics.box.mr, 0.85),
    }

    all_pass = True
    for name, (value, target) in targets.items():
        status = 'PASS' if value >= target else 'FAIL'
        if status == 'FAIL':
            all_pass = False
        print(f'  {name}: {value:.4f} (hedef >= {target}) [{status}]')

    print()
    if all_pass:
        print('  TÜM METRİKLER HEDEFTE!')
    else:
        print('  BAZI METRİKLER HEDEF ALTINDA — hiperparametre/veri iyileştirmesi gerekebilir')
    print('=' * 60)


if __name__ == '__main__':
    main()