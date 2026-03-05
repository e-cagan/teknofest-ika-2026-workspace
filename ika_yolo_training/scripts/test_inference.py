"""
Eğitilmiş modelle hızlı görsel test.
Webcam veya görüntü dosyası üzerinde çalıştır.

Kullanım:
  python scripts/test_inference.py                          # webcam
  python scripts/test_inference.py --source test_image.jpg  # dosya
  python scripts/test_inference.py --source test_video.mp4  # video
"""

import argparse
from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default='runs/detect/ika_training/cone_v1/weights/best.pt')
    parser.add_argument('--source', default=0, help='0=webcam, veya dosya yolu')
    parser.add_argument('--conf', type=float, default=0.35)
    parser.add_argument('--imgsz', type=int, default=640)
    args = parser.parse_args()

    model = YOLO(args.model)

    results = model.predict(
        source=args.source,
        conf=args.conf,
        imgsz=args.imgsz,
        show=True,
        save=True,
        stream=True,
    )

    for r in results:
        boxes = r.boxes
        print(f'Frame: {len(boxes)} koni tespit edildi')
        for box in boxes:
            conf = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            print(f'  Koni: ({x1:.0f},{y1:.0f})-({x2:.0f},{y2:.0f}) conf={conf:.2f}')


if __name__ == '__main__':
    main()