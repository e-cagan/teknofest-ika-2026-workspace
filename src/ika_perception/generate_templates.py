"""
Tabela template'leri oluştur (1-11).
Arial Black fontu ile, şartnameye uygun.
"""
import cv2
import numpy as np
import os

try:
    from PIL import Image, ImageDraw, ImageFont
    USE_PIL = True
except ImportError:
    USE_PIL = False

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), 'ika_perception', 'templates')
SIZE = 128  # piksel — template boyutu


def generate_with_pil():
    """PIL ile Arial Black font kullanarak template oluştur."""
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # Arial Black font bul
    font_candidates = [
        '/usr/share/fonts/truetype/msttcorefonts/Arial_Black.ttf',
        '/usr/share/fonts/truetype/freefont/FreeMonoBold.ttf',
        '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',
        '/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf',
    ]

    font_path = None
    for path in font_candidates:
        if os.path.exists(path):
            font_path = path
            break

    for i in range(1, 12):
        img = Image.new('L', (SIZE, SIZE), 0)  # Siyah arka plan
        draw = ImageDraw.Draw(img)

        text = str(i)

        # Font boyutunu ayarla — tek haneli ve çift haneli farklı
        font_size = 90 if i < 10 else 70
        if font_path:
            font = ImageFont.truetype(font_path, font_size)
        else:
            font = ImageFont.load_default()

        # Metni ortala
        bbox = draw.textbbox((0, 0), text, font=font)
        tw = bbox[2] - bbox[0]
        th = bbox[3] - bbox[1]
        x = (SIZE - tw) // 2 - bbox[0]
        y = (SIZE - th) // 2 - bbox[1]

        draw.text((x, y), text, fill=255, font=font)  # Beyaz numara

        # Kaydet
        np_img = np.array(img)
        path = os.path.join(OUTPUT_DIR, f'{i}.png')
        cv2.imwrite(path, np_img)
        print(f'  {path}')


def generate_with_opencv():
    """PIL yoksa OpenCV putText ile fallback."""
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    for i in range(1, 12):
        img = np.zeros((SIZE, SIZE), dtype=np.uint8)
        text = str(i)

        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 3.0 if i < 10 else 2.0
        thickness = 8 if i < 10 else 6

        (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
        x = (SIZE - tw) // 2
        y = (SIZE + th) // 2

        cv2.putText(img, text, (x, y), font, scale, 255, thickness, cv2.LINE_AA)

        path = os.path.join(OUTPUT_DIR, f'{i}.png')
        cv2.imwrite(path, img)
        print(f'  {path}')


if __name__ == '__main__':
    print(f'Template dizini: {OUTPUT_DIR}')
    if USE_PIL:
        print('PIL kullanılıyor (daha iyi font desteği)')
        generate_with_pil()
    else:
        print('OpenCV fallback (PIL yok — pip install Pillow ile yükleyin)')
        generate_with_opencv()
    print('Tamamlandı!')