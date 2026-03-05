"""
Roboflow'dan dataset indir.

İlk çalıştırmada Roboflow API key istenecek.
https://app.roboflow.com/settings/api adresinden al.

Kullanım:
  python scripts/download_dataset.py
"""

import os
import sys
from dotenv import load_dotenv
from roboflow import Roboflow

# .env'den API key yükle
load_dotenv()

api_key = os.getenv('ROBOFLOW_API_KEY')
if not api_key:
    print('HATA: ROBOFLOW_API_KEY bulunamadı!')
    print('  .env dosyası oluştur:')
    print('  echo "ROBOFLOW_API_KEY=rf_xxxxx" > .env')
    sys.exit(1)

rf = Roboflow(api_key=api_key)

# Ana dataset: 4030 görüntü (en büyük koni dataset'i)
# Bu URL'yi kendi seçtiğin dataset ile değiştir
project = rf.workspace("potato-defect-detecti-a4pov").project("traffic-cone-tfgzt")
version = project.version(1)
dataset = version.download("yolov8")

# Opsiyonel: İkinci dataset (robotik yarışma perspektifi)
# project2 = rf.workspace().project("traffic-cones-4laxg")
# version2 = project2.version(2)
# dataset2 = version2.download("yolov8", location="datasets/cones_extra")

print('\nDataset indirildi!')
print('Birden fazla dataset birleştirmek için:')
print('  Roboflow web arayüzünde "Merge" özelliğini kullan')
print('  veya images/labels klasörlerini manuel birleştir')