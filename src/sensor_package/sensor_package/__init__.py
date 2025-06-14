"""
sensor_package - ROS 2 Sıcaklık Sensörü Simülasyon Paketi

Bu paket ROS 2 öğrenmek için tasarlanmış örnek bir projedir.
Sıcaklık sensörü simülasyonu yapmaktadır.

Modüller:
- temperature_publisher: Sıcaklık verisi yayınlar
- temperature_subscriber: Sıcaklık verisi dinler ve analiz eder  
- temperature_service: Sıcaklık alarm sınırlarını ayarlar

Kullanım:
    ros2 run sensor_package temperature_publisher
    ros2 run sensor_package temperature_subscriber
    ros2 run sensor_package temperature_service
"""

__version__ = '0.0.1'
__author__ = 'ROS 2 Öğrencisi'
__email__ = 'test@test.com'

# Paket bilgileri
PACKAGE_NAME = 'sensor_package'
ROS_VERSION = 'ROS 2 Jazzy'

# Paket modüllerini içe aktar (isteğe bağlı)
try:
    from . import temperature_publisher
    from . import temperature_subscriber
    from . import temperature_service
except ImportError:
    # Modüller henüz yüklenmemişse sessizce geç
    pass