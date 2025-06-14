
Bu proje publisher/subscriber kavramlarını, servis/client yapısını ve temel ROS 2 araçlarını gösterecek.

## Proje: Robot Sensör Simülasyonu

Bir robot sensörü simüle eden, sıcaklık verisi yayınlayan ve bu veriyi işleyen bir sistem oluşturacağız.

### Dizin Yapısı
```
ros2_sensor_ws/
├── src/
│   └── sensor_package/
│       ├── sensor_package/
│       │   ├── __init__.py
│       │   ├── temperature_publisher.py
│       │   ├── temperature_subscriber.py
│       │   └── temperature_service.py
│       ├── resource/
│       │   └── sensor_package
│       ├── test/
│       │   ├── test_copyright.py
│       │   ├── test_flake8.py
│       │   └── test_pep257.py
│       ├── package.xml
│       └── setup.py
├── install/
├── build/
└── log/
```
### Adım 1: Workspace Oluşturma

```bash
# Workspace dizini oluştur
mkdir -p ~/ros2_sensor_ws/src
cd ~/ros2_sensor_ws

# Paket oluştur
cd src
ros2 pkg create --build-type ament_python sensor_package
```

### Adım 2: Package.xml Dosyası
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>sensor_package</name>
  <version>0.0.0</version>
  <description>ROS 2 öğrenmek için örnek sensör paketi</description>
  <maintainer email="test@test.com">developer</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>example_interfaces</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```
### Adım 3: Setup.py Dosyası
```python
from setuptools import setup

package_name = 'sensor_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='test@test.com',
    description='ROS 2 öğrenmek için örnek sensör paketi',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'temperature_publisher = sensor_package.temperature_publisher:main',
            'temperature_subscriber = sensor_package.temperature_subscriber:main',
            'temperature_service = sensor_package.temperature_service:main',
        ],
    },
)
```
### Adım 4: Temperature Publisher Node
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
import random
import time


class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        
        # Publisher oluştur - Temperature mesajları yayınlayacak
        self.publisher_ = self.create_publisher(
            Temperature, 
            'sensor/temperature', 
            10  # QoS (Quality of Service) - mesaj kuyruğu boyutu
        )
        
        # Timer oluştur - her 1 saniyede bir mesaj yayınla
        timer_period = 1.0  # saniye
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Başlangıç sıcaklığı
        self.base_temperature = 25.0
        
        self.get_logger().info('Sıcaklık sensörü başlatıldı...')

    def timer_callback(self):
        """Her timer tetiklendiğinde çalışır"""
        # Temperature mesajı oluştur
        msg = Temperature()
        
        # Header bilgileri
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'temperature_sensor'
        
        # Rastgele sıcaklık değeri üret (20-30 derece arası)
        temperature_variation = random.uniform(-5.0, 5.0)
        msg.temperature = self.base_temperature + temperature_variation
        
        # Varyans (sensör güvenilirliği)
        msg.variance = 0.1
        
        # Mesajı yayınla
        self.publisher_.publish(msg)
        
        # Log mesajı
        self.get_logger().info(f'Sıcaklık yayınlandı: {msg.temperature:.2f}°C')


def main(args=None):
    # ROS 2 Python client library'yi başlat
    rclpy.init(args=args)
    
    # Node oluştur
    temperature_publisher = TemperaturePublisher()
    
    try:
        # Node'u çalıştır (sonsuz döngü)
        rclpy.spin(temperature_publisher)
    except KeyboardInterrupt:
        pass
    
    # Temizlik işlemleri
    temperature_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
### Adım 5: Temperature Subscriber Node
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature


class TemperatureSubscriber(Node):
    def __init__(self):
        super().__init__('temperature_subscriber')
        
        # Subscriber oluştur - Temperature mesajlarını dinle
        self.subscription = self.create_subscription(
            Temperature,
            'sensor/temperature',
            self.temperature_callback,
            10  # QoS - mesaj kuyruğu boyutu
        )
        
        # Sıcaklık verilerini saklamak için listeler
        self.temperature_history = []
        self.max_history = 10  # Son 10 ölçümü sakla
        
        # Alarm sınırları
        self.min_temp = 15.0
        self.max_temp = 35.0
        
        self.get_logger().info('Sıcaklık abonesi başlatıldı...')

    def temperature_callback(self, msg):
        """Yeni sıcaklık mesajı geldiğinde çalışır"""
        current_temp = msg.temperature
        
        # Sıcaklık geçmişine ekle
        self.temperature_history.append(current_temp)
        if len(self.temperature_history) > self.max_history:
            self.temperature_history.pop(0)  # Eski veriyi sil
        
        # Ortalama sıcaklık hesapla
        avg_temp = sum(self.temperature_history) / len(self.temperature_history)
        
        # Log mesajı
        self.get_logger().info(
            f'Alınan sıcaklık: {current_temp:.2f}°C | '
            f'Ortalama: {avg_temp:.2f}°C | '
            f'Veri sayısı: {len(self.temperature_history)}'
        )
        
        # Alarm kontrolü
        self.check_temperature_alarm(current_temp)
        
        # Her 5 ölçümde bir istatistik göster
        if len(self.temperature_history) == self.max_history:
            self.show_statistics()

    def check_temperature_alarm(self, temperature):
        """Sıcaklık sınırlarını kontrol et"""
        if temperature < self.min_temp:
            self.get_logger().warn(f'⚠️  DÜŞÜK SICAKLIK ALARMI: {temperature:.2f}°C')
        elif temperature > self.max_temp:
            self.get_logger().warn(f'⚠️  YÜKSEK SICAKLIK ALARMI: {temperature:.2f}°C')

    def show_statistics(self):
        """Sıcaklık istatistiklerini göster"""
        if not self.temperature_history:
            return
            
        min_temp = min(self.temperature_history)
        max_temp = max(self.temperature_history)
        avg_temp = sum(self.temperature_history) / len(self.temperature_history)
        
        self.get_logger().info(
            f'📊 İstatistikler - Min: {min_temp:.2f}°C | '
            f'Max: {max_temp:.2f}°C | Ortalama: {avg_temp:.2f}°C'
        )


def main(args=None):
    # ROS 2 Python client library'yi başlat
    rclpy.init(args=args)
    
    # Node oluştur
    temperature_subscriber = TemperatureSubscriber()
    
    try:
        # Node'u çalıştır
        rclpy.spin(temperature_subscriber)
    except KeyboardInterrupt:
        pass
    
    # Temizlik işlemleri
    temperature_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
### Adım 6: Temperature Service Node
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from sensor_msgs.msg import Temperature


class TemperatureService(Node):
    def __init__(self):
        super().__init__('temperature_service')
        
        # Service oluştur - sıcaklık alarm sınırlarını ayarlamak için
        self.srv = self.create_service(
            AddTwoInts, 
            'set_temperature_limits', 
            self.set_temperature_limits_callback
        )
        
        # Subscriber - mevcut sıcaklığı takip etmek için
        self.subscription = self.create_subscription(
            Temperature,
            'sensor/temperature',
            self.temperature_callback,
            10
        )
        
        # Mevcut değerler
        self.current_temperature = 0.0
        self.min_limit = 15.0
        self.max_limit = 35.0
        
        self.get_logger().info('Sıcaklık servisi başlatıldı...')
        self.get_logger().info(f'Mevcut sınırlar: Min={self.min_limit}°C, Max={self.max_limit}°C')

    def temperature_callback(self, msg):
        """Mevcut sıcaklığı güncelle"""
        self.current_temperature = msg.temperature

    def set_temperature_limits_callback(self, request, response):
        """
        Servis callback - sıcaklık sınırlarını ayarla
        request.a = minimum sıcaklık
        request.b = maksimum sıcaklık
        response.sum = durum kodu (0=başarılı, -1=hata)
        """
        min_temp = float(request.a)
        max_temp = float(request.b)
        
        # Giriş kontrolü
        if min_temp >= max_temp:
            self.get_logger().error(
                f'Hatalı sınırlar: Min({min_temp}) >= Max({max_temp})'
            )
            response.sum = -1  # Hata kodu
            return response
        
        if min_temp < -50 or max_temp > 100:
            self.get_logger().error(
                f'Sınırlar gerçekçi değil: Min={min_temp}, Max={max_temp}'
            )
            response.sum = -1  # Hata kodu
            return response
        
        # Sınırları güncelle
        self.min_limit = min_temp
        self.max_limit = max_temp
        
        self.get_logger().info(
            f'Yeni sıcaklık sınırları ayarlandı: Min={min_temp}°C, Max={max_temp}°C'
        )
        
        # Mevcut sıcaklığı kontrol et
        self.check_current_temperature()
        
        response.sum = 0  # Başarı kodu
        return response

    def check_current_temperature(self):
        """Mevcut sıcaklığın yeni sınırlara uygun olup olmadığını kontrol et"""
        if self.current_temperature < self.min_limit:
            self.get_logger().warn(
                f'🚨 Mevcut sıcaklık ({self.current_temperature:.2f}°C) '
                f'minimum sınırın ({self.min_limit}°C) altında!'
            )
        elif self.current_temperature > self.max_limit:
            self.get_logger().warn(
                f'🚨 Mevcut sıcaklık ({self.current_temperature:.2f}°C) '
                f'maksimum sınırın ({self.max_limit}°C) üstünde!'
            )
        else:
            self.get_logger().info(
                f'✅ Mevcut sıcaklık ({self.current_temperature:.2f}°C) sınırlar içinde'
            )


def main(args=None):
    rclpy.init(args=args)
    
    temperature_service = TemperatureService()
    
    try:
        rclpy.spin(temperature_service)
    except KeyboardInterrupt:
        pass
    
    temperature_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
### Adım 7: **init**.py Dosyası
```python
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
```
### Adım 8: Projeyi Derleme ve Çalıştırma

```bash
# Workspace dizinine git
cd ~/ros2_sensor_ws

# Projeyi derle
colcon build --packages-select sensor_package

# Setup script'i çalıştır
source install/setup.bash

# ROS 2 environment'ı kontrol et
echo $ROS_DISTRO  # jazzy çıkmalı
```

### Adım 9: Uygulamayı Test Etme

**Terminal 1 - Publisher:**

```bash
cd ~/ros2_sensor_ws
source install/setup.bash
ros2 run sensor_package temperature_publisher
```

**Terminal 2 - Subscriber:**

```bash
cd ~/ros2_sensor_ws
source install/setup.bash
ros2 run sensor_package temperature_subscriber
```

**Terminal 3 - Service:**

```bash
cd ~/ros2_sensor_ws
source install/setup.bash
ros2 run sensor_package temperature_service
```

**Terminal 4 - Service Test:**

```bash
# Sıcaklık sınırlarını değiştir (min=10, max=40)
source install/setup.bash
ros2 service call /set_temperature_limits example_interfaces/srv/AddTwoInts "{a: 10, b: 40}"
```

### ROS 2 Araçları ile Analiz

```bash
# Aktif node'ları listele
ros2 node list

# Topic'leri listele
ros2 topic list

# Topic'teki mesajları dinle
ros2 topic echo /sensor/temperature

# Service'leri listele
ros2 service list

# Topic hızını ölç
ros2 topic hz /sensor/temperature

# Node bilgilerini görüntüle
ros2 node info /temperature_publisher
```

### Öğrenilen ROS 2 Kavramları

1. **Node**: Bağımsız çalışan süreçler
2. **Publisher/Subscriber**: Asenkron mesaj iletişimi
3. **Service/Client**: Senkron istek-yanıt iletişimi
4. **Topic**: Mesajların yayınlandığı kanallar
5. **Message**: Yapılandırılmış veri tipleri
6. **QoS**: Mesaj kalitesi ayarları
7. **Timer**: Periyodik görevler
8. **Callback**: Olay tetikli fonksiyonlar

Bu proje ROS 2'nin temel yapılarını anlamanız için ideal bir başlangıç noktası. Her dosyayı inceleyip çalıştırarak ROS 2'nin nasıl çalıştığını görebilirsiniz.



