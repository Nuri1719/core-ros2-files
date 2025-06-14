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