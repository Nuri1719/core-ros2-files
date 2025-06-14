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