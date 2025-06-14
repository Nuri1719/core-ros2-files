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