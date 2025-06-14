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