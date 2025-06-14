import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/cnosmn/Desktop/ros2_sensor_ws/install/sensor_package'
