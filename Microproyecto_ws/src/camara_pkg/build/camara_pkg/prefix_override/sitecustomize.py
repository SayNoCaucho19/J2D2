import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jean/ros2_ws/src/camara_pkg/install/camara_pkg'
