import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/camilo/J2D2/Microproyecto_ws/install/camara_pkg'
