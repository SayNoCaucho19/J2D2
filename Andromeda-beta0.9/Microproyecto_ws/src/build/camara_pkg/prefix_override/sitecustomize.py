import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mich-amu/uaows/src/J2D2/Microproyecto_ws/src/install/camara_pkg'
