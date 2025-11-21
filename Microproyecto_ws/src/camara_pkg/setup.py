#!/usr/bin/env python3

import os
from glob import glob
from setuptools import setup

package_name = 'camara_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),

        # URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),

        # Mesh files
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.STL')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.dae')),

        # Config files (RVIZ, YAML, etc.)
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='camilo',
    maintainer_email='tu_correo@example.com',
    description='Paquete ROS2 que incluye cámara, GUI, teleop y monitor de voltaje.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Nodos originales
            'camara_node = camara_pkg.camara_node:main',
            'camera_gui_node = camara_pkg.camera_gui_node:main',
            'guidos = camara_pkg.guidos:main',
            'salvatore_mqtt_bridge = camara_pkg.salvatore_mqtt_bridge:main',
            'teleop = camara_pkg.teleop:main',
            'movemotors = camara_pkg.movemotors:main',

            # Nodos del monitor de voltaje
            'mqtt_bridge_node = camara_pkg.mqtt_bridge_node:main',
           
        ],
    },
)
