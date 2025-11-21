import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Obtener el directorio del paquete
    pkg_share = get_package_share_directory('camara_pkg')
    
    # Ruta al archivo URDF
    urdf_file = os.path.join(pkg_share, 'urdf', 'turtlesimm.urdf')
    
    # Verificar que el archivo existe
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")
    
    # Leer el contenido del URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Argumentos de lanzamiento
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # ============================================================
    # Nodo robot_state_publisher - Publica las transformaciones
    # ============================================================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time,
            'publish_frequency': 20.0  # Hz
        }]
    )
    
    # ============================================================
    # Nodo teleop - Control con joystick
    # ============================================================
    teleop_node = Node(
        package='camara_pkg',
        executable='teleop',
        name='teleop_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'baud_rate': 115200,
            'max_speed': 0.4,
            'deadzone': 0.1
        }]
    )
    
    # ============================================================
    # Nodo guidos - Comunicación con Arduino
    # ============================================================
    guidos_node = Node(
        package='camara_pkg',
        executable='guidos',
        name='guidos_node',
        output='screen'
    )
    
    # ============================================================
    # Nodo movemotors - Actualiza joint_states para RViz
    # ============================================================
    movemotors_node = Node(
        package='camara_pkg',
        executable='movemotors',
        name='movemotors_node',
        output='screen',
        parameters=[{
            'wheel_base': 0.3,      # Ajusta según tu robot (metros)
            'wheel_radius': 0.05    # Ajusta según tu robot (metros)
        }]
    )
    
    # ============================================================
    # Nodo RVIZ2 - Visualización
    # ============================================================
    rviz_config = os.path.join(pkg_share, 'config', 'view_robot.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # ============================================================
    # Retornar Launch Description
    # ============================================================
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        robot_state_publisher_node,
        # joint_state_publisher_gui,  # ❌ COMENTADO - Conflicto con movemotors
        teleop_node,
        guidos_node,
        movemotors_node,  # ✅ AHORA SÍ INCLUIDO
        rviz_node
    ])