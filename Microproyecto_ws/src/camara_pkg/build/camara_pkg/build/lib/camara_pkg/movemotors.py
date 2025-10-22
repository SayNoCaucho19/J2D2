#!/usr/bin/env python3
"""
Nodo que convierte comandos /cmd_vel en movimiento de articulaciones
para visualización en RViz
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math


class MoveMotors(Node):
    def __init__(self):
        super().__init__('move_motors')
        
        # Parámetros del robot
        self.declare_parameter('wheel_base', 0.3)  # Distancia entre hélices (m)
        self.declare_parameter('wheel_radius', 0.05)  # Radio de las hélices (m)
        
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        
        # Suscripción a cmd_vel
        self.sub = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.callback, 
            10
        )
        
        # Publicador de estados de articulaciones
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Variables de estado
        self.left_angle = 0.0
        self.right_angle = 0.0
        self.linear_x = 0.0
        self.angular_z = 0.0
        
        # Timer para actualizar a 20Hz
        self.timer = self.create_timer(0.05, self.update)
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('Nodo MoveMotors iniciado correctamente')
        self.get_logger().info(f'wheel_base = {self.wheel_base} m')
        self.get_logger().info(f'wheel_radius = {self.wheel_radius} m')
        self.get_logger().info('Esperando mensajes en /cmd_vel...')
        self.get_logger().info('=' * 50)

    def callback(self, msg):
        """Recibe comandos de velocidad"""
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z
        
        self.get_logger().info(
            f'📥 Recibido cmd_vel: linear.x={msg.linear.x:.3f} m/s, '
            f'angular.z={msg.angular.z:.3f} rad/s',
            throttle_duration_sec=1.0
        )

    def update(self):
        """Actualiza posiciones de las articulaciones"""
        dt = 0.05
        
        # Cálculo correcto de velocidades diferenciales
        # v = (v_left + v_right) / 2
        # w = (v_right - v_left) / wheel_base
        # Despejando:
        left_speed = self.linear_x - (self.angular_z * self.wheel_base / 2.0)
        right_speed = self.linear_x + (self.angular_z * self.wheel_base / 2.0)
        
        # Convertir velocidad lineal a velocidad angular de las ruedas/hélices
        # omega = v / r
        left_angular_vel = left_speed / self.wheel_radius
        right_angular_vel = right_speed / self.wheel_radius
        
        # Integrar para obtener posición angular
        self.left_angle += left_angular_vel * dt
        self.right_angle += right_angular_vel * dt
        
        # Crear mensaje JointState
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()  # ¡CRÍTICO para RViz!
        js.header.frame_id = ''
        # Nombres exactos del URDF: turtlesimm.urdf
        js.name = ['Motorizq_joint', 'Motorder_joint']  # Izquierdo, Derecho
        js.position = [self.left_angle, self.right_angle]
        js.velocity = [left_angular_vel, right_angular_vel]
        js.effort = []
        
        # Publicar
        self.pub.publish(js)
        
        # Log para depuración (solo cuando hay movimiento)
        if abs(self.linear_x) > 0.01 or abs(self.angular_z) > 0.01:
            self.get_logger().info(
                f'🔄 Publicando: L={math.degrees(self.left_angle):.1f}° '
                f'({left_angular_vel:.2f} rad/s) | '
                f'R={math.degrees(self.right_angle):.1f}° '
                f'({right_angular_vel:.2f} rad/s)',
                throttle_duration_sec=0.5
            )


def main(args=None):
    """Función principal"""
    rclpy.init(args=args)
    node = MoveMotors()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo detenido por el usuario')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()