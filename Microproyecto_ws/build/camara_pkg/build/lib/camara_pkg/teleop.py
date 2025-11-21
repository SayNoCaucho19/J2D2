#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import paho.mqtt.client as mqtt
import json
import time

class JoystickToSerial(Node):
    def __init__(self):
        super().__init__('joystick_to_serial')
        
        # Parámetros configurables
        self.declare_parameter('mqtt_broker', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_topic', 'aquabot/motors')
        self.declare_parameter('mqtt_status_topic', 'aquabot/status')
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('min_ratio', 0.3)

        # Obtener parámetros
        self.mqtt_broker = self.get_parameter('mqtt_broker').value
        self.mqtt_port = self.get_parameter('mqtt_port').value
        self.mqtt_topic = self.get_parameter('mqtt_topic').value
        self.mqtt_status_topic = self.get_parameter('mqtt_status_topic').value
        self.max_speed = self.get_parameter('max_speed').value
        self.deadzone = self.get_parameter('deadzone').value
        self.min_ratio = self.get_parameter('min_ratio').value
        
        # Variables de estado
        self.left_motor = 0.0
        self.right_motor = 0.0
        self.last_joy_time = time.time()
        self.connection_active = False
        self.esp32_online = False
        self.last_esp32_ping = time.time()
        
        # Configurar cliente MQTT
        self.mqtt_client = mqtt.Client(client_id="ros2_joystick_controller")
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        # Conectar a broker MQTT
        try:
            self.get_logger().info(f'Conectando a broker MQTT: {self.mqtt_broker}:{self.mqtt_port}')
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f'Error conectando a MQTT broker: {e}')
        
        # Publisher para RViz
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Suscriptor al joystick
        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Timers
        self.timeout_timer = self.create_timer(0.1, self.timeout_check)
        self.send_timer = self.create_timer(0.05, self.send_commands)  # 20 Hz
        self.heartbeat_timer = self.create_timer(2.0, self.check_esp32_heartbeat)
        
        self.get_logger().info('Nodo Teleop iniciado con MQTT')

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback cuando se conecta a MQTT"""
        if rc == 0:
            self.connection_active = True
            self.get_logger().info('✓ Conectado al broker MQTT')
            # Suscribirse al topic de status del ESP32
            self.mqtt_client.subscribe(self.mqtt_status_topic)
            self.get_logger().info(f'Suscrito a: {self.mqtt_status_topic}')
        else:
            self.get_logger().error(f'Fallo en conexión MQTT. Código: {rc}')
            self.connection_active = False

    def on_mqtt_disconnect(self, client, userdata, rc):
        """Callback cuando se desconecta de MQTT"""
        self.connection_active = False
        self.esp32_online = False
        self.get_logger().warn('Desconectado del broker MQTT')

    def on_mqtt_message(self, client, userdata, msg):
        """Callback para mensajes MQTT recibidos"""
        try:
            if msg.topic == self.mqtt_status_topic:
                payload = json.loads(msg.payload.decode())
                if payload.get('type') == 'heartbeat':
                    self.esp32_online = True
                    self.last_esp32_ping = time.time()
                    self.get_logger().info(
                        f"ESP32 online - Uptime: {payload.get('uptime', 0)}s, WiFi: {payload.get('wifi_rssi', 0)}dBm",
                        throttle_duration_sec=5.0
                    )
        except Exception as e:
            self.get_logger().error(f'Error procesando mensaje MQTT: {e}')

    def joy_callback(self, msg):
        """Procesar señales del joystick"""
        self.last_joy_time = time.time()
        
        if len(msg.axes) < 4:
            return
        
        stick_y = msg.axes[1]  # Avance/retroceso
        stick_x = msg.axes[3]  # Giro

        # Zona muerta
        if abs(stick_x) < self.deadzone:
            stick_x = 0.0
        if abs(stick_y) < self.deadzone:
            stick_y = 0.0

        # Cálculo de velocidades
        v = stick_y * self.max_speed
        w = stick_x * self.max_speed

        # Conversión a velocidades de motores diferenciales
        wheel_base = 0.3  # distancia entre hélices en metros
        self.left_motor = v - (w * wheel_base / 2.0)
        self.right_motor = v + (w * wheel_base / 2.0)

        # Normalizar si exceden el rango [-1.0, 1.0]
        max_val = max(abs(self.left_motor), abs(self.right_motor))
        if max_val > 1.0:
            self.left_motor /= max_val
            self.right_motor /= max_val

        # Publicar en /cmd_vel para sincronizar con RViz
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_vel_pub.publish(twist)

        # Parada de emergencia
        if len(msg.buttons) > 0 and msg.buttons[0]:
            self.emergency_stop()

        status = "🟢 MQTT" if self.connection_active else "🔴 MQTT OFF"
        esp_status = "🟢 ESP32" if self.esp32_online else "🔴 ESP32 OFF"
        
        self.get_logger().info(
            f'{status} | {esp_status} | v={v:.2f} m/s | w={w:.2f} rad/s | L={self.left_motor:.3f} | R={self.right_motor:.3f}',
            throttle_duration_sec=0.5
        )

    def send_commands(self):
        """Enviar comandos por MQTT"""
        if not self.connection_active:
            return

        try:
            # Crear mensaje JSON con valores float de -1.0 a 1.0
            message = {
                "left": round(self.left_motor, 3),
                "right": round(self.right_motor, 3),
                "timestamp": time.time()
            }
            
            # Publicar con QoS 0 para menor latencia
            result = self.mqtt_client.publish(
                self.mqtt_topic,
                json.dumps(message),
                qos=0  # QoS 0 = más rápido, sin confirmación
            )
            
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                self.get_logger().error(f'Error publicando a MQTT: {result.rc}')
                
        except Exception as e:
            self.get_logger().error(f'Error enviando comando MQTT: {e}')

    def timeout_check(self):
        """Detener motores si no hay señal"""
        if time.time() - self.last_joy_time > 1.0:
            if self.left_motor != 0 or self.right_motor != 0:
                self.get_logger().warn('⚠️  Timeout del joystick - Deteniendo motores')
                self.left_motor = 0.0
                self.right_motor = 0.0
                self.send_commands()

    def check_esp32_heartbeat(self):
        """Verificar si el ESP32 sigue respondiendo"""
        if self.connection_active:
            time_since_ping = time.time() - self.last_esp32_ping
            if time_since_ping > 5.0 and self.esp32_online:
                self.esp32_online = False
                self.get_logger().warn('⚠️  ESP32 no responde (sin heartbeat)')

    def emergency_stop(self):
        """Parada de emergencia"""
        self.get_logger().warn('🚨 ¡PARADA DE EMERGENCIA!')
        self.left_motor = 0.0
        self.right_motor = 0.0
        
        if self.connection_active:
            try:
                # Enviar múltiples comandos de parada
                stop_msg = json.dumps({"left": 0.0, "right": 0.0, "emergency": True})
                for _ in range(3):
                    self.mqtt_client.publish(self.mqtt_topic, stop_msg, qos=2)
                    time.sleep(0.05)
            except Exception as e:
                self.get_logger().error(f'Error en parada de emergencia: {e}')

    def destroy_node(self):
        """Limpiar recursos al cerrar"""
        self.get_logger().info('Cerrando nodo...')
        self.emergency_stop()
        if self.connection_active:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JoystickToSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()