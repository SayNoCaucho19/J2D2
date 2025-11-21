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
        self.declare_parameter('mqtt_broker', 'test.mosquitto.org')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_topic', 'aquabot/motors')
        self.declare_parameter('mqtt_status_topic', 'aquabot/status')
        self.declare_parameter('max_speed', 0.25)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('turn_deadzone', 0.05)
        self.declare_parameter('wheel_base', 0.3)
        
        # PARÁMETROS DE CALIBRACIÓN
        self.declare_parameter('left_trim', 0.0)
        self.declare_parameter('right_trim', 0.0)
        self.declare_parameter('left_gain', 0.25)
        self.declare_parameter('right_gain', 1.0)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        
        # PARÁMETROS DE ACELERACIÓN SUAVE
        self.declare_parameter('acceleration_time', 2.0)
        self.declare_parameter('deceleration_time', 1.0)
        self.declare_parameter('force_equal_speed', True)

        # Obtener parámetros
        self.mqtt_broker = self.get_parameter('mqtt_broker').value
        self.mqtt_port = self.get_parameter('mqtt_port').value
        self.mqtt_topic = self.get_parameter('mqtt_topic').value
        self.mqtt_status_topic = self.get_parameter('mqtt_status_topic').value
        self.max_speed = self.get_parameter('max_speed').value
        self.deadzone = self.get_parameter('deadzone').value
        self.turn_deadzone = self.get_parameter('turn_deadzone').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.left_trim = self.get_parameter('left_trim').value
        self.right_trim = self.get_parameter('right_trim').value
        self.left_gain = self.get_parameter('left_gain').value
        self.right_gain = self.get_parameter('right_gain').value
        self.invert_left = self.get_parameter('invert_left').value
        self.invert_right = self.get_parameter('invert_right').value
        self.acceleration_time = self.get_parameter('acceleration_time').value
        self.deceleration_time = self.get_parameter('deceleration_time').value
        self.force_equal_speed = self.get_parameter('force_equal_speed').value
        
        # Variables de estado
        self.left_motor_target = 0.0
        self.right_motor_target = 0.0
        self.left_motor_current = 0.0
        self.right_motor_current = 0.0
        self.last_joy_time = time.time()
        self.last_update_time = time.time()
        self.connection_active = False
        self.esp32_online = False
        self.last_esp32_ping = time.time()
        
        # Estadísticas para debug
        self.debug_counter = 0
        self.last_left_pulse = 0
        self.last_right_pulse = 0
        
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
        self.send_timer = self.create_timer(0.05, self.send_commands)
        self.heartbeat_timer = self.create_timer(2.0, self.check_esp32_heartbeat)
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('Nodo Teleop - MODO ARRANQUE SUAVE + CALIBRACIÓN')
        self.get_logger().info(f'  Acceleration time: {self.acceleration_time}s')
        self.get_logger().info(f'  Deceleration time: {self.deceleration_time}s')
        self.get_logger().info(f'  Left gain: {self.left_gain:.3f} | Right gain: {self.right_gain:.3f}')
        self.get_logger().info(f'  Left trim: {self.left_trim:+.3f} | Right trim: {self.right_trim:+.3f}')
        self.get_logger().info(f'  Force equal speed: {self.force_equal_speed}')
        self.get_logger().info('=' * 70)

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback cuando se conecta a MQTT"""
        if rc == 0:
            self.connection_active = True
            self.get_logger().info('✓ Conectado al broker MQTT')
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
                    
                    if 'left_pulse' in payload and 'right_pulse' in payload:
                        self.last_left_pulse = payload['left_pulse']
                        self.last_right_pulse = payload['right_pulse']
                    
                    self.get_logger().info(
                        f"ESP32 online - Uptime: {payload.get('uptime', 0)}s, "
                        f"WiFi: {payload.get('wifi_rssi', 0)}dBm, "
                        f"Pulsos: L={self.last_left_pulse}µs R={self.last_right_pulse}µs",
                        throttle_duration_sec=5.0
                    )
        except Exception as e:
            self.get_logger().error(f'Error procesando mensaje MQTT: {e}')

    def apply_deadzone(self, value, deadzone):
        """Aplicar zona muerta con rampa suave"""
        if abs(value) < deadzone:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - deadzone) / (1.0 - deadzone)

    def smooth_acceleration(self, current, target, dt, accel_time, decel_time):
        """Aplicar aceleración/deceleración suave"""
        error = target - current
        
        if abs(error) < 0.001:
            return target
        
        # Determinar si está acelerando o frenando
        if abs(target) > abs(current):
            # Acelerando
            max_change = dt / accel_time
        else:
            # Frenando
            max_change = dt / decel_time
        
        # Limitar el cambio
        if abs(error) > max_change:
            return current + (max_change if error > 0 else -max_change)
        else:
            return target

    def joy_callback(self, msg):
        """Procesar señales del joystick - CON ARRANQUE SUAVE"""
        self.last_joy_time = time.time()
        
        if len(msg.axes) < 4:
            return
        
        stick_y = msg.axes[1]
        stick_x = msg.axes[3]

        stick_y = self.apply_deadzone(stick_y, self.deadzone)
        stick_x = self.apply_deadzone(stick_x, self.turn_deadzone)

        # ========== CALCULAR VELOCIDADES OBJETIVO ==========
        if self.force_equal_speed and abs(stick_x) < self.turn_deadzone:
            # MODO LÍNEA RECTA: Ambos motores iguales
            speed = stick_y * self.max_speed
            left_target = speed
            right_target = speed
            mode = "RECTA"
        else:
            # MODO GIRO: Velocidad diferencial
            v = stick_y * self.max_speed
            w = stick_x * self.max_speed
            
            left_target = v - (w * self.wheel_base / 2.0)
            right_target = v + (w * self.wheel_base / 2.0)
            
            max_val = max(abs(left_target), abs(right_target))
            if max_val > 1.0:
                left_target /= max_val
                right_target /= max_val
            
            mode = "GIRO"

        # ========== APLICAR GANANCIA INDIVIDUAL POR MOTOR ==========
        # Esto permite compensar motores con diferente potencia
        left_target *= self.left_gain
        right_target *= self.right_gain

        # Aplicar trim (offset constante)
        left_target += self.left_trim
        right_target += self.right_trim

        # Invertir si es necesario
        if self.invert_left:
            left_target = -left_target
        if self.invert_right:
            right_target = -right_target

        # Limitar a rango válido
        left_target = max(-1.0, min(1.0, left_target))
        right_target = max(-1.0, min(1.0, right_target))

        # Guardar objetivos
        self.left_motor_target = left_target
        self.right_motor_target = right_target

        # Publicar en /cmd_vel
        twist = Twist()
        twist.linear.x = stick_y * self.max_speed
        twist.angular.z = stick_x * self.max_speed
        self.cmd_vel_pub.publish(twist)

        # Parada de emergencia
        if len(msg.buttons) > 0 and msg.buttons[0]:
            self.emergency_stop()

        # Log detallado
        self.debug_counter += 1
        if self.debug_counter % 10 == 0:
            status = "🟢 MQTT" if self.connection_active else "🔴 MQTT OFF"
            esp_status = "🟢 ESP32" if self.esp32_online else "🔴 ESP32 OFF"
            
            self.get_logger().info(
                f'{status} | {esp_status} | {mode:5s} | '
                f'Joy: X={stick_x:+.2f} Y={stick_y:+.2f} | '
                f'Target: L={self.left_motor_target:+.3f} R={self.right_motor_target:+.3f} | '
                f'Current: L={self.left_motor_current:+.3f} R={self.right_motor_current:+.3f}'
            )

    def send_commands(self):
        """Enviar comandos por MQTT con aceleración suave"""
        if not self.connection_active:
            return

        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        # Aplicar aceleración suave a cada motor
        self.left_motor_current = self.smooth_acceleration(
            self.left_motor_current,
            self.left_motor_target,
            dt,
            self.acceleration_time,
            self.deceleration_time
        )

        self.right_motor_current = self.smooth_acceleration(
            self.right_motor_current,
            self.right_motor_target,
            dt,
            self.acceleration_time,
            self.deceleration_time
        )

        try:
            message = {
                "left": round(self.left_motor_current, 3),
                "right": round(self.right_motor_current, 3),
                "timestamp": time.time()
            }
            
            result = self.mqtt_client.publish(
                self.mqtt_topic,
                json.dumps(message),
                qos=0
            )
            
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                self.get_logger().error(f'Error publicando a MQTT: {result.rc}')
                
        except Exception as e:
            self.get_logger().error(f'Error enviando comando MQTT: {e}')

    def timeout_check(self):
        """Detener motores si no hay señal"""
        if time.time() - self.last_joy_time > 1.0:
            if self.left_motor_target != 0 or self.right_motor_target != 0:
                self.get_logger().warn('⚠️  Timeout del joystick - Deteniendo motores')
                self.left_motor_target = 0.0
                self.right_motor_target = 0.0

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
        self.left_motor_target = 0.0
        self.right_motor_target = 0.0
        self.left_motor_current = 0.0
        self.right_motor_current = 0.0
        
        if self.connection_active:
            try:
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