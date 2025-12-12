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
        self.declare_parameter('speed_multiplier_forward', 0.3)   # Multiplicador adelante
        self.declare_parameter('speed_multiplier_reverse', 0.2)   # Multiplicador reversa

        # Obtener parámetros
        self.mqtt_broker = self.get_parameter('mqtt_broker').value
        self.mqtt_port = self.get_parameter('mqtt_port').value
        self.mqtt_topic = self.get_parameter('mqtt_topic').value
        self.mqtt_status_topic = self.get_parameter('mqtt_status_topic').value
        self.max_speed = self.get_parameter('max_speed').value
        self.deadzone = self.get_parameter('deadzone').value
        self.speed_multiplier_forward = self.get_parameter('speed_multiplier_forward').value
        self.speed_multiplier_reverse = self.get_parameter('speed_multiplier_reverse').value
        
        # Variables de estado
        self.left_motor = 0.0
        self.right_motor = 0.0
        self.last_joy_time = time.time()
        self.connection_active = False
        self.esp32_online = False
        self.last_esp32_ping = time.time()
        
        # Control dinámico de velocidad (separado para adelante/atrás)
        self.current_multiplier_forward = self.speed_multiplier_forward
        self.current_multiplier_reverse = self.speed_multiplier_reverse
        self.min_multiplier = 0.1
        self.max_multiplier = 1.0
        self.multiplier_step = 0.05
        
        # Publisher para RViz
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Suscriptor al joystick
        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Configurar cliente MQTT
        self.setup_mqtt()
        
        # Timers
        self.timeout_timer = self.create_timer(0.1, self.timeout_check)
        self.send_timer = self.create_timer(0.05, self.send_commands)
        self.heartbeat_timer = self.create_timer(2.0, self.check_esp32_heartbeat)
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('🎮 Nodo Joystick to MQTT iniciado (Control de Velocidad)')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'📡 Broker MQTT: {self.mqtt_broker}:{self.mqtt_port}')
        self.get_logger().info(f'📤 Topic comandos: {self.mqtt_topic}')
        self.get_logger().info(f'📥 Topic status: {self.mqtt_status_topic}')
        self.get_logger().info(f'🕹️  Suscrito a: /joy')
        self.get_logger().info(f'🚗 Publicando a: /cmd_vel')
        self.get_logger().info(f'⚡ Max speed: {self.max_speed}')
        self.get_logger().info(f'🎯 Deadzone: {self.deadzone}')
        self.get_logger().info(f'🔧 Multiplicador adelante: {self.current_multiplier_forward:.2f}')
        self.get_logger().info(f'🔧 Multiplicador reversa: {self.current_multiplier_reverse:.2f}')
        self.get_logger().info('')
        self.get_logger().info('CONTROLES:')
        self.get_logger().info('  • Stick Izquierdo (Y): Adelante/Atrás')
        self.get_logger().info('  • Stick Derecho (X): Giro Izquierda/Derecha')
        self.get_logger().info('  • R1 (Botón 5): Aumentar velocidad (+5%)')
        self.get_logger().info('  • L1 (Botón 4): Disminuir velocidad (-5%)')
        self.get_logger().info('  • A/X (Botón 0): Parada de emergencia')
        self.get_logger().info('=' * 70)

    def setup_mqtt(self):
        """Configurar y conectar cliente MQTT"""
        self.mqtt_client = mqtt.Client(client_id="ros2_joystick_controller")
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        try:
            self.get_logger().info(f'🔌 Conectando a broker MQTT...')
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f'❌ Error conectando a MQTT broker: {e}')
            self.get_logger().warn('⚠️  Continuando sin MQTT (solo RViz)')

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback cuando se conecta a MQTT"""
        if rc == 0:
            self.connection_active = True
            self.get_logger().info('✅ Conectado al broker MQTT')
            self.mqtt_client.subscribe(self.mqtt_status_topic)
            self.get_logger().info(f'📥 Suscrito a: {self.mqtt_status_topic}')
        else:
            self.get_logger().error(f'❌ Fallo en conexión MQTT. Código: {rc}')
            self.connection_active = False

    def on_mqtt_disconnect(self, client, userdata, rc):
        """Callback cuando se desconecta de MQTT"""
        self.connection_active = False
        self.esp32_online = False
        self.get_logger().warn('⚠️  Desconectado del broker MQTT')

    def on_mqtt_message(self, client, userdata, msg):
        """Callback para mensajes MQTT recibidos"""
        try:
            if msg.topic == self.mqtt_status_topic:
                payload = json.loads(msg.payload.decode())
                if payload.get('type') == 'heartbeat':
                    if not self.esp32_online:
                        self.get_logger().info('✅ ESP32 detectado y online')
                    self.esp32_online = True
                    self.last_esp32_ping = time.time()
        except Exception as e:
            self.get_logger().error(f'❌ Error procesando mensaje MQTT: {e}')

    def joy_callback(self, msg):
        """Procesar señales del joystick"""
        self.last_joy_time = time.time()
        
        # Verificar que tengamos suficientes ejes y botones
        if len(msg.axes) < 4:
            self.get_logger().warn('⚠️  Joystick sin suficientes ejes', throttle_duration_sec=2.0)
            return
        
        # ===== CONTROL DE VELOCIDAD CON BOTONES =====
        if len(msg.buttons) >= 6:
            # R1 (botón 5): Aumentar velocidad
            if msg.buttons[5]:
                old_fwd = self.current_multiplier_forward
                old_rev = self.current_multiplier_reverse
                
                self.current_multiplier_forward = min(
                    self.current_multiplier_forward + self.multiplier_step, 
                    self.max_multiplier
                )
                self.current_multiplier_reverse = min(
                    self.current_multiplier_reverse + self.multiplier_step, 
                    self.max_multiplier
                )
                
                if self.current_multiplier_forward != old_fwd:
                    self.get_logger().info(
                        f'⬆️  Velocidad aumentada: Fwd={self.current_multiplier_forward:.0%} Rev={self.current_multiplier_reverse:.0%}',
                        throttle_duration_sec=0.3
                    )
            
            # L1 (botón 4): Disminuir velocidad
            if msg.buttons[4]:
                old_fwd = self.current_multiplier_forward
                old_rev = self.current_multiplier_reverse
                
                self.current_multiplier_forward = max(
                    self.current_multiplier_forward - self.multiplier_step, 
                    self.min_multiplier
                )
                self.current_multiplier_reverse = max(
                    self.current_multiplier_reverse - self.multiplier_step, 
                    self.min_multiplier
                )
                
                if self.current_multiplier_forward != old_fwd:
                    self.get_logger().info(
                        f'⬇️  Velocidad reducida: Fwd={self.current_multiplier_forward:.0%} Rev={self.current_multiplier_reverse:.0%}',
                        throttle_duration_sec=0.3
                    )
        
        # Leer ejes del joystick
        stick_y = msg.axes[1]  # Avance/retroceso (Stick izquierdo vertical)
        stick_x = msg.axes[3]  # Giro (Stick derecho horizontal)
        
        # Aplicar zona muerta
        if abs(stick_x) < self.deadzone:
            stick_x = 0.0
        if abs(stick_y) < self.deadzone:
            stick_y = 0.0

        # Aplicar multiplicador de velocidad según dirección
        # Si v > 0 (adelante): usar multiplicador de adelante
        # Si v < 0 (reversa): usar multiplicador de reversa
        if stick_y > 0:
            v = stick_y * self.max_speed * self.current_multiplier_forward
        elif stick_y < 0:
            v = stick_y * self.max_speed * self.current_multiplier_reverse
        else:
            v = 0.0
        
        # El giro usa el multiplicador promedio
        avg_multiplier = (self.current_multiplier_forward + self.current_multiplier_reverse) / 2.0
        w = stick_x * self.max_speed * avg_multiplier

        # ===== CÁLCULO DE MOTORES =====
        # Conversión a velocidades de motores diferenciales
        wheel_base = 0.3  # distancia entre hélices en metros
        self.left_motor = v - (w * wheel_base / 2.0)
        self.right_motor = v + (w * wheel_base / 2.0)

        # Normalizar si exceden el rango [-1.0, 1.0]
        max_val = max(abs(self.left_motor), abs(self.right_motor))
        if max_val > 1.0:
            self.left_motor /= max_val
            self.right_motor /= max_val

        # Publicar en /cmd_vel SIEMPRE (para RViz y simulación)
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.cmd_vel_pub.publish(twist)

        # Botón de parada de emergencia (botón A/X)
        if len(msg.buttons) > 0 and msg.buttons[0]:
            self.emergency_stop()
            return

        # Log de estado (solo si hay movimiento)
        if abs(v) > 0.01 or abs(w) > 0.01:
            mqtt_icon = "🟢" if self.connection_active else "🔴"
            esp_icon = "🟢" if self.esp32_online else "🔴"
            
            # Mostrar multiplicador actual según dirección
            current_mult = self.current_multiplier_forward if v >= 0 else self.current_multiplier_reverse
            direction = "FWD" if v >= 0 else "REV"
            
            self.get_logger().info(
                f'{mqtt_icon}MQTT {esp_icon}ESP32 | {direction}:{current_mult:.0%} | '
                f'Lin:{v:+.2f} Ang:{w:+.2f} | L:{self.left_motor:+.2f} R:{self.right_motor:+.2f}',
                throttle_duration_sec=0.5
            )

    def send_commands(self):
        """Enviar comandos por MQTT"""
        if not self.connection_active:
            return

        try:
            # Crear mensaje JSON con valores float de -1.0 a 1.0
            message = {
                "left": round(float(self.left_motor), 3),
                "right": round(float(self.right_motor), 3),
                "speed_mult_fwd": round(float(self.current_multiplier_forward), 2),
                "speed_mult_rev": round(float(self.current_multiplier_reverse), 2),
                "timestamp": time.time()
            }
            
            # Publicar con QoS 0 para menor latencia
            result = self.mqtt_client.publish(
                self.mqtt_topic,
                json.dumps(message),
                qos=0
            )
            
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                self.get_logger().error(
                    f'❌ Error publicando a MQTT: {result.rc}',
                    throttle_duration_sec=1.0
                )
                
        except Exception as e:
            self.get_logger().error(
                f'❌ Error enviando comando MQTT: {e}',
                throttle_duration_sec=1.0
            )

    def timeout_check(self):
        """Detener motores si no hay señal del joystick"""
        time_since_joy = time.time() - self.last_joy_time
        
        if time_since_joy > 0.5:
            if self.left_motor != 0.0 or self.right_motor != 0.0:
                self.get_logger().warn('⚠️  Timeout del joystick - Deteniendo motores')
                self.left_motor = 0.0
                self.right_motor = 0.0
                
                # Publicar parada en cmd_vel
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                
                # Enviar parada por MQTT si está conectado
                if self.connection_active:
                    self.send_commands()

    def check_esp32_heartbeat(self):
        """Verificar si el ESP32 sigue respondiendo"""
        if self.connection_active:
            time_since_ping = time.time() - self.last_esp32_ping
            if time_since_ping > 5.0:
                if self.esp32_online:
                    self.esp32_online = False
                    self.get_logger().warn('⚠️  ESP32 no responde (sin heartbeat > 5s)')

    def emergency_stop(self):
        """Parada de emergencia"""
        self.get_logger().warn('🚨 ¡PARADA DE EMERGENCIA ACTIVADA!')
        
        # Detener motores
        self.left_motor = 0.0
        self.right_motor = 0.0
        
        # Publicar parada en cmd_vel
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        # Enviar parada por MQTT si está disponible
        if self.connection_active:
            try:
                stop_msg = json.dumps({
                    "left": 0.0,
                    "right": 0.0,
                    "emergency": True
                })
                # Enviar 3 veces para asegurar recepción
                for _ in range(3):
                    self.mqtt_client.publish(self.mqtt_topic, stop_msg, qos=1)
                    time.sleep(0.02)
            except Exception as e:
                self.get_logger().error(f'❌ Error en parada de emergencia MQTT: {e}')

    def destroy_node(self):
        """Limpiar recursos al cerrar"""
        self.get_logger().info('🛑 Cerrando nodo...')
        
        # Parada de emergencia antes de cerrar
        self.emergency_stop()
        
        # Cerrar MQTT
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
        node.get_logger().info('🛑 Interrupción por teclado')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()