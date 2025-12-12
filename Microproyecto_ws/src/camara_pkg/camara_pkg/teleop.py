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
        
        # Multiplicadores separados por motor y dirección
        self.declare_parameter('left_motor_forward', 0.7)    # Motor izquierdo adelante
        self.declare_parameter('left_motor_reverse', 0.0)    # Motor izquierdo reversa
        self.declare_parameter('right_motor_forward', 0.7)   # Motor derecho adelante
        self.declare_parameter('right_motor_reverse', 0.0)   # Motor derecho reversa

        # Obtener parámetros
        self.mqtt_broker = self.get_parameter('mqtt_broker').value
        self.mqtt_port = self.get_parameter('mqtt_port').value
        self.mqtt_topic = self.get_parameter('mqtt_topic').value
        self.mqtt_status_topic = self.get_parameter('mqtt_status_topic').value
        self.max_speed = self.get_parameter('max_speed').value
        self.deadzone = self.get_parameter('deadzone').value
        
        # Multiplicadores individuales
        self.left_fwd = self.get_parameter('left_motor_forward').value
        self.left_rev = self.get_parameter('left_motor_reverse').value
        self.right_fwd = self.get_parameter('right_motor_forward').value
        self.right_rev = self.get_parameter('right_motor_reverse').value
        
        # Variables de estado
        self.left_motor = 0.0
        self.right_motor = 0.0
        self.last_joy_time = time.time()
        self.connection_active = False
        self.esp32_online = False
        self.last_esp32_ping = time.time()
        
        # Control dinámico de velocidad
        self.current_left_fwd = self.left_fwd
        self.current_left_rev = self.left_rev
        self.current_right_fwd = self.right_fwd
        self.current_right_rev = self.right_rev
        
        self.min_multiplier = 0.1
        self.max_multiplier = 1.0
        self.multiplier_step = 0.05
        
        # Modo de ajuste (qué motor estamos ajustando)
        self.adjustment_mode = "ALL"  # ALL, LEFT, RIGHT
        
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
        
        self.print_config()

    def print_config(self):
        """Mostrar configuración inicial"""
        self.get_logger().info('=' * 80)
        self.get_logger().info('🎮 Nodo Joystick - Control Individual de Motores')
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'📡 Broker MQTT: {self.mqtt_broker}:{self.mqtt_port}')
        self.get_logger().info(f'📤 Topic comandos: {self.mqtt_topic}')
        self.get_logger().info(f'📥 Topic status: {self.mqtt_status_topic}')
        self.get_logger().info(f'🕹️  Suscrito a: /joy')
        self.get_logger().info(f'🚗 Publicando a: /cmd_vel')
        self.get_logger().info('')
        self.get_logger().info('⚙️  CONFIGURACIÓN DE MOTORES:')
        self.get_logger().info(f'   Motor Izquierdo  - Adelante: {self.current_left_fwd:.0%} | Reversa: {self.current_left_rev:.0%}')
        self.get_logger().info(f'   Motor Derecho    - Adelante: {self.current_right_fwd:.0%} | Reversa: {self.current_right_rev:.0%}')
        self.get_logger().info('')
        self.get_logger().info('🎮 CONTROLES:')
        self.get_logger().info('   Movimiento:')
        self.get_logger().info('     • Stick Izquierdo (Y): Adelante/Atrás')
        self.get_logger().info('     • Stick Derecho (X): Giro Izquierda/Derecha')
        self.get_logger().info('')
        self.get_logger().info('   Ajuste de Velocidad:')
        self.get_logger().info('     • R1 (Botón 5): Aumentar velocidad (+5%)')
        self.get_logger().info('     • L1 (Botón 4): Disminuir velocidad (-5%)')
        self.get_logger().info('')
        self.get_logger().info('   Selección de Motor:')
        self.get_logger().info('     • SELECT (Botón 8): Cambiar modo (AMBOS → IZQUIERDO → DERECHO)')
        self.get_logger().info('     • START (Botón 9): 🔄 RESET velocidades a valores iniciales')
        self.get_logger().info('     • A/X (Botón 0): Parada de emergencia')
        self.get_logger().info('=' * 80)

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
        
        # ===== CAMBIAR MODO DE AJUSTE =====
        if len(msg.buttons) >= 10:
            # SELECT/BACK (botón 8): Cambiar modo de ajuste
            if msg.buttons[8]:
                old_mode = self.adjustment_mode
                if self.adjustment_mode == "ALL":
                    self.adjustment_mode = "LEFT"
                elif self.adjustment_mode == "LEFT":
                    self.adjustment_mode = "RIGHT"
                else:
                    self.adjustment_mode = "ALL"
                
                print("\n" + "="*70)
                print(f"🔧 CAMBIO DE MODO: {old_mode} → {self.adjustment_mode}")
                print("="*70)
                if self.adjustment_mode == "ALL":
                    print("📋 Ahora ajustarás: AMBOS MOTORES (Izquierdo y Derecho)")
                elif self.adjustment_mode == "LEFT":
                    print("📋 Ahora ajustarás: MOTOR IZQUIERDO únicamente")
                else:
                    print("📋 Ahora ajustarás: MOTOR DERECHO únicamente")
                print("="*70 + "\n")
                
                time.sleep(0.3)  # Debounce
            
            # START (botón 9): Reset de velocidades
            if msg.buttons[9]:
                self.reset_speeds()
                time.sleep(0.3)  # Debounce
        
        # ===== CONTROL DE VELOCIDAD CON BOTONES =====
        if len(msg.buttons) >= 6:
            # Variable estática para debounce
            if not hasattr(self, '_last_button_time'):
                self._last_button_time = {'increase': 0, 'decrease': 0}
            
            current_time = time.time()
            
            # R1 (botón 5): Aumentar velocidad
            if msg.buttons[5] and (current_time - self._last_button_time['increase'] > 0.2):
                self.adjust_speed(increment=True)
                self._last_button_time['increase'] = current_time
            
            # L1 (botón 4): Disminuir velocidad
            if msg.buttons[4] and (current_time - self._last_button_time['decrease'] > 0.2):
                self.adjust_speed(increment=False)
                self._last_button_time['decrease'] = current_time
        
        # Leer ejes del joystick
        stick_y = msg.axes[1]  # Avance/retroceso
        stick_x = msg.axes[3]  # Giro
        
        # Aplicar zona muerta
        if abs(stick_x) < self.deadzone:
            stick_x = 0.0
        if abs(stick_y) < self.deadzone:
            stick_y = 0.0

        # Calcular velocidades base
        v = stick_y * self.max_speed
        w = stick_x * self.max_speed

        # ===== CÁLCULO DE MOTORES CON MULTIPLICADORES INDIVIDUALES =====
        wheel_base = 0.3
        left_raw = v - (w * wheel_base / 2.0)
        right_raw = v + (w * wheel_base / 2.0)
        
        # Aplicar multiplicadores individuales según dirección
        self.left_motor = self.apply_motor_multiplier(left_raw, "left")
        self.right_motor = self.apply_motor_multiplier(right_raw, "right")

        # Publicar en /cmd_vel para RViz
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.cmd_vel_pub.publish(twist)

        # Botón de parada de emergencia
        if len(msg.buttons) > 0 and msg.buttons[0]:
            self.emergency_stop()
            return

        # Log de estado
        if abs(v) > 0.01 or abs(w) > 0.01:
            mqtt_icon = "🟢" if self.connection_active else "🔴"
            esp_icon = "🟢" if self.esp32_online else "🔴"
            
            # Determinar multiplicadores actuales
            left_mult = self.current_left_fwd if left_raw >= 0 else self.current_left_rev
            right_mult = self.current_right_fwd if right_raw >= 0 else self.current_right_rev
            
            self.get_logger().info(
                f'{mqtt_icon}MQTT {esp_icon}ESP32 | Mode:{self.adjustment_mode} | '
                f'L:{left_mult:.0%}({self.left_motor:+.2f}) R:{right_mult:.0%}({self.right_motor:+.2f})',
                throttle_duration_sec=0.5
            )

    def apply_motor_multiplier(self, raw_value, motor_side):
        """Aplicar multiplicador específico según motor y dirección"""
        if abs(raw_value) < 0.01:
            return 0.0
        
        if motor_side == "left":
            multiplier = self.current_left_fwd if raw_value > 0 else self.current_left_rev
        else:  # right
            multiplier = self.current_right_fwd if raw_value > 0 else self.current_right_rev
        
        result = raw_value * multiplier
        return max(-1.0, min(1.0, result))

    def adjust_speed(self, increment):
        """Ajustar velocidad según el modo actual"""
        old_vals = (self.current_left_fwd, self.current_left_rev, 
                    self.current_right_fwd, self.current_right_rev)
        
        if self.adjustment_mode == "ALL":
            # Ajustar ambos motores
            if increment:
                self.current_left_fwd = min(self.current_left_fwd + self.multiplier_step, self.max_multiplier)
                self.current_left_rev = min(self.current_left_rev + self.multiplier_step, self.max_multiplier)
                self.current_right_fwd = min(self.current_right_fwd + self.multiplier_step, self.max_multiplier)
                self.current_right_rev = min(self.current_right_rev + self.multiplier_step, self.max_multiplier)
            else:
                self.current_left_fwd = max(self.current_left_fwd - self.multiplier_step, self.min_multiplier)
                self.current_left_rev = max(self.current_left_rev - self.multiplier_step, self.min_multiplier)
                self.current_right_fwd = max(self.current_right_fwd - self.multiplier_step, self.min_multiplier)
                self.current_right_rev = max(self.current_right_rev - self.multiplier_step, self.min_multiplier)
                
        elif self.adjustment_mode == "LEFT":
            # Ajustar solo motor izquierdo
            if increment:
                self.current_left_fwd = min(self.current_left_fwd + self.multiplier_step, self.max_multiplier)
                self.current_left_rev = min(self.current_left_rev + self.multiplier_step, self.max_multiplier)
            else:
                self.current_left_fwd = max(self.current_left_fwd - self.multiplier_step, self.min_multiplier)
                self.current_left_rev = max(self.current_left_rev - self.multiplier_step, self.min_multiplier)
                
        else:  # RIGHT
            # Ajustar solo motor derecho
            if increment:
                self.current_right_fwd = min(self.current_right_fwd + self.multiplier_step, self.max_multiplier)
                self.current_right_rev = min(self.current_right_rev + self.multiplier_step, self.max_multiplier)
            else:
                self.current_right_fwd = max(self.current_right_fwd - self.multiplier_step, self.min_multiplier)
                self.current_right_rev = max(self.current_right_rev - self.multiplier_step, self.min_multiplier)
        
        # Siempre mostrar el cambio (sin throttle)
        new_vals = (self.current_left_fwd, self.current_left_rev, 
                    self.current_right_fwd, self.current_right_rev)
        
        if old_vals != new_vals:
            direction = "⬆️ AUMENTAR" if increment else "⬇️ DISMINUIR"
            
            # Mostrar cambios detallados
            print("\n" + "="*70)
            print(f"🎮 {direction} VELOCIDAD - Modo: {self.adjustment_mode}")
            print("="*70)
            print(f"🔧 Motor Izquierdo:")
            print(f"   Adelante: {old_vals[0]:.0%} → {self.current_left_fwd:.0%} [{self.current_left_fwd:+.2f}]")
            print(f"   Reversa:  {old_vals[1]:.0%} → {self.current_left_rev:.0%} [{self.current_left_rev:+.2f}]")
            print(f"🔧 Motor Derecho:")
            print(f"   Adelante: {old_vals[2]:.0%} → {self.current_right_fwd:.0%} [{self.current_right_fwd:+.2f}]")
            print(f"   Reversa:  {old_vals[3]:.0%} → {self.current_right_rev:.0%} [{self.current_right_rev:+.2f}]")
            print("="*70 + "\n")
            
            # También log normal de ROS
            self.get_logger().info(
                f'{direction} {self.adjustment_mode} | '
                f'L_FWD:{self.current_left_fwd:.0%} L_REV:{self.current_left_rev:.0%} | '
                f'R_FWD:{self.current_right_fwd:.0%} R_REV:{self.current_right_rev:.0%}'
            )

    def reset_speeds(self):
        """Resetear todas las velocidades a los valores iniciales"""
        old_vals = (self.current_left_fwd, self.current_left_rev,
                    self.current_right_fwd, self.current_right_rev)
        
        # Restaurar a valores iniciales de parámetros
        self.current_left_fwd = self.left_fwd
        self.current_left_rev = self.left_rev
        self.current_right_fwd = self.right_fwd
        self.current_right_rev = self.right_rev
        
        # Volver a modo ALL
        self.adjustment_mode = "ALL"
        
        # Mostrar cambios detallados
        print("\n" + "="*70)
        print("🔄 RESET - RESTAURANDO VELOCIDADES INICIALES")
        print("="*70)
        print(f"🔧 Motor Izquierdo:")
        print(f"   Adelante: {old_vals[0]:.0%} → {self.current_left_fwd:.0%} [{self.current_left_fwd:+.2f}]")
        print(f"   Reversa:  {old_vals[1]:.0%} → {self.current_left_rev:.0%} [{self.current_left_rev:+.2f}]")
        print(f"🔧 Motor Derecho:")
        print(f"   Adelante: {old_vals[2]:.0%} → {self.current_right_fwd:.0%} [{self.current_right_fwd:+.2f}]")
        print(f"   Reversa:  {old_vals[3]:.0%} → {self.current_right_rev:.0%} [{self.current_right_rev:+.2f}]")
        print(f"🎮 Modo: {self.adjustment_mode}")
        print("="*70 + "\n")
        
        self.get_logger().warn('🔄 RESET completado - Vuelto a valores iniciales')

    def send_commands(self):
        """Enviar comandos por MQTT"""
        if not self.connection_active:
            return

        try:
            message = {
                "left": round(float(self.left_motor), 3),
                "right": round(float(self.right_motor), 3),
                "left_fwd_mult": round(float(self.current_left_fwd), 2),
                "left_rev_mult": round(float(self.current_left_rev), 2),
                "right_fwd_mult": round(float(self.current_right_fwd), 2),
                "right_rev_mult": round(float(self.current_right_rev), 2),
                "timestamp": time.time()
            }
            
            result = self.mqtt_client.publish(self.mqtt_topic, json.dumps(message), qos=0)
            
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
                
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                
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
        
        self.left_motor = 0.0
        self.right_motor = 0.0
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        if self.connection_active:
            try:
                stop_msg = json.dumps({
                    "left": 0.0,
                    "right": 0.0,
                    "emergency": True
                })
                for _ in range(3):
                    self.mqtt_client.publish(self.mqtt_topic, stop_msg, qos=1)
                    time.sleep(0.02)
            except Exception as e:
                self.get_logger().error(f'❌ Error en parada de emergencia MQTT: {e}')

    def destroy_node(self):
        """Limpiar recursos al cerrar"""
        self.get_logger().info('🛑 Cerrando nodo...')
        self.emergency_stop()
        
        if self.connection_active:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=None)
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