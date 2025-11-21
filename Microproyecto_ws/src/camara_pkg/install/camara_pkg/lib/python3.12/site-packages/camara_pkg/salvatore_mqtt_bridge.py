#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import paho.mqtt.client as mqtt
import json
import base64
import cv2
import numpy as np
from cv_bridge import CvBridge
import threading
import time

class SalvatoreMQTTBridge(Node):
    def __init__(self):
        super().__init__('salvatore_mqtt_bridge')
        
        # Configuración MQTT
        self.mqtt_broker = "test.mosquitto.org"
        self.mqtt_port = 1883
        self.mqtt_client_id = "SALVATORE_ROS2_BRIDGE"
        
        # Tópicos MQTT (deben coincidir con ESP32-CAM)
        self.mqtt_topic_image = "salvatore_rov_2024/camera/image"
        self.mqtt_topic_status = "salvatore_rov_2024/camera/status" 
        self.mqtt_topic_control = "salvatore_rov_2024/camera/control"
        
        # Publishers ROS2
        self.image_publisher = self.create_publisher(
            Image, 
            '/salvatore/camera/image_raw', 
            10
        )
        
        self.status_publisher = self.create_publisher(
            String,
            '/salvatore/camera/status',
            10
        )
        
        # Subscribers ROS2 (para enviar comandos a ESP32)
        self.command_subscriber = self.create_subscription(
            String,
            '/salvatore/camera/commands',
            self.ros_command_callback,
            10
        )
        
        # Bridge para conversión de imágenes
        self.cv_bridge = CvBridge()
        
        # Variables para reconstruir imagen por chunks
        self.image_chunks = {}
        self.current_image_metadata = {}
        
        # Cliente MQTT
        self.mqtt_client = mqtt.Client(client_id=self.mqtt_client_id)
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        
        # Estado de conexión
        self.mqtt_connected = False
        
        # Inicializar conexión MQTT
        self.connect_mqtt()
        
        # Timer para enviar heartbeat y verificar conexión
        self.heartbeat_timer = self.create_timer(5.0, self.heartbeat_callback)
        
        self.get_logger().info('🌊 SALVATORE MQTT-ROS2 Bridge iniciado')
        self.get_logger().info(f'📡 Conectando a broker: {self.mqtt_broker}:{self.mqtt_port}')
    
    def connect_mqtt(self):
        """Conectar al broker MQTT"""
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()  # Iniciar loop en thread separado
            self.get_logger().info('🔗 Iniciando conexión MQTT...')
        except Exception as e:
            self.get_logger().error(f'❌ Error conectando MQTT: {e}')
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback cuando se conecta MQTT"""
        if rc == 0:
            self.mqtt_connected = True
            self.get_logger().info('✅ MQTT Conectado exitosamente!')
            
            # Suscribirse a todos los tópicos de SALVATORE
            topics = [
                (self.mqtt_topic_image, 0),
                (self.mqtt_topic_image + "/meta", 0),  
                (self.mqtt_topic_status, 0)
            ]
            
            for topic, qos in topics:
                client.subscribe(topic, qos)
                self.get_logger().info(f'📥 Suscrito a: {topic}')
                
            # Solicitar status inicial de ESP32-CAM
            self.send_mqtt_command("GET_STATUS")
            
        else:
            self.get_logger().error(f'❌ Error MQTT conexión, código: {rc}')
            self.mqtt_connected = False
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        """Callback cuando se desconecta MQTT"""
        self.mqtt_connected = False
        self.get_logger().warning(f'⚠️ MQTT Desconectado, código: {rc}')
    
    def on_mqtt_message(self, client, userdata, msg):
        """Procesar mensajes MQTT recibidos"""
        try:
            topic = msg.topic
            payload = msg.payload.decode('utf-8')
            
            self.get_logger().debug(f'📨 MQTT recibido en {topic}')
            
            if topic == self.mqtt_topic_status:
                self.handle_status_message(payload)
                
            elif topic == self.mqtt_topic_image + "/meta":
                self.handle_image_metadata(payload)
                
            elif topic == self.mqtt_topic_image:
                self.handle_image_chunk(payload)
                
        except Exception as e:
            self.get_logger().error(f'❌ Error procesando mensaje MQTT: {e}')
    
    def handle_status_message(self, payload):
        """Manejar mensajes de status de ESP32-CAM"""
        try:
            status_data = json.loads(payload)
            
            # Log del status recibido
            device = status_data.get('device', 'Unknown')
            status = status_data.get('status', 'Unknown')
            timestamp = status_data.get('timestamp', 0)
            
            self.get_logger().info(f'📊 Status {device}: {status}')
            
            # Publicar status en ROS2
            ros_status_msg = String()
            ros_status_msg.data = payload
            self.status_publisher.publish(ros_status_msg)
            
        except json.JSONDecodeError:
            self.get_logger().error('❌ Error decodificando JSON de status')
    
    def handle_image_metadata(self, payload):
        """Manejar metadata de imagen"""
        try:
            self.current_image_metadata = json.loads(payload)
            self.image_chunks.clear()  # Limpiar chunks anteriores
            
            width = self.current_image_metadata.get('width', 0)
            height = self.current_image_metadata.get('height', 0) 
            size = self.current_image_metadata.get('size', 0)
            
            self.get_logger().info(f'📸 Nueva imagen: {width}x{height}, {size} bytes')
            
        except json.JSONDecodeError:
            self.get_logger().error('❌ Error decodificando metadata de imagen')
    
    def handle_image_chunk(self, payload):
        """Manejar chunks de imagen y reconstruir"""
        try:
            chunk_data = json.loads(payload)
            
            chunk_num = chunk_data.get('chunk', 0)
            total_chunks = chunk_data.get('total', 1)
            chunk_content = chunk_data.get('data', '')
            
            # Almacenar chunk
            self.image_chunks[chunk_num] = chunk_content
            
            self.get_logger().debug(f'📦 Chunk {chunk_num + 1}/{total_chunks} recibido')
            
            # Si tenemos todos los chunks, reconstruir imagen
            if len(self.image_chunks) == total_chunks:
                self.reconstruct_and_publish_image()
                
        except json.JSONDecodeError:
            self.get_logger().error('❌ Error decodificando chunk de imagen')
    
    def reconstruct_and_publish_image(self):
        """Reconstruir imagen de chunks y publicar en ROS2"""
        try:
            # Ordenar chunks y concatenar
            sorted_chunks = []
            for i in sorted(self.image_chunks.keys()):
                sorted_chunks.append(self.image_chunks[i])
            
            # Reconstruir imagen base64 completa
            complete_image_b64 = ''.join(sorted_chunks)
            
            # Decodificar de base64 a bytes
            image_bytes = base64.b64decode(complete_image_b64)
            
            # Convertir bytes a imagen OpenCV
            nparr = np.frombuffer(image_bytes, np.uint8)
            cv_image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if cv_image is not None:
                # Convertir de BGR (OpenCV) a RGB (ROS2)
                cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                
                # Convertir a mensaje ROS2 Image
                ros_image = self.cv_bridge.cv2_to_imgmsg(cv_image_rgb, encoding='rgb8')
                
                # Añadir timestamp
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = "salvatore_camera"
                
                # Publicar en ROS2
                self.image_publisher.publish(ros_image)
                
                height, width = cv_image.shape[:2]
                self.get_logger().info(f'✅ Imagen publicada en ROS2: {width}x{height}')
                
            else:
                self.get_logger().error('❌ Error decodificando imagen JPEG')
                
        except Exception as e:
            self.get_logger().error(f'❌ Error reconstruyendo imagen: {e}')
        finally:
            # Limpiar chunks procesados
            self.image_chunks.clear()
    
    def ros_command_callback(self, msg):
        """Recibir comandos desde ROS2 y enviarlos por MQTT"""
        try:
            command_data = json.loads(msg.data)
            command = command_data.get('command', '')
            
            self.get_logger().info(f'📤 Enviando comando a ESP32: {command}')
            self.send_mqtt_command(command)
            
        except Exception as e:
            self.get_logger().error(f'❌ Error procesando comando ROS2: {e}')
    
    def send_mqtt_command(self, command):
        """Enviar comando a ESP32-CAM por MQTT"""
        if self.mqtt_connected:
            command_msg = {
                "command": command,
                "timestamp": int(time.time() * 1000),
                "source": "ROS2_BRIDGE"
            }
            
            payload = json.dumps(command_msg)
            self.mqtt_client.publish(self.mqtt_topic_control, payload)
            self.get_logger().debug(f'📡 Comando MQTT enviado: {command}')
        else:
            self.get_logger().warning('⚠️ No se puede enviar comando - MQTT desconectado')
    
    def heartbeat_callback(self):
        """Enviar heartbeat y verificar conexión"""
        if self.mqtt_connected:
            self.get_logger().debug('💓 MQTT Bridge activo - recibiendo imágenes submarinas')
        else:
            self.get_logger().warning('⚠️ MQTT desconectado - reintentando conexión...')
            try:
                self.mqtt_client.reconnect()
            except:
                pass
    
    def destroy_node(self):
        """Limpiar al destruir nodo"""
        if hasattr(self, 'mqtt_client'):
            self.mqtt_client.disconnect()
            self.mqtt_client.loop_stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        bridge = SalvatoreMQTTBridge()
        rclpy.spin(bridge)
        
    except KeyboardInterrupt:
        print('\n🌊 Deteniendo SALVATORE MQTT Bridge...')
        
    finally:
        if 'bridge' in locals():
            bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()