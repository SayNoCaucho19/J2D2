#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import paho.mqtt.client as mqtt
import json


class MQTTBridgeNode(Node):

    def __init__(self):
        super().__init__("mqtt_bridge_node")

        # ===================== CONFIG MQTT ======================
        # CORREGIDO: Usar broker local en lugar de HiveMQ público
        self.MQTT_BROKER = "localhost"  # Broker Mosquitto en tu PC
        self.MQTT_PORT = 1883
        self.MQTT_TOPIC = "aquabot/battery"
        
        # Topics adicionales para monitoreo completo
        self.MQTT_TOPIC_STATUS = "aquabot/status"
        self.MQTT_TOPIC_COMMANDS = "aquabot/commands"
        # ========================================================

        # ============= ROS2 PUBLISHERS ==========================
        self.voltage_pub = self.create_publisher(Float32, "voltage_data", 10)
        self.battery_percent_pub = self.create_publisher(Float32, "battery_percentage", 10)

        # ============= CONFIGURAR CLIENTE MQTT ==================
        self.mqtt_client = mqtt.Client(client_id="ros2_bridge", clean_session=True)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.on_disconnect = self.on_disconnect

        # Intentar conexión con manejo de errores mejorado
        try:
            self.get_logger().info(
                f"Conectando a MQTT broker {self.MQTT_BROKER}:{self.MQTT_PORT}"
            )
            self.mqtt_client.connect(self.MQTT_BROKER, self.MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info("✓ Cliente MQTT iniciado correctamente")
        except ConnectionRefusedError:
            self.get_logger().error(
                "✗ ERROR: Conexión rechazada. ¿Mosquitto está corriendo?"
            )
            self.get_logger().error("   Ejecuta: sudo systemctl start mosquitto")
        except OSError as e:
            self.get_logger().error(f"✗ ERROR de red: {str(e)}")
            self.get_logger().error(
                "   Verifica que Mosquitto escuche en 0.0.0.0:1883"
            )
        except Exception as e:
            self.get_logger().error(f"✗ ERROR conectando a MQTT: {str(e)}")

    # =============================================================
    # MQTT: al conectar
    # =============================================================
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("✓ Conectado exitosamente a MQTT broker")
            
            # Suscribirse a múltiples topics
            client.subscribe(self.MQTT_TOPIC)
            self.get_logger().info(f"✓ Suscrito a: {self.MQTT_TOPIC}")
            
            client.subscribe(self.MQTT_TOPIC_STATUS)
            self.get_logger().info(f"✓ Suscrito a: {self.MQTT_TOPIC_STATUS}")
            
        else:
            error_messages = {
                1: "Protocolo incorrecto",
                2: "Client ID rechazado",
                3: "Servidor no disponible",
                4: "Usuario/contraseña incorrectos",
                5: "No autorizado"
            }
            error = error_messages.get(rc, f"Error desconocido ({rc})")
            self.get_logger().error(f"✗ Falló conexión MQTT: {error}")

    # =============================================================
    # MQTT: al desconectar
    # =============================================================
    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            self.get_logger().warning(
                f"⚠ Desconexión inesperada de MQTT (rc={rc}). Intentando reconectar..."
            )

    # =============================================================
    # MQTT: al recibir mensaje
    # =============================================================
    def on_message(self, client, userdata, msg):
        payload = msg.payload.decode("utf-8")
        self.get_logger().info(f"📨 [{msg.topic}] {payload[:100]}...")  # Muestra primeros 100 chars

        try:
            data = json.loads(payload)

            # Procesar voltaje de batería
            if "battery_voltage" in data:
                voltage = float(data["battery_voltage"])

                ros_msg = Float32()
                ros_msg.data = voltage
                self.voltage_pub.publish(ros_msg)

                self.get_logger().info(f"📊 Voltaje publicado en ROS2: {voltage:.2f}V")

            # Procesar porcentaje de batería
            if "battery_percentage" in data:
                percentage = float(data["battery_percentage"])

                ros_msg = Float32()
                ros_msg.data = percentage
                self.battery_percent_pub.publish(ros_msg)

                self.get_logger().info(f"🔋 Batería: {percentage:.1f}%")

            # Mostrar estado de batería si existe
            if "battery_status" in data:
                status = data["battery_status"]
                self.get_logger().info(f"📈 Estado: {status}")

            # Detectar alertas críticas
            if data.get("type") == "battery_alert":
                self.get_logger().warning(
                    f"⚠️ ALERTA: {data.get('message', 'Batería baja')}"
                )

            # Detectar paradas de emergencia
            if data.get("type") == "emergency_stop":
                self.get_logger().error("🛑 PARADA DE EMERGENCIA DETECTADA")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"✗ JSON inválido: {str(e)}")
        except ValueError as e:
            self.get_logger().error(f"✗ Error de conversión: {str(e)}")
        except Exception as e:
            self.get_logger().error(f"✗ Error procesando mensaje: {str(e)}")

    # =============================================================
    def destroy_node(self):
        self.get_logger().info("Cerrando conexión MQTT...")
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MQTTBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción por teclado detectada")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()