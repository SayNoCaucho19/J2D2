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
        self.MQTT_BROKER = "localhost"
        self.MQTT_PORT = 1883
        self.MQTT_TOPIC = "aquabot/battery"
        self.MQTT_TOPIC_STATUS = "aquabot/status"
        self.MQTT_TOPIC_COMMANDS = "aquabot/commands"
        # ========================================================

        # ============= ROS2 PUBLISHERS ==========================
        self.voltage_pub = self.create_publisher(Float32, "voltage_data", 10)

        # 👉 PUBLICAR TOPIC "peso"
        self.peso_pub = self.create_publisher(Float32, "peso", 10)

        # ============= CONFIG MQTT CLIENT =======================
        self.mqtt_client = mqtt.Client(client_id="ros2_bridge", clean_session=True)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.on_disconnect = self.on_disconnect

        try:
            self.get_logger().info(
                f"Conectando a MQTT broker {self.MQTT_BROKER}:{self.MQTT_PORT}"
            )
            self.mqtt_client.connect(self.MQTT_BROKER, self.MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info("✓ Cliente MQTT iniciado")
        except Exception as e:
            self.get_logger().error(f"✗ ERROR conectando a MQTT: {str(e)}")

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("✓ Conectado a MQTT")
            client.subscribe(self.MQTT_TOPIC)
            client.subscribe(self.MQTT_TOPIC_STATUS)
            client.subscribe("aquabot/peso")   # Suscribir peso REAL
        else:
            self.get_logger().error(f"✗ Falló conexión MQTT rc={rc}")

    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            self.get_logger().warning(
                f"⚠ Desconexión inesperada de MQTT (rc={rc}). Reintentando..."
            )

    def on_message(self, client, userdata, msg):
        payload = msg.payload.decode("utf-8")
        self.get_logger().info(f"📨 [{msg.topic}] {payload[:100]}")

        try:
            data = json.loads(payload)

            # ====================================================
            #  🟢 PROCESO EXCLUSIVO PARA aquabot/peso
            # ====================================================
            if msg.topic == "aquabot/peso":
                if "valor_g" in data:
                    peso = float(data["valor_g"])
                    msg_ros = Float32()
                    msg_ros.data = peso
                    self.peso_pub.publish(msg_ros)
                    self.get_logger().info(f"⚖️ Peso publicado: {peso}")
                else:
                    self.get_logger().error("❌ JSON recibido no tiene 'valor_g'")
                return  # MUY IMPORTANTE

            # ====================================================
            #  Voltaje
            # ====================================================
            if "battery_voltage" in data:
                voltage = float(data["battery_voltage"])
                msg_ros = Float32()
                msg_ros.data = voltage
                self.voltage_pub.publish(msg_ros)

            # ====================================================
            #  Peso ANTIGUO (si viniera en battery_percentage)
            # ====================================================
            #if "battery_percentage" in data:
             #   peso_valor = float(data["battery_percentage"])
              #  msg_ros = Float32()
               # msg_ros.data = peso_valor
                #self.peso_pub.publish(msg_ros)
                #self.get_logger().info(f"⚖️ Peso publicado (legacy): {peso_valor}")

        except Exception as e:
            self.get_logger().error(f"✗ Error procesando mensaje: {str(e)}")

    def destroy_node(self):
        self.get_logger().info("Cerrando MQTT...")
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MQTTBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
