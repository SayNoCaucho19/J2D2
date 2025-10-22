import rclpy
from rclpy.node import Node
import requests
import webbrowser
import os
import time

BASE_URL = "http://192.168.52.1"
STREAM_URL = f"{BASE_URL}:81/stream"

class CamaraNode(Node):
    def __init__(self):
        super().__init__("camara_node")
        self.get_logger().info("📷 Nodo de cámara iniciado")
        self.configurar_y_abrir()

    def configurar_y_abrir(self):
        try:
            # Cambiar resolución a SVGA (800x600)
            url_resolucion = f"{BASE_URL}/control?var=framesize&val=6"
            r1 = requests.get(url_resolucion, timeout=2)
            if r1.status_code == 200:
                self.get_logger().info("✅ Resolución puesta en SVGA (800x600)")
            else:
                self.get_logger().warn("⚠️ No se pudo cambiar resolución")

            time.sleep(1)

            # Crear HTML fullscreen
            html_content = f"""
            <html>
            <head>
                <title>ESP32-CAM Stream</title>
                <style>
                    body {{
                        margin: 0;
                        background: black;
                        display: flex;
                        justify-content: center;
                        align-items: center;
                        height: 100vh;
                    }}
                    img {{
                        width: 100%;
                        height: 100%;
                        object-fit: contain;
                    }}
                </style>
            </head>
            <body>
                <img src="{STREAM_URL}" />
            </body>
            </html>
            """

            filename = "esp32_stream.html"
            with open(filename, "w") as f:
                f.write(html_content)

            # Abrir navegador
            url = f"file://{os.path.abspath(filename)}"
            webbrowser.open(url)

        except Exception as e:
            self.get_logger().error(f"❌ Error configurando la cámara: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CamaraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
