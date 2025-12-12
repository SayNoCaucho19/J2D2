#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import customtkinter as ctk
from PIL import Image as PILImage, ImageTk
import threading
from tkinter import messagebox
import queue
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32
import requests
from io import BytesIO

# Configurar CustomTkinter con tema marino
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# Colores tema marino
OCEAN_COLORS = {
    "deep_blue": "#0B2545",
    "ocean_blue": "#134E6F",
    "sea_blue": "#1B6EC2",
    "aqua_blue": "#2E8BC0",
    "light_aqua": "#52B2CF",
    "foam": "#B8D4E3",
    "white": "#FFFFFF",
    "coral": "#FF7F7F",
    "seaweed": "#4F7942",
    "gold": "#FFD700",
}

class AquaCleanInterface(Node):
    def __init__(self):
        super().__init__('aquaclean_interface')
        
        # ---- CONFIGURACIÓN ESP32-CAM ----
        self.esp32_cam_url = "http://10.42.0.202/stream"
        self.camera_thread = None
        self.camera_running = False
        
        # ---- SUSCRIPCIÓN AL VOLTAJE ----
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.voltage_sub = self.create_subscription(
            Float32,
            '/voltage_data',
            self.voltage_callback,
            qos
        )

        # ---- SUSCRIPCIÓN AL PESO ----
        self.weight_sub = self.create_subscription(
            Float32,
            '/peso',
            self.weight_callback,
            qos
        )

        # ---- Variables ----
        self.weight_value_ros = 0.0
        self.voltage_value = 0.0
        self.cv_bridge = CvBridge()
        
        # Queue para comunicación thread-safe entre ROS2 y GUI
        self.gui_queue = queue.Queue()
        self.queue = queue.Queue()

        # Variables de estado
        self.connection_status = "DESCONECTADO"
        self.battery_level = 85.0
        self.battery_voltage = 12.6
        self.cargo_weight = 0
        self.camera_active = False
        self.current_image = None
        self.voltage_label = None

        # Inicializar interfaz gráfica
        self.init_gui()
        
        self.get_logger().info('SALVATORE Interface Marina iniciada')
        
    def start_esp32_camera_stream(self):
        """Iniciar el stream de la ESP32-CAM en un hilo separado"""
        if not self.camera_running:
            self.camera_running = True
            self.camera_thread = threading.Thread(target=self.camera_reconnect_loop, daemon=True)
            self.camera_thread.start()
            self.get_logger().info('🎥 Stream ESP32-CAM iniciado con auto-reconexión')
    
    def stop_esp32_camera_stream(self):
        """Detener el stream de la ESP32-CAM"""
        self.camera_running = False
        if self.camera_thread:
            self.camera_thread.join(timeout=2)
        self.get_logger().info('🎥 Stream ESP32-CAM detenido')
    
    def manual_reconnect_camera(self):
        """Reconectar manualmente la cámara"""
        self.get_logger().info('🔄 Reconexión manual solicitada')
        
        # Detener stream actual si existe
        if self.camera_running:
            self.stop_esp32_camera_stream()
            # Esperar un momento para que se detenga
            threading.Event().wait(1)
        
        # Limpiar la cola de frames
        while not self.gui_queue.empty():
            try:
                self.gui_queue.get_nowait()
            except:
                break
        
        # Reiniciar stream
        self.start_esp32_camera_stream()
        
        # Actualizar indicador
        if hasattr(self, 'camera_indicator'):
            self.camera_indicator.configure(
                text="🟡 Reconectando manualmente...",
                text_color=OCEAN_COLORS["gold"]
            )
    
    def camera_reconnect_loop(self):
        """Loop que maneja la reconexión automática de la cámara"""
        retry_delay = 5  # segundos entre reintentos
        attempt = 0
        
        self.get_logger().info('🔄 Loop de reconexión iniciado')
        
        while self.camera_running:
            attempt += 1
            
            if attempt == 1:
                self.get_logger().info(f'🔄 Intento de conexión a ESP32-CAM...')
            else:
                self.get_logger().info(f'🔄 Reintento #{attempt} de conexión...')
                # Esperar antes de reintentar (verificando camera_running cada segundo)
                for i in range(retry_delay):
                    if not self.camera_running:
                        self.get_logger().info('🛑 Loop de reconexión detenido por usuario')
                        return
                    threading.Event().wait(1)
            
            try:
                # Intentar leer el stream
                self.read_esp32_stream()
                
                # Si read_esp32_stream termina normalmente (no por excepción),
                # significa que el stream se cerró correctamente
                if self.camera_running:
                    self.get_logger().warning('⚠️ Stream terminó inesperadamente. Reconectando...')
                    
            except requests.exceptions.ConnectionError as e:
                if self.camera_running:
                    self.get_logger().warning(f'⚠️ Error de conexión. Reintentando en {retry_delay}s...')
                    
            except requests.exceptions.Timeout as e:
                if self.camera_running:
                    self.get_logger().warning(f'⚠️ Timeout de conexión. Reintentando en {retry_delay}s...')
                    
            except Exception as e:
                if self.camera_running:
                    self.get_logger().warning(f'⚠️ Error: {type(e).__name__}. Reintentando en {retry_delay}s...')
        
        self.get_logger().info('🛑 Loop de reconexión finalizado')
    
    def read_esp32_stream(self):
        """Leer el stream MJPEG de la ESP32-CAM"""
        try:
            # Conectar al stream con timeout más largo
            self.get_logger().info(f'🔄 Conectando a {self.esp32_cam_url}...')
            
            response = requests.get(
                self.esp32_cam_url, 
                stream=True, 
                timeout=15,
                headers={'User-Agent': 'Mozilla/5.0'}
            )
            
            if response.status_code == 200:
                self.get_logger().info('✅ Conexión exitosa con ESP32-CAM!')
                
                bytes_buffer = b''
                frame_count = 0
                
                # Leer el stream continuamente
                for chunk in response.iter_content(chunk_size=1024):
                    if not self.camera_running:
                        self.get_logger().info('🛑 Deteniendo stream...')
                        break
                    
                    bytes_buffer += chunk
                    
                    # Buscar inicio de JPEG
                    start = bytes_buffer.find(b'\xff\xd8')
                    # Buscar fin de JPEG
                    end = bytes_buffer.find(b'\xff\xd9')
                    
                    # Si encontramos un frame completo
                    if start != -1 and end != -1 and end > start:
                        # Extraer el frame JPEG completo
                        jpg_data = bytes_buffer[start:end+2]
                        # Limpiar el buffer
                        bytes_buffer = bytes_buffer[end+2:]
                        
                        try:
                            # Decodificar JPEG con OpenCV
                            np_arr = np.frombuffer(jpg_data, dtype=np.uint8)
                            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                            
                            if frame is not None:
                                # Convertir de BGR a RGB
                                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                                
                                # Limpiar cola si tiene muchos frames acumulados
                                while self.gui_queue.qsize() > 2:
                                    try:
                                        self.gui_queue.get_nowait()
                                    except:
                                        break
                                
                                # Agregar nuevo frame
                                self.gui_queue.put(frame_rgb)
                                frame_count += 1
                                
                                # Log de progreso
                                if frame_count == 1:
                                    self.get_logger().info('🎥 Primer frame recibido!')
                                elif frame_count % 50 == 0:
                                    self.get_logger().info(f'📹 Frames procesados: {frame_count}')
                            else:
                                if frame_count % 10 == 0:
                                    self.get_logger().warning('⚠️ Frame decodificado es None')
                                    
                        except Exception as decode_error:
                            self.get_logger().error(f'❌ Error decodificando: {decode_error}')
                    
                    # Evitar que el buffer crezca demasiado
                    if len(bytes_buffer) > 200000:
                        self.get_logger().warning('⚠️ Buffer muy grande, limpiando...')
                        # Buscar el último inicio de JPEG válido
                        last_start = bytes_buffer.rfind(b'\xff\xd8')
                        if last_start != -1:
                            bytes_buffer = bytes_buffer[last_start:]
                        else:
                            bytes_buffer = b''
                
                self.get_logger().info(f'✓ Stream finalizado. Total frames: {frame_count}')
                
            else:
                self.get_logger().error(f'❌ HTTP {response.status_code} - Verifica la URL')
                
        except requests.exceptions.ConnectionError:
            self.get_logger().error(f'❌ No se puede conectar a {self.esp32_cam_url}')
            self.get_logger().error('   Verifica que:')
            self.get_logger().error('   1. La ESP32-CAM esté encendida')
            self.get_logger().error('   2. Estés en la misma red WiFi')
            self.get_logger().error('   3. La IP sea correcta (prueba en navegador)')
        except requests.exceptions.Timeout:
            self.get_logger().error(f'❌ Timeout - La cámara no responde en {self.esp32_cam_url}')
        except Exception as e:
            self.get_logger().error(f'❌ Error inesperado: {type(e).__name__}: {e}')
        finally:
            self.camera_running = False
            self.get_logger().info('🛑 Hilo de cámara finalizado')
        
    def calculate_battery_percentage(self, voltage):
        MAX_VOLTAGE = 16.8
        MIN_VOLTAGE = 13.2

        if voltage >= MAX_VOLTAGE:
            return 100.0
        if voltage <= MIN_VOLTAGE:
            return 0.0

        percent = (voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE) * 100
        return round(percent, 1)

    def voltage_callback(self, msg):
        self.voltage_value = msg.data
        battery_percent = self.calculate_battery_percentage(self.voltage_value)
        self.queue.put(("voltage", self.voltage_value))
        self.queue.put(("battery_percent", battery_percent))

    def weight_callback(self, msg):
        self.weight_value_ros = msg.data
        self.queue.put(("weight", self.weight_value_ros))

    def init_gui(self):
        """Inicializar la interfaz gráfica con tema marino"""
        self.root = ctk.CTk()
        self.root.title("SALVATORE - Robot Marino")
        self.root.geometry("900x800")
        self.root.configure(fg_color=OCEAN_COLORS["deep_blue"])
        
        self.current_screen = "lobby"
        self.animation_running = True
        
        self.configure_colors()
        self.create_lobby_screen()
        self.process_gui_updates()
    
    def configure_colors(self):
        """Configurar colores personalizados"""
        ctk.ThemeManager.theme["CTkFrame"]["fg_color"] = [OCEAN_COLORS["ocean_blue"], OCEAN_COLORS["ocean_blue"]]
        ctk.ThemeManager.theme["CTkButton"]["fg_color"] = [OCEAN_COLORS["sea_blue"], OCEAN_COLORS["aqua_blue"]]
        ctk.ThemeManager.theme["CTkButton"]["hover_color"] = [OCEAN_COLORS["light_aqua"], OCEAN_COLORS["light_aqua"]]

    def process_gui_updates(self):
        try:
            # Procesar frames de la cámara
            if not self.gui_queue.empty():
                cv_image = self.gui_queue.get()

                # Redimensionar y mostrar solo si camera_display existe
                if hasattr(self, 'camera_display'):
                    image_pil = PILImage.fromarray(cv_image)
                    image_pil = image_pil.resize((640, 480))
                    image_tk = ImageTk.PhotoImage(image_pil)

                    self.camera_display.configure(image=image_tk, text="")
                    self.camera_display.image = image_tk
                    
                    # Actualizar indicador a verde (conectado)
                    if hasattr(self, 'camera_indicator'):
                        self.camera_indicator.configure(
                            text="🟢 Cámara transmitiendo en vivo",
                            text_color=OCEAN_COLORS["seaweed"]
                        )
            else:
                # Si la cola está vacía y estamos intentando conectar
                if hasattr(self, 'camera_indicator') and self.camera_running:
                    # Mantener indicador amarillo si estamos intentando conectar
                    current_text = self.camera_indicator.cget("text")
                    if "🔴" in current_text or "🟡" in current_text:
                        pass  # No cambiar, mantener estado actual
                    elif "🟢" in current_text:
                        # Si estaba verde y ahora no hay frames, cambiar a amarillo
                        self.camera_indicator.configure(
                            text="🟡 Reconectando...",
                            text_color=OCEAN_COLORS["gold"]
                        )

            # Procesar eventos de voltaje y peso
            if not self.queue.empty():
                event = self.queue.get()
                
                if event[0] == "voltage" and hasattr(self, 'voltage_label') and self.voltage_label is not None:
                    new_voltage = event[1]
                    self.voltage_label.configure(text=f"{new_voltage:.2f} V")
                    
                if event[0] == "battery_percent" and hasattr(self, 'battery_value') and self.battery_value is not None:
                    percent = event[1]
                    self.battery_value.configure(text=f"{percent}%")

                    if percent > 60:
                        self.battery_value.configure(text_color=OCEAN_COLORS["seaweed"])
                    elif percent > 30:
                        self.battery_value.configure(text_color=OCEAN_COLORS["gold"])
                    else:
                        self.battery_value.configure(text_color=OCEAN_COLORS["coral"])
                        
                if event[0] == "weight" and hasattr(self, "weight_value") and self.weight_value is not None:
                    new_weight = event[1]
                    self.weight_value.configure(text=f"{new_weight:.2f} g")

        except Exception as e:
            self.get_logger().error(f"❌ Error actualizando GUI: {e}")

        self.root.after(50, self.process_gui_updates)

    def create_lobby_screen(self):
        """Crear pantalla de lobby marina"""
        # Detener cámara si está activa
        self.stop_esp32_camera_stream()
        
        for widget in self.root.winfo_children():
            widget.destroy()
            
        main_frame = ctk.CTkFrame(
            self.root,
            fg_color=OCEAN_COLORS["deep_blue"],
            corner_radius=0
        )
        main_frame.pack(fill="both", expand=True)
        
        header_frame = ctk.CTkFrame(main_frame, fg_color="transparent", height=200)
        header_frame.pack(fill="x", padx=20, pady=(40, 20))
        
        title_label = ctk.CTkLabel(
            header_frame,
            text="🌊 SALVATORE 🌊",
            font=ctk.CTkFont(size=56, weight="bold"),
            text_color=OCEAN_COLORS["foam"]
        )
        title_label.pack(pady=(10, 10))
        
        subtitle_label = ctk.CTkLabel(
            header_frame,
            text="ROV para la recolección de basura superficial acuática",
            font=ctk.CTkFont(size=22, weight="normal"),
            text_color=OCEAN_COLORS["light_aqua"]
        )
        subtitle_label.pack(pady=(0, 10))
        
        line_frame = ctk.CTkFrame(
            header_frame,
            height=4,
            fg_color=OCEAN_COLORS["aqua_blue"]
        )
        line_frame.pack(fill="x", padx=200, pady=20)
        
        center_frame = ctk.CTkFrame(main_frame, fg_color="transparent")
        center_frame.pack(expand=True, fill="both", padx=40, pady=20)
        
        button_frame = ctk.CTkFrame(
            center_frame,
            fg_color=OCEAN_COLORS["ocean_blue"],
            corner_radius=25,
            border_width=2,
            border_color=OCEAN_COLORS["aqua_blue"]
        )
        button_frame.pack(pady=40, padx=100)
        
        buttons_data = [
            ("START", self.show_dashboard, OCEAN_COLORS["sea_blue"]),
            ("MANUAL DE USUARIO", self.show_manual, OCEAN_COLORS["aqua_blue"]),
            ("ℹ ABOUT US!", self.show_about, OCEAN_COLORS["light_aqua"]),
            ("CONFIGURACIÓN", self.show_settings, OCEAN_COLORS["ocean_blue"])
        ]
        
        for text, command, color in buttons_data:
            btn = ctk.CTkButton(
                button_frame,
                text=text,
                font=ctk.CTkFont(size=18, weight="bold"),
                width=350,
                height=60,
                fg_color=color,
                hover_color=OCEAN_COLORS["light_aqua"],
                corner_radius=15,
                command=command,
                text_color=OCEAN_COLORS["white"]
            )
            btn.pack(pady=15, padx=30)

    def show_dashboard(self):
        """Mostrar dashboard principal"""
        self.current_screen = "dashboard"
        
        # Iniciar stream de ESP32-CAM
        self.start_esp32_camera_stream()
        
        for widget in self.root.winfo_children():
            widget.destroy()
            
        main_frame = ctk.CTkFrame(
            self.root,
            fg_color=OCEAN_COLORS["deep_blue"],
            corner_radius=0
        )
        main_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        self.create_dashboard_header(main_frame)
        
        content_frame = ctk.CTkFrame(main_frame, fg_color="transparent")
        content_frame.pack(fill="both", expand=True, padx=20, pady=10)
        
        top_frame = ctk.CTkFrame(content_frame, fg_color="transparent", height=130)
        top_frame.pack(fill="x", pady=(0, 15))
        top_frame.pack_propagate(False)
        
        top_frame.grid_columnconfigure(0, weight=1)
        top_frame.grid_columnconfigure(1, weight=1)
        top_frame.grid_columnconfigure(2, weight=1)
        top_frame.grid_rowconfigure(0, weight=1)
        
        self.create_small_status_cards(top_frame)
        
        bottom_frame = ctk.CTkFrame(content_frame, fg_color="transparent")
        bottom_frame.pack(fill="both", expand=True, pady=(15, 0))
        
        self.create_large_camera_card(bottom_frame)
        
    def create_dashboard_header(self, parent):
        """Crear header del dashboard"""
        header_frame = ctk.CTkFrame(
            parent,
            fg_color=OCEAN_COLORS["ocean_blue"],
            corner_radius=15,
            border_width=2,
            border_color=OCEAN_COLORS["aqua_blue"]
        )
        header_frame.pack(fill="x", padx=10, pady=(10, 20))
        
        header_content = ctk.CTkFrame(header_frame, fg_color="transparent")
        header_content.pack(fill="both", expand=True, padx=20, pady=15)
        
        back_button = ctk.CTkButton(
            header_content,
            text="← Volver",
            font=ctk.CTkFont(size=11, weight="bold"),
            width=150,
            height=40,
            fg_color=OCEAN_COLORS["aqua_blue"],
            hover_color=OCEAN_COLORS["gold"],
            corner_radius=20,
            command=self.create_lobby_screen
        )
        back_button.pack(side="left")
        
        title_frame = ctk.CTkFrame(header_content, fg_color="transparent")
        title_frame.pack(side="left", expand=True, fill="x")
        
        dashboard_title = ctk.CTkLabel(
            title_frame,
            text="🌊 SALVATORE - PANEL DE CONTROL 🌊",
            font=ctk.CTkFont(size=32, weight="bold"),
            text_color=OCEAN_COLORS["foam"]
        )
        dashboard_title.pack(pady=5)

    def create_small_status_cards(self, parent):
        """Crear 3 tarjetas pequeñas en fila superior"""
        self.connection_card = self.create_small_status_card(
            parent, 0, "CONEXIÓN ROS2", "🌐", 
            OCEAN_COLORS["sea_blue"], "connection"
        )
        
        self.battery_card = self.create_small_status_card(
            parent, 1, "BATERÍA MARINA", "🔋",
            OCEAN_COLORS["seaweed"], "battery"
        )
        
        self.weight_card = self.create_small_status_card(
            parent, 2, "CARGA RECOLECTADA", "⚖️",
            OCEAN_COLORS["coral"], "weight"
        )
        
    def create_small_status_card(self, parent, col, title, icon, color, card_type):
        """Crear una tarjeta de estado pequeña"""
        card = ctk.CTkFrame(
            parent,
            fg_color=OCEAN_COLORS["ocean_blue"],
            corner_radius=15,
            border_width=2,
            border_color=color
        )
        card.grid(row=0, column=col, padx=8, pady=5, sticky="nsew")
        
        header = ctk.CTkFrame(card, fg_color=color, corner_radius=10, height=28)
        header.pack(fill="x", padx=8, pady=(8, 3))
        header.pack_propagate(False)
        
        title_label = ctk.CTkLabel(
            header,
            text=title,
            font=ctk.CTkFont(size=10, weight="bold"),
            text_color=OCEAN_COLORS["white"]
        )
        title_label.pack(pady=5)
        
        icon_label = ctk.CTkLabel(card, text=icon, font=ctk.CTkFont(size=24))
        icon_label.pack(pady=(3, 5))
        
        if card_type == "connection":
            self.connection_value = ctk.CTkLabel(
                card,
                text="ROS2 ACTIVO",
                font=ctk.CTkFont(size=12, weight="bold"),
                text_color=OCEAN_COLORS["seaweed"]
            )
            self.connection_value.pack(pady=1)
            
            self.connection_status_label = ctk.CTkLabel(
                card,
                text="Navegando",
                font=ctk.CTkFont(size=9),
                text_color=OCEAN_COLORS["light_aqua"]
            )
            self.connection_status_label.pack(pady=(0, 5))
            
        elif card_type == "battery":
            self.battery_value = ctk.CTkLabel(
                card,
                text="85%",
                font=ctk.CTkFont(size=16, weight="bold"),
                text_color=OCEAN_COLORS["seaweed"]
            )
            self.battery_value.pack(pady=1)
            
            self.voltage_label = ctk.CTkLabel(
                card,
                text="0.00 V",
                font=ctk.CTkFont(size=9),
                text_color=OCEAN_COLORS["light_aqua"]
            )
            self.voltage_label.pack(pady=(0, 5))
            
        elif card_type == "weight":
            self.weight_value = ctk.CTkLabel(
                card,
                text="0 G",
                font=ctk.CTkFont(size=14, weight="bold"),
                text_color=OCEAN_COLORS["gold"]
            )
            self.weight_value.pack(pady=1)
            
            self.weight_status_label = ctk.CTkLabel(
                card,
                text="Recolectado",
                font=ctk.CTkFont(size=9),
                text_color=OCEAN_COLORS["light_aqua"]
            )
            self.weight_status_label.pack(pady=(0, 5))
            
        return card
        
    def create_large_camera_card(self, parent):
        """Crear tarjeta de cámara"""
        container_frame = ctk.CTkFrame(parent, fg_color="transparent")
        container_frame.pack(expand=True, fill="both")
        
        camera_card = ctk.CTkFrame(
            container_frame,
            fg_color=OCEAN_COLORS["ocean_blue"],
            corner_radius=20,
            border_width=3,
            border_color=OCEAN_COLORS["aqua_blue"]
        )
        camera_card.pack(expand=True, padx=60, pady=10, fill="both")
        
        header = ctk.CTkFrame(
            camera_card,
            fg_color=OCEAN_COLORS["aqua_blue"],
            corner_radius=15,
            height=40
        )
        header.pack(fill="x", padx=15, pady=(15, 8))
        header.pack_propagate(False)
        
        title_label = ctk.CTkLabel(
            header,
            text="🎥 VISIÓN SUBMARINA 🎥",
            font=ctk.CTkFont(size=20, weight="bold"),
            text_color=OCEAN_COLORS["white"]
        )
        title_label.pack(pady=8)
        
        self.camera_frame = ctk.CTkFrame(
            camera_card,
            fg_color=OCEAN_COLORS["deep_blue"],
            corner_radius=15,
            border_width=2,
            border_color=OCEAN_COLORS["light_aqua"]
        )
        self.camera_frame.pack(padx=30, pady=(8, 10), fill="both", expand=True)
        
        self.camera_display = ctk.CTkLabel(
            self.camera_frame,
            text="🌊 CÁMARA SUBMARINA 🌊\n\nConectando con ESP32-CAM...\n\n🐠 🐟 🐙",
            font=ctk.CTkFont(size=22),
            text_color=OCEAN_COLORS["foam"],
            justify="center"
        )
        self.camera_display.pack(expand=True)
        
        indicator_frame = ctk.CTkFrame(camera_card, fg_color="transparent")
        indicator_frame.pack(fill="x", padx=20, pady=(0, 12))
        
        # Frame para indicador y botón en la misma línea
        status_row = ctk.CTkFrame(indicator_frame, fg_color="transparent")
        status_row.pack(fill="x")
        
        # Indicador de estado
        self.camera_indicator = ctk.CTkLabel(
            status_row,
            text="🟡 Conectando a cámara...",
            font=ctk.CTkFont(size=12, weight="bold"),
            text_color=OCEAN_COLORS["gold"]
        )
        self.camera_indicator.pack(side="left", pady=5)
        
        # Botón de reconexión manual
        reconnect_button = ctk.CTkButton(
            status_row,
            text="🔄 Reconectar",
            font=ctk.CTkFont(size=11, weight="bold"),
            width=120,
            height=30,
            fg_color=OCEAN_COLORS["sea_blue"],
            hover_color=OCEAN_COLORS["aqua_blue"],
            corner_radius=15,
            command=self.manual_reconnect_camera
        )
        reconnect_button.pack(side="right", padx=10)
        
        # Información de la cámara
        self.camera_info = ctk.CTkLabel(
            indicator_frame,
            text=f"Stream: {self.esp32_cam_url}",
            font=ctk.CTkFont(size=10),
            text_color=OCEAN_COLORS["light_aqua"]
        )
        self.camera_info.pack()

        return camera_card

    def show_manual(self):
        """Mostrar manual"""
        manual_text = "🌊 MANUAL DEL NAVEGANTE - SALVATORE 🌊\n\n"
        messagebox.showinfo("🌊 Manual del Navegante", manual_text)
        
    def show_about(self):
        """Mostrar about"""
        about_text = "🌊 SALVATORE - ROV Marino 🌊\n\n"
        messagebox.showinfo("🌊 Acerca de SALVATORE", about_text)
        
    def show_settings(self):
        """Mostrar configuraciones"""
        settings_text = "⚙️ CONFIGURACIÓN MARINA ⚙️\n\n"
        messagebox.showinfo("⚙️ Configuración Marina", settings_text)

    def run(self):
        """Ejecutar la interfaz"""
        try:
            self.root.mainloop()
        except Exception as e:
            self.get_logger().error(f'Error en interfaz: {e}')
        finally:
            self.cleanup()
        
    def cleanup(self):
        """Limpieza al cerrar"""
        self.animation_running = False
        self.stop_esp32_camera_stream()
        if hasattr(self, 'root'):
            try:
                self.root.quit()
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    interface = AquaCleanInterface()

    ros_thread = threading.Thread(target=rclpy.spin, args=(interface,), daemon=True)
    ros_thread.start()

    try:
        interface.run()
    except KeyboardInterrupt:
        pass
    finally:
        interface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()