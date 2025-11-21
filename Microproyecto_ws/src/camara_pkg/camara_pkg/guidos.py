#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import customtkinter as ctk
from PIL import Image as PILImage, ImageTk
import threading
import tkinter as tk
from tkinter import messagebox
import queue
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import requests
from io import BytesIO

# Configurar CustomTkinter con tema marino
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# Colores tema marino
OCEAN_COLORS = {
    "deep_blue": "#000000",
    "ocean_blue": "#01115A",
    "sea_blue": "#1A2531",
    "aqua_blue": "#2E8BC0",
    "light_aqua": "#52B2CF",
    "foam": "#B8D4E3",
    "white": "#FFFFFF",
    "seaweed": "#FFFFFF",
    "coral": "#000000",
    "gold": "#050505",
}

class AquaCleanInterface(Node):
    def __init__(self):
        super().__init__('aquaclean_interface')
        self.cv_bridge = CvBridge()

        # Suscriptor ROS2 (opcional)
        self.image_subscriber = self.create_subscription(
            RosImage,
            '/salvatore/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Queue OPTIMIZADA con tamaño limitado (solo frame más reciente)
        self.gui_queue = queue.Queue(maxsize=2)
        
        # Variables de estado
        self.connection_status = "DESCONECTADO"
        self.battery_level = 85.0
        self.battery_voltage = 12.6
        self.cargo_weight = 15.2
        self.camera_active = False
        self.current_image = None
        self.frames_received = 0
        self.current_fps = 0.0
        self.last_frame_time = time.time()
        self.display_fps = 0.0
        
        # ============================================
        # CONFIGURACIÓN HTTP STREAM ESP32-CAM
        # ============================================
        self.esp32_stream_url = "http://10.94.210.1/stream"
        self.stream_thread = None
        self.stream_running = False
        
        # Iniciar stream ESP32-CAM
        self.start_esp32_stream()   
        
        # Inicializar interfaz gráfica
        self.init_gui()
        
        self.get_logger().info('🌊 ANDROMEDA Interface iniciada')
        self.get_logger().info(f'📹 Stream ESP32-CAM: {self.esp32_stream_url}')

    def start_esp32_stream(self):
        """Iniciar captura de stream HTTP del ESP32-CAM"""
        def stream_worker():
            self.stream_running = True
            self.get_logger().info(f'🔌 Conectando a ESP32-CAM stream...')
            
            while self.stream_running:
                try:
                    # Conectar al stream MJPEG
                    response = requests.get(self.esp32_stream_url, stream=True, timeout=5)
                    
                    if response.status_code == 200:
                        self.connection_status = "CONECTADO"
                        self.camera_active = True
                        self.get_logger().info('✅ Conectado a ESP32-CAM')
                        
                        bytes_data = bytes()
                        
                        # Leer stream MJPEG
                        for chunk in response.iter_content(chunk_size=1024):
                            if not self.stream_running:
                                break
                                
                            bytes_data += chunk
                            
                            # Buscar inicio y fin de frame JPEG
                            a = bytes_data.find(b'\xff\xd8')  # Inicio JPEG
                            b = bytes_data.find(b'\xff\xd9')  # Fin JPEG
                            
                            if a != -1 and b != -1:
                                jpg = bytes_data[a:b+2]
                                bytes_data = bytes_data[b+2:]
                                
                                # Decodificar JPEG
                                nparr = np.frombuffer(jpg, np.uint8)
                                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                                
                                if frame is not None:
                                    # Convertir BGR a RGB
                                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                                    
                                    # Si la cola está llena, descartar frame viejo
                                    if self.gui_queue.full():
                                        try:
                                            self.gui_queue.get_nowait()
                                        except queue.Empty:
                                            pass
                                    
                                    # Enviar a cola
                                    try:
                                        self.gui_queue.put_nowait(frame_rgb)
                                        self.frames_received += 1
                                    except queue.Full:
                                        pass
                    else:
                        self.get_logger().error(f'❌ Error HTTP: {response.status_code}')
                        self.connection_status = "ERROR"
                        self.camera_active = False
                        time.sleep(2)
                        
                except requests.exceptions.ConnectionError:
                    self.get_logger().warn('⚠️  No se puede conectar al ESP32-CAM')
                    self.connection_status = "DESCONECTADO"
                    self.camera_active = False
                    time.sleep(2)
                    
                except requests.exceptions.Timeout:
                    self.get_logger().warn('⚠️  Timeout conectando a ESP32-CAM')
                    self.connection_status = "TIMEOUT"
                    self.camera_active = False
                    time.sleep(2)
                    
                except Exception as e:
                    self.get_logger().error(f'❌ Error en stream: {e}')
                    self.connection_status = "ERROR"
                    self.camera_active = False
                    time.sleep(2)
        
        # Iniciar thread del stream
        self.stream_thread = threading.Thread(target=stream_worker, daemon=True)
        self.stream_thread.start()
    
    def stop_esp32_stream(self):
        """Detener stream ESP32-CAM"""
        self.stream_running = False
        if self.stream_thread:
            self.stream_thread.join(timeout=2)
    
    def image_callback(self, msg):
        """Recibir imagen ROS2 (opcional)"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            if not self.gui_queue.full():
                self.gui_queue.put_nowait(cv_image)
        except Exception as e:
            self.get_logger().error(f"❌ Error ROS2: {e}")

    def init_gui(self):
        """Inicializar la interfaz gráfica"""
        
        self.root = ctk.CTk()
        self.root.title("ANDROMEDA - Robot Marino")
        self.root.geometry("900x800")
        self.root.configure(fg_color=OCEAN_COLORS["deep_blue"])
        
        self.current_screen = "lobby"
        self.animation_running = True
        
        self.configure_colors()
        self.create_lobby_screen()
        
        # Timer OPTIMIZADO - procesamiento más rápido
        self.process_gui_updates()
    
    def configure_colors(self):
        """Configurar colores personalizados"""
        ctk.ThemeManager.theme["CTkFrame"]["fg_color"] = [OCEAN_COLORS["ocean_blue"], OCEAN_COLORS["ocean_blue"]]
        ctk.ThemeManager.theme["CTkButton"]["fg_color"] = [OCEAN_COLORS["sea_blue"], OCEAN_COLORS["aqua_blue"]]
        ctk.ThemeManager.theme["CTkButton"]["hover_color"] = [OCEAN_COLORS["light_aqua"], OCEAN_COLORS["light_aqua"]]

    def process_gui_updates(self):
        """Procesar imágenes OPTIMIZADO para mínima latencia"""
        try:
            # Procesar TODOS los frames disponibles (descartar viejos)
            processed = False
            while not self.gui_queue.empty():
                try:
                    cv_image = self.gui_queue.get_nowait()
                    processed = True
                except queue.Empty:
                    break
            
            # Mostrar solo el frame más reciente
            if processed:
                # Calcular FPS de display
                current_time = time.time()
                time_diff = current_time - self.last_frame_time
                if time_diff > 0:
                    self.display_fps = 1.0 / time_diff
                self.last_frame_time = current_time
                
                # Redimensionar con interpolación rápida
                height, width = cv_image.shape[:2]
                target_width, target_height = 640, 480
                
                # Usar cv2.resize (más rápido que PIL)
                cv_resized = cv2.resize(
                    cv_image, 
                    (target_width, target_height),
                    interpolation=cv2.INTER_LINEAR
                )
                
                # Convertir a PIL y luego a Tkinter
                image_pil = PILImage.fromarray(cv_resized)
                image_tk = ImageTk.PhotoImage(image_pil)

                if hasattr(self, 'camera_display'):
                    self.camera_display.configure(image=image_tk, text="")
                    self.camera_display.image = image_tk
                    
                    # Actualizar indicador
                    if hasattr(self, 'camera_indicator'):
                        self.camera_indicator.configure(
                            text="🟢 Cámara ESP32 activa",
                            text_color=OCEAN_COLORS["seaweed"]
                        )
                    
                    # Actualizar info con FPS
                    if hasattr(self, 'camera_info'):
                        info_text = f"Resolución: {width}x{height} | "
                        info_text += f"Display: {self.display_fps:.1f} FPS | "
                        info_text += f"Frames: {self.frames_received}"
                        self.camera_info.configure(text=info_text)
                        
        except Exception as e:
            self.get_logger().error(f"❌ Error actualizando GUI: {e}")

        if hasattr(self, 'root'):
            # Actualizar cada 20ms (50 FPS máximo de display)
            self.root.after(20, self.process_gui_updates)

    def create_lobby_screen(self):
        """Crear pantalla de lobby marina"""
        
        for widget in self.root.winfo_children():
            widget.destroy()
            
        main_frame = ctk.CTkFrame(
            self.root,
            fg_color=OCEAN_COLORS["deep_blue"],
            corner_radius=0
        )
        main_frame.pack(fill="both", expand=True)
        
        header_frame = ctk.CTkFrame(
            main_frame,
            fg_color="transparent",
            height=200
        )
        
        header_frame.pack(fill="x", padx=20, pady=(40, 20))
        
        title_label = ctk.CTkLabel(
            header_frame,
            text="🌊 ANDROMEDA 🌊",
            font=ctk.CTkFont(size=56, weight="bold"),
            text_color=OCEAN_COLORS["foam"]
        )
        title_label.pack(pady=(10, 10))
        
        subtitle_label = ctk.CTkLabel(
            header_frame,
            text="ROV para la recolección de basura superficial acuatica",
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
        
        center_frame = ctk.CTkFrame(
            main_frame,
            fg_color="transparent"
        )
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
            ("ℹ️ ABOUT US!", self.show_about, OCEAN_COLORS["sea_blue"]),
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
        """Mostrar dashboard principal marino"""
        
        self.current_screen = "dashboard"
        
        for widget in self.root.winfo_children():
            widget.destroy()
            
        main_frame = ctk.CTkFrame(
            self.root,
            fg_color=OCEAN_COLORS["deep_blue"],
            corner_radius=0
        )
        main_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        self.create_dashboard_header(main_frame)
        
        content_frame = ctk.CTkFrame(
            main_frame,
            fg_color="transparent"
        )
        content_frame.pack(fill="both", expand=True, padx=20, pady=10)
        
        top_frame = ctk.CTkFrame(
            content_frame,
            fg_color="transparent",
            height=130
        )
        top_frame.pack(fill="x", pady=(0, 15))
        top_frame.pack_propagate(False)
        
        top_frame.grid_columnconfigure(0, weight=1)
        top_frame.grid_columnconfigure(1, weight=1)
        top_frame.grid_columnconfigure(2, weight=1)
        top_frame.grid_rowconfigure(0, weight=1)
        
        self.create_small_status_cards(top_frame)
        
        bottom_frame = ctk.CTkFrame(
            content_frame,
            fg_color="transparent"
        )
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
        
        header_content = ctk.CTkFrame(
            header_frame,
            fg_color="transparent"
        )
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
        
        title_frame = ctk.CTkFrame(
            header_content,
            fg_color="transparent"
        )
        title_frame.pack(side="left", expand=True, fill="x")
        
        dashboard_title = ctk.CTkLabel(
            title_frame,
            text="🌊 ANDROMEDA - PANEL DE CONTROL 🌊",
            font=ctk.CTkFont(size=32, weight="bold"),
            text_color=OCEAN_COLORS["foam"]
        )
        dashboard_title.pack(pady=5)

    def create_small_status_cards(self, parent):
        """Crear 3 tarjetas pequeñas en fila superior"""
        
        self.connection_card = self.create_small_status_card(
            parent, 0, "CONEXIÓN ESP32", "🌐", 
            OCEAN_COLORS["aqua_blue"], "connection"
        )
        
        self.battery_card = self.create_small_status_card(
            parent, 1, "BATERÍA MARINA", "🔋",
            OCEAN_COLORS["aqua_blue"], "battery"
        )
        
        self.weight_card = self.create_small_status_card(
            parent, 2, "CARGA RECOLECTADA", "⚖️",
            OCEAN_COLORS["aqua_blue"], "weight"
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
        
        header = ctk.CTkFrame(
            card,
            fg_color=color,
            corner_radius=10,
            height=28
        )
        header.pack(fill="x", padx=8, pady=(8, 3))
        header.pack_propagate(False)
        
        title_label = ctk.CTkLabel(
            header,
            text=title,
            font=ctk.CTkFont(size=10, weight="bold"),
            text_color=OCEAN_COLORS["white"]
        )
        title_label.pack(pady=5)
        
        icon_label = ctk.CTkLabel(
            card,
            text=icon,
            font=ctk.CTkFont(size=24)
        )
        icon_label.pack(pady=(3, 5))
        
        if card_type == "connection":
            self.connection_value = ctk.CTkLabel(
                card,
                text="HTTP STREAM" if self.connection_status == "CONECTADO" else "DESCONECTADO",
                font=ctk.CTkFont(size=12, weight="bold"),
                text_color=OCEAN_COLORS["seaweed"] if self.connection_status == "CONECTADO" else OCEAN_COLORS["coral"]
            )
            self.connection_value.pack(pady=1)
            
            self.connection_status_label = ctk.CTkLabel(
                card,
                text="192.168.72.1",
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
            
            self.battery_status_label = ctk.CTkLabel(
                card,
                text="12.6V",
                font=ctk.CTkFont(size=9),
                text_color=OCEAN_COLORS["light_aqua"]
            )
            self.battery_status_label.pack(pady=(0, 5))
            
        elif card_type == "weight":
            self.weight_value = ctk.CTkLabel(
                card,
                text="15.2 KG",
                font=ctk.CTkFont(size=14, weight="bold"),
                text_color=OCEAN_COLORS["seaweed"]
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
        
        container_frame = ctk.CTkFrame(
            parent,
            fg_color="transparent"
        )
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
            text="🎥 CÁMARA ESP32-CAM 🎥",
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
            text="🌊 Conectando a ESP32-CAM... 🌊",
            font=ctk.CTkFont(size=22),
            text_color=OCEAN_COLORS["foam"],
            justify="center"
        )
        self.camera_display.pack(expand=True)
        
        indicator_frame = ctk.CTkFrame(
            camera_card,
            fg_color="transparent"
        )
        indicator_frame.pack(fill="x", padx=20, pady=(0, 12))
        
        self.camera_indicator = ctk.CTkLabel(
            indicator_frame,
            text="🔴 Conectando a HTTP stream...",
            font=ctk.CTkFont(size=12, weight="bold"),
            text_color=OCEAN_COLORS["coral"]
        )
        self.camera_indicator.pack(pady=5)
        
        self.camera_info = ctk.CTkLabel(
            indicator_frame,
            text="http://192.168.72.1/stream",
            font=ctk.CTkFont(size=10),
            text_color=OCEAN_COLORS["light_aqua"]
        )
        self.camera_info.pack()
        
        return camera_card

    def show_manual(self):
        """Mostrar manual marino"""
        manual_text = """
        🌊 MANUAL DEL NAVEGANTE - ANDROMEDA 🌊
        
        1. Presiona START para acceder al panel de control
        2. La cámara ESP32-CAM se conecta automáticamente via HTTP
        3. El stream se obtiene de: http://192.168.72.1/stream
        4. Monitorea el estado de conexión en tiempo real
        
        CARACTERÍSTICAS:
        - Stream MJPEG directo desde ESP32-CAM
        - Baja latencia (sin broker intermedio)
        - Reconexión automática en caso de fallo
        """
        messagebox.showinfo("🌊 Manual del Navegante", manual_text)
        
    def show_about(self):
        """Mostrar información"""
        about_text = """
        🌊 ANDROMEDA - Robot Marino de Limpieza 🌊
        
        Sistema de visión con ESP32-CAM
        Stream HTTP directo para baja latencia
        Versión 2.0 - Sin MQTT
        """
        messagebox.showinfo("🌊 Acerca de ANDROMEDA", about_text)
        
    def show_settings(self):
        """Mostrar configuraciones"""
        settings_text = f"""
        ⚙️ CONFIGURACIÓN ⚙️
        
        Stream URL: {self.esp32_stream_url}
        Protocolo: HTTP (MJPEG)
        Queue: Limitada a 2 frames
        Update: 20ms (50 FPS display)
        Reconexión: Automática
        """
        messagebox.showinfo("⚙️ Configuración", settings_text)

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
        self.stop_esp32_stream()
        if hasattr(self, 'root'):
            try:
                self.root.quit()
            except:
                pass

    def __del__(self):
        """Destructor"""
        self.cleanup()

def main(args=None):
    rclpy.init(args=args)
    interface = AquaCleanInterface()

    # Hilo separado para ROS2
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