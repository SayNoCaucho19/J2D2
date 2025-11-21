 #!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import customtkinter as ctk
from PIL import Image as PILImage, ImageTk, ImageDraw
import threading
import tkinter as tk
from tkinter import messagebox
import queue
import math
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Configurar CustomTkinter con tema marino
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# Colores tema marino
OCEAN_COLORS = {
    "deep_blue": "#0B2545",      # Azul profundo
    "ocean_blue": "#134E6F",     # Azul océano
    "sea_blue": "#1B6EC2",       # Azul mar
    "aqua_blue": "#2E8BC0",      # Azul agua
    "light_aqua": "#52B2CF",     # Azul claro
    "foam": "#B8D4E3",           # Espuma de mar
    "white": "#FFFFFF",          # Blanco
    "coral": "#FF7F7F",          # Coral para alertas
    "seaweed": "#4F7942",        # Verde alga
    "gold": "#FFD700",           # Oro para acentos
}

class AquaCleanInterface(Node):
    def __init__(self):
        super().__init__('aquaclean_interface')
                # ---- SUSCRIPCIÓN AL VOLTAJE ----
        from std_msgs.msg import Float32
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.voltage_sub = self.create_subscription(
            Float32,
            '/voltage_data',
            self.voltage_callback,
            qos
        )

        self.voltage_value = 0.0

        self.cv_bridge = CvBridge()

        self.image_subscriber = self.create_subscription(RosImage,'/salvatore/camera/image_raw',self.image_callback,10)
        

        

        
        # Queue para comunicación thread-safe entre ROS2 y GUI
        self.gui_queue = queue.Queue()
        self.queue = queue.Queue()

        
        # Variables de estado (thread-safe)
        self.connection_status = "DESCONECTADO"
        self.battery_level = 85.0  # Placeholder
        self.battery_voltage = 12.6
        self.cargo_weight = 15.2
        self.camera_active = False
        self.current_image = None
        self.voltage_label = None

        # Inicializar interfaz gráfica
        self.init_gui()
        
        self.get_logger().info('SALVATORE Interface Marina iniciada')

    
    def image_callback(self, msg):
        """Recibir imagen ROS2 y pasarla a la cola de la GUI"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self.gui_queue.put(cv_image)
        except Exception as e:
            self.get_logger().error(f"❌ Error procesando imagen ROS2: {e}")


    # ---- CALLBACK DEL VOLTAJE ----
    def voltage_callback(self, msg):
        print("Voltaje recibido:", msg.data)
        self.voltage_value = msg.data
        self.queue.put(("voltage", self.voltage_value))


    

    def init_gui(self):
        """Inicializar la interfaz gráfica con tema marino"""
        
        # Ventana principal con colores marinos
        self.root = ctk.CTk()
        self.root.title("SALVATORE - Robot Marino")
        self.root.geometry("900x800")
        self.root.configure(fg_color=OCEAN_COLORS["deep_blue"])
        
        # Variables para la interfaz
        self.current_screen = "lobby"
        self.animation_running = True
        
        # Configurar colores personalizados
        self.configure_colors()
        
        # Inicializar pantalla de lobby
        self.create_lobby_screen()
        
        # Timer para procesar queue de GUI
        self.process_gui_updates()
        
    
    
    def configure_colors(self):
        """Configurar colores personalizados"""
        # Personalizar colores de CTk
        ctk.ThemeManager.theme["CTkFrame"]["fg_color"] = [OCEAN_COLORS["ocean_blue"], OCEAN_COLORS["ocean_blue"]]
        ctk.ThemeManager.theme["CTkButton"]["fg_color"] = [OCEAN_COLORS["sea_blue"], OCEAN_COLORS["aqua_blue"]]
        ctk.ThemeManager.theme["CTkButton"]["hover_color"] = [OCEAN_COLORS["light_aqua"], OCEAN_COLORS["light_aqua"]]

    def process_gui_updates(self):
        try:
            if not self.gui_queue.empty():
                cv_image = self.gui_queue.get()

                image_pil = PILImage.fromarray(cv_image)
                image_pil = image_pil.resize((640, 480))
                image_tk = ImageTk.PhotoImage(image_pil)

                self.camera_display.configure(image=image_tk, text="")
                self.camera_display.image = image_tk

            if not self.queue.empty():
                event = self.queue.get()
                if event[0] == "voltage" and self.voltage_label is not None:
                    new_voltage = event[1]
                    self.voltage_label.configure(text=f"{new_voltage:.2f} V")

            # ← ← ← AGREGAR ESTO SIEMPRE
            if self.voltage_label is not None:
                self.voltage_label.configure(text=f"{self.voltage_value:.2f} V")

        except Exception as e:
            self.get_logger().error(f"❌ Error actualizando GUI: {e}")

        self.root.after(50, self.process_gui_updates)




    def create_lobby_screen(self):
        """Crear pantalla de lobby marina"""
        
        # Limpiar ventana
        for widget in self.root.winfo_children():
            widget.destroy()
            
        # Frame principal con gradiente marino
        main_frame = ctk.CTkFrame(
            self.root,
            fg_color=OCEAN_COLORS["deep_blue"],
            corner_radius=0
        )
        main_frame.pack(fill="both", expand=True)
        
        # Header con título animado
        header_frame = ctk.CTkFrame(
            main_frame,
            fg_color="transparent",
            height=200
        )
        
        header_frame.pack(fill="x", padx=20, pady=(40, 20))
        
        # Título principal con efectos
        title_label = ctk.CTkLabel(
            header_frame,
            text="🌊 SALVATORE 🌊",
            font=ctk.CTkFont(size=56, weight="bold"),
            text_color=OCEAN_COLORS["foam"]
        )
        title_label.pack(pady=(10, 10))
        
        # Subtítulo con descripción
        subtitle_label = ctk.CTkLabel(
            header_frame,
            text="ROV para la recolección de basura superficial acuatica",
            font=ctk.CTkFont(size=22, weight="normal"),
            text_color=OCEAN_COLORS["light_aqua"]
        )
        subtitle_label.pack(pady=(0, 10))
        
        # Línea decorativa
        line_frame = ctk.CTkFrame(
            header_frame,
            height=4,
            fg_color=OCEAN_COLORS["aqua_blue"]
        )
        line_frame.pack(fill="x", padx=200, pady=20)
        
        # Frame central para botones
        center_frame = ctk.CTkFrame(
            main_frame,
            fg_color="transparent"
        )
        center_frame.pack(expand=True, fill="both", padx=40, pady=20)
        
        # Frame para botones con diseño marino
        button_frame = ctk.CTkFrame(
            center_frame,
            fg_color=OCEAN_COLORS["ocean_blue"],
            corner_radius=25,
            border_width=2,
            border_color=OCEAN_COLORS["aqua_blue"]
        )
        button_frame.pack(pady=40, padx=100)
        
        # Botones del menú con estilo marino
        buttons_data = [
            ("START", self.show_dashboard, OCEAN_COLORS["sea_blue"]),
            ("MANUAL DE USUARIO", self.show_manual, OCEAN_COLORS["aqua_blue"]),
            ("ℹABOUT US!", self.show_about, OCEAN_COLORS["light_aqua"]),
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
        """Mostrar dashboard principal marino con nuevo layout mejorado"""
        
        self.current_screen = "dashboard"
        
        # Limpiar ventana
        for widget in self.root.winfo_children():
            widget.destroy()
            
        # Frame principal
        main_frame = ctk.CTkFrame(
            self.root,
            fg_color=OCEAN_COLORS["deep_blue"],
            corner_radius=0
        )
        main_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        # Header del dashboard
        self.create_dashboard_header(main_frame)
        
        # Frame principal para todo el contenido del dashboard
        content_frame = ctk.CTkFrame(
            main_frame,
            fg_color="transparent"
        )
        content_frame.pack(fill="both", expand=True, padx=20, pady=10)
        
        # Frame superior para tarjetas pequeñas en fila (MÁS CORTO)
        top_frame = ctk.CTkFrame(
            content_frame,
            fg_color="transparent",
            height=130  # Reducido de 180 a 130
        )
        top_frame.pack(fill="x", pady=(0, 15))
        top_frame.pack_propagate(False)  # Mantener altura fija
        
        # Configurar grid para 3 columnas iguales
        top_frame.grid_columnconfigure(0, weight=1)
        top_frame.grid_columnconfigure(1, weight=1)
        top_frame.grid_columnconfigure(2, weight=1)
        top_frame.grid_rowconfigure(0, weight=1)
        
        # Crear las 3 tarjetas pequeñas superiores (más cortas)
        self.create_small_status_cards(top_frame)
        
        # Frame inferior para la cámara grande y MÁS CUADRADA
        bottom_frame = ctk.CTkFrame(
            content_frame,
            fg_color="transparent"
        )
        bottom_frame.pack(fill="both", expand=True, pady=(15, 0))
        
        # Crear la tarjeta de cámara grande y más cuadrada
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
        
        # Frame interno para layout
        header_content = ctk.CTkFrame(
            header_frame,
            fg_color="transparent"
        )
        header_content.pack(fill="both", expand=True, padx=20, pady=15)
        
        # Botón volver (izquierda)
        back_button = ctk.CTkButton(
            header_content,
            text="← Volver",
            font=ctk.CTkFont(size=11, weight="bold"),
            width=150,
            height=40,
            fg_color=OCEAN_COLORS["aqua_blue"],
            hover_color= OCEAN_COLORS["gold"],
            corner_radius=20,
            command=self.create_lobby_screen
        )
        back_button.pack(side="left")
        
        # Título central
        title_frame = ctk.CTkFrame(
            header_content,
            fg_color="transparent"
        )
        title_frame.pack(side="left", expand=True, fill="x")
        
        dashboard_title = ctk.CTkLabel(
            title_frame,
            text="🌊 SALVATORE- PANEL DE CONTROL🌊",
            font=ctk.CTkFont(size=32, weight="bold"),
            text_color=OCEAN_COLORS["foam"]
        )
        dashboard_title.pack(pady=5)

    def create_small_status_cards(self, parent):
        """Crear 3 tarjetas pequeñas y más cortas en fila superior"""
        
        # Tarjeta de Conexión
        self.connection_card = self.create_small_status_card(
            parent, 0, "CONEXIÓN ROS2", "🌐", 
            OCEAN_COLORS["sea_blue"], "connection"
        )
        
        # Tarjeta de Batería  
        self.battery_card = self.create_small_status_card(
            parent, 1, "BATERÍA MARINA", "🔋",
            OCEAN_COLORS["seaweed"], "battery"
        )
        
        # Tarjeta de Peso
        self.weight_card = self.create_small_status_card(
            parent, 2, "CARGA RECOLECTADA", "⚖️",
            OCEAN_COLORS["coral"], "weight"
        )
        
    def create_small_status_card(self, parent, col, title, icon, color, card_type):
        """Crear una tarjeta de estado pequeña y más corta"""
        
        card = ctk.CTkFrame(
            parent,
            fg_color=OCEAN_COLORS["ocean_blue"],
            corner_radius=15,
            border_width=2,
            border_color=color
        )
        card.grid(row=0, column=col, padx=8, pady=5, sticky="nsew")
        
        # Header más compacto (reducido)
        header = ctk.CTkFrame(
            card,
            fg_color=color,
            corner_radius=10,
            height=28  # Reducido de 35 a 28
        )
        header.pack(fill="x", padx=8, pady=(8, 3))  # Menos padding
        header.pack_propagate(False)
        
        # Título más compacto
        title_label = ctk.CTkLabel(
            header,
            text=title,
            font=ctk.CTkFont(size=10, weight="bold"),  # Reducido de 12 a 10
            text_color=OCEAN_COLORS["white"]
        )
        title_label.pack(pady=5)  # Reducido padding
        
        # Icono más pequeño
        icon_label = ctk.CTkLabel(
            card,
            text=icon,
            font=ctk.CTkFont(size=24)  # Reducido de 32 a 24
        )
        icon_label.pack(pady=(3, 5))  # Menos padding
        
        # Valor principal compacto
        if card_type == "connection":
            self.connection_value = ctk.CTkLabel(
                card,
                text="ROS2 ACTIVO",
                font=ctk.CTkFont(size=12, weight="bold"),  # Reducido de 14 a 12
                text_color=OCEAN_COLORS["seaweed"]
            )
            self.connection_value.pack(pady=1)
            
            self.connection_status_label = ctk.CTkLabel(
                card,
                text="Navegando",
                font=ctk.CTkFont(size=9),  # Reducido de 10 a 9
                text_color=OCEAN_COLORS["light_aqua"]
            )
            self.connection_status_label.pack(pady=(0, 5))
            
        elif card_type == "battery":
            self.battery_value = ctk.CTkLabel(
                card,
                text="85%",
                font=ctk.CTkFont(size=16, weight="bold"),  # Reducido de 18 a 16
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
                text="15.2 KG",
                font=ctk.CTkFont(size=14, weight="bold"),  # Reducido de 16 a 14
                text_color=OCEAN_COLORS["gold"]
            )
            self.weight_value.pack(pady=1)
            
            self.weight_status_label = ctk.CTkLabel(
                card,
                text="Recolectado",
                font=ctk.CTkFont(size=9),  # Reducido de 10 a 9
                text_color=OCEAN_COLORS["light_aqua"]
            )
            self.weight_status_label.pack(pady=(0, 5))
            
        return card
        
    def create_large_camera_card(self, parent):
        """Crear tarjeta de cámara más cuadrada y proporcionada"""
        
        # Frame contenedor para centrar la cámara
        container_frame = ctk.CTkFrame(
            parent,
            fg_color="transparent"
        )
        container_frame.pack(expand=True, fill="both")
        
        # Tarjeta de cámara más cuadrada - controlar mejor las proporciones
        camera_card = ctk.CTkFrame(
            container_frame,
            fg_color=OCEAN_COLORS["ocean_blue"],
            corner_radius=20,
            border_width=3,
            border_color=OCEAN_COLORS["aqua_blue"]
        )
        # Hacer la tarjeta más cuadrada limitando su expansión horizontal
        camera_card.pack(expand=True, padx=60, pady=10, fill="both")  # Más padding horizontal
        
        # Header de la cámara más pequeño
        header = ctk.CTkFrame(
            camera_card,
            fg_color=OCEAN_COLORS["aqua_blue"],
            corner_radius=15,
            height=40  # Reducido de 50 a 40
        )
        header.pack(fill="x", padx=15, pady=(15, 8))  # Menos padding
        header.pack_propagate(False)
        
        title_label = ctk.CTkLabel(
            header,
            text="🎥 VISIÓN SUBMARINA 🎥",
            font=ctk.CTkFont(size=20, weight="bold"),  # Reducido de 24 a 20
            text_color=OCEAN_COLORS["white"]
        )
        title_label.pack(pady=8)  # Reducido de 12 a 8
        
        # Frame principal para la cámara (más cuadrado y centrado)
        self.camera_frame = ctk.CTkFrame(
            camera_card,
            fg_color=OCEAN_COLORS["deep_blue"],
            corner_radius=15,
            border_width=2,
            border_color=OCEAN_COLORS["light_aqua"]
        )
        # Limitar la expansión para hacer más cuadrado
        self.camera_frame.pack(padx=30, pady=(8, 10), fill="both", expand=True)  # Más padding horizontal
        
        # Display de la cámara
        self.camera_display = ctk.CTkLabel(
            self.camera_frame,
            text="🌊 CÁMARA SUBMARINA 🌊\n\nConectando con el océano...\n\n🐠 🐟 🐙",
            font=ctk.CTkFont(size=22),  # Reducido de 24 a 22
            text_color=OCEAN_COLORS["foam"],
            justify="center"
        )
        self.camera_display.pack(expand=True)
        
        # Frame inferior para indicadores más compacto
        indicator_frame = ctk.CTkFrame(
            camera_card,
            fg_color="transparent"
        )
        indicator_frame.pack(fill="x", padx=20, pady=(0, 12))  # Menos padding
        
        # Indicador de estado de cámara
        self.camera_indicator = ctk.CTkLabel(
            indicator_frame,
            text="🔴 Cámara en modo exploración marina",
            font=ctk.CTkFont(size=12, weight="bold"),  # Reducido de 14 a 12
            text_color=OCEAN_COLORS["coral"]
        )
        self.camera_indicator.pack(pady=5)  # Reducido de 8 a 5
        
        # Información adicional
        self.camera_info = ctk.CTkLabel(
            indicator_frame,
            text="Resolución: 1080p | Visión nocturna: ON | Zoom: 1.0x",
            font=ctk.CTkFont(size=10),  # Reducido de 12 a 10
            text_color=OCEAN_COLORS["light_aqua"]
        )
        self.camera_info.pack()


        
        return camera_card

            
    def update_connection_display(self, status):
        """Actualizar display de conexión"""
        pass  # Placeholder
        
    def update_battery_display(self, data):
        """Actualizar display de batería"""
        pass  # Placeholder
            
    def update_weight_display(self, weight):
        """Actualizar display de peso"""
        pass  # Placeholder
            
    def update_camera_display(self, cv_image):
        """Actualizar display de cámara"""
        pass  # Placeholder

    def show_manual(self):
        """Mostrar manual marino"""
        manual_text = """
        🌊 MANUAL DEL NAVEGANTE - AQUACLEAN 🌊
        
        
        """
        messagebox.showinfo("🌊 Manual del Navegante", manual_text)
        
    def show_about(self):
        """"""
        about_text = """
       
        """
        messagebox.showinfo("🌊 Acerca de AquaClean", about_text)
        
    def show_settings(self):
        """Mostrar configuraciones marinas"""
        settings_text = """
    
        """
        messagebox.showinfo("⚙️ Configuración Marina", settings_text)

    def run(self):
        """Ejecutar la interfaz marina"""
        # Ejecutar interfaz gráfica en el hilo principal
        try:
            self.root.mainloop()
        except Exception as e:
            self.get_logger().error(f'Error en interfaz: {e}')
        finally:
            self.cleanup()
        
    def cleanup(self):
        """Limpieza al cerrar"""
        self.animation_running = False
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
        interface.run()   # Esto mantiene la GUI viva
    except KeyboardInterrupt:
        pass
    finally:
        interface.destroy_node()
        rclpy.shutdown()

    

if __name__ == '__main__':
    main()  
