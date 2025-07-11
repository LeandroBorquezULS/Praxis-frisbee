import tkinter as tk
from tkinter import messagebox, simpledialog, scrolledtext
import threading
from datetime import datetime
import socket


import ESP32

class ESPApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Control ESP32")
        self.sock = None
        self.esp_addr = None
        self.escucha_activa = threading.Event()
        self.hilo_escucha = None

        self.status_label = tk.Label(root, text="Estado: Desconectado", fg="red", font=("Arial", 12))
        self.status_label.pack(pady=10)

        self.btn_conectar_esp = tk.Button(root, text="Conectar ESP", command=self.conectar_esp)
        self.btn_conectar_esp.pack(pady=5)

        self.btn_calibrar_esp = tk.Button(root, text="Calibrar ESP", command=self.calibrar_esp)
        self.btn_calibrar_esp.pack(pady=5)

        self.btn_info_esp = tk.Button(root, text="Pedir INFO ESP", command=self.info_esp)
        self.btn_info_esp.pack(pady=5)

        self.btn_iniciar = tk.Button(root, text="Iniciar Lanzamiento", command=self.iniciar_bucle)
        self.btn_iniciar.pack(pady=5)

        self.btn_escuchar = tk.Button(root, text="Escuchar ESP", command=self.escuchar_esp, state=tk.DISABLED)
        self.btn_escuchar.pack(pady=5)

        self.btn_tcp = tk.Button(root, text="Iniciar TCP con ESP", command=self.iniciar_tcp)
        self.btn_tcp.pack(pady=5)

        self.btn_wifi = tk.Button(root, text="Conectar WiFi", command=self.conectar_wifi)
        self.btn_wifi.pack(pady=5)

        tk.Label(root, text="Consola de mensajes:", font=("Arial", 10)).pack(pady=(10, 0))
        self.text_area = scrolledtext.ScrolledText(root, width=60, height=15, state=tk.DISABLED)
        self.text_area.pack(padx=10, pady=5)

    def actualizar_estado(self, conectado, ip=None):
        if conectado:
            self.status_label.config(text=f"Estado: Conectado ({ip})", fg="green")
        else:
            self.status_label.config(text="Estado: Desconectado", fg="red")

    def log_mensaje(self, mensaje):
        self.text_area.config(state=tk.NORMAL)
        self.text_area.insert(tk.END, mensaje + "\n")
        self.text_area.see(tk.END)
        self.text_area.config(state=tk.DISABLED)

    def conectar_esp(self):
        def tarea_conectar():
            self.sock = ESP32.crear_socket()
            try:
                self.log_mensaje("[PC] Esperando conexión de la ESP...")
                self.esp_addr = ESP32.conectar_ESP(self.sock)
                self.actualizar_estado(True, self.esp_addr[0])
                self.log_mensaje(f"[PC] Conectado a {self.esp_addr[0]}")
                self.btn_escuchar.config(state=tk.NORMAL)
            except Exception as e:
                self.log_mensaje(f"[ERROR] {e}")
                messagebox.showerror("Error", f"No se pudo conectar: {e}")
                self.actualizar_estado(False)

        threading.Thread(target=tarea_conectar, daemon=True).start()
    
    def calibrar_esp(self):
        if not (self.sock and self.esp_addr):
            messagebox.showwarning("Advertencia", "Debe conectar primero con la ESP.")
            return

        self.log_mensaje("[PC] Enviando comando de calibración...")
        ESP32.enviar_comando(self.sock, self.esp_addr, "CALIBRATE")

    def info_esp(self):
        if not (self.sock and self.esp_addr):
            messagebox.showwarning("Advertencia", "Debe conectar primero con la ESP.")
            return

        self.log_mensaje("[PC] Solicitando información del ESP...")
        ESP32.enviar_comando(self.sock, self.esp_addr, "INFO")

    def iniciar_bucle(self):
        if not (self.sock and self.esp_addr):
            messagebox.showwarning("Advertencia", "Debe conectar primero con la ESP.")
            return
        self.log_mensaje("[PC] Enviando comando START...")
        ESP32.enviar_comando(self.sock, self.esp_addr, "START")

    def escuchar_esp(self):
        if not (self.sock and self.esp_addr):
            messagebox.showwarning("Advertencia", "Debe conectar primero con la ESP.")
            return

        if self.hilo_escucha and self.hilo_escucha.is_alive():
            self.log_mensaje("[PC] Escucha ya está activa.")
            return
        
    def escuchar_esp(self):
        if not (self.sock and self.esp_addr):
            messagebox.showwarning("Advertencia", "Debe conectar primero con la ESP.")
            return

        if self.hilo_escucha and self.hilo_escucha.is_alive():
            self.log_mensaje("[PC] Escucha ya está activa.")
            return

        def tarea_escuchar():
            self.escucha_activa.set()
            self.log_mensaje("[PC] Escuchando datos de la ESP...")
            while self.escucha_activa.is_set():
                try:
                    self.sock.settimeout(1.0)
                    data, addr = self.sock.recvfrom(ESP32.BUFFER_SIZE)
                    msg = data.decode(errors="ignore").strip()
                    if addr == self.esp_addr:
                        self.log_mensaje(f"[ESP] {msg}")
                    else:
                        self.log_mensaje(f"[IGNORADO] Paquete desde {addr[0]}")
                except socket.timeout:
                    continue
                except Exception as e:
                    self.log_mensaje(f"[ERROR] {e}")
                    break
            self.log_mensaje("[PC] Escucha detenida.")

        self.hilo_escucha = threading.Thread(target=tarea_escuchar, daemon=True)
        self.hilo_escucha.start()


    def conectar_wifi(self):
        ssid = simpledialog.askstring("SSID", "Ingrese el nombre de la red WiFi:")
        password = simpledialog.askstring("Contraseña", "Ingrese la contraseña de la red WiFi:", show='*')

        if ssid and password:
            def tarea_wifi():
                self.log_mensaje(f"[PC] Enviando credenciales WiFi...")
                ESP32.conectar_WIFI(ssid, password)
                self.log_mensaje("[PC] Proceso de conexión WiFi finalizado.")

            threading.Thread(target=tarea_wifi, daemon=True).start()
        else:
            self.log_mensaje("[PC] SSID o contraseña no ingresados.")

    def iniciar_tcp(self):
        if not self.esp_addr:
            self.log_mensaje("[PC] IP no disponible.")
            return

        def tarea_tcp():
            ip = self.esp_addr[0]  # extraer solo la IP
            self.log_mensaje(f"[PC] Iniciando conexión TCP a {ip}...")
            ESP32.enviar_comando(self.sock, self.esp_addr, "DATOS")
            ESP32.recibir_datos_TCP(ip, log_callback=self.log_mensaje)
            self.log_mensaje("[PC] Conexión TCP finalizada.")

        threading.Thread(target=tarea_tcp, daemon=True).start()

    def detener_escucha(self):
        if self.escucha_activa.is_set():
            self.escucha_activa.clear()

    def log_mensaje(self, mensaje):
        timestamp = datetime.now().strftime("%H:%M:%S")
        mensaje = f"[{timestamp}] {mensaje}"
        self.text_area.config(state=tk.NORMAL)
        self.text_area.insert(tk.END, mensaje + "\n")
        self.text_area.see(tk.END)
        self.text_area.config(state=tk.DISABLED)
        # También guardar en archivo si lo deseas
        with open("log_pc.txt", "a") as f:
            f.write(mensaje + "\n")

if __name__ == "__main__":
    root = tk.Tk()
    app = ESPApp(root)
    root.mainloop()
