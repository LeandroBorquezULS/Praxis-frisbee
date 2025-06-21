import tkinter as tk
from tkinter import messagebox, simpledialog, scrolledtext
import threading

import ESP32

class ESPApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Control ESP32")
        self.sock = None
        self.esp_addr = None

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

        def tarea_escuchar():
            self.log_mensaje("[PC] Escuchando datos de la ESP...")
            while True:
                try:
                    data, addr = self.sock.recvfrom(ESP32.BUFFER_SIZE)
                    msg = data.decode(errors="ignore").strip()
                    if addr == self.esp_addr:
                        self.log_mensaje(f"[ESP] {msg}")
                    else:
                        self.log_mensaje(f"[IGNORADO] Paquete desde {addr[0]}")
                except Exception as e:
                    self.log_mensaje(f"[ERROR] {e}")
                    break

        threading.Thread(target=tarea_escuchar, daemon=True).start()

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

if __name__ == "__main__":
    root = tk.Tk()
    app = ESPApp(root)
    root.mainloop()
