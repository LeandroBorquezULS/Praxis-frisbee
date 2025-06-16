import serial
import time
import socket
from serial.tools import list_ports

LISTEN_PORT = 4210  # Puerto en el que escucha el PC para todo
BUFFER_SIZE = 1024

def crear_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Reutilización del puerto
    sock.bind(("", LISTEN_PORT))  # Escucha en todas las interfaces
    return sock

def conectar_ESP(sock):
    """Espera un PING de la ESP y responde con un ACK"""
    print("[PC] Esperando conexión de la ESP...\n")
    while True:
        data, addr = sock.recvfrom(BUFFER_SIZE)
        message = data.decode(errors="ignore").strip()

        if message == "ESP_PING":
            print(f"[ESP] PING recibido desde {addr[0]}:{addr[1]}")
            sock.sendto(b"ACK", addr)
            print(f"[PC] ACK enviado a {addr[0]}:{addr[1]}")
            print("[PC] Conexión establecida. Esperando datos...\n")
            return addr


def escuchar_ESP(sock, esp_address):
    """Escucha datos desde la ESP después del ACK"""
    while True:
        data, addr = sock.recvfrom(BUFFER_SIZE)
        if addr == esp_address:
            print(f"[ESP] {data.decode(errors='ignore').strip()}")
        else:
            print(f"[IGNORADO] Paquete de {addr}, se esperaba de {esp_address}")

def conectar_WIFI(ssid, password):
    puerto_esp = detectar_puerto_esp()
    if puerto_esp:
        enviar_credenciales(puerto_esp, ssid, password)

def detectar_puerto_esp():
    puertos = list_ports.comports()
    esp_puertos = []

    for puerto in puertos:
        # Filtrar por nombre o fabricante típico del ESP32
        if "USB" in puerto.description or "CP210" in puerto.description or "CH340" in puerto.description:
            esp_puertos.append(puerto.device)

    if not esp_puertos:
        print("No se encontró ningún ESP32 conectado.")
        return None
    elif len(esp_puertos) == 1:
        print(f"Puerto detectado automáticamente: {esp_puertos[0]}")
        return esp_puertos[0]
    else:
        print("Se encontraron varios dispositivos. Selecciona uno:")
        for i, p in enumerate(esp_puertos):
            print(f"[{i}] {p}")
        idx = int(input("Ingresa el número del puerto a usar: "))
        return esp_puertos[idx]

def enviar_credenciales(puerto, ssid, password, baudios=115200):
    try:
        with serial.Serial(puerto, baudios, timeout=2) as ser:
            time.sleep(2)  # Esperar a que el ESP reinicie

            print(f"Enviando SSID: {ssid}")
            ser.write((ssid + '\n').encode())
            time.sleep(0.5)

            print(f"Enviando Password: {password}")
            ser.write((password + '\n').encode())
            time.sleep(0.5)

            print("Credenciales enviadas correctamente.\n")

            # Leer respuestas del ESP
            while ser.in_waiting:
                print("ESP:", ser.readline().decode().strip())

    except serial.SerialException as e:
        print(f"Error de conexión: {e}")

if __name__ == "__main__":
    sock = crear_socket()
    esp_addr = conectar_ESP(sock)
    escuchar_ESP(sock, esp_addr)