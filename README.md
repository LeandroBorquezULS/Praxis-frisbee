# Praxis-frisbee
# Control de Lanzamiento con ESP32 e IMU (BMI160)

Este proyecto permite recibir datos de un módulo ESP32 con sensor IMU (BMI160) a través de una interfaz gráfica desarrollada en Python utilizando `Tkinter`. La comunicación entre el PC y la ESP32 se realiza mediante UDP, y se utiliza conexión WiFi para enviar datos de velocidad inicial y otros parámetros.

## Estructura del Proyecto

- **PC (Python)**:
  - Interfaz gráfica (`Tkinter`) para interactuar con la ESP32.
  - Comunicación UDP con la ESP32.
  - Recepción de datos IMU y eventos desde la ESP32.
  - Módulo `ESP32.py` con funciones de red y comunicación serial.

- **ESP32 (Arduino)**:
  - Detección de lanzamiento mediante aceleración.
  - Calibración del sensor BMI160.
  - Envío de datos IMU al PC por UDP.

---

## Requisitos

### Software

- **Python 3.x**
- **Librerías Python**:
  - `tkinter` (incluido en la mayoría de las instalaciones)
  - `pyserial`
  - `pyserial.tools.list_ports`

Instalación rápida:

```bash
pip install pyserial
