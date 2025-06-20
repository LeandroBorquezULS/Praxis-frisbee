#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <BMI160Gen.h>
#include <math.h>
#include <Preferences.h>

IPAddress pcIP;
uint16_t pcPort;
char ssid[64];
char password[64];
const int udpPort = 4210;
bool conectado = false;
unsigned long lastBlinkTime = 0;
bool ledState = false;
bool vueloDetectado = false;
unsigned long ultimoMovimiento = 0;
const unsigned long TIEMPO_INACTIVIDAD = 3000; // ms
bool lanzamiento = false;
int dt = 0;
float accel_array[10];


WiFiUDP udp;

// I2C config
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_ADDR 0x68
#define LED_BUILTIN 2

bool connectToSavedWiFi(unsigned long timeout_ms = 10000);
bool receiveWiFiCredentialsOverSerial(String &ssid, String &password);
void saveWiFiCredentials(const String &ssid, const String &password);

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!connectToSavedWiFi()) {
    Serial.println("No se pudo conectar. Esperando datos por Serial...");
    String ssid, password;
    if (receiveWiFiCredentialsOverSerial(ssid, password)) {
      saveWiFiCredentials(ssid, password);
      Serial.println("Datos guardados. Reiniciando...");
      ESP.restart();
    } else {
      Serial.println("Error al recibir datos.");
    }
  } else {
    Serial.println("Conectado a WiFi.");
    Serial.println(WiFi.localIP());   
  }

  // Inicial led
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Iniciar IMU
  Wire.begin(SDA_PIN, SCL_PIN);
  BMI160.begin(BMI160GenClass::I2C_MODE, I2C_ADDR);
  BMI160.setAccelerometerRange(16);  // Cambiar a ±16g
  //  BMI160.setAccelRate(BMI160_ACCEL_RANGE_16G);

  udp.begin(udpPort);
}

void loop() {
  // Esperar hasta que reciba "ACK"
  if (!conectado){
    connectToPC();
  }

  // Una vez conectado, registrar datos crudos
  int ax_raw, ay_raw, az_raw;
  int gx_raw, gy_raw, gz_raw;
  BMI160.readMotionSensor(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw);

  // Conversión de aceleración a G (±2G → 16384 LSB/G)
  float ax_g = ax_raw / 2048.0; //para ±16G
  float ay_g = ay_raw / 2048.0;
  float az_g = az_raw / 2048.0;

  // Calcular aceleración neta
  float net_accel_g = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

  // Verificar si la aceleración neta supera el umbral
  if (net_accel_g > 2.0 && !lanzamiento || dt > 0) {
    float temp = (net_accel_g * net_accel_g) - 1;
    float orizontal_accel_g = (temp > 0) ? sqrt(temp) : 0;
    accel_array[dt] = orizontal_accel_g;
    dt++;
    if (dt > 10){
      lanzamiento = true;
      dt = 0;
      udp.beginPacket(pcIP, pcPort);
      udp.print("Lanzamiento");
      udp.endPacket();
      float velocity = 0.0;
      for (int i = 0; i < 10; i++) {
        float accel_ms2 = accel_array[i] * 9.8;
        velocity += accel_ms2 * 0.05;
      }
      sendInitialSpeed(velocity);
    }
    
  }
  if (lanzamiento == true){
    sendRawData(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw);
  }

  delay(50); // Evita saturar el canal
}


void sendRawData(int ax, int ay, int az, int gx, int gy, int gz) {
  String msg = String(ax) + "," + String(ay) + "," + String(az)
             + "," + String(gx) + "," + String(gy) + "," + String(gz);
  udp.beginPacket(pcIP, pcPort);
  udp.print(msg);
  udp.endPacket();
}

void sendPresenceBroadcast() {
  udp.beginPacket("255.255.255.255", udpPort);
  udp.print("ESP_PING");
  udp.endPacket();
}

bool connectToSavedWiFi(unsigned long timeout_ms) {
    Preferences prefs;
    prefs.begin("wifi", true);
    String ssid = prefs.getString("ssid", "");
    String password = prefs.getString("password", "");
    prefs.end();

    Serial.print("Intentando conectar a: ");
    Serial.println(ssid);
    Serial.print("Con contraseña: ");
    Serial.println(password); // ¡Ojo! Solo con fines de depuración

    if (ssid == "" || password == "") return false;

    WiFi.begin(ssid.c_str(), password.c_str());

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeout_ms) {
        delay(500);
    }

    return WiFi.status() == WL_CONNECTED;
}

bool receiveWiFiCredentialsOverSerial(String &ssid, String &password) {
    Serial.println("Esperando SSID...");
    while (Serial.available() == 0) delay(100);
    ssid = Serial.readStringUntil('\n');
    ssid.trim();

    Serial.println("Esperando contraseña...");
    while (Serial.available() == 0) delay(100);
    password = Serial.readStringUntil('\n');
    password.trim();
    Serial.println("Credenciales guardadas:");
    /*
    Serial.print("SSID: ");
    Serial.println(ssid);
    Serial.print("Password: ");
    Serial.println(password);
    */

    return (ssid.length() > 0 && password.length() > 0);
}

void saveWiFiCredentials(const String &ssid, const String &password) {
    Preferences prefs;
    prefs.begin("wifi", false);

    // Borrar credenciales previas
    prefs.clear();  

    // Guardar nuevas credenciales
    prefs.putString("ssid", ssid);
    prefs.putString("password", password);
    prefs.end();

    Serial.println("Credenciales guardadas:");
/*
  Serial.print("SSID: ");
    Serial.println(ssid);
    Serial.print("Password: ");
    Serial.println(password);
*/
}


void connectToPC(){
  while (!conectado) {
    sendPresenceBroadcast();

    int packetSize = udp.parsePacket();
    if (packetSize > 0) {
      char incoming[32];
      int len = udp.read(incoming, sizeof(incoming) - 1);
      if (len > 0) {
        incoming[len] = '\0';

        if (strcmp(incoming, "ACK") == 0) {
          conectado = true;
          pcIP = udp.remoteIP();      // Guarda IP del remitente
          pcPort = udp.remotePort();  // Guarda puerto del remitente

          Serial.print("Conectado a: ");
          Serial.print(pcIP);
          Serial.print(":");
          Serial.println(pcPort);

          digitalWrite(LED_BUILTIN, HIGH);  // LED encendido
        }
      }
    }

    // LED parpadea mientras espera conexión
    if (millis() - lastBlinkTime >= 500 && !conectado) {
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
      lastBlinkTime = millis();
    }

    delay(100);
  }
}

void sendInitialSpeed(float initial_speed) {
  udp.beginPacket(pcIP, pcPort);
  udp.print("Velocidad inicial: ");
  udp.print(initial_speed);  // en m/s
  udp.endPacket();
}


