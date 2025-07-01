#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <BMI160Gen.h>
#include <math.h>
#include <Preferences.h>

//variables de red
IPAddress pcIP;
char ssid[64];
char password[64];
const int udpPort = 4210;
WiFiServer tcpServer(5000);
bool conectado = false;
bool iniciado = false;

//variables led
unsigned long lastBlinkTime = 0;
bool ledState = false;

//variables lanzamiento
bool vueloDetectado = false;
unsigned long ultimoMovimiento = 0;
const unsigned long TIEMPO_INACTIVIDAD = 3000; // ms
bool lanzamiento = false;
int sampleIndex = 0;
float accelSamples[10];
int stationaryCycles = 0;
const float g = 9.8;
int ax_offset = 0;
int ay_offset = 0;
int az_offset = 0;
bool datosPendientes = false;



WiFiUDP udp;

// I2C config
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_ADDR 0x68
#define LED_BUILTIN 2

bool connectToSavedWiFi(unsigned long timeout_ms = 10000);
bool receiveWiFiCredentialsOverSerial(String &ssid, String &password);
void saveWiFiCredentials(const String &ssid, const String &password);
void setLED(bool on);
void blinkWhileWaiting();

typedef struct IMUData {
    float ax, ay, az;
    float gx, gy, gz;
    uint32_t timestamp;
    struct IMUData* next;
} IMUData;

IMUData* head = NULL;
IMUData* tail = NULL;


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
  cargarOffsets();

  udp.begin(udpPort);
  tcpServer.begin();

  esperarComandoInicial();  // Aquí se interpreta "CALIBRATE", "INFO", "ACK", etc.
}

void loop() {
  // Una vez conectado, registrar datos crudos
  int ax_raw, ay_raw, az_raw;
  int gx_raw, gy_raw, gz_raw;
  BMI160.readMotionSensor(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw);

  // Conversión de aceleración a G (±16G → 2048 LSB/G)
  float ax_g = (ax_raw - ax_offset) / 2048.0;
  float ay_g = (ay_raw - ay_offset) / 2048.0;
  float az_g = (az_raw - az_offset) / 2048.0;

  // Calcular aceleración neta
  float net_accel_g = sqrt(ax_g * ax_g + ay_g * ay_g);

  // Verificar si la aceleración neta supera el umbral
  if (net_accel_g > 2.0 && !lanzamiento || sampleIndex > 0) {
    accelSamples[sampleIndex] = net_accel_g;
    sampleIndex++;
    lanzamiento = true;
    if (sampleIndex > 10){
      sampleIndex = 0;
      sendUDPMessage("lanzamiento");
      float velocity = 0.0;
      for (int i = 0; i < 10; i++) {
        float accel_ms2 = accelSamples[i] * g;
        velocity += accel_ms2 * 0.05;
      }
      String initialVelocity = "Velocidad inicial: " + String(velocity) + "m/s";
      sendUDPMessage(initialVelocity);
    }
    
  }
  if (lanzamiento){
    addData(ax_g, ay_g, az_g, gx_raw, gy_raw, gz_raw, millis());
    float giro_total = abs(gx_raw) + abs(gy_raw) + abs(gz_raw);
    if (giro_total < 100.0) {
      stationaryCycles++;
    } else {
      stationaryCycles = 0;
    }

    // Si el frisbee estuvo quieto por N ciclos, finalizar vuelo
    if (stationaryCycles >= 10) {
      sendUDPMessage("Vuelo finalizado");
      // Reiniciar todo para permitir nuevo lanzamiento
      lanzamiento = false;
      stationaryCycles = 0;

      if (WiFi.status() != WL_CONNECTED) {
        conectado = false;
      }
      esperarComandoInicial();
    }
  }

  delay(50); // Evita saturar el canal
}

void esperarComandoInicial() {
  iniciado = false;
  Serial.println("Esperando comandos desde PC...");

  while (!iniciado) {
    if (!conectado) {
      sendPresenceBroadcast();
      blinkWhileWaiting();
    }

    int packetSize = udp.parsePacket();
    if (packetSize > 0) {
      char incoming[64];
      int len = udp.read(incoming, sizeof(incoming) - 1);
      if (len > 0) {
        incoming[len] = '\0';
        String comando = String(incoming);

        comando.trim();
        comando.toUpperCase();  // Comandos insensibles a mayúsculas

        if (comando == "ACK") {
          pcIP = udp.remoteIP();
          Serial.println("Conexión establecida.");
          conectado = true;
          setLED(true);
        } else if (comando == "INFO") {
          sendInfo();
        } else if (comando == "CALIBRATE") {
          calibrarIMU();
        } else if (comando == "RESET") {
          ESP.restart();
        } else if (comando == "START"){
          iniciado = true;
          if (datosPendientes) {
            Serial.println("Advertencia: lanzamiento anterior no fue enviado. Datos descartados.");
            clearData();
          }
        } else if (comando == "DATOS"){
          waitForTCPConnection();
        } else {
          Serial.println("Comando no reconocido: " + comando);
        }
      }
    }

    delay(100);
  }
}

void sendInfo() {
  String info = "INFO\n";
  info += "SSID: " + String(WiFi.SSID()) + "\n";
  info += "IP: " + WiFi.localIP().toString() + "\n";
  info += "IMU: BMI160\n";
  info += "Offsets:\n";
  info += "  ax_offset: " + String(ax_offset) + "\n";
  info += "  ay_offset: " + String(ay_offset) + "\n";
  info += "  az_offset: " + String(az_offset) + "\n";

  sendUDPMessage(info);

  Serial.println("Información enviada:");
  Serial.println(info);
}

void calibrarIMU() {
  const int muestras = 100;
  long suma_ax = 0, suma_ay = 0, suma_az = 0;

  Serial.println("Calibrando... mantén la ESP inmóvil y plana.");

  for (int i = 0; i < muestras; i++) {
    int ax, ay, az, gx, gy, gz;
    BMI160.readMotionSensor(ax, ay, az, gx, gy, gz);

    suma_ax += ax;
    suma_ay += ay;
    suma_az += az;

    delay(10);
  }

  // Promedios
  ax_offset = suma_ax / muestras;
  ay_offset = suma_ay / muestras;
  az_offset = (suma_az / muestras) - 2048; // +1g = -2048 LSB (eje z hacia arriba)

  Serial.println("Calibración completada.");
  guardarOffsets();
  sendUDPMessage("Calibración completada y guardada.");
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
}

void sendUDPMessage(const String& msg) {
  udp.beginPacket(pcIP, udpPort);
  udp.print(msg);
  udp.endPacket();
}

void setLED(bool on){
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
}

void blinkWhileWaiting(){
  // LED parpadea mientras espera conexión
  if (millis() - lastBlinkTime >= 500) {
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
    lastBlinkTime = millis();
  }
}

void guardarOffsets() {
  Preferences prefs;
  prefs.begin("imu_offsets", false);

  prefs.putInt("ax_offset", ax_offset);
  prefs.putInt("ay_offset", ay_offset);
  prefs.putInt("az_offset", az_offset);

  prefs.end();

  Serial.println("Offsets guardados en memoria:");
  Serial.println("ax_offset: " + String(ax_offset));
  Serial.println("ay_offset: " + String(ay_offset));
  Serial.println("az_offset: " + String(az_offset));
}

void cargarOffsets() {
  Preferences prefs;
  prefs.begin("imu_offsets", true);  // true = solo lectura

  ax_offset = prefs.getInt("ax_offset", 0);
  ay_offset = prefs.getInt("ay_offset", 0);
  az_offset = prefs.getInt("az_offset", 0);

  prefs.end();

  Serial.println("Offsets cargados de memoria:");
  Serial.println("ax_offset: " + String(ax_offset));
  Serial.println("ay_offset: " + String(ay_offset));
  Serial.println("az_offset: " + String(az_offset));
}

void addData(float ax, float ay, float az, float gx, float gy, float gz, uint32_t timestamp) {
  datosPendientes = true;
  IMUData* newNode = (IMUData*)malloc(sizeof(IMUData));
  if (newNode == NULL) {
      Serial.println("Error: Memoria insuficiente.");
      return;
  }

  newNode->ax = ax;
  newNode->ay = ay;
  newNode->az = az;
  newNode->gx = gx;
  newNode->gy = gy;
  newNode->gz = gz;
  newNode->timestamp = timestamp;
  newNode->next = NULL;

  if (head == NULL) {
      head = tail = newNode;
  } else {
      tail->next = newNode;
      tail = newNode;
  }
}

void sendAndClearData(WiFiClient& client) {
  
  datosPendientes = false;
  IMUData* current = head;
  while (current != NULL) {
      client.printf("t=%lu, acc=(%.2f, %.2f, %.2f), gyr=(%.2f, %.2f, %.2f)\n",
          current->timestamp, current->ax, current->ay, current->az,
          current->gx, current->gy, current->gz);

      IMUData* next = current->next;
      free(current);
      current = next;
      delay(10);
  }

  head = tail = NULL;
}


void clearData() {
  datosPendientes = false;
  IMUData* current = head;
  while (current != NULL) {
      IMUData* next = current->next;
      free(current);
      current = next;
  }
  head = tail = NULL;
}

void waitForTCPConnection(){
  
  bool TCPConect = false;
  while(!TCPConect){
    WiFiClient client = tcpServer.available();
    if (client) {
      Serial.println("Cliente conectado. Enviando datos almacenados...");
      sendAndClearData(client);
      client.stop();
      Serial.println("Datos enviados. Cliente desconectado.");
      TCPConect = true;
    }
  }
}
