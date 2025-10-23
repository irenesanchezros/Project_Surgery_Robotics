#include <WiFi.h>
#include <WiFiUdp.h>
#include "MPU9250.h"
#include <Wire.h>                 // I2C IMU
#include <ArduinoJson.h>          // V. 7.x compatible amb .h
#include <IMU_RoboticsUB.h>

//======================
// Config bàsica
//======================
const char *deviceId = "G1_Gri";

// Wi-Fi
const char *ssid     = "Robotics_UB";
const char *password = "rUBot_xx";

// UDP
// Receptors de sempre (servo ESP32 + PC)
IPAddress receiverESP32IP(192, 168, 1, 13);
IPAddress receiverComputerIP(192, 168, 1, 15);
const int udpPort = 12345;
WiFiUDP udp;

// IMPORTANT: si el teu Endo no envia al gripper, canvia l'Endo per enviar també a la IP del gripper
// o configura broadcast a 192.168.1.255.
// Aquest codi ja escolta al port udpPort per rebre l'Endo.

// Botons
const int PIN_S1 = 14;  // Recalibrar offsets (posem a zero)
const int PIN_S2 = 27;  // Zero suau només gripper (opcional)
int s1Status = HIGH;
int s2Status = HIGH;

// IMU
IMU imu;

// Ori gripper (absoluta i relativa)
float Gri_roll  = 0.0f, Gri_pitch = 0.0f, Gri_yaw  = 0.0f;
float Gri_roll0 = 0.0f, Gri_pitch0 = 0.0f, Gri_yaw0 = 0.0f;   // offset (zero)

// Ori Endo (rebuda per UDP)
float Endo_roll = 0.0f, Endo_pitch = 0.0f, Endo_yaw = 0.0f;
float Endo_roll0 = 0.0f, Endo_pitch0 = 0.0f, Endo_yaw0 = 0.0f;
bool  endoSeen = false;                // hem rebut com a mínim un paquet de l'Endo?
uint32_t endoLastMs = 0;               // últim cop rebut (ms)
const uint32_t ENDO_TIMEOUT_MS = 500;  // si passa això, considerem que no hi ha Endo actiu

// Vibració (si el vols usar com a feedback, opcional)
const int vibrationPin = 23;

//======================
// Helpers
//======================
static inline float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void connectToWiFi() {
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());
}

// Llegeix IMU del gripper i botons
void updateGripperIMU() {
  imu.ReadSensor();
  float* rpw = imu.GetRPW();
  Gri_roll  = rpw[0];
  Gri_pitch = rpw[1];
  Gri_yaw   = rpw[2];

  s1Status = digitalRead(PIN_S1);
  s2Status = digitalRead(PIN_S2);
}

// Escolta paquets UDP i, si són de l'Endo, actualitza la seva orientació
void pollEndoUdp() {
  int packetSize = udp.parsePacket();
  if (packetSize <= 0) return;

  // Llegim el paquet
  static char rxBuf[512];
  int len = udp.read(rxBuf, sizeof(rxBuf) - 1);
  if (len <= 0) return;
  rxBuf[len] = '\0';

  // Intentem parsejar JSON
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, rxBuf);
  if (err) return;

  // Mirem si és l'Endo
  const char* dev = doc["device"] | "";
  if (strcmp(dev, "G1_Endo") != 0) {
    // Podria ser el nostre propi paquet o d'algun altre dispositiu -> ignorem
    return;
  }

  Endo_roll  = doc["roll"]  | Endo_roll;
  Endo_pitch = doc["pitch"] | Endo_pitch;
  Endo_yaw   = doc["yaw"]   | Endo_yaw;

  endoSeen   = true;
  endoLastMs = millis();
}

// Retorna true si l’Endo és “vàlid” (hem rebut fa poc)
bool isEndoAlive() {
  if (!endoSeen) return false;
  return (millis() - endoLastMs) < ENDO_TIMEOUT_MS;
}

// Envia paquet UDP amb YAW independent
void sendOrientationUDP(float yaw_ind, float roll_ind, float pitch_ind) {
  JsonDocument doc;

  // Camps “canònics” cap als consumidors existents
  doc["device"] = deviceId;
  doc["roll"]   = roll_ind;     // independent
  doc["pitch"]  = pitch_ind;    // independent
  doc["yaw"]    = yaw_ind;      // independent

  // Estat botons
  doc["s1"] = s1Status;
  doc["s2"] = s2Status;

  // Camps extra (debug/telemetria)
  doc["raw_roll"]  = Gri_roll;
  doc["raw_pitch"] = Gri_pitch;
  doc["raw_yaw"]   = Gri_yaw;
  doc["endo_alive"] = isEndoAlive();

  char jsonBuffer[512];
  size_t n = serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));

  // Enviem a Servos
  udp.beginPacket(receiverESP32IP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, n);
  udp.endPacket();

  // Enviem a PC
  udp.beginPacket(receiverComputerIP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, n);
  udp.endPacket();
}

// Recalibra offsets (gripper i, si el tenim, també Endo)
void doFullRecalibration() {
  Gri_roll0  = Gri_roll;
  Gri_pitch0 = Gri_pitch;
  Gri_yaw0   = Gri_yaw;

  if (isEndoAlive()) {
    Endo_roll0  = Endo_roll;
    Endo_pitch0 = Endo_pitch;
    Endo_yaw0   = Endo_yaw;
  }

  // Feedback opcional
  digitalWrite(vibrationPin, HIGH);
  delay(60);
  digitalWrite(vibrationPin, LOW);
}

// Zero suau del gripper (no toca Endo)
void doSoftZeroGripper() {
  Gri_roll0  = Gri_roll;
  Gri_pitch0 = Gri_pitch;
  Gri_yaw0   = Gri_yaw;

  digitalWrite(vibrationPin, HIGH);
  delay(30);
  digitalWrite(vibrationPin, LOW);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(500);

  pinMode(PIN_S1, INPUT);
  pinMode(PIN_S2, INPUT);
  pinMode(vibrationPin, OUTPUT);
  digitalWrite(vibrationPin, LOW);

  // IMU
  imu.Install();

  // Xarxa
  connectToWiFi();
  udp.begin(udpPort);
  Serial.println("UDP initialized");

  // Lectura inicial per tenir valors
  updateGripperIMU();
  // Inicialitzem zeros
  doFullRecalibration();
}

void loop() {
  // 1) Rebem Endo (si hi ha paquets circulant)
  pollEndoUdp();

  // 2) Actualitzem IMU del gripper i botons
  updateGripperIMU();

  // 3) Gestió de botons: S1 = recalibra tot; S2 = zero suau (només gripper)
  static int lastS1 = HIGH, lastS2 = HIGH;
  if (s1Status == LOW && lastS1 == HIGH) {
    doFullRecalibration();
  }
  if (s2Status == LOW && lastS2 == HIGH) {
    doSoftZeroGripper();
  }
  lastS1 = s1Status;
  lastS2 = s2Status;

  // 4) Càlcul relatiu i independent
  // Relatiu gripper
  float roll_rel  = wrap180(Gri_roll  - Gri_roll0);
  float pitch_rel = wrap180(Gri_pitch - Gri_pitch0);
  float yaw_rel   = wrap180(Gri_yaw   - Gri_yaw0);

  float roll_ind  = roll_rel;
  float pitch_ind = pitch_rel;
  float yaw_ind   = yaw_rel;

  // Si tenim Endo “viu”, restem la seva part per obtenir independència (relativa)
  if (isEndoAlive()) {
    float endo_roll_rel  = wrap180(Endo_roll  - Endo_roll0);
    float endo_pitch_rel = wrap180(Endo_pitch - Endo_pitch0);
    float endo_yaw_rel   = wrap180(Endo_yaw   - Endo_yaw0);

    // Decouple: independent = gripper_rel - endo_rel
    roll_ind  = wrap180(roll_rel  - endo_roll_rel);
    pitch_ind = wrap180(pitch_rel - endo_pitch_rel);
    yaw_ind   = wrap180(yaw_rel   - endo_yaw_rel);
  }

  // 5) Enviem
  sendOrientationUDP(yaw_ind, roll_ind, pitch_ind);

  delay(10);
}