#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ArduinoJson.h> // Compatible amb versió 7.4.2
#include <ESP32Servo.h>

// Device ID
const char *deviceId = "G1_Servos";

// Wi-Fi credentials
const char *ssid = "Robotics_UB";
const char *password = "rUBot_xx";

// UDP settings
IPAddress receiverESP32IP(192, 168, 1, 11);
IPAddress receiverComputerIP(192, 168, 1, 15);
const int udpPort = 12345;
WiFiUDP udp;

// Servo settings
Servo servo_yaw;
Servo servo_pitch;
Servo servo_roll1;
Servo servo_roll2;

// Pins
const int PIN_ANALOG_YAW = 36;
const int PIN_SIGNAL_YAW = 32;
const int PIN_ANALOG_PITCH = 39;
const int PIN_SIGNAL_PITCH = 33;
const int PIN_ANALOG_ROLL1 = 34;
const int PIN_SIGNAL_ROLL1 = 25;
const int PIN_ANALOG_ROLL2 = 35;
const int PIN_SIGNAL_ROLL2 = 27;

const float Rshunt = 1.6;

// Variables
float Gri_roll = 0.0, Gri_pitch = 0.0, Gri_yaw = 0.0;
float Torque_roll1 = 0.0, Torque_roll2 = 0.0, Torque_pitch = 0.0, Torque_yaw = 0.0;
int s1 = 1, s2 = 1;

float prevRoll1 = 0, prevRoll2 = 0, prevPitch = 0, prevYaw = 0; ###
float sumRoll1 = 0, sumRoll2 = 0, sumPitch = 0, sumYaw = 0;
float OldValueRoll = 0, OldValuePitch = 0, OldValueYaw = 0;
float roll = 0, pitch = 0, yaw = 0;
float initial_yaw = 0;
bool yaw_initialized = false;

int constrainAngle(int angle) {
  return constrain(angle, 0, 180);
}

void connectToWiFi() {
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");
  Serial.println("IP Address: " + WiFi.localIP().toString());
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void receiveOrientationUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    byte packetBuffer[512];
    int len = udp.read(packetBuffer, 512);
    if (len > 0) {
      packetBuffer[len] = '\0';
      Serial.print("Received packet size: ");
      Serial.println(packetSize);
      Serial.print("Received: ");
      Serial.println((char*)packetBuffer);

      JsonDocument doc;  // ✅ Versió 7
      DeserializationError error = deserializeJson(doc, packetBuffer);
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
      }

      const char* device = doc["device"];
      if (strcmp(device, "G5_Gri") == 0) {
        Gri_roll = round(doc["roll"].as<float>());
        Gri_pitch = round(doc["pitch"].as<float>());
        Gri_yaw = round(doc["yaw"].as<float>());
        s1 = doc["s1"];
        s2 = doc["s2"];
        Serial.print("Gri_Roll: "); Serial.print(Gri_roll);
        Serial.print(" Gri_Pitch: "); Serial.print(Gri_pitch);
        Serial.print(" Gri_Yaw: "); Serial.println(Gri_yaw);
        Serial.print("S1: "); Serial.print(s1);
        Serial.print(" S2: "); Serial.println(s2);
      } else {
        Serial.println("Unknown device.");
      }
    }
  }
}

float getCurrent(uint32_t integrationTimeMs, int pin) {
  uint32_t startTime = millis();
  float integratedCurrent = 0;
  while (millis() < startTime + integrationTimeMs) {
    uint16_t adcValue = analogRead(pin);
    integratedCurrent += ((float)adcValue / 4095.0 * 3.3) / Rshunt;
  }
  return integratedCurrent;
}

float getTorque(float& sum, int analogPin, float& previous) {
  float current = getCurrent(20, analogPin);
  sum += current;
  float diff = abs(sum - previous);
  previous = sum;
  return diff;
}

void moveServos() {
  // Store and update current angles
  roll = Gri_roll;
  OldValueRoll = roll;
  pitch = Gri_pitch;
  OldValuePitch = pitch;
  yaw = Gri_yaw;
  OldValueYaw = yaw;

  // Initialize yaw reference on first execution
  if (!yaw_initialized) {
    initial_yaw = yaw;
    yaw_initialized = true;
    Serial.println("Yaw reference initialized to: " + String(initial_yaw));
  }

  // Apply roll angle (with gripper opening delta)
  float delta = 0;
  if (s1 == 0) {
    delta = 40;  // Opening angle when S1 is pressed
    Serial.println("S1 pressed → Opening gripper");
  }

  // Roll is controlled by two opposing servos
  // servo_roll1 moves in positive direction
  // servo_roll2 moves in negative direction to create the roll motion
  int roll1_angle = constrainAngle(90 + roll + delta);  // 90° is the neutral position
  int roll2_angle = constrainAngle(90 - roll);          // Opposing motion
  
  servo_roll1.write(roll1_angle);
  servo_roll2.write(roll2_angle);

  // Apply pitch angle from 90° neutral position
  int pitch_angle = constrainAngle(90 + pitch);
  servo_pitch.write(pitch_angle);

  // Apply yaw angle relative to initial position
  // This makes it independent of geographical North
  if (yaw_initialized) {
    float relative_yaw = yaw - initial_yaw;
    // Handle angle wrapping for smoother motion
    if (relative_yaw > 180) relative_yaw -= 360;
    if (relative_yaw < -180) relative_yaw += 360;
    
    int yaw_angle = constrainAngle(90 + relative_yaw);
    servo_yaw.write(yaw_angle);
  }

  // Debug output
  Serial.print("Angles - Roll1: "); Serial.print(roll1_angle);
  Serial.print(" Roll2: "); Serial.print(roll2_angle);
  Serial.print(" Pitch: "); Serial.print(pitch_angle);
  if (yaw_initialized) {
    Serial.print(" Yaw: "); Serial.print(yaw_angle);
  }
  Serial.println();

}

void sendTorqueUDP() {
  // Calculate torques for each axis
  Torque_roll1 = getTorque(sumRoll1, PIN_ANALOG_ROLL1, prevRoll1);
  Torque_roll2 = getTorque(sumRoll2, PIN_ANALOG_ROLL2, prevRoll2);
  Torque_pitch = getTorque(sumPitch, PIN_ANALOG_PITCH, prevPitch);
  Torque_yaw = getTorque(sumYaw, PIN_ANALOG_YAW, prevYaw);

  JsonDocument doc;
  doc["device"] = deviceId;
  doc["Torque_roll1"] = Torque_roll1;
  doc["Torque_roll2"] = Torque_roll2;
  doc["Torque_pitch"] = Torque_pitch;
  doc["Torque_yaw"] = Torque_yaw;

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));

  // Send to Gripper ESP32
  udp.beginPacket(receiverESP32IP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, strlen(jsonBuffer));
  udp.endPacket();

  // Send to Computer
  udp.beginPacket(receiverComputerIP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, strlen(jsonBuffer));
  udp.endPacket();

  // Debug print
  Serial.print("Torques - Roll1: "); Serial.print(Torque_roll1);
  Serial.print(" Roll2: "); Serial.print(Torque_roll2);
  Serial.print(" Pitch: "); Serial.print(Torque_pitch);
  Serial.print(" Yaw: "); Serial.println(Torque_yaw);
}


void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  connectToWiFi();
  udp.begin(udpPort);
  Serial.println("UDP initialized");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo_yaw.setPeriodHertz(50);
  servo_pitch.setPeriodHertz(50);
  servo_roll1.setPeriodHertz(50);
  servo_roll2.setPeriodHertz(50);

  servo_yaw.attach(PIN_SIGNAL_YAW);
  servo_pitch.attach(PIN_SIGNAL_PITCH);
  servo_roll1.attach(PIN_SIGNAL_ROLL1);
  servo_roll2.attach(PIN_SIGNAL_ROLL2);

  pinMode(PIN_ANALOG_YAW, INPUT);
  pinMode(PIN_ANALOG_PITCH, INPUT);
  pinMode(PIN_ANALOG_ROLL1, INPUT);
  pinMode(PIN_ANALOG_ROLL2, INPUT);

  servo_yaw.write(90);
  servo_pitch.write(90);
  servo_roll1.write(90);
  servo_roll2.write(90);
}

void loop() {
  receiveOrientationUDP();
  sendTorqueUDP();
  moveServos();
  delay(20);
}
