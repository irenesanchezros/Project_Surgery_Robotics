#include <WiFi.h>
#include <WiFiUdp.h>
#include "MPU9250.h"
#include <Wire.h> // Needed for I2C to read IMU
#include <ArduinoJson.h> // Compatible amb versió 7.4.2
#include <IMU_RoboticsUB.h> // Custom IMU library

// Device ID
const char *deviceId = "G1_Gri";

// Wi-Fi credentials
const char *ssid = "Robotics_UB";
const char *password = "rUBot_xx";

// Vibration motor settings
const int vibrationPin = 23; // Pin for the vibration motor

// Botons
const int PIN_S1 = 14;
const int PIN_S2 = 27;
int s1Status = HIGH;
int s2Status = HIGH;

// UDP settings
IPAddress receiverESP32IP(192, 168, 1, 13); // IP of receiver ESP32
IPAddress receiverComputerIP(192, 168, 1, 15); // IP of PC
const int udpPort = 12345;
WiFiUDP udp;

// IMU object
IMU imu;

// Orientation data
float Gri_roll = 0.0, Gri_pitch = 0.0, Gri_yaw = 0.0;

// Torque data
float Torque_roll1 = 0.0;
float Torque_pitch = 0.0;
float Torque_yaw = 0.0;

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

void updateOrientation() {
 // Llegeix FIFO del DMP i actualitza càlculs interns
  imu.ReadSensor();
  // Obté els angles (roll, pitch, yaw) via GetRPW()
  float* rpw = imu.GetRPW();
  Gri_roll  = rpw[0];
  Gri_pitch = rpw[1];
  Gri_yaw   = rpw[2];
  s1Status = digitalRead(PIN_S1);
  s2Status = digitalRead(PIN_S2);
}

void sendOrientationUDP() {
  JsonDocument doc;
  doc["device"] = deviceId;
  doc["roll"] = Gri_roll;
  doc["pitch"] = Gri_pitch;
  doc["yaw"] = Gri_yaw;
  doc["s1"] = s1Status;
  doc["s2"] = s2Status;

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));

  // Send to ESP32 Servos
  udp.beginPacket(receiverESP32IP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, strlen(jsonBuffer));
  udp.endPacket();

  // Send to Computer
  udp.beginPacket(receiverComputerIP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, strlen(jsonBuffer));
  udp.endPacket();
}

void receiveTorquesUDP() {
    char buffer[255];
    int packetSize = udp.parsePacket();

    if (!packetSize) {
        return;
    }

    int len = udp.read(buffer, sizeof(buffer) - 1);
    if (len <= 0) {
        Serial.println("UDP read returned 0");
        return;
    }
    buffer[len] = '\0';

    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, buffer);

    if (error) {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        return;
    }

    // Verify sender device
    if (doc.containsKey("device") && strcmp(doc["device"], "G1_Servos") == 0) {
        // Get torque values with flexible key names
        if (doc.containsKey("Torque_Roll_1")) Torque_roll1 = doc["Torque_Roll_1"].as<float>();
        if (doc.containsKey("Torque_Pitch")) Torque_pitch = doc["Torque_Pitch"].as<float>();
        if (doc.containsKey("Torque_Yaw")) Torque_yaw = doc["Torque_Yaw"].as<float>();

        // Vibration motor control based on absolute torque values
        float totalTorque = fabs(Torque_roll1) + fabs(Torque_pitch) + fabs(Torque_yaw);
        int vibrationValue = constrain((int)(totalTorque * 2.5f), 0, 255);
        ledcWrite(0, vibrationValue);

        // Debug output
        Serial.print("Torques (Roll,Pitch,Yaw): ");
        Serial.print(Torque_roll1, 2); Serial.print(", ");
        Serial.print(Torque_pitch, 2); Serial.print(", ");
        Serial.println(Torque_yaw, 2);
        Serial.print("Vibration value: ");
        Serial.println(vibrationValue);
    }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  // Configure PWM for the vibration motor (channel 0)
  ledcSetup(0, 5000, 8); // Channel 0, frequency 5kHz, resolution 8 bits
  ledcAttachPin(vibrationPin, 0); // Attach the vibration motor to channel 0

 // Inicialitza IMU (amb DMP)
  imu.Install();


  connectToWiFi();
  udp.begin(udpPort);
  Serial.println("UDP initialized");

  pinMode(PIN_S1, INPUT);
  pinMode(PIN_S2, INPUT);
}

void loop() {
    receiveTorquesUDP();
    updateOrientation();
    sendOrientationUDP();
    delay(10);
}
