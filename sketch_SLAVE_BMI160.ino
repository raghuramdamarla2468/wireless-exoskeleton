#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include "DFRobot_BMI160.h"

DFRobot_BMI160 bmi160;

// -------- STRUCT TO SEND ROLL & PITCH --------
typedef struct {
  float roll;
  float pitch;
  float yaw;
} struct_message;

struct_message myData;

// MASTER MAC
uint8_t masterAddress[] = {0xEC, 0xE3, 0x34, 0x99, 0xF4, 0x50};

// ---------- CORRECT CALLBACK FOR ESP32 CORE 3.x ----------
void onSendStatus(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("SEND: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

// ---------- FILTER VARIABLES ----------
float rollFiltered = 0, pitchFiltered = 0, yawFiltered = 0;

void setup() {
  Serial.begin(115200);

  // ---------- START I2C ----------
  Wire.begin(21, 22);
  Serial.println("Initialising BMI160...");

  // BMI INIT
  if (bmi160.I2cInit(0x69) != BMI160_OK) {
    if (bmi160.I2cInit(0x68) != BMI160_OK) {
      Serial.println("BMI160 Init Failed!");
      while (1);
    }
  }

  Serial.println("BMI160 Ready!");

  // ---------- START WIFI ----------
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP NOW FAILED!");
    return;
  }

  // Register callback (CORRECT FORMAT)
  esp_now_register_send_cb(onSendStatus);

  // ---------- ADD MASTER ----------
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, masterAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer!");
    return;
  }

  Serial.println("SLAVE READY & SENDING...");
}

void loop() {

  int16_t accel[3], gyro[3];

  // Read BMI160
  bmi160.getAccelData(accel);
  bmi160.getGyroData(gyro);

  // Gyro values (your original logic)
  float gx = gyro[0] / 131.0;
  float gy = gyro[1] / 131.0;
  float gz = gyro[2] / 131.0;

  // Apply smoothing
  rollFiltered  = 0.9 * rollFiltered  + 0.1 * gx;
  pitchFiltered = 0.9 * pitchFiltered + 0.1 * gy;
  yawFiltered = 0.9 * yawFiltered + 0.1 * gz;

  // Load into struct
  myData.roll  = rollFiltered;
  myData.pitch = pitchFiltered;
  myData.yaw = yawFiltered;

  // ---------- SEND TO MASTER ----------
  esp_now_send(masterAddress, (uint8_t *)&myData, sizeof(myData));

  // Debug
  Serial.print("ROLL: ");
  Serial.print(rollFiltered);
  Serial.print(" | PITCH: ");
  Serial.println(pitchFiltered);
  Serial.print(" | YAW: ");
  Serial.println(yawFiltered);

  delay(50);
}
