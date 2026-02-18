#include <WiFi.h>
#include <esp_now.h>

// --- MAC address of Master ---
uint8_t masterAddress[] = {0xEC, 0xE3, 0x34, 0x99, 0xF4, 0x50};

// --- FSR Pin ---
#define FSR1_PIN 34  // Analog Pin for FSR Pad
#define FSR2_PIN 35  // Analog Pin for FSR Strip

// --- Data structure ---
typedef struct {
  int pressure;
} FSR_Data;

FSR_Data fsr1Data;
FSR_Data fsr2Data;

// --- Callback (send confirmation) ---
void onDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "✅ Success" : "❌ Fail");
}

void setup() {
  Serial.begin(115200);
  pinMode(FSR1_PIN, INPUT);
  pinMode(FSR2_PIN, INPUT);

  // Initialize Wi-Fi in STA mode
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW Init Failed");
    return;
  }

  esp_now_register_send_cb(onDataSent);

  // Register master as peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, masterAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("❌ Failed to add Master as peer");
    return;
  }

  Serial.println("✅ FSR Slave Ready to Send Data...");
}

void loop() {
  // Read FSR value
  fsr1Data.pressure = analogRead(FSR1_PIN);
  fsr2Data.pressure = analogRead(FSR2_PIN);

  // Give FSR strip (2) threshhold as it is more sensitive
  if (fsr2Data.pressure < 3500) {
    fsr2Data.pressure = 0;
  }


  // Send highest pressure value
  if (fsr1Data.pressure > fsr2Data.pressure) {
    esp_now_send(masterAddress, (uint8_t *)&fsr1Data, sizeof(fsr1Data));
  } else {
    esp_now_send(masterAddress, (uint8_t *)&fsr2Data, sizeof(fsr2Data));
  }
  

  // Print value to serial monitor
  Serial.print("📤 Pressure: ");
  Serial.println(max(fsr1Data.pressure,fsr2Data.pressure));

  delay(200);
}
