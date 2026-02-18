#include <WiFi.h>
#include <esp_now.h>

// ------------------ SLAVE MAC ADDRESSES ------------------

// BMI SLAVES
uint8_t bmiSlave1[] = {0x88, 0x57, 0x21, 0x2F, 0x1B, 0x54};
uint8_t bmiSlave2[] = {0xEC, 0xE3, 0x34, 0x9A, 0xE2, 0xEC};

// FSR SLAVES
uint8_t fsrSlave1[] = {0x00, 0x4B, 0x12, 0x97, 0x39, 0x94};
uint8_t fsrSlave2[] = {0x78, 0x1C, 0x3C, 0xF5, 0xE4, 0x74};

// ------------------ STRUCTS ------------------
typedef struct {
  float roll;
  float pitch;
  float yaw;
} BMI160_Data;

typedef struct {
  int pressure;
} FSR_Data;

BMI160_Data bmi1;
BMI160_Data bmi2;

FSR_Data fsr1;
FSR_Data fsr2;

// ------------------ MOTOR PINS ------------------
#define RPWM 25
#define LPWM 26

// ------------------ STATES ------------------
bool isBending = false;
bool isStanding = false;

bool fsr1Pressure = false;
bool fsr2Pressure = false;
bool isLifting = false;

bool motorRunning = false;
unsigned long motorStartTime = 0;

int motorMode = 0;
const int pressureThreshold = 3000;
const unsigned long runTime = 1250; // 1 sec

// ------------------ MOTOR CONTROL ------------------
void motorCW() {
  analogWrite(RPWM, 200);
  analogWrite(LPWM, 0);
  motorStartTime = millis();
  motorRunning = true;
  Serial.println("🔥🔥🔥 MOTOR → CLOCKWISE (1 SEC)");
}

void motorCCW() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 200);
  motorStartTime = millis();
  motorRunning = true;
  Serial.println("💧💧💧 MOTOR → ANTICLOCKWISE (1 SEC)");
}

void motorStop() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  motorRunning = false;
  Serial.println("🛑 MOTOR STOPPED");
}

// ------------------ CALLBACK ------------------
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {

  // ------------------ BMI #1 ------------------
  if (memcmp(info->src_addr, bmiSlave1, 6) == 0) {
    memcpy(&bmi1, data, sizeof(bmi1));

    Serial.println("--------- BMI #1 ---------");
    Serial.print("🌀 Roll: ");   Serial.print(bmi1.roll);
    Serial.print(" | 🎯 Pitch: "); Serial.print(bmi1.pitch);
    Serial.print(" | 🧭 Yaw: ");   Serial.println(bmi1.yaw);

    if (bmi1.yaw < 50) {
      isStanding = true;
      isBending = false;
      Serial.println("💧 POSITION: STANDING 💧");
    }
    else if (bmi1.yaw > 80) {
      isStanding = false;
      isBending = true;
      Serial.println("🔥 POSITION: BENDING 🔥");
    }
  }

  // ------------------ BMI #2 ------------------
  else if (memcmp(info->src_addr, bmiSlave2, 6) == 0) {
    memcpy(&bmi2, data, sizeof(bmi2));

    Serial.println("--------- BMI #2 ---------");
    Serial.print("🌀 Roll: ");   Serial.print(bmi2.roll);
    Serial.print(" | 🎯 Pitch: "); Serial.print(bmi2.pitch);
    Serial.print(" | 🧭 Yaw: ");   Serial.println(bmi2.yaw);

    if (bmi2.yaw < 50) {
      isStanding = true;
      isBending = false;
      Serial.println("💧 POSITION: STANDING (BMI2) 💧");
    }
    else if (bmi2.yaw > 80) {
      isStanding = false;
      isBending = true;
      Serial.println("🔥 POSITION: BENDING (BMI2) 🔥");
    }
  }

  // ------------------ FSR #1 ------------------
  else if (memcmp(info->src_addr, fsrSlave1, 6) == 0) {
    memcpy(&fsr1, data, sizeof(fsr1));
    Serial.print("📩 FSR #1 Pressure: "); Serial.print(fsr1.pressure);

    if (fsr1.pressure >= pressureThreshold) {
      fsr1Pressure = true;
      Serial.println(" 🔥 (fsr1 Pressure)");
    } else {
      fsr1Pressure = false;
      Serial.println(" 💧 (fsr1 no Pressure)");
    }
  }

  // ------------------ FSR #2 ------------------
  else if (memcmp(info->src_addr, fsrSlave2, 6) == 0) {
    memcpy(&fsr2, data, sizeof(fsr2));
    Serial.print("📩 FSR #2 Pressure: "); Serial.print(fsr2.pressure);

    if (fsr2.pressure >= pressureThreshold) {
      fsr2Pressure = true;
      Serial.println(" 🔥 (fsr2 Pressure)");
    } else {
      fsr2Pressure = false;
      Serial.println(" 💧 (fsr2 no Pressure)");
    }
  }

  // Lifting logic
  isLifting = (fsr1Pressure || fsr2Pressure);

  // ------------------ STATE MACHINE ------------------

  // Step 1: Bending + Pressure → Clockwise
  if (!motorRunning && (motorMode == 0 || motorMode == 1)) {
    if (isBending && isLifting) {
      motorCW();
      motorMode = 3; // wait for standing
    }
  }

  // Step 2: Standing + No Pressure → Anticlockwise
  else if (!motorRunning && motorMode == 3) {
    if (isStanding && !isLifting) {
      motorCCW();
      motorMode = 1; // restart loop
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);

  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(onDataRecv);

  Serial.println("MASTER READY — 4 SLAVES CONNECTED");
}

void loop() {
  if (motorRunning) {
    Serial.println("🔥");
  }
  if (motorRunning && (millis() - motorStartTime >= runTime)) {
    motorStop();
  }
}
