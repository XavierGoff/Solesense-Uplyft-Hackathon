#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"


MAX30105 particleSensor;

// =======================
// FSR SETUP
// =======================
const int fsrPins[3] = {A0, A1, A2};
const char* fsrNames[3] = {"TOE", "HEEL", "MIDDLE"};

const int threshold = 70;
const int releaseThreshold = 40;
const unsigned long minPulseGap = 40;
const unsigned long maxPulseGap = 700;
const unsigned long tapWindow = 3000;
const int minTapCount = 4;

bool sensorPressed[3] = {false, false, false};
unsigned long lastTapTime[3] = {0, 0, 0};
unsigned long tapStartTime[3] = {0, 0, 0};
int tapCount[3] = {0, 0, 0};
bool tapDetected[3] = {false, false, false};

// =======================
// MPU6050 SETUP
// =======================
const int MPU_ADDR = 0x68;

int16_t gyroX_raw, gyroY_raw, gyroZ_raw;

float gyroX_offset = 0;
float gyroY_offset = 0;
float gyroZ_offset = 0;

float gyroX, gyroY, gyroZ;
float gyroMag = 0;
float smoothGyroMag = 0;

const float alpha = 0.2;
const float shakeOnThreshold = 70.0;
const float shakeOffThreshold = 40.0;
const unsigned long shakeOnTime = 300;
const unsigned long shakeOffTime = 500;

bool shaking = false;
unsigned long aboveThresholdStart = 0;
unsigned long belowThresholdStart = 0;

// =======================
// HEART RATE SETUP
// =======================
const byte RATE_SIZE = 8;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute = 0;
int beatAvg = 0;
bool hrReady = false;

// =======================
// SETUP
// =======================
void setup() {
  Serial.begin(9600);
  Wire.begin();

  wakeMPU();
  setGyroRange();

  Serial.println("Keep MPU still for calibration...");
  calibrateGyro();
  Serial.println("Calibration done.");
  Serial.println("Combined FSR tap + MPU shake detection started...");

  if (particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    particleSensor.setup(0x1F, 4, 2, 400, 411, 4096);
    hrReady = true;
    Serial.println("MAX30102 ready");
  } else {
    Serial.println("MAX30102 not found - skipping HR");
  }
}

// =======================
// LOOP
// =======================
void loop() {
  unsigned long now = millis();

  if (hrReady) updateHeartRate();

  updateFSRTaps(now);
  updateShakeDetection(now);
  printCombinedStatus();

  delay(20);
}

// =======================
// FSR TAP DETECTION
// =======================
void updateFSRTaps(unsigned long now) {
  for (int i = 0; i < 3; i++) {
    int value = analogRead(fsrPins[i]);

    if (!sensorPressed[i] && value > threshold) {
      sensorPressed[i] = true;
      unsigned long gap = now - lastTapTime[i];

      if (lastTapTime[i] == 0) {
        tapStartTime[i] = now;
        tapCount[i] = 1;
      } else if (gap >= minPulseGap && gap <= maxPulseGap) {
        tapCount[i]++;
      } else {
        tapStartTime[i] = now;
        tapCount[i] = 1;
        tapDetected[i] = false;
      }
      lastTapTime[i] = now;
    }

    if (sensorPressed[i] && value < releaseThreshold) {
      sensorPressed[i] = false;
    }

    if (tapCount[i] >= minTapCount && tapStartTime[i] != 0 && (now - tapStartTime[i] >= tapWindow)) {
      tapDetected[i] = true;
    }

    if (lastTapTime[i] != 0 && (now - lastTapTime[i] > maxPulseGap)) {
      tapCount[i] = 0;
      tapStartTime[i] = 0;
      tapDetected[i] = false;
      sensorPressed[i] = false;
    }
  }
}

// =======================
// HEART RATE DETECTION
// =======================
void updateHeartRate() {
  long irValue = particleSensor.getIR();

  if (irValue < 50000) {
    beatAvg = 0;
    lastBeat = 0;
    for (byte x = 0; x < RATE_SIZE; x++) rates[x] = 0;
    rateSpot = 0;
    return;
  }

  if (checkForBeat(irValue) == true) {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60000.0 / delta;

    if (beatsPerMinute > 50 && beatsPerMinute < 150) {
      rates[rateSpot++ % RATE_SIZE] = (byte)beatsPerMinute;
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
}

// =======================
// SHAKE DETECTION
// =======================
void updateShakeDetection(unsigned long now) {
  readGyro();

  gyroX = (gyroX_raw - gyroX_offset) / 131.0;
  gyroY = (gyroY_raw - gyroY_offset) / 131.0;
  gyroZ = (gyroZ_raw - gyroZ_offset) / 131.0;

  gyroMag = sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);
  smoothGyroMag = alpha * gyroMag + (1.0 - alpha) * smoothGyroMag;

  if (!shaking) {
    if (smoothGyroMag > shakeOnThreshold) {
      if (aboveThresholdStart == 0) aboveThresholdStart = now;
      if (now - aboveThresholdStart >= shakeOnTime) {
        shaking = true;
        belowThresholdStart = 0;
      }
    } else {
      aboveThresholdStart = 0;
    }
  } else {
    if (smoothGyroMag < shakeOffThreshold) {
      if (belowThresholdStart == 0) belowThresholdStart = now;
      if (now - belowThresholdStart >= shakeOffTime) {
        shaking = false;
        aboveThresholdStart = 0;
      }
    } else {
      belowThresholdStart = 0;
    }
  }
}

// =======================
// SERIAL OUTPUT
// =======================
void printCombinedStatus() {
  for (int i = 0; i < 3; i++) {
    int value = analogRead(fsrPins[i]);
    Serial.print(fsrNames[i]);
    Serial.print(":");
    Serial.print(value);
    Serial.print(" C:");
    Serial.print(tapCount[i]);
    Serial.print(" ");
  }

  Serial.print("| ");

  bool anyTap = false;
  for (int i = 0; i < 3; i++) {
    if (tapDetected[i]) {
      Serial.print(fsrNames[i]);
      Serial.print("_TAP ");
      anyTap = true;
    }
  }
  if (!anyTap) Serial.print("NO_TAP ");

  Serial.print("| Gyro:");
  Serial.print(gyroMag, 1);
  Serial.print(" Smooth:");
  Serial.print(smoothGyroMag, 1);
  Serial.print(" | ");

  if (shaking) {
    Serial.print("SHAKING");
  } else {
    Serial.print("NOT SHAKING");
  }

  Serial.print(" | HR:");
  Serial.print(beatAvg);
  Serial.println();
}

// =======================
// MPU6050 HELPERS
// =======================
void wakeMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void setGyroRange() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

void readGyro() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  gyroX_raw = Wire.read() << 8 | Wire.read();
  gyroY_raw = Wire.read() << 8 | Wire.read();
  gyroZ_raw = Wire.read() << 8 | Wire.read();
}

void calibrateGyro() {
  const int samples = 1000;
  long gx = 0, gy = 0, gz = 0;
  for (int i = 0; i < samples; i++) {
    readGyro();
    gx += gyroX_raw;
    gy += gyroY_raw;
    gz += gyroZ_raw;
    delay(3);
  }
  gyroX_offset = gx / (float)samples;
  gyroY_offset = gy / (float)samples;
  gyroZ_offset = gz / (float)samples;
}
