#include <Wire.h>
#include <math.h>

// MPU-9265 I2C address (AD0 = GND → 0x68, AD0 = VCC → 0x69)
const uint8_t MPU_ADDR = 0x68;

// MPU registers
const uint8_t PWR_MGMT_1   = 0x6B;
const uint8_t ACCEL_XOUT_H = 0x3B;
const uint8_t GYRO_XOUT_H  = 0x43;
const uint8_t INT_PIN_CFG  = 0x37;

// Raw sensor data
int16_t AcX, AcY, AcZ;
int16_t GyX, GyY, GyZ;

// Offsets
int16_t AcX_offset = 0;
int16_t AcY_offset = 0;
int16_t AcZ_offset = 0;
int16_t GyX_offset = 0;
int16_t GyY_offset = 0;
int16_t GyZ_offset = 0;

// Complementary filter constants
const float GYRO_TRUST_FACTOR = 0.98;
const float ACC_TRUST_FACTOR  = 0.02;

float angle = 0.0;
unsigned long lastPrint = 0;
unsigned long last_time;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // ESP32: SDA=21, SCL=22
  delay(100);

  // Wake up MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0); // Exit sleep mode
  Wire.endTransmission(true);
  delay(50);

  // Enable bypass mode for magnetometer (optional)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(INT_PIN_CFG);
  Wire.write(0x02); // BYPASS_EN
  Wire.endTransmission(true);
  delay(50);

  // Calibrate offsets
  calibrateSensors();

  last_time = millis();
}

void loop() {
  readMPU();

  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0; // convert ms to seconds
  last_time = current_time;

  // Gyro rate in deg/s (assuming ±250 deg/s)
  float gyro_x_rate = (GyX - GyX_offset) / 131.0;

  // Angle from accelerometer (X-axis tilt)
  float angle_acc = atan2(AcY - AcY_offset, AcZ - AcZ_offset) * 180.0 / PI;

  // Complementary filter
  angle = GYRO_TRUST_FACTOR * (angle + gyro_x_rate * dt) + ACC_TRUST_FACTOR * angle_acc;

  // Print every 1 second
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    Serial.print("AcX: "); Serial.print(AcX - AcX_offset);
    Serial.print("  AcY: "); Serial.print(AcY - AcY_offset);
    Serial.print("  AcZ: "); Serial.print(AcZ - AcZ_offset);

    Serial.print("  GyX: "); Serial.print(GyX - GyX_offset);
    Serial.print("  GyY: "); Serial.print(GyY - GyY_offset);
    Serial.print("  GyZ: "); Serial.print(GyZ - GyZ_offset);

    Serial.print("  Angle: "); Serial.println(angle);
  }
}

// Read raw accelerometer and gyro
void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  AcX = Wire.read()<<8 | Wire.read();
  AcY = Wire.read()<<8 | Wire.read();
  AcZ = Wire.read()<<8 | Wire.read();

  Wire.read(); Wire.read(); // skip temperature

  GyX = Wire.read()<<8 | Wire.read();
  GyY = Wire.read()<<8 | Wire.read();
  GyZ = Wire.read()<<8 | Wire.read();

  delay(2); // small delay to avoid I2C NACKs
}

// Improved calibration: average 500 readings
void calibrateSensors() {
  long AcX_sum=0, AcY_sum=0, AcZ_sum=0;
  long GyX_sum=0, GyY_sum=0, GyZ_sum=0;
  const int samples = 500; // more samples for better calibration

  Serial.println("Calibrating... Keep MPU flat and still!");

  for(int i=0; i<samples; i++){
    readMPU();
    AcX_sum += AcX;
    AcY_sum += AcY;
    AcZ_sum += AcZ;
    GyX_sum += GyX;
    GyY_sum += GyY;
    GyZ_sum += GyZ;
    delay(10);
  }

  AcX_offset = AcX_sum / samples;
  AcY_offset = AcY_sum / samples;
  AcZ_offset = AcZ_sum / samples - 16384; // gravity
  GyX_offset = GyX_sum / samples;
  GyY_offset = GyY_sum / samples;
  GyZ_offset = GyZ_sum / samples;

  Serial.println("Calibration done.");

  // Initialize complementary filter angle to accelerometer
  angle = atan2(AcY - AcY_offset, AcZ - AcZ_offset) * 180.0 / PI;
}