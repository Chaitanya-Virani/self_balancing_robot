#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// -------------------------- PIN DEFINITIONS --------------------------
const int PIN_STBY  = 23;

// Motor A (Left)
const int PIN_PWMA  = 4;
const int PIN_AIN1  = 16;
const int PIN_AIN2  = 17;

// Motor B (Right)
const int PIN_PWMB  = 25;
const int PIN_BIN1  = 27;
const int PIN_BIN2  = 26;

// -------------------------- ENCODERS --------------------------
const int ENC_L_A = 34;
const int ENC_L_B = 35;
const int ENC_R_A = 32;
const int ENC_R_B = 33;

// -------------------------- PWM CONFIG --------------------------
const int PWM_FREQ = 20000;
const int PWM_RES  = 8;
const int PWM_CH_L = 0;
const int PWM_CH_R = 1;

// -------------------------- CONTROL CONFIG --------------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

double Kp = 18.0, Ki = 0.2, Kd = 0.6;
double setpoint = 0.0;
const double SAFE_ANGLE_DEG = 45.0;

volatile long encLeftCount = 0, encRightCount = 0;
const double WHEEL_DIAM_MM = 44.0;
const double WHEEL_CIRCUMFERENCE_MM = WHEEL_DIAM_MM * 3.14159265358979323846;
const int ENCODER_PPR = 20;

double integralTerm = 0.0, lastError = 0.0;
unsigned long lastTimeMicros = 0;

// -------------------------- UTILS --------------------------
inline void safeStandby(bool en) {
  digitalWrite(PIN_STBY, en ? HIGH : LOW);
}

// ✅ Motor Direction Logic
void setMotorLeft(int pwmVal) {
  if (pwmVal > 0) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
    ledcWrite(PWM_CH_L, min(pwmVal, 255));
  } else if (pwmVal < 0) {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
    ledcWrite(PWM_CH_L, min(-pwmVal, 255));
  } else {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    ledcWrite(PWM_CH_L, 0);
  }
}

void setMotorRight(int pwmVal) {
  if (pwmVal > 0) {
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
    ledcWrite(PWM_CH_R, min(pwmVal, 255));
  } else if (pwmVal < 0) {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
    ledcWrite(PWM_CH_R, min(-pwmVal, 255));
  } else {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);
    ledcWrite(PWM_CH_R, 0);
  }
}

// -------------------------- ENCODER ISRs --------------------------
void IRAM_ATTR isrEncLeftA() { bool A = digitalRead(ENC_L_A); bool B = digitalRead(ENC_L_B); if (A == B) encLeftCount++; else encLeftCount--; }
void IRAM_ATTR isrEncLeftB() { bool A = digitalRead(ENC_L_A); bool B = digitalRead(ENC_L_B); if (A != B) encLeftCount++; else encLeftCount--; }
void IRAM_ATTR isrEncRightA(){ bool A = digitalRead(ENC_R_A); bool B = digitalRead(ENC_R_B); if (A == B) encRightCount++; else encRightCount--; }
void IRAM_ATTR isrEncRightB(){ bool A = digitalRead(ENC_R_A); bool B = digitalRead(ENC_R_B); if (A != B) encRightCount++; else encRightCount--; }

// -------------------------- MOTOR TEST --------------------------
void motorTest() {
  Serial.println("Motor Test Start...");
  setMotorLeft(150);
  setMotorRight(150);
  delay(1000);
  setMotorLeft(-150);
  setMotorRight(-150);
  delay(1000);
  setMotorLeft(0);
  setMotorRight(0);
  Serial.println("Motor Test Done");
  delay(1000);
}

// -------------------------- SERIAL PID TUNING --------------------------
void handleSerialInput() {
  static String input = "";

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (input.length() > 0) {
        input.trim();
        if (input.startsWith("Kp=")) Kp = input.substring(3).toFloat();
        else if (input.startsWith("Ki=")) Ki = input.substring(3).toFloat();
        else if (input.startsWith("Kd=")) Kd = input.substring(3).toFloat();
        else if (input.startsWith("set=")) setpoint = input.substring(4).toFloat();
        else if (input == "show") {
          Serial.printf("Kp=%.3f | Ki=%.3f | Kd=%.3f | Set=%.2f\n", Kp, Ki, Kd, setpoint);
        } else {
          Serial.println("Unknown command. Use: Kp= | Ki= | Kd= | set= | show");
        }
        input = "";
      }
    } else {
      input += c;
    }
  }
}

// -------------------------- SETUP --------------------------
void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(PIN_STBY, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);

  ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWMA, PWM_CH_L);
  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWMB, PWM_CH_R);

  pinMode(ENC_L_A, INPUT);
  pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT);
  pinMode(ENC_R_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrEncLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), isrEncLeftB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrEncRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), isrEncRightB, CHANGE);

  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check wiring!");
    while (1) delay(10);
  }

  bno.setExtCrystalUse(true);
  safeStandby(true);
  motorTest();

  lastTimeMicros = micros();
  Serial.println("Setup complete.");
  Serial.println("Type commands like: Kp=200 | Ki=15 | Kd=60 | set=1 | show");
}

// -------------------------- LOOP --------------------------
void loop() {
  handleSerialInput();

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  double pitchDeg = -euler.z();

  unsigned long now = micros();
  double dt = (now - lastTimeMicros) / 1000000.0;
  if (dt <= 0) dt = 0.001;

  double error = pitchDeg - setpoint;
  integralTerm += error * dt;
  double derivative = (error - lastError) / dt;

  double pid_out = Kp * error + Ki * integralTerm + Kd * derivative;
  pid_out = constrain(pid_out, -255, 255);

  if (abs(pitchDeg) > SAFE_ANGLE_DEG) {
    setMotorLeft(0);
    setMotorRight(0);
    Serial.print("SAFE STOP | Angle: ");
    Serial.println(pitchDeg, 1);
    integralTerm = 0;
    delay(200);
  } else {
    int pwmCmd = (int)pid_out;
    setMotorLeft(pwmCmd);
    setMotorRight(pwmCmd);
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.printf("Pitch: %.2f | PID: %.2f | Kp: %.2f Ki: %.2f Kd: %.2f\n",
                  pitchDeg, pid_out, Kp, Ki, Kd);
  }

  lastError = error;
  lastTimeMicros = now;
  delay(2);
}
