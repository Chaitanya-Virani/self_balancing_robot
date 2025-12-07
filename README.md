# Self-Balancing Robot (ESP32 + PID)
![Self Balancing Robot](/media/img/cover_image.jpeg)
## Project Overview
This project aims to build a self-balancing two-wheel robot that stabilizes itself using a PID control algorithm. The robot operates on the principle of an inverted pendulum, where the motors adjust their torque proportional to the tilt error to keep the chassis upright.

## Hardware Requirements
Based on the project specifications:

| Component | Specification | Quantity |
|-----------|---------------|----------|
| **Microcontroller** | ESP32 DevKit V1 (Dual-core, WiFi, BT) | 1 |
| **IMU Sensor** | BNO055 (9DOF, I2C) | 1 |
| **Motor Driver** | TB6612FNG (Dual H-Bridge, 1.2A) | 1 |
| **Motors** | N20 Magnetic Encoder Motors (12V, 300 RPM) | 2 |
| **Power Source** | IMR 18650 Li-ion cells (3.7V, 2000 mAh) | 3 |
| **Chassis** | Foam Board / Custom Frame | 1 |

## Software Dependencies
To compile this project, you need the Arduino IDE with the ESP32 board support installed.

**Required Libraries:**
1.  `Adafruit_BNO055`
2.  `Adafruit_Sensor`
3.  `Wire` (Standard Arduino Library)

## Wiring & Pinout
The wiring follows the logic defined in the firmware and hardware summary:

| Component | Pin Function | ESP32 GPIO |
| :--- | :--- | :--- |
| **Motor Driver (TB6612FNG)** | PWMA (Left Speed) | 4 |
| | AIN1 (Left Direction) | 16 |
| | AIN2 (Left Direction) | 17 |
| | PWMB (Right Speed) | 25 |
| | BIN1 (Right Direction) | 27 |
| | BIN2 (Right Direction) | 26 |
| | STBY (Standby) | 23 |
| **IMU (BNO055)** | SDA (Data) | 21 |
| | SCL (Clock) | 22 |
| | Power | 3.3V or 5V |
| **Encoders (N20 Motors)** | Left Encoder A | 34 |
| | Left Encoder B | 35 |
| | Right Encoder A | 32 |
| | Right Encoder B | 33 |

## Control Theory
The robot utilizes a PID (Proportional-Integral-Derivative) controller.
* **Input:** Current Tilt Angle (Pitch) from BNO055.
* **Setpoint:** $0^{\circ}$ (Perfectly upright).
*  **Algorithm:** $\text{Output} = K_p \cdot \text{Error} + K_i \cdot \int \text{Error} \cdot dt + K_d \cdot \frac{d(\text{Error})}{dt}$
* **Output:** Motor speed and direction via PWM.

## Installation & Execution
1.  **Hardware Assembly:** Mount the motors and electronics on the chassis. Ensure the BNO055 is mounted flat and securely.
2.  **Library Installation:** Open Arduino IDE > Sketch > Include Library > Manage Libraries. Search for and install "Adafruit BNO055".
3.  **Upload:** Connect the ESP32 via USB and upload the provided `.ino` file.
4.  **Calibration:** * Place the robot on a flat surface.
    * Open the Serial Monitor (Baud: 115200).
    * Ensure the BNO055 is detected.

## Tuning Guide 
The PID values can be tuned in real-time via the Serial Monitor. 
* **Initial Values:** `Kp=18.0`, `Ki=0.2`, `Kd=0.6`.
* **Commands:** Type the following into the Serial Monitor to adjust values:
    * `Kp=20` (Sets Proportional gain)
    * `Ki=0.5` (Sets Integral gain)
    * `Kd=10` (Sets Derivative gain)
    * `show` (Displays current values)

**Tuning Strategy:**
1.  Set `Ki` and `Kd` to 0. Increase `Kp` until the robot starts oscillating correctly around the setpoint.
2.  Increase `Kd` to dampen the oscillations.
3.  Increase `Ki` slightly to correct any steady-state error (falling to one side).

## Safety Features
* **Safe Angle Stop:** If the robot tilts beyond 45 degrees, the motors will automatically shut off to prevent damage.
## Gallery
[📂 Click here to view the Project Gallery](media/)
