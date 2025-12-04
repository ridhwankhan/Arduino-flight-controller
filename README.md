# 🚀 Custom High-Performance Quadcopter Flight Controller 🚁

| 🛠️ Tech Stack | 🧠 Core Logic | 🎯 Hardware Focus | 💻 Language |
| :---: | :---: | :---: | :---: |
| **Custom FC (Not YMFC-AL)** | **MPU-6050/6500** | **Tuned for F405 Frame** | **C/C++ (Arduino)** |

This repository houses a meticulously engineered, highly optimized, and **bug-fixed** flight controller based on the original YMFC-AL project structure. This code is specifically tailored for enhanced performance and custom frame configurations, moving far beyond the foundation of the original YMFC-AL firmware.

**⚠️ SAFETY FIRST: Always remove all propellers before connecting the LiPo battery, powering up the flight controller, or running any calibration/setup code.**

***

## ✨ Core Features & Optimizations

This is **Custom FC Code**, optimized for modern setups (e.g., F405 style) and high-speed operation.

* **Fixed Critical YMFC Bugs:** Includes fixes for instability and logic issues found in the latest YMFC-AL release. The custom setup program (`main-setup-by-ridhwan-all-works-fine.ino`) includes increased tolerance for stick center positions and avoids fatal errors by using default values where possible to ensure a complete setup.
* **Precision Tuning:** Specialized PID and sensor fusion parameters integrated into the final flight code (`fc-ridhwan.ino`) for tighter control and superior stability on custom airframes.
* **Loop Time Integrity:** The main control loop is rigorously managed to run at **250Hz** (4000µs loop time), crucial for accurate angle calculations and flight control.
* **Efficient Interrupts:** Refactored Interrupt Service Routine (ISR) logic provides more deterministic and faster processing of incoming RC signals.

***

## 🛠️ Hardware & Wiring Diagram

The code is designed for an Arduino UNO (or ATmega328P compatible) and an MPU-6050/6500 IMU.

| Component | Arduino Pin | Function | Schematic Reference |
| :--- | :--- | :--- | :--- |
| **Roll (CH1)** | Digital Pin 8 (PCINT0) | RC Input | Receiver |
| **Pitch (CH2)** | Digital Pin 9 (PCINT1) | RC Input | Receiver |
| **Throttle (CH3)**| Digital Pin 10 (PCINT2) | RC Input | Receiver |
| **Yaw (CH4)** | Digital Pin 11 (PCINT3) | RC Input | Receiver |
| **ESC 1 (Right-Front)**| Digital Pin 4 | ESC Output | M1 |
| **ESC 2 (Right-Rear)**| Digital Pin 5 | ESC Output | M2 |
| **ESC 3 (Left-Rear)**| Digital Pin 6 | ESC Output | M3 |
| **ESC 4 (Left-Front)**| Digital Pin 7 | ESC Output | M4 |
| **MPU-6050 SDA**| Analog Pin A4 (SDA) | I2C Comm | MPU-6050 |
| **MPU-6050 SCL**| Analog Pin A5 (SCL) | I2C Comm | MPU-6050 |
| **Battery Sense**| Analog Pin A0 | Voltage Monitoring | R3/R2 Divider |
| **LED** | Digital Pin 12 | Status/Warning | LED |

***

## 📋 Detailed Installation & Quick Start Guide

Follow these steps precisely, in order. The flight controller is entirely reliant on the calibration data stored in the EEPROM by the setup programs.

### **STEP 0: Clean the EEPROM (Recommended)**

Before starting, it is best practice to clear any old EEPROM data.

1.  Open `Clear_all_EEPROM_data_on_an_Arduino/Clear_EEPROM_data/Clear_EEPROM_data.ino`.
2.  Upload this sketch to your Arduino UNO.
3.  Open the Serial Monitor at 9600 baud. It should print **"EEPROM cleared successfully."**.
4.  Remove power and USB, then proceed to Step 1.

### **STEP 1: Run Transmitter & Gyro Setup**

This step detects your receiver stick endpoints, assigns channels, and calibrates the Gyro offset.

1.  Open **`YMFC-AL_setup/main-setup-by-ridhwan-all-works-fine/main-setup-by-ridhwan-all-works-fine.ino`**.
2.  Upload the sketch to your Arduino UNO.
3.  Open the **Serial Monitor** and set the baud rate to **57600**.
4.  **Follow the on-screen instructions precisely.** This involves:
    * Waiting for valid RC signals.
    * Centering all sticks and subtrims.
    * Moving each stick (Throttle, Roll, Pitch, Yaw) to its maximum and minimum extents as prompted.
    * Holding the quadcopter still for the 8-second Gyro calibration.
    * Tilting the quadcopter to map the Roll, Pitch, and Yaw axes correctly.
5.  Once complete, the program will print **"Setup is finished."** and **"You can now calibrate the esc's..."**.

### **STEP 2: Calibrate ESCs**

The ESC calibration procedure ensures all motors respond identically to the flight controller's commands.

1.  Open **`YMFC-AL_esc_calibrate/YMFC-AL_esc_calibrate.ino`**.
2.  Upload the sketch to your Arduino UNO.
3.  Open the **Serial Monitor** at **57600 baud**.
4.  Follow the standard ESC calibration ritual:
    * The code immediately sets the output to **MAX throttle (2000µs)** for all ESC pins.
    * **Now, connect your flight battery.** The ESCs should emit a tone indicating max throttle registered.
    * The program will automatically drop the pulse to **MIN throttle (1000µs)**. The ESCs should respond with battery cell count beeps and a final long beep, confirming calibration.
    * Disconnect the flight battery immediately after calibration is confirmed.

### **STEP 3: Upload Flight Controller Code & Fly!**

1.  Open **`YMFC-AL_Flight_controller/fc-ridhwan/fc-ridhwan.ino`**.
2.  Review and adjust PID gains (lines 8-20) if necessary, based on your frame size and weight.
3.  Upload the sketch to your Arduino UNO.
4.  **Arming the Motors:** Ensure throttle stick is at minimum and push the Yaw stick **to the left** (min yaw).
5.  **Disarming the Motors:** Push the Throttle stick to minimum and push the Yaw stick **to the right** (max yaw).
6.  **Flying:** Slowly increase throttle. The PID loop will control the motors based on your Roll and Pitch inputs.

***

## ⚠️ Critical Warning

The code relies on a strict loop timing of 4000µs. If the loop time is longer or shorter, the angle calculation (IMU fusion) will be incorrect, potentially leading to instability or crashes. If the time exceeds 4050µs, the status LED (D12) will turn on. Avoid adding any time-consuming functions (like excessive `Serial.print` statements) to the main `loop()` function.
