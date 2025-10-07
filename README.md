
# 🚗 Smart Vehicle Security & Safety System (STM32F405 + FreeRTOS)

## 📖 Overview

This project implements a **vehicle security and safety system** on the STM32F405 microcontroller using **FreeRTOS**.
It combines **remote authentication, EEPROM-based user codes, LCD display, motor control, and multi-sensor monitoring** to provide a reliable embedded safety solution.

System protects against:

* 🔥 **Over-Temperature**
* ⚡ **High Vibration (Accidental Detection)**
* 🚧 **Collision Detection (IR sensor)**

On detection, the system:

* Turns **Engine OFF**
* Displays alert messages on **LCD**
* Sends **alerts via Bluetooth (UART2)**

Authentication is required before engine operation using a **Remote Control and SPI EEPROM-based user codes**.



## ⚙️ Features

* **Remote-Controlled Power-On & Authentication**

  * User selects ID (1–5) and enters a 4-digit security code stored in **SPI EEPROM (M95640)**.
  * LCD prompts guide the process.
* **Password Management**

  * Default code = `1234`
  * Change password menu (via remote).
* **Sensor Monitoring**

  * **IR sensor (PC13)** → Collision Detection
  * **SW420 vibration sensor (PB5)** → Accidental Detection
  * **Temperature via ADC (PC2, ADC1_IN12)** → Overheat Detection
* **Engine Control**

  * Motor simulated using **PA9 / PA10** GPIO pins.
* **LCD Display**

  * Shows system status, menu, and alerts.
* **LED Indicators**

  * **Green LED (PC8)** → Access Granted / Normal
  * **Red LED (PC12)** → Access Denied / Fault
* **Bluetooth Alerts**

  * Sent through **UART2**:

    * `"STATUS: Engine ON"`
    * `"ALERT: Over Temp!"`
    * `"ALERT: Vibration detected!"`
    * `"ALERT: Collision detected!"`



## 🔌 Hardware Connections

| Module / Signal             | STM32F405 Pin                              |
| --------------------------- | ------------------------------------------ |
| LCD RS / EN / D4–D7         | As defined in `lcd.h`                      |
| IR Obstacle Sensor          | PC13                                       |
| Vibration Sensor SW420      | PB5                                        |
| Temperature Sensor          | PC2 (ADC1_IN12)                            |
| Motor Control               | PA9, PA10                                  |
| EEPROM (SPI M95640)         | SPI1 (PA5=SCK, PA6=MISO, PA7=MOSI, PA4=CS) |
| Bluetooth Module (HC-05/06) | USART2 (PA2=TX, PA3=RX)                    |
| LED Green                   | PC8                                        |
| LED Red                     | PC12                                       |
| IR Remote Receiver          | TIM2 CH1 (via external pin)                |



## 🛠 Software Architecture

* **RTOS Tasks**

  * `Task_Remote` → Handles remote authentication, password verification, and menu.
  * `TempTask` → Reads ADC temperature, compares against threshold.
  * `VibrationTask` → Monitors SW420 vibration sensor.
  * `CollisionTask` → Monitors IR sensor on PC13.
  * `EngineTask` → Controls motor state, updates LCD, and sends Bluetooth alerts.



## 🚀 How It Works

1. On power-up, LCD shows **Welcome**.
2. User presses `*` on remote to start login.
3. User enters ID (1–5) and 4-digit security code (`#` to confirm).
4. If valid → Access Granted (Green LED ON).
   If invalid → Access Denied (Red LED ON).
5. Once inside:

   * System monitors sensors continuously.
   * If **any fault** → Engine stops + LCD shows alert + Bluetooth sends alert.
   * If safe → Engine remains ON.

