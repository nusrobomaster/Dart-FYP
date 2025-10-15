# Strain Gauge Sensor & Screen Sensor Interface

## 📘 Overview
This project interfaces a **strain gauge sensor** (for force/load measurement) and a **screen display** (for visual output) with a microcontroller.  
It provides live readings of strain data and displays them on the connected screen for real-time monitoring.

---

## ⚙️ Hardware Components
- **Strain Gauge Sensor** (e.g. HX711 amplifier module)
- **Screen Sensor/Display** (e.g. ILI9341, ILI9488, ST7789, etc.)
- **Microcontroller** (e.g. STM32, ESP32, Raspberry Pi, etc.)
- **Power Supply**
- **Connecting wires and breadboard/PCB**

---

## 🧩 Pin Layout

### Strain Gauge Sensor
| Pin | Microcontroller Pin | Description |
|------|----------------------|--------------|
| VCC | *[5V]* | Power supply (2.7V–5V) |
| GND | *[Gnd]* | Ground connection |
| DT  | *[I2]* / PF0 | Data output (HX711 DOUT) |
| SCK | *[I1]* / PF1 | Clock input (HX711 SCK) |


---

### Screen Sensor / Display
| Pin | Microcontroller Pin | Description |
|------|----------------------|--------------|
| VCC | *[3V]* | Power supply |
| GND | *[GND]* | Ground |
| CS  | *[M1]* / PC3 | Chip Select |
| DC/RS | *[L1]* / PC2 | Data/Command |
| MOSI | *[K1]* / PE6 | SPI Data |
| MISO | *[J1]* / PE5 | SPI Data (if used) |
| SCK | *[K2]* / PE12 | SPI Clock |
| RESET | *[L2]* / PB0 | Hardware reset |
| LED | *[M2]* / PB1 | LED |
---

## 🧠 System Description
1. **Strain Gauge Sensor** measures deformation and outputs an analog signal.  
2. **HX711 module** amplifies and digitizes the signal.  
3. **Microcontroller** reads the data via GPIO/SPI/I²C.  
4. **Screen sensor** displays the processed value (e.g. force, weight, strain).  
5. Optionally, calibration routines and tare functionality are implemented for accuracy.

---

## 💻 Software Requirements
- Platform: STM32CubeIDE 
- Libraries:
  - `hx711.h` for the strain gauge ADC  
  - `LCD_tft` for the driver of the screen
  - `lvgl.h` or `TFT_eSPI.h` (tbc)
- RTOS: FreeRTOS / CMSIS-RTOS2

---

## 🔧 Setup Instructions
1. Wire the strain gauge and screen according to the pin layout above.  
2. Flash the firmware using your IDE.  
3. Power up the system and verify sensor readings via serial output.  
4. Confirm screen initialization (display logo or welcome message).  
5. Calibrate the strain gauge by placing known weights and recording readings.

---

## 🧮 Calibration Notes
Collect data for at least two known weights (e.g. 0 g and 500 g).  
Compute the **scale factor** as:

