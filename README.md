# 🌬️ SGP40-AlcSense: Ultra-Compact IoT Breathalyzer

![Project Status](https://img.shields.io/badge/Status-Prototype-orange) ![License](https://img.shields.io/badge/License-MIT-blue) ![Hardware](https://img.shields.io/badge/Hardware-nRF52%20%7C%20SGP40-green)

**SGP40-AlcSense** is a portable, USB-C rechargeable breath analysis device (Breathalyzer) measuring only **30mm x 20mm**.

This project combines advanced **Sensirion SGP40 VOC** sensor technology with the power of the **Nordic nRF52832** SoC to measure alcohol intensity (VOC Index) and provide instant visual feedback via an RGB LED or transmit data to a mobile app via Bluetooth Low Energy (BLE).

---

## 📸 Project Images

| PCB Top View | PCB Bottom View | 3D Render |
| :---: | :---: | :---: |
| *[Add Top PCB Image Here]* | *[Add Bottom PCB Image Here]* | *[Add 3D Render Here]* |

---

## 🌟 Key Features

* **Ultra-Compact Design:** Keychain-sized (30x20mm) 2-layer PCB.
* **High-Sensitivity Detection:** Sensirion SGP40 MOx sensor sensitive to Alcohol (Ethanol) vapor.
* **Wireless Connectivity:** Bluetooth 5.0 (BLE) support via nRF52832 (ARM Cortex-M4F).
* **Visual Feedback:** RGB LED changes color based on ambient alcohol levels (Green/Blue/Red).
* **Power Management:**
    * Integrated Li-Po Battery Charging Circuit (MCP73831).
    * DC-DC Buck Converter mode for low power consumption.
    * USB Type-C Charging Input.
* **RF Design:** Johanson 2450AT18A Chip Antenna with impedance matching (Pi-Network).

---

## 🛠️ Hardware Specifications

The project is built on the following core components:

| Component | Model | Description |
| :--- | :--- | :--- |
| **MCU** | `nRF52832-QFAA` | Main Processor & BLE Module. |
| **Sensor** | `SGP40-D-R4` | Digital VOC (Volatile Organic Compound) Sensor. |
| **Charger IC** | `MCP73831` | Li-Ion/Li-Po Charge Controller (CC/CV). |
| **Regulator** | `XC6206` | 3.3V LDO Voltage Regulator. |
| **Antenna** | `2450AT18A0110` | 2.4GHz SMD Chip Antenna. |
| **LED** | `XL-1608RGBC` | 0603 SMD RGB LED (Common Anode). |
| **Connector** | `USB-C 16-Pin` | Power input & Charging. |

---

## 🔌 Pinout Map

Pin definitions for firmware development (Arduino/PlatformIO/Segger):

```cpp
// RGB LED Connections (Active LOW)
#define PIN_LED_RED    P0.06
#define PIN_LED_GREEN  P0.07
#define PIN_LED_BLUE   P0.08

// I2C (SGP40 Sensor)
#define PIN_SDA        P0.26
#define PIN_SCL        P0.27

// Battery Monitoring (Optional)
#define PIN_VBAT_SENSE P0.xx  // (ADC Channel)