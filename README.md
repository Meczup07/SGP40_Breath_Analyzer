# 🌬️ SGP40-AlcSense: Ultra-Compact IoT Breathalyzer

![Project Status](https://img.shields.io/badge/Status-Prototype-orange) ![License](https://img.shields.io/badge/License-MIT-blue) ![Hardware](https://img.shields.io/badge/Hardware-nRF52%20%7C%20SGP40-green)

**SGP40-AlcSense** is a portable, USB-C rechargeable breath analysis device (Breathalyzer) measuring only **30mm x 20mm**.

This project combines advanced **Sensirion SGP40 VOC** sensor technology with the power of the **Nordic nRF52832** SoC to measure alcohol intensity (VOC Index) and provide instant visual feedback via an RGB LED or transmit data to a mobile app via Bluetooth Low Energy (BLE).

---

## 📸 Project Images

<img width="798" height="713" alt="1" src="https://github.com/user-attachments/assets/c0f1d064-9244-4460-ad38-2b4a213009cc" />

<img width="760" height="694" alt="2" src="https://github.com/user-attachments/assets/6e43f0ba-f4aa-441f-baec-7cfdd439d7fb" />

---

## 🌟 Key Features

* **Ultra-Compact Design:** Keychain-sized (30x20mm) 2-layer PCB.
* **High-Sensitivity Detection:** Sensirion SGP40 MOx sensor sensitive to Alcohol (Ethanol) vapor.
* **Wireless Connectivity:** Bluetooth 5.0 (BLE) support via nRF52832 (ARM Cortex-M4F).
* **Visual Feedback:** RGB LED changes color based on ambient alcohol levels (Green/Yellow/Red).
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
| **LED** | `XL-1608RGBC` | 0603 SMD RGB LED (Common Cathode). |
| **Connector** | `USB-C 16-Pin` | Power input & Charging. |

---

## 📐 Schematic & PCB Design

- **PCB Size:** 30mm x 20mm
- **Layers:** 2-layer PCB
- **Thickness:** 1.6mm
- **Design Tool:** Altium Designer / KiCAD

Full schematic and Gerber files are available in the [hardware/](hardware/) directory.

---

## 📌 Pinout Map

Pin definitions for firmware development:
```cpp
// RGB LED Connections (Common Cathode - Active LOW)
#define PIN_LED_RED    8   // P0.08
#define PIN_LED_GREEN  9   // P0.09
#define PIN_LED_BLUE   10  // P0.10

// I2C (SGP40 Sensor)
#define PIN_SDA        6   // P0.06 (check your schematic!)
#define PIN_SCL        7   // P0.07 (check your schematic!)

// SWD Programming Interface
#define PIN_SWDCLK     25  // P0.25
#define PIN_SWDIO      26  // P0.26

// Battery Monitoring (Optional)
#define PIN_VBAT_SENSE 3   // P0.03 (ADC Channel)
```

⚠️ **Important:** Verify pin assignments against your actual schematic before programming!

---

## 🚦 LED Status Indicators

The RGB LED provides real-time feedback on alcohol detection levels:

| VOC Index | LED Color | Status | Description |
|-----------|-----------|--------|-------------|
| **0 - 100** | 🟢 Green | Safe | No alcohol detected |
| **100 - 250** | 🟡 Yellow | Moderate | Low alcohol level |
| **250+** | 🔴 Red | High | Dangerous alcohol level |

---

## 🔧 Software Development

### Prerequisites

Before building the firmware, you'll need:

1. **Segger Embedded Studio** (v7.22 or newer) - [Download](https://www.segger.com/downloads/embedded-studio/)
2. **Nordic nRF5 SDK v17.1.0** - [Download](https://www.nordicsemi.com/Products/Development-software/nrf5-sdk/download)
3. **J-Link Debugger** or **ST-Link V2** (for SWD programming)

---

### 📦 Step 1: Install nRF5 SDK

1. Download **nRF5 SDK v17.1.0** from Nordic's website:
```
   https://www.nordicsemi.com/Products/Development-software/nrf5-sdk/download
```

2. Extract the SDK to a convenient location:
   - **Windows:** `C:\nRF5_SDK_17.1.0`
   - **macOS/Linux:** `~/nRF5_SDK_17.1.0`

3. **Important:** Do NOT rename the SDK folder. Keep it as `nRF5_SDK_17.1.0`.

---

### 📥 Step 2: Clone This Repository
```bash
git clone https://github.com/Meczup07/SGP40_Breath_Analyzer.git
cd SGP40_Breath_Analyzer
```

---

### 🔨 Step 3: Build the Firmware

#### Option A: Using Segger Embedded Studio (Recommended)

1. Open Segger Embedded Studio

2. Navigate to the SDK example template:
```
   C:\nRF5_SDK_17.1.0\examples\ble_peripheral\ble_app_template\pca10040\s132\ses\
```

3. Open the project file:
```
   ble_app_template_pca10040_s132.emProject
```

4. In the **Project Explorer** (left panel), locate:
```
   Application → main.c
```

5. Replace the contents of `main.c` with the code from this repository's `src/main.c`

6. **Build the project:**
   - Press `F7` or go to **Build → Build Solution**

7. **Connect your J-Link debugger** to the SWD pins:
```
   J-Link    →    nRF52832
   --------       ---------
   SWDCLK   →    P0.25
   SWDIO    →    P0.26
   VDD      →    3.3V
   GND      →    GND
```

8. **Flash the firmware:**
   - Press `F5` or go to **Debug → Go**

---

#### Option B: Using Command Line (Advanced)
```bash
# Navigate to SDK example directory
cd C:\nRF5_SDK_17.1.0\examples\ble_peripheral\ble_app_template\pca10040\s132\armgcc

# Copy your main.c
cp /path/to/SGP40_Breath_Analyzer/src/main.c ../main.c

# Build
make

# Flash (requires nrfjprog)
make flash
```

---

### 📱 Step 4: Test with Mobile App

1. **Install nRF Connect app:**
   - [Android](https://play.google.com/store/apps/details?id=no.nordicsemi.android.mcp)
   - [iOS](https://apps.apple.com/app/nrf-connect-for-mobile/id1054362403)

2. **Power on your device** (via USB-C or battery)

3. **Open nRF Connect** and tap **SCAN**

4. Look for **"AlkolOlcer"** or **"SGP40_Sensor"** in the device list

5. **Connect** and navigate to the service UUID `0x1815`

6. **Enable notifications** to receive real-time VOC index values

---

## 📊 BLE Service Specification

The device exposes a custom Bluetooth service for reading VOC data:

| Parameter | Value |
|-----------|-------|
| **Service UUID** | `0x1815` |
| **Characteristic UUID** | `0x2A56` |
| **Data Type** | `uint16_t` (2 bytes) |
| **Properties** | Read, Notify |
| **Update Rate** | 1 second |

### Example Data Format
```
VOC Index: 0-500
- 0-100:   Safe (Green LED)
- 100-250: Moderate (Yellow LED)  
- 250+:    High (Red LED)
```

---

## 🔋 Power Consumption

| Mode | Current Draw | Notes |
|------|--------------|-------|
| **Active (BLE + Sensor)** | ~5-8 mA | Normal operation |
| **Sleep Mode** | ~2-5 µA | Deep sleep with RTC |
| **Charging** | 500 mA max | USB-C input |

**Estimated Battery Life:** ~48 hours continuous use with 500mAh Li-Po battery.

---

## 🐛 Troubleshooting

### Device not appearing in BLE scan

**Solution:**
1. Check if SoftDevice is programmed:
```bash
   nrfjprog --program s132_nrf52_7.3.0_softdevice.hex --chiperase
   nrfjprog --reset
```
2. Verify antenna connections
3. Check `sdk_config.h` has `NRF_SDH_BLE_ENABLED = 1`

### SGP40 sensor not responding

**Solution:**
1. Verify I2C connections (SDA, SCL, VDD, GND)
2. Check pull-up resistors (4.7kΩ required on SDA/SCL)
3. Scan I2C bus to confirm SGP40 address `0x59`:
```c
   for (uint8_t addr = 0x01; addr < 0x7F; addr++) {
       ret = nrf_drv_twi_tx(&m_twi, addr, NULL, 0, false);
       if (ret == NRF_SUCCESS) {
           NRF_LOG_INFO("I2C device found at: 0x%02X", addr);
       }
   }
```

### RGB LED not working

**Solution:**
1. Verify pin assignments in code match your PCB
2. Check if LED is common cathode or common anode
3. Confirm current limiting resistors (1kΩ recommended)
4. Test individual colors:
```c
   nrf_gpio_pin_clear(PIN_LED_RED);   // Turn on red
   nrf_gpio_pin_set(PIN_LED_RED);     // Turn off red
```

---

## 📚 Documentation

- [nRF52832 Product Specification](https://infocenter.nordicsemi.com/pdf/nRF52832_PS_v1.4.pdf)
- [SGP40 Datasheet](https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/9_Gas_Sensors/Datasheets/Sensirion_Gas_Sensors_Datasheet_SGP40.pdf)
- [Nordic DevZone](https://devzone.nordicsemi.com/)
- [Segger Wiki](https://wiki.segger.com/Main_Page)

---

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

### Development Workflow

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/amazing-feature`
3. Commit your changes: `git commit -m 'Add amazing feature'`
4. Push to the branch: `git push origin feature/amazing-feature`
5. Open a Pull Request

---

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## ⚠️ Legal Disclaimer

This device is designed for **educational and hobby purposes only**. It is **NOT** certified for:
- Traffic law enforcement
- Medical diagnosis
- Legal alcohol testing

For official alcohol testing, use certified breathalyzer devices approved by relevant authorities.

---

## 🙏 Acknowledgments

- **Nordic Semiconductor** - For the excellent nRF5 SDK and documentation
- **Sensirion** - For the SGP40 sensor and application notes
- **Segger** - For the outstanding Embedded Studio IDE

---

## 📧 Contact

**Project Maintainer:** [Your Name]
- GitHub: [@Meczup07](https://github.com/Meczup07)
- Email: your.email@example.com

---

## 🌟 Support

If you find this project helpful, please consider giving it a ⭐ on GitHub!

---

<div align="center">

**Made with ❤️ for the maker community**

[⬆ Back to Top](#-sgp40-alcsense-ultra-compact-iot-breathalyzer)

</div>
