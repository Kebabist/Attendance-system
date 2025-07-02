# Smart Attendance System

A comprehensive IoT-based attendance and room occupancy management system using ESP32 devices, Flask web server, and LoRa communication for reliable operation in environments with limited network connectivity.

---

## 🔧 System Architecture

The system consists of three main components:

### 1. Flask Web Server (`server.py`)
- Python-based backend with SQLite database
- Real-time web dashboard (Bootstrap UI)
- REST API endpoints for device communication
- Firmware management and OTA update coordination
- Iran timezone support for local time handling

### 2. Master ESP32 Device (`master.c`)
- WiFi gateway to Flask server
- LoRa communication hub for slave devices
- Data aggregation and forwarding
- Web interface for local device management
- Support for both WiFi and LoRa OTA updates

### 3. Slave ESP32 Device (`slave.c`)
- RFID reader for attendance scanning
- Dual ultrasonic sensors for directional people counting
- 16x2 LCD display and 4x4 keypad interface
- LoRa communication with master device
- WiFi capability for direct OTA updates
- Local admin menu for device management

---

## ✨ Key Features

### Backend Dashboard
- **Real-time Monitoring:** Live device status, attendance records, and people counts
- **Student Management:** Add/view registered students with complete profiles
- **Device Management:** Monitor ESP32 devices with status, IP, firmware version, and heartbeat
- **Firmware Management:** Upload and push OTA updates to devices
- **People Counting:** Aggregate occupancy data from all slave devices
- **Event Logging:** Comprehensive system activity logs
- **Auto-refresh:** Dashboard updates every 10-30 seconds

### Smart People Counting
- **Directional Detection:** Two ultrasonic sensors detect entry/exit direction
- **State Machine Logic:** Robust sequence detection prevents false counts
- **Dynamic Calibration:** Automatic baseline adjustment for different environments
- **Filtering:** Multi-sample averaging for noise reduction
- **Real-time Display:** Live count updates on LCD and dashboard

### Communication System
- **Hybrid Connectivity:** LoRa for slave-to-master, WiFi for master-to-server
- **Heartbeat Monitoring:** Regular status updates with health metrics
- **Reliable Messaging:** ACK/NACK protocol for LoRa communication
- **Automatic Reconnection:** WiFi fallback and recovery mechanisms

### OTA Update System
- **Dual Mode Updates:** WiFi OTA for direct updates, LoRa OTA for remote slaves
- **Progress Monitoring:** Real-time update progress with visual feedback
- **Chunked Transfer:** Large firmware files split into manageable LoRa packets
- **Integrity Verification:** CRC32 checksums ensure data integrity
- **Web Interface:** ElegantOTA integration for easy updates

---

## 🛠️ Hardware Requirements

### Server
- Computer with Python 3.7+ and network connectivity

### Master Device
- **ESP32 Dev Module**
- **LoRa Module:** SX1276/SX1278 (433MHz)

### Each Slave Device
- **ESP32 Dev Module**
- **LoRa Module:** SX1276/SX1278 (433MHz)
- **RFID Reader:** MFRC522
- **Display:** 16x2 I2C LCD (address 0x27)
- **Input:** 4x4 Matrix Keypad
- **Sensors:** 2x HC-SR04 Ultrasonic Sensors
- **RFID Cards/Tags**

---

## 📋 Software Requirements

### Backend

```bash
pip install Flask Flask-Cors pytz
```

### ESP32 Development
- **Arduino IDE** with ESP32 board support  
  Board Manager URL:  
  `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`

#### Required Libraries (Install via Library Manager)
- `LoRa` by Sandeep Mistry
- `ElegantOTA` by Ayush Sharma
- `ArduinoJson` by Benoit Blanchon
- `MFRC522` by GithubCommunity
- `LiquidCrystal_I2C` by Frank de Brabander
- `Keypad` by Mark Stanley

---

## 🚀 Installation & Setup

### 1. Backend Server Setup

```bash
# Clone the repository
git clone <repository-url>
cd esp32-attendance-system

# Install dependencies
pip install Flask Flask-Cors pytz

# Run the server
python server.py
```

Access the dashboard at: [http://localhost:5001](http://localhost:5001)

### 2. ESP32 Configuration

#### Arduino IDE Setup
1. Install Arduino IDE
2. Add ESP32 board support via Board Manager
3. Select **"ESP32 Dev Module"** as target board
4. Install required libraries via Library Manager

#### Hardware Wiring

**Master Device - LoRa Connections:**
```
ESP32    LoRa Module
VCC   -> 3.3V
GND   -> GND  
SCK   -> GPIO 5
MISO  -> GPIO 19
MOSI  -> GPIO 23
NSS   -> GPIO 5
RST   -> GPIO 14
DIO0  -> GPIO 2
```

**Slave Device - Complete Wiring:**
```
MFRC522 RFID:        LCD (I2C):         Keypad:
VCC -> 3.3V          VCC -> 5V          Rows: 12,14,27,26
GND -> GND           GND -> GND         Cols: 25,33,32
SCK -> GPIO 18       SDA -> GPIO 21
MISO -> GPIO 19      SCL -> GPIO 22
MOSI -> GPIO 23
SDA -> GPIO 21       Ultrasonic Sensors:
SCL -> GPIO 22       Sensor 1: Trig=4, Echo=5
RST -> GPIO 17       Sensor 2: Trig=2, Echo=16
SS -> GPIO 13
```

#### Code Configuration
1. **Update WiFi credentials** in both `master.c` and `slave.c`
2. **Set server IP address** in `FLASK_SERVER` and `SERVER_URL` constants
3. **Verify pin definitions** match your hardware wiring
4. **Upload firmware** to respective ESP32 devices

---

## 📱 Usage Guide

### Web Dashboard
- **Attendance Tab:** View real-time attendance records
- **Students Tab:** Add/manage student profiles
- **Devices Tab:** Monitor ESP32 status, reboot devices, view heartbeats
- **Firmware Tab:** Upload .bin files and push OTA updates
- **Logs Tab:** System event monitoring and debugging

### Slave Device Operation
- **RFID Attendance:** Tap registered cards on reader
- **Manual Entry:** Press any key → enter 8-digit ID → press `*` to confirm
- **Admin Menu:** Enter admin ID `10111213` → press `*`
  - Navigate with `2`/`8` keys
  - Select with `*`, exit with `#`
- **People Counting:** Automatic - sensors detect entry/exit direction

### Master Device
- **Web Interface:** Access via `http://<master-ip>/` for local control
- **OTA Updates:** Automatic coordination of firmware updates
- **Status Monitoring:** Real-time slave device management

---

## 📊 System Features

### Advanced People Counting
- **Bidirectional Detection:** Distinguishes between entries and exits
- **State Machine:** IDLE → SENSOR1_TRIGGERED → BOTH_TRIGGERED → SENSOR2_TRIGGERED
- **Debouncing:** Prevents false triggering from sensor noise
- **Calibration:** Auto-adjusts to room baseline distances

### Robust Communication
- **LoRa Protocol:** Custom message format with ACK/NACK
- **Heartbeat System:** 30-second LoRa, 60-second WiFi intervals
- **Error Recovery:** Automatic reconnection and retry mechanisms
- **Data Validation:** CRC checksums and duplicate prevention

### Security & Reliability
- **Database Locking:** Thread-safe SQLite operations
- **Duplicate Prevention:** 5-minute cooldown for attendance records
- **Iran Timezone:** Accurate local time handling
- **System Logging:** Comprehensive event tracking

---

## 🖼️ Screenshots

| Dashboard Overview | Device Management | Attendance Records |
|:------------------:|:----------------:|:-----------------:|
| ![Dashboard](https://github.com/user-attachments/assets/8ec19873-63b7-4139-ae15-b68b3fadfada) | ![Devices](https://github.com/user-attachments/assets/dd2ebd9f-2724-4863-80c3-1498970f4836) | ![Attendance](https://github.com/user-attachments/assets/d6bea974-7cff-40ef-949f-6b04c661cd69) |

---

## 🔧 Troubleshooting

### Common Issues
- **WiFi Connection:** Check credentials and network connectivity
- **LoRa Communication:** Verify wiring and antenna connections
- **RFID Reading:** Ensure proper power (3.3V) and SPI connections
- **LCD Display:** Confirm I2C address (default 0x27)
- **People Counting:** Calibrate sensors and check detection thresholds

### Debug Information
- **Serial Monitor:** 115200 baud for ESP32 debug output
- **Dashboard Logs:** Real-time system events and errors
- **Device Status:** Check heartbeat timestamps and firmware versions

---

## 📄 License


---

## 🙏 Acknowledgments

- [ESP32 Arduino Core](https://github.com/espressif/arduino-esp32)
- [ElegantOTA](https://github.com/ayushsharma82/ElegantOTA)
- [LoRa Library](https://github.com/sandeepmistry/arduino-LoRa)
- [ArduinoJson](https://github.com/bblanchon/ArduinoJson)
- [MFRC522](https://github.com/miguelbalboa/rfid)
- [LiquidCrystal_I2C](https://github.com/johnrickman/LiquidCrystal_I2C)
- [Keypad](https://github.com/Chris--A/Keypad)

---
