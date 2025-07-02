# Attendance-system
![image](https://github.com/user-attachments/assets/dd2ebd9f-2724-4863-80c3-1498970f4836)
![image](https://github.com/user-attachments/assets/d6bea974-7cff-40ef-949f-6b04c661cd69)
![image](https://github.com/user-attachments/assets/8ec19873-63b7-4139-ae15-b68b3fadfada)
IoT-Based Smart Attendance System with People Counting
This project is a comprehensive, IoT-based attendance and room occupancy management system. It utilizes a network of ESP32 devices, a central Flask server, and LoRa (Long Range) communication. The use of LoRa is a key feature, enabling the system to operate effectively in locations with poor or no standard network connectivity, ensuring that attendance and sensor data can always be relayed from slave nodes to the central master device.

The system provides real-time attendance tracking and remote device management. A standout feature is its accurate people counting, achieved by using a pair of ultrasonic sensors on each slave device. By detecting the sequence in which the sensors are triggered, the system can determine the direction of movement (in or out of a room) and maintain a precise, real-time occupancy count.

System Architecture
The system consists of three main components:

Flask Web Server (server.py): A Python-based backend that serves as the central hub for data management and control. It provides a web dashboard, REST APIs for the devices, and manages a SQLite database.

Master ESP32 Device (master.c): The primary hardware node that acts as a gateway. It communicates with the backend server via WiFi and with slave devices via LoRa. It aggregates data from slaves and can relay commands, including Over-the-Air (OTA) firmware updates.

Slave ESP32 Device (slave.c): Peripheral nodes responsible for data acquisition. Each slave is equipped with an RFID reader for attendance, ultrasonic sensors for people counting, an LCD display, and a keypad for user interaction.

Key Features
Backend & Dashboard
Real-time Web Dashboard: A single-page application to monitor all system components.

Student Management: Register, view, and manage student data.

Attendance Logging: Tracks attendance records with student details, timestamps, and the device used.

Device Monitoring: View all connected master and slave devices, their online/offline status, IP address, firmware version, and last heartbeat.

People Counting: Aggregates and displays the total number of people detected by all slave devices.

Firmware Management: Upload new firmware binaries (.bin) to the server.

Over-the-Air (OTA) Updates: Push firmware updates from the dashboard to both master and slave devices over WiFi.

System Event Logging: A comprehensive log of all major system events for debugging and auditing.

Master Device
WiFi & LoRa Gateway: Connects to the local WiFi network to communicate with the server and uses LoRa to communicate with slave devices.

Data Aggregation: Forwards attendance data and heartbeats from slaves to the server.

LoRa OTA Capability: Can facilitate firmware updates for slave devices over the LoRa network (future enhancement).

Remote Control: Can be rebooted or managed remotely via the web dashboard.

Slave Device
Multi-modal Input:

RFID Reader: For quick and easy attendance marking with RFID cards/tags.

Keypad: For manual entry of student IDs and accessing an admin menu.

User Feedback: An I2C LCD display shows system status, instructions, and confirmation messages.

People Counting: Uses two ultrasonic sensors to detect the direction of movement. By placing the sensors at an entrance and monitoring which one is triggered first, the system can reliably distinguish between an entry and an exit, allowing it to maintain an accurate room occupancy count.

Dual-Mode Communication: Sends data to the master via LoRa. It can also connect to WiFi directly for firmware updates.

On-device Admin Menu: Provides local administrative functions like clearing data, testing sensors, and viewing status without needing the dashboard.

Hardware & Software Requirements
Hardware
Server: Any computer capable of running Python 3 and Flask.

ESP32 Devices: At least one ESP32 for the master and one for each slave node.

LoRa Modules: SX1276/SX1278 for each ESP32.

For Each Slave Device:

MFRC522 RFID Reader

16x2 or 20x4 I2C LCD Display

4x3 or 4x4 Matrix Keypad

2x HC-SR04 Ultrasonic Sensors

RFID Cards/Tags

Software
Backend:

Python 3

Flask (pip install Flask Flask-Cors)

ESP32 Development:

Arduino IDE

ESP32 Board Support Package

Required Arduino Libraries:

LoRa by Sandeep Mistry

ElegantOTA by Ayush Sharma

ArduinoJson by Benoit Blanchon

MFRC522 by GithubCommunity

LiquidCrystal_I2C by Frank de Brabander

Keypad by Mark Stanley

Setup & Installation
1. Backend Server (server.py)
Ensure Python 3 is installed.

Install the required libraries:

pip install Flask Flask-Cors

Run the server:

python server.py

The server will start on port 5001. Access the dashboard by navigating to http://<your-server-ip>:5001 in a web browser.

2. ESP32 Devices (master.c & slave.c)
Setup Arduino IDE:

Install the Arduino IDE.

Add the ESP32 board manager URL in File > Preferences: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

Install the "esp32" board from Tools > Board > Boards Manager.

Select "ESP32 Dev Module" as your board.

Install Libraries:

Open the Library Manager (Tools > Manage Libraries...) and install all the libraries listed in the "Software Requirements" section.

Configure the Code:

In master.c:

Update WIFI_SSID and WIFI_PASSWORD with your network credentials.

Update SERVER_URL and FLASK_SERVER to point to your backend server's IP address (e.g., http://192.168.1.100:5001).

In slave.c:

Update SLAVE_WIFI_SSID and SLAVE_WIFI_PASSWORD.

Update FLASK_SERVER to your server's IP.

Verify that the GPIO pin definitions match your hardware wiring for the RFID, LCD, Keypad, and LoRa modules.

Upload the Code:

Connect your ESP32 device via USB.

Select the correct COM port in the Arduino IDE.

Upload the master.c sketch to the master device.

Upload the slave.c sketch to each slave device.

How to Use
Dashboard
Attendance: View a real-time list of all attendance records.

Students: Add new students who can then be marked present.

Devices: Monitor the status of all your ESP32 devices. If a device is online, you can use the buttons to get its status or reboot it.

Firmware: Upload a new firmware.bin file. Once uploaded, it will appear in the list. You can then select an online device from the dropdown and click "Push to Device" to start the OTA update.

Logs: See a running log of system activities.

Slave Device
RFID: Simply tap a registered RFID card on the reader to mark attendance.

Keypad:

Press any key to start entering an ID.

Type the 8-digit student ID.

Press * to confirm or # to cancel.

Admin Menu:

Enter the admin ID (10111213) and press *.

Use 2 and 8 to navigate the menu options.

Press * to select an option or # to exit.
