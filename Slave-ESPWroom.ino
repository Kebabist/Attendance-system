/*
 * ESP32 Slave Device - RFID Attendance System with LoRa Communication
 * 
 * SETUP INSTRUCTIONS:
 * ==================
 * 
 * 1. INSTALL ESP32 BOARD SUPPORT IN ARDUINO IDE:
 *    - Open Arduino IDE
 *    - Go to File → Preferences
 *    - In "Additional Board Manager URLs" add:
 *      https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
 *    - Go to Tools → Board → Boards Manager
 *    - Search for "esp32" and install "ESP32 by Espressif Systems"
 *    - After installation, go to Tools → Board → ESP32 Arduino → ESP32 Dev Module
 *    - Select the correct COM port under Tools → Port
 * 
 * 2. REQUIRED LIBRARIES TO INSTALL:
 *    Install these libraries via Arduino IDE Library Manager (Tools → Manage Libraries):
 *    
 *    a) MFRC522 Library (for RFID):
 *       - Search: "MFRC522"
 *       - Install: "MFRC522 by GithubCommunity" (latest version)
 *    
 *    b) LiquidCrystal I2C Library (for LCD Display):
 *       - Search: "LiquidCrystal I2C"
 *       - Install: "LiquidCrystal I2C by Frank de Brabander" (latest version)
 *    
 *    c) Keypad Library (for 4x4 Keypad):
 *       - Search: "Keypad"
 *       - Install: "Keypad by Mark Stanley" (latest version)
 *    
 *    d) LoRa Library:
 *       - Search: "LoRa"
 *       - Install: "LoRa by Sandeep Mistry" (latest version)
 *    
 *    e) ElegantOTA Library:
 *       - Search: "ElegantOTA"
 *       - Install: "ElegantOTA by Ayush Sharma" (latest version)
 * 
 * 3. BUILT-IN LIBRARIES (No installation needed):
 *    - SPI (Arduino Core)
 *    - Wire (Arduino Core) 
 *    - WiFi (ESP32 Core)
 *    - WebServer (ESP32 Core)
 *    - Update (ESP32 Core)
 *    - HTTPClient (ESP32 Core)
 *    - SPIFFS (ESP32 Core)
 * 
 * 4. BOARD CONFIGURATION:
 *    - Board: "ESP32 Dev Module"
 *    - Upload Speed: 921600
 *    - CPU Frequency: 240MHz (WiFi/BT)
 *    - Flash Frequency: 80MHz
 *    - Flash Mode: QIO
 *    - Flash Size: 4MB (32Mb)
 *    - Partition Scheme: Default 4MB with spiffs
 *    - Core Debug Level: None
 *    - PSRAM: Disabled
 * 
 * 5. HARDWARE CONNECTIONS:
 *    
 *    MFRC522 RFID Module:
 *    - VCC → 3.3V
 *    - GND → GND
 *    - SCK → GPIO 18 (SPI SCK)
 *    - MISO → GPIO 19 (SPI MISO)
 *    - MOSI → GPIO 23 (SPI MOSI)
 *    - SDA/SS → GPIO 21 (or as defined in code)
 *    - RST → GPIO 22 (or as defined in code)
 *    
 *    I2C LCD Display (16x2 or 20x4):
 *    - VCC → 5V or 3.3V (depending on LCD module)
 *    - GND → GND
 *    - SDA → GPIO 21 (I2C SDA)
 *    - SCL → GPIO 22 (I2C SCL)
 *    
 *    4x4 Matrix Keypad:
 *    - Connect row and column pins as defined in code
 *    - Typically uses GPIO pins 13, 12, 14, 27 for rows
 *    - And GPIO pins 26, 25, 33, 32 for columns
 *    
 *    LoRa Module (SX1276/SX1278):
 *    - VCC → 3.3V
 *    - GND → GND
 *    - SCK → GPIO 18 (SPI SCK)
 *    - MISO → GPIO 19 (SPI MISO)
 *    - MOSI → GPIO 23 (SPI MOSI)
 *    - NSS/CS → GPIO 5 (or as defined in code)
 *    - RST → GPIO 14 (or as defined in code)
 *    - DIO0 → GPIO 2 (or as defined in code)
 * 
 * 6. BEFORE UPLOADING:
 *    - Update WiFi credentials (SLAVE_WIFI_SSID and SLAVE_WIFI_PASSWORD)
 *    - Update Flask server URL (FLASK_SERVER)
 *    - Verify all hardware connections
 *    - Ensure pin definitions match your wiring
 *    - Check that master device is running and accessible
 * 
 * 7. FEATURES:
 *    - RFID card/tag reading for attendance
 *    - LCD display for user feedback
 *    - Keypad input for manual entry
 *    - LoRa communication with master device
 *    - WiFi OTA firmware update capability
 *    - LoRa-based OTA firmware updates
 *    - People counting sensor integration
 *    - Heartbeat reporting to master
 * 
 * 8. TROUBLESHOOTING:
 *    - If compilation fails, ensure all libraries are installed correctly
 *    - If upload fails, check COM port and board selection
 *    - If RFID not working, verify SPI connections and power
 *    - If LCD not displaying, check I2C address and connections
 *    - If LoRa communication fails, check wiring and antenna
 *    - If WiFi OTA fails, verify network credentials and connectivity
 * 
 * Author: Kebabist
 * Date: June 2025
 * Version: 1.0
 */

#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <LoRa.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include <Update.h>
#include <HTTPClient.h>
#include <SPIFFS.h>

// WiFi credentials for direct WiFi OTA updates if available
const char* SLAVE_WIFI_SSID = "Your Wifi SSID";
const char* SLAVE_WIFI_PASSWORD = "Your Wifi Password";

// Base URL of the Flask server for sending heartbeats when WiFi is connected
const char* FLASK_SERVER = "http://Replace with server IP:5001";

// Web server instance for handling OTA updates
WebServer server(80);

// Variables for WiFi-based Over-The-Air (OTA) updates
bool wifiOtaMode = false; // Flag indicating if the device is in WiFi OTA mode
unsigned long wifiOtaTimeout = 0; // Timestamp to manage WiFi OTA mode timeout
const unsigned long WIFI_OTA_TIMEOUT = 300000; // Timeout duration for WiFi OTA mode (5 minutes)

// Variables for LoRa-based Over-The-Air (OTA) updates
bool loraOtaMode = false; // Flag indicating if the device is in LoRa OTA mode
bool loraOtaInProgress = false; // Flag indicating if a LoRa OTA update is currently active
String loraOtaStatus = "Ready"; // Current status of the LoRa OTA process
size_t loraOtaProgress = 0; // Bytes received so far during LoRa OTA
size_t loraOtaTotal = 0; // Total size of the firmware for LoRa OTA
size_t loraOtaExpectedChunks = 0; // Total number of chunks expected for LoRa OTA
size_t loraOtaReceivedChunks = 0; // Number of chunks received so far during LoRa OTA
uint32_t loraOtaCRC = 0; // CRC32 checksum of the firmware for LoRa OTA
unsigned long loraOtaStartTime = 0; // Timestamp when LoRa OTA started
unsigned long loraOtaLastActivity = 0; // Timestamp of the last activity during LoRa OTA
const unsigned long LORA_OTA_TIMEOUT = 600000; // Timeout duration for the entire LoRa OTA process (10 minutes)

// Buffers and tracking for LoRa OTA firmware chunks
const size_t LORA_CHUNK_SIZE = 200; // Maximum size of a LoRa payload chunk for OTA
uint8_t* loraOtaBuffer = nullptr; // Buffer to store the incoming firmware data during LoRa OTA
bool* loraOtaChunkReceived = nullptr; // Array to track which firmware chunks have been received

// Variables for tracking general OTA update progress (primarily for WiFi OTA)
volatile bool otaInProgress = false; // Flag indicating if any OTA update is active
volatile size_t otaProgress = 0; // Current progress of the OTA update (bytes)
volatile size_t otaTotal = 0; // Total size of the OTA update (bytes)
volatile bool otaActive = false; // Flag indicating if OTA functionality is generally active
String otaStatus = "Ready"; // Current status message for OTA
unsigned long otaStartTime = 0; // Timestamp when the OTA update began
unsigned long lastOTAActivity = 0; // Timestamp of the last OTA-related activity

// Variables for sending heartbeats
unsigned long lastHeartbeat = 0; // Timestamp of the last heartbeat sent via WiFi
const unsigned long HEARTBEAT_INTERVAL = 60000; // Interval for sending WiFi heartbeats (1 minute)

// Device identification string
String deviceId = ""; // Unique identifier for this slave device
bool isWifiAvailable = false; // Flag indicating if WiFi connection is currently active

// Pin definitions for various peripherals
#define RST_PIN 17     // RFID MFRC522 Reset Pin
#define SS_PIN 13      // RFID MFRC522 Slave Select Pin
#define TRIG_PIN_1 4   // Ultrasonic Sensor 1 Trigger Pin
#define ECHO_PIN_1 5   // Ultrasonic Sensor 1 Echo Pin
#define TRIG_PIN_2 2   // Ultrasonic Sensor 2 Trigger Pin
#define ECHO_PIN_2 16  // Ultrasonic Sensor 2 Echo Pin
// LoRa module pin definitions (Note: LORA_SCK, MISO, MOSI are standard SPI pins, often managed by the library)
#define LORA_SCK 5     // LoRa SCK (Serial Clock) - Standard SPI
#define LORA_MISO 19   // LoRa MISO (Master In Slave Out) - Standard SPI
#define LORA_MOSI 23   // LoRa MOSI (Master Out Slave In) - Standard SPI
#define LORA_RESET -1  // LoRa Reset Pin (-1 if not used or tied to ESP32 RST)
#define LORA_SS 17     // LoRa Slave Select Pin
#define LORA_DIO0 15   // LoRa DIO0 Pin (Interrupt for packet reception)

// Keypad configuration
const byte ROW_NUM = 4; // Number of rows on the keypad
const byte COL_NUM = 3; // Number of columns on the keypad
char keys[ROW_NUM][COL_NUM] = { // Keymap defining the characters for each key
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
byte pin_rows[ROW_NUM] = {12, 14, 27, 26}; // ESP32 pins connected to keypad rows
byte pin_cols[COL_NUM] = {25, 33, 32};    // ESP32 pins connected to keypad columns
Keypad keypad = Keypad(makeKeymap(keys), pin_rows, pin_cols, ROW_NUM, COL_NUM); // Keypad library instance

// LCD I2C configuration
LiquidCrystal_I2C lcd(0x27, 16, 2); // LCD instance (I2C address 0x27, 16 columns, 2 rows)

// RFID MFRC522 configuration
MFRC522 mfrc522(SS_PIN, RST_PIN); // RFID library instance

// System operation variables
String studentID = ""; // Stores student ID entered via keypad
String students[70];   // Array to store scanned/entered student UIDs (max 70)
int studentCount = 0;  // Current number of students in the list
bool isEnteringID = false; // Flag indicating if student ID entry via keypad is active

// People counter configuration and state variables
#define MIN_DISTANCE 10     // Minimum distance for sensor detection (cm)
#define MAX_DISTANCE 150    // Maximum reliable distance for sensor (cm)
#define DEBOUNCE_DELAY 400  // Debounce delay for sensor detections (ms)

int currentPeople = 0; // Current count of people
unsigned long lastDetectionTime1 = 0; // Timestamp of last detection by sensor 1
unsigned long lastDetectionTime2 = 0; // Timestamp of last detection by sensor 2
unsigned long lastUpdateTime = 0;     // Timestamp of last display update

// LoRa communication variables
unsigned long lastSendTime = 0; // Timestamp of the last LoRa UID transmission
const long SEND_INTERVAL = 2000; // Minimum interval between sending UIDs via LoRa (ms)
String lastSentUID = ""; // Stores the last UID sent via LoRa to avoid duplicates
unsigned long lastLoRaHeartbeat = 0; // Timestamp of the last LoRa heartbeat sent
const unsigned long LORA_HEARTBEAT_INTERVAL = 30000; // Interval for sending LoRa heartbeats (30 seconds)

// states for the entry/exit detection state machine
enum DetectionState {
  DETECTION_IDLE,      // No detection in progress
  ENTRY_S1_ACTIVE,     // Entry sequence: First sensor triggered  
  ENTRY_TRANSITION,    // Entry sequence: Moving between sensors
  EXIT_S2_ACTIVE,      // Exit sequence: First sensor triggered
  EXIT_TRANSITION      // Exit sequence: Moving between sensors
};

DetectionState currentState = DETECTION_IDLE;
unsigned long stateStartTime = 0;
const unsigned long SEQUENCE_TIMEOUT = 1000;  // Max time for complete entry/exit sequence (2s)
const unsigned long MIN_TRANSITION_TIME = 50; // Min time between sensors for valid detection

// Number of readings to average for filtering
const int FILTER_SAMPLES = 3;
float sensor1History[FILTER_SAMPLES] = {0};
float sensor2History[FILTER_SAMPLES] = {0};
int historyIndex = 0;

// Variables for dynamic threshold adjustment
float baseline1 = -1;  // Baseline distance for sensor 1 (measured when no one is present)
float baseline2 = -1;  // Baseline distance for sensor 2
const float DETECTION_PERCENTAGE = 0.8; // Detect when distance < 70% of baseline

// Function prototypes
void tryWiFiConnection();
bool initializeLoRa();
void displaySystemStatus();
void sendLoRaHeartbeat();
void sendLoRaMessage(String message);
void receiveLoRaMessages();
void processLoRaMessage(String message);
void handleLoRaOTAStart(String message);
void handleLoRaOTAChunk(String message);
void handleLoRaOTAEnd(String message);
void handleLoRaOTAAbort();
void resetLoRaOTA();
void handleLoRaOTAMode();
uint32_t calculateCRC32(uint8_t* data, size_t length);
uint8_t* base64_decode(const char* input, size_t input_length, size_t* output_length);
void setupWiFiOTA();
void handleWiFiOTAMode();
void sendHeartbeat();
void handleLoRaACK(String message);
void handleRegularLoRaMessage(String message);
void handlePeopleCounting();
float getFilteredDistance(int trigPin, int echoPin);
float getDistance(int trigPin, int echoPin);
bool isSensorReadingValid(float currentDistance, float prevDistance);
void calibrateSensors();
bool isSensor1Active(float distance);
bool isSensor2Active(float distance);
void checkTimeBasedReset();
void checkSensorHealth();
void updateDisplay();
void handleRFID();
void handleKeypad();
void sendUIDViaLoRa(String uid);
void adminMenu();
void executeAdminCommand(int option);
void resetOTAProgress();

// Setup function: initializes hardware and software components
void setup() {
  Serial.begin(115200); // Initialize serial communication
  while (!Serial); // Wait for serial port to connect

  Serial.println("\n=======================================");
  Serial.println("🚀 Enhanced Slave Device with LoRa OTA");
  Serial.println("=======================================");

  // Initialize SPIFFS (SPI Flash File System)
  if (!SPIFFS.begin(true)) { // true to format SPIFFS if mount fails
    Serial.println("❌ SPIFFS Mount Failed");
  } else {
    Serial.println("✅ SPIFFS initialized for LoRa OTA");
  }

  tryWiFiConnection(); // Attempt to connect to WiFi; sets deviceId based on success

  Serial.println("📱 Device ID: " + deviceId);

  // Initialize RFID reader
  SPI.begin(); // Initialize SPI bus
  mfrc522.PCD_Init(); // Initialize MFRC522 module
  Serial.println("✅ RFID Initialized");

  initializeLoRa(); // Initialize LoRa module

  calibrateSensors();

  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);

  // Initialize LCD display
  lcd.init(); // Initialize LCD
  lcd.backlight(); // Turn on LCD backlight
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  delay(1000);

  displaySystemStatus(); // Display initial system status on LCD
  Serial.println("✅ System Initialized");

  // Setup WiFi OTA functionality only if WiFi is connected
  if (isWifiAvailable) {
    setupWiFiOTA(); // Configure web server for WiFi OTA
    sendHeartbeat(); // Send initial WiFi heartbeat
  }

  sendLoRaHeartbeat(); // Send initial LoRa heartbeat
}

// Main loop: continuously executes the device's core logic
void loop() {
  // Handle WiFi-related tasks if WiFi is available
  if (isWifiAvailable) {
    if (WiFi.status() != WL_CONNECTED) { // Check if WiFi is still connected
      Serial.println("⚠️ WiFi disconnected, trying to reconnect...");
      tryWiFiConnection(); // Attempt to reconnect
    } else {
      server.handleClient(); // Handle incoming HTTP requests for OTA server
      ElegantOTA.loop(); // Process ElegantOTA tasks

      // Check for WiFi OTA timeout
      if (otaInProgress && (millis() - lastOTAActivity > 30000)) {
        Serial.println("⚠️ Slave OTA timeout detected, resetting progress tracking");
        resetOTAProgress(); // Reset WiFi OTA progress variables
      }

      // Send periodic WiFi heartbeat
      if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
        sendHeartbeat();
        lastHeartbeat = millis();
      }
    }
  }

  // Check for LoRa OTA timeout
  if (loraOtaInProgress && (millis() - loraOtaLastActivity > LORA_OTA_TIMEOUT)) {
    Serial.println("⏱️ LoRa OTA timeout detected");
    resetLoRaOTA(); // Reset LoRa OTA state
  }

  // If in LoRa OTA mode, prioritize LoRa OTA handling
  if (loraOtaMode) {
    handleLoRaOTAMode(); // Dedicated handler for LoRa OTA operations
    return; // Skip normal operations during LoRa OTA
  }

  // If in WiFi OTA mode, prioritize WiFi OTA handling
  if (wifiOtaMode) {
    handleWiFiOTAMode(); // Dedicated handler for WiFi OTA operations
    return; // Skip normal operations during WiFi OTA
  }

  // Normal device operations
  handlePeopleCounting(); // Manage people counting using ultrasonic sensors

  checkTimeBasedReset();

  // Update LCD display periodically
  if(millis() - lastUpdateTime > 500) {
    lastUpdateTime = millis();
    updateDisplay();
  }

  handleRFID(); // Check for RFID card scans
  handleKeypad(); // Check for keypad input
  receiveLoRaMessages(); // Check for incoming LoRa messages

  // Send periodic LoRa heartbeat
  if (millis() - lastLoRaHeartbeat > LORA_HEARTBEAT_INTERVAL) {
    sendLoRaHeartbeat();
    lastLoRaHeartbeat = millis();
  }

  delay(100); // Short delay to allow system tasks and prevent busy-looping
}

// Attempts to connect to the configured WiFi network
void tryWiFiConnection() {
  Serial.print("🔍 Attempting WiFi connection...");
  WiFi.begin(SLAVE_WIFI_SSID, SLAVE_WIFI_PASSWORD); // Initiate WiFi connection

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) { // Retry for up to 20 seconds
    delay(1000);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) { // If connection is successful
    isWifiAvailable = true;
    Serial.println("\n✅ WiFi connected!");
    Serial.println("📡 IP Address: " + WiFi.localIP().toString());

    // Generate device ID based on IP address for easier identification
    String ipStr = WiFi.localIP().toString();
    ipStr.replace(".", ""); // Remove dots from IP for a cleaner ID
    deviceId = "SLAVE-" + ipStr;
    Serial.println("📱 Updated Device ID: " + deviceId);
  } else { // If connection fails
    isWifiAvailable = false;
    Serial.println("\n⚠️ WiFi not available - LoRa only mode");
    WiFi.disconnect(true); // Disconnect and turn off WiFi radio
    // Generate device ID based on MAC address for LoRa-only operation
    deviceId = "SLAVE-" + String((uint32_t)ESP.getEfuseMac(), HEX); // Use part of MAC address
  }
}

// Initializes the LoRa module with retry mechanism
bool initializeLoRa() {
  LoRa.setPins(LORA_SS, LORA_RESET, LORA_DIO0); // Configure LoRa pins

  for (int attempt = 1; attempt <= 3; attempt++) { // Try initialization up to 3 times
    if (LoRa.begin(433E6)) { // Initialize LoRa at 433 MHz
      // Set LoRa parameters for communication
      LoRa.setSyncWord(0xF3);         // Custom sync word to differentiate network
      LoRa.setSpreadingFactor(12);    // Spreading factor (higher for longer range)
      LoRa.setSignalBandwidth(125E3); // Signal bandwidth (125 kHz)
      LoRa.setCodingRate4(5);         // Coding rate (4/5)
      Serial.println("✅ LoRa initialized successfully!");
      return true; // Initialization successful
    }
    Serial.printf("⚠️ LoRa initialization attempt %d failed\n", attempt);
    delay(1000 * attempt); // Increasing delay between attempts
  }

  Serial.println("❌ LoRa initialization failed after multiple attempts");
  return false; // Initialization failed
}

// Displays the current system status (WiFi/LoRa mode and Device ID) on the LCD
void displaySystemStatus() {
  lcd.clear();
  lcd.setCursor(0, 0);
  if (isWifiAvailable) {
    lcd.print("WiFi+LoRa Ready");
  } else {
    lcd.print("LoRa Only Mode");
  }
  lcd.setCursor(0, 1);
  lcd.print(deviceId.substring(0, 16)); // Display first 16 chars of device ID
  delay(2000);
}

// ============================================
// LoRa OTA Implementation
// ============================================

// Sends a heartbeat message via LoRa to the master device
void sendLoRaHeartbeat() {
  // Construct heartbeat message: HEARTBEAT:<DeviceID>:<PeopleCount>:<FreeHeap>
  String heartbeat = "HEARTBEAT:" + deviceId + ":" + String(currentPeople) + ":" + String(ESP.getFreeHeap());
  sendLoRaMessage(heartbeat); // Send the message
  Serial.println("💓 LoRa heartbeat sent to master");
}

// Sends a string message via the LoRa module
void sendLoRaMessage(String message) {
  LoRa.beginPacket(); // Start LoRa packet
  LoRa.print(message);  // Add message payload
  LoRa.endPacket();   // Send the packet
}

// Checks for and processes incoming messages from the LoRa module
void receiveLoRaMessages() {
  int packetSize = LoRa.parsePacket(); // Check if a LoRa packet has been received
  if (packetSize) { // If a packet is available
    String message = "";
    while (LoRa.available()) { // Read all bytes of the packet
      message += (char)LoRa.read();
    }
    Serial.println("📨 LoRa received: " + message);
    processLoRaMessage(message); // Process the received message
  }
}

// Processes different types of messages received via LoRa
void processLoRaMessage(String message) {
  if (message.startsWith("OTA_START:")) { // Master initiated LoRa OTA
    handleLoRaOTAStart(message);
  } else if (message.startsWith("OTA_CHUNK:")) { // Master sent a firmware chunk
    handleLoRaOTAChunk(message);
  } else if (message.startsWith("OTA_END:")) { // Master signaled end of firmware transmission
    handleLoRaOTAEnd(message);
  } else if (message.startsWith("OTA_ABORT")) { // Master aborted LoRa OTA
    handleLoRaOTAAbort();
  } else if (message.startsWith("ACK:")) { // Master acknowledged a UID
    handleLoRaACK(message);
  } else { // Other types of messages (e.g., commands, simple UIDs if master sends them)
    handleRegularLoRaMessage(message);
  }
}

// Handles the "OTA_START" command from the master to initiate LoRa OTA
void handleLoRaOTAStart(String message) {
  // Expected format: OTA_START:<total_size>:<total_chunks>:<crc32_hex>
  int firstColon = message.indexOf(':', 10); // Skip "OTA_START:"
  int secondColon = message.indexOf(':', firstColon + 1);
  int thirdColon = message.indexOf(':', secondColon + 1);

  if (firstColon == -1 || secondColon == -1 || thirdColon == -1) {
    Serial.println("❌ Invalid OTA_START format");
    sendLoRaMessage("OTA_NACK:INVALID_FORMAT"); // Send Negative Acknowledgment
    return;
  }

  // Parse OTA parameters from the message
  loraOtaTotal = message.substring(10, firstColon).toInt();
  loraOtaExpectedChunks = message.substring(firstColon + 1, secondColon).toInt();
  loraOtaCRC = strtoul(message.substring(secondColon + 1, thirdColon).c_str(), NULL, 16); // Parse CRC from hex

  Serial.println("=======================================");
  Serial.println("🔄 LoRa OTA Update Started by Master");
  Serial.println("=======================================");
  Serial.printf("📦 Total size: %u bytes\n", loraOtaTotal);
  Serial.printf("📊 Expected chunks: %u\n", loraOtaExpectedChunks);
  Serial.printf("🔐 CRC32: 0x%08X\n", loraOtaCRC);

  // Free any existing buffers and allocate new ones for the incoming firmware
  if (loraOtaBuffer != nullptr) free(loraOtaBuffer);
  if (loraOtaChunkReceived != nullptr) free(loraOtaChunkReceived);

  loraOtaBuffer = (uint8_t*)malloc(loraOtaTotal); // Buffer for the entire firmware
  loraOtaChunkReceived = (bool*)calloc(loraOtaExpectedChunks, sizeof(bool)); // Array to track received chunks

  if (loraOtaBuffer == nullptr || loraOtaChunkReceived == nullptr) {
    Serial.println("❌ Failed to allocate memory for LoRa OTA");
    sendLoRaMessage("OTA_NACK:MEMORY_ERROR");
    resetLoRaOTA(); // Clean up and reset OTA state
    return;
  }

  // Initialize LoRa OTA state variables
  loraOtaMode = true; // Enter LoRa OTA mode
  loraOtaInProgress = true;
  loraOtaProgress = 0;
  loraOtaReceivedChunks = 0;
  loraOtaStartTime = millis();
  loraOtaLastActivity = millis();
  loraOtaStatus = "Receiving firmware chunks...";

  // Update LCD display
  lcd.clear();
  lcd.print("LoRa OTA Started");
  lcd.setCursor(0, 1);
  lcd.print("Receiving...");

  sendLoRaMessage("OTA_ACK:READY"); // Acknowledge readiness to master
  Serial.println("✅ LoRa OTA initialized successfully, waiting for chunks.");
}

// Handles an incoming firmware chunk during LoRa OTA
void handleLoRaOTAChunk(String message) {
  if (!loraOtaInProgress) {
    Serial.println("⚠️ Received OTA chunk but not in LoRa OTA mode");
    return;
  }

  // Expected format: OTA_CHUNK:<chunk_id>:<base64_data>
  int firstColon = message.indexOf(':', 10); // Skip "OTA_CHUNK:"
  int secondColon = message.indexOf(':', firstColon + 1);

  if (firstColon == -1 || secondColon == -1) {
    Serial.println("❌ Invalid OTA_CHUNK format");
    sendLoRaMessage("OTA_NACK:INVALID_CHUNK_FORMAT");
    return;
  }

  size_t chunkId = message.substring(10, firstColon).toInt(); // Parse chunk ID
  String chunkDataB64 = message.substring(secondColon + 1); // Get Base64 encoded chunk data

  loraOtaLastActivity = millis(); // Update activity timestamp

  if (chunkId >= loraOtaExpectedChunks) { // Validate chunk ID
    Serial.printf("❌ Invalid chunk ID: %u (expected < %u)\n", chunkId, loraOtaExpectedChunks);
    sendLoRaMessage("OTA_NACK:INVALID_CHUNK_ID:" + String(chunkId));
    return;
  }

  if (loraOtaChunkReceived[chunkId]) { // If chunk already received, re-ACK
    Serial.printf("ℹ️ Chunk %u already received, sending ACK\n", chunkId);
    sendLoRaMessage("OTA_ACK:" + String(chunkId));
    return;
  }

  // Decode Base64 data
  size_t decodedLength = 0;
  uint8_t* decodedData = base64_decode(chunkDataB64.c_str(), chunkDataB64.length(), &decodedLength);

  if (decodedData == nullptr || decodedLength == 0) {
    Serial.printf("❌ Failed to decode chunk %u\n", chunkId);
    sendLoRaMessage("OTA_NACK:DECODE_ERROR:" + String(chunkId));
    if(decodedData) free(decodedData);
    return;
  }

  // Calculate offset and size for storing the chunk
  size_t chunkOffset = chunkId * LORA_CHUNK_SIZE;
  // Ensure actualChunkSize does not cause buffer overflow, especially for the last chunk
  size_t actualChunkSize = (chunkOffset + decodedLength > loraOtaTotal) ? (loraOtaTotal - chunkOffset) : decodedLength;


  if (chunkOffset + actualChunkSize > loraOtaTotal) { // Boundary check
    Serial.printf("❌ Chunk %u (offset %u, size %u) would exceed total buffer size %u\n", chunkId, chunkOffset, actualChunkSize, loraOtaTotal);
    free(decodedData);
    sendLoRaMessage("OTA_NACK:BUFFER_OVERFLOW:" + String(chunkId));
    return;
  }

  memcpy(loraOtaBuffer + chunkOffset, decodedData, actualChunkSize); // Copy decoded data to buffer
  free(decodedData); // Free temporary decoded data buffer

  loraOtaChunkReceived[chunkId] = true; // Mark chunk as received
  loraOtaReceivedChunks++;
  loraOtaProgress += actualChunkSize; // Update overall progress

  float progressPercent = (float)loraOtaReceivedChunks / loraOtaExpectedChunks * 100.0;
  loraOtaStatus = "Received " + String(loraOtaReceivedChunks) + "/" + String(loraOtaExpectedChunks) + " chunks";

  Serial.printf("📦 Chunk %u received (%u bytes) - Progress: %.1f%%\n",
                chunkId, actualChunkSize, progressPercent);

  // Update LCD with progress
  lcd.clear();
  lcd.print("LoRa OTA");
  lcd.setCursor(0, 1);
  lcd.print(String((int)progressPercent) + "% " + String(loraOtaReceivedChunks) + "/" + String(loraOtaExpectedChunks));

  sendLoRaMessage("OTA_ACK:" + String(chunkId)); // Acknowledge receipt of the chunk

  // If all chunks are received, notify master
  if (loraOtaReceivedChunks >= loraOtaExpectedChunks) {
    Serial.println("✅ All chunks received, ready for verification by master.");
    sendLoRaMessage("OTA_ACK:ALL_CHUNKS_RECEIVED");
  }
}

// Handles the "OTA_END" command from the master, signaling firmware transmission completion
void handleLoRaOTAEnd(String message) {
  if (!loraOtaInProgress) {
    Serial.println("⚠️ Received OTA_END but not in LoRa OTA mode");
    return;
  }
  loraOtaLastActivity = millis();
  Serial.println("🔍 Master signaled OTA_END. Verifying received firmware...");

  // Verify all chunks have been received
  for (size_t i = 0; i < loraOtaExpectedChunks; i++) {
    if (!loraOtaChunkReceived[i]) {
      Serial.printf("❌ Missing chunk %u during final verification\n", i);
      sendLoRaMessage("OTA_NACK:MISSING_CHUNKS"); // Notify master about missing chunks
      // Optionally, could request specific missing chunks here if protocol supported it
      return;
    }
  }

  // Verify CRC32 checksum of the assembled firmware
  uint32_t calculatedCRC = calculateCRC32(loraOtaBuffer, loraOtaTotal);
  if (calculatedCRC != loraOtaCRC) {
    Serial.printf("❌ CRC mismatch! Expected: 0x%08X, Calculated: 0x%08X\n", loraOtaCRC, calculatedCRC);
    sendLoRaMessage("OTA_NACK:CRC_MISMATCH");
    resetLoRaOTA(); // Abort OTA on CRC failure
    return;
  }

  Serial.println("✅ Firmware verification successful! Proceeding with flash update.");

  lcd.clear();
  lcd.print("Verifying...");
  lcd.setCursor(0, 1);
  lcd.print("Please wait...");

  // Begin the firmware update process using the Update library
  if (!Update.begin(loraOtaTotal)) { // Pass the total firmware size
    Serial.println("❌ Update.begin() failed!");
    Update.printError(Serial);
    sendLoRaMessage("OTA_NACK:UPDATE_BEGIN_FAILED");
    resetLoRaOTA();
    return;
  }

  // Write the firmware data to flash
  size_t written = Update.write(loraOtaBuffer, loraOtaTotal);
  if (written != loraOtaTotal) {
    Serial.printf("❌ Update.write() failed! Written: %u, Expected: %u\n", written, loraOtaTotal);
    Update.printError(Serial);
    sendLoRaMessage("OTA_NACK:WRITE_FAILED");
    resetLoRaOTA();
    return;
  }

  // Finalize the update
  if (!Update.end(true)) { // true to commit the update
    Serial.println("❌ Update.end() failed!");
    Update.printError(Serial);
    sendLoRaMessage("OTA_NACK:UPDATE_END_FAILED");
    resetLoRaOTA();
    return;
  }

  Serial.println("✅ LoRa OTA Update successful! Firmware flashed.");
  sendLoRaMessage("OTA_ACK:SUCCESS"); // Notify master of successful update

  lcd.clear();
  lcd.print("OTA Success!");
  lcd.setCursor(0, 1);
  lcd.print("Rebooting...");

  delay(2000); // Allow time for message to send and display to be seen
  resetLoRaOTA(); // Clean up LoRa OTA state
  ESP.restart(); // Reboot the device with the new firmware
}

// Handles the "OTA_ABORT" command from the master
void handleLoRaOTAAbort() {
  Serial.println("⚠️ LoRa OTA aborted by master command.");
  resetLoRaOTA(); // Reset LoRa OTA state and clean up

  lcd.clear();
  lcd.print("OTA Aborted");
  delay(2000);
  // Return to normal operation after displaying message
  displaySystemStatus(); // Or a default ready screen
}

// Resets all LoRa OTA state variables and frees allocated memory
void resetLoRaOTA() {
  loraOtaMode = false;
  loraOtaInProgress = false;
  loraOtaStatus = "Ready";
  loraOtaProgress = 0;
  loraOtaTotal = 0;
  loraOtaExpectedChunks = 0;
  loraOtaReceivedChunks = 0;
  loraOtaCRC = 0;

  // Free dynamically allocated buffers
  if (loraOtaBuffer != nullptr) {
    free(loraOtaBuffer);
    loraOtaBuffer = nullptr;
  }
  if (loraOtaChunkReceived != nullptr) {
    free(loraOtaChunkReceived);
    loraOtaChunkReceived = nullptr;
  }
  Serial.println("🔄 LoRa OTA state reset and memory freed.");
}

// Manages operations while the device is in LoRa OTA mode
void handleLoRaOTAMode() {
  receiveLoRaMessages(); // Continuously check for LoRa messages (chunks, commands)

  // Update LCD display with LoRa OTA progress periodically
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate > 1000) { // Update display every second
    if (loraOtaInProgress) {
      float progressPercent = loraOtaExpectedChunks > 0 ?
        (float)loraOtaReceivedChunks / loraOtaExpectedChunks * 100.0 : 0.0;

      lcd.clear();
      lcd.print("LoRa OTA Active");
      lcd.setCursor(0, 1);
      lcd.print(String((int)progressPercent) + "% " + String(loraOtaReceivedChunks) + "/" + String(loraOtaExpectedChunks));
    } else if (loraOtaMode) { // In LoRa OTA mode but not actively receiving (e.g., waiting for start)
        lcd.clear();
        lcd.print("LoRa OTA Mode");
        lcd.setCursor(0, 1);
        lcd.print("Waiting...");
    }
    lastDisplayUpdate = millis();
  }
  delay(50); // Short delay to prevent busy loop, allow processing
}

// ============================================
// Utility Functions
// ============================================

// Calculates CRC32 checksum for a block of data
uint32_t calculateCRC32(uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFF; // Initial CRC value
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i]; // XOR byte into CRC
    for (int j = 0; j < 8; j++) { // Process each bit
      if (crc & 1) { // If LSB is 1
        crc = (crc >> 1) ^ 0xEDB88320; // Shift and XOR with polynomial
      } else {
        crc >>= 1; // Just shift
      }
    }
  }
  return ~crc; // Return final CRC (inverted)
}

// Decodes a Base64 encoded string into a binary data buffer
uint8_t* base64_decode(const char* input, size_t input_length, size_t* output_length) {
  const char base64_chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  if (input_length % 4 != 0) { // Base64 string length must be a multiple of 4
      *output_length = 0;
      return nullptr;
  }

  // Calculate the length of the decoded output
  *output_length = input_length / 4 * 3;
  if (input[input_length - 1] == '=') (*output_length)--; // Adjust for padding characters
  if (input[input_length - 2] == '=') (*output_length)--;

  uint8_t* output = (uint8_t*)malloc(*output_length); // Allocate memory for the output
  if (output == nullptr) {
      *output_length = 0;
      return nullptr; // Memory allocation failed
  }

  // Perform the decoding
  for (size_t i = 0, j = 0; i < input_length;) {
    // Get the values of 4 Base64 characters
    uint32_t sextet_a = input[i] == '=' ? 0 & i++ : strchr(base64_chars, input[i++]) - base64_chars;
    uint32_t sextet_b = input[i] == '=' ? 0 & i++ : strchr(base64_chars, input[i++]) - base64_chars;
    uint32_t sextet_c = input[i] == '=' ? 0 & i++ : strchr(base64_chars, input[i++]) - base64_chars;
    uint32_t sextet_d = input[i] == '=' ? 0 & i++ : strchr(base64_chars, input[i++]) - base64_chars;

    uint32_t triple = (sextet_a << 18) + (sextet_b << 12) + (sextet_c << 6) + sextet_d; // Combine into 24 bits

    // Extract 3 bytes from the 24 bits
    if (j < *output_length) output[j++] = (triple >> 16) & 0xFF;
    if (j < *output_length) output[j++] = (triple >> 8) & 0xFF;
    if (j < *output_length) output[j++] = triple & 0xFF;
  }
  return output;
}

// ============================================
// WiFi OTA Functions (when WiFi available)
// ============================================

// Sets up the web server for WiFi-based OTA updates
void setupWiFiOTA() {
  Serial.println("=======================================");
  Serial.println("🔧 Setting up Enhanced Slave OTA Server (WiFi)");
  Serial.println("=======================================");

  server.enableCORS(true); // Enable Cross-Origin Resource Sharing for web interface

  // Route for the main HTML page of the OTA web interface
  server.on("/", []() {
    String html = "<!DOCTYPE html><html><head><title>Enhanced Slave Device Control</title>";
    html += "<meta charset='UTF-8'>";
    html += "<meta http-equiv='refresh' content='3'>"; // Auto-refresh page every 3 seconds
    html += "<style>body{font-family:Arial;margin:40px;} button{padding:10px 20px;margin:10px;font-size:16px;}";
    html += ".progress{background:#f0f0f0;border-radius:5px;padding:3px;margin:10px 0;} ";
    html += ".progress-bar{background:#4CAF50;height:25px;border-radius:3px;transition:width 0.3s;text-align:center;line-height:25px;color:white;font-weight:bold;}";
    html += ".status{padding:10px;margin:10px 0;border-radius:5px;} .uploading{background:#fff3cd;border:1px solid #ffeaa7;} .ready{background:#d4edda;border:1px solid #c3e6cb;}";
    html += ".debug{font-size:10px;color:#666;font-family:monospace;}";
    html += "</style></head>";
    html += "<body><h1>Enhanced Slave Device Control Panel</h1>";

    // Display device status information
    html += "<div class='status " + String(otaInProgress ? "uploading" : "ready") + "'>";
    html += "<p><strong>Slave Status:</strong> " + otaStatus + "</p>";
    html += "<p><strong>WiFi:</strong> Connected</p>";
    html += "<p><strong>IP Address:</strong> " + WiFi.localIP().toString() + "</p>";
    html += "<p><strong>Device ID:</strong> " + deviceId + "</p>";
    html += "<p><strong>People Count:</strong> " + String(currentPeople) + "</p>";
    html += "<p><strong>Free Heap:</strong> " + String(ESP.getFreeHeap()) + " bytes</p>";

    // Display debug information about OTA states
    html += "<div class='debug'>";
    html += "WiFi OTA: InProgress=" + String(otaInProgress ? "true" : "false");
    html += ", Progress=" + String(otaProgress) + "/" + String(otaTotal) + "<br>";
    html += "LoRa OTA: Mode=" + String(loraOtaMode ? "true" : "false");
    html += ", Status=" + loraOtaStatus;
    html += "</div>";
    html += "</div>";

    // Display progress bar if WiFi OTA is in progress
    if (otaInProgress) {
      int percent = otaTotal > 0 ? (otaProgress * 100) / otaTotal : 0;
      html += "<div class='progress'>";
      html += "<div class='progress-bar' style='width:" + String(percent) + "%'>";
      html += String(percent) + "%";
      html += "</div></div>";
      html += "<p>Status: " + otaStatus + "</p>";
    }

    // Display control buttons if no OTA is active
    if (!otaInProgress) {
      html += "<h2>Device Controls</h2>";
      html += "<p><a href='/update' style='background:#007bff;color:white;padding:10px 20px;text-decoration:none;border-radius:5px;'>Firmware Update (ElegantOTA)</a></p>";
      html += "<p><a href='/reboot' style='background:#dc3545;color:white;padding:10px 20px;text-decoration:none;border-radius:5px;'>Reboot Device</a></p>";
    }

    html += "<hr>";
    html += "<h2>System Status</h2>";
    html += "<p>Connection: WiFi + LoRa</p>";
    html += "</body></html>";

    server.sendHeader("Cache-Control", "no-cache"); // Prevent browser caching of this page
    server.send(200, "text/html", html);
  });

  // Route to provide device status as JSON
  server.on("/status", []() {
    String json = "{";
    json += "\"deviceId\":\"" + deviceId + "\",";
    json += "\"deviceType\":\"slave\",";
    json += "\"peopleCount\":" + String(currentPeople) + ",";
    json += "\"connectionType\":\"wifi_lora\",";
    json += "\"uptime\":" + String(millis()) + ",";
    json += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";
    json += "\"otaInProgress\":" + String(otaInProgress ? "true" : "false") + ",";
    json += "\"otaActive\":" + String(otaActive ? "true" : "false") + ",";
    json += "\"otaProgress\":" + String(otaProgress) + ",";
    json += "\"otaTotal\":" + String(otaTotal) + ",";
    json += "\"otaPercent\":" + String(otaTotal > 0 ? (otaProgress * 100) / otaTotal : 0) + ",";
    json += "\"otaStatus\":\"" + otaStatus + "\",";
    json += "\"loraOtaMode\":" + String(loraOtaMode ? "true" : "false") + ",";
    json += "\"loraOtaStatus\":\"" + loraOtaStatus + "\"";
    json += "}";

    server.sendHeader("Access-Control-Allow-Origin", "*"); // Allow cross-origin requests
    server.sendHeader("Cache-Control", "no-cache");
    server.send(200, "application/json", json);
  });

  // Route to provide OTA progress as JSON
  server.on("/ota_progress", []() {
    String json = "{";
    json += "\"inProgress\":" + String(otaInProgress ? "true" : "false") + ",";
    int percent = 0;
    if (otaInProgress && otaTotal > 0) {
      percent = (int)((otaProgress * 100) / otaTotal);
    }
    json += "\"percent\":" + String(percent) + ",";
    json += "\"progress\":" + String((size_t)otaProgress) + ",";
    json += "\"total\":" + String((size_t)otaTotal) + ",";
    json += "\"status\":\"" + otaStatus + "\"";
    json += "}";

    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Cache-Control", "no-cache");
    server.send(200, "application/json", json);
  });

  // Route to handle reboot command via HTTP GET
  server.on("/reboot", HTTP_GET, []() {
    Serial.println("🔄 Reboot request received via WiFi");
    server.send(200, "text/plain", "Rebooting slave device...");
    server.handleClient(); // Ensure response is sent before reboot
    delay(500);
    ESP.restart(); // Reboot the ESP32
  });

  // Initialize ElegantOTA library for handling web-based firmware updates
  ElegantOTA.begin(&server); // Pass the WebServer instance
  ElegantOTA.setAutoReboot(true); // Automatically reboot after successful update

  // Callback function when ElegantOTA update starts
  ElegantOTA.onStart([]() {
    Serial.println("🔄 WiFi OTA Update Started (ElegantOTA)");
    otaInProgress = true;
    otaActive = true;
    otaProgress = 0;
    otaTotal = 0; // Total size will be known from onProgress
    otaStatus = "Starting ElegantOTA...";
    otaStartTime = millis();
    lastOTAActivity = millis();
    Serial.println("✅ Ready for firmware upload via ElegantOTA.");
    lcd.clear();
    lcd.print("WiFi OTA Started");
    lcd.setCursor(0, 1);
    lcd.print("Updating...");
  });

  // Callback function for ElegantOTA progress updates
  ElegantOTA.onProgress([](size_t current, size_t final_val) { // Renamed 'final' to avoid conflict
    otaProgress = current;
    otaTotal = final_val; // Update total size
    lastOTAActivity = millis();
    float percent = (float)current / final_val * 100.0;
    otaStatus = "Uploading (ElegantOTA): " + String((int)percent) + "%";
    Serial.printf("📊 WiFi OTA Progress (ElegantOTA): %u/%u bytes (%.1f%%)\n", current, final_val, percent);

    static int lastDisplayPercent = -1;
    int currentDisplayPercent = (int)percent;
    if (currentDisplayPercent != lastDisplayPercent && currentDisplayPercent % 5 == 0) { // Update LCD every 5%
      lcd.clear();
      lcd.print("WiFi OTA");
      lcd.setCursor(0, 1);
      lcd.print(String(currentDisplayPercent) + "% Updating");
      lastDisplayPercent = currentDisplayPercent;
    }
  });

  // Callback function when ElegantOTA update ends
  ElegantOTA.onEnd([](bool success) {
    if (success) {
      Serial.println("✅ WiFi OTA Update (ElegantOTA) completed successfully!");
      otaStatus = "Completed (ElegantOTA)";
      lcd.clear();
      lcd.print("OTA Success!");
      lcd.setCursor(0, 1);
      lcd.print("Rebooting...");
      delay(2000); // Allow time for display
    } else {
      Serial.println("❌ WiFi OTA Update (ElegantOTA) failed!");
      otaStatus = "Failed (ElegantOTA)";
      lcd.clear();
      lcd.print("OTA Failed!");
      delay(2000);
    }
    otaInProgress = false; // Reset OTA flags; ElegantOTA handles reboot if autoReboot is true
    otaActive = false;
  });

  // Endpoint for firmware updates pushed directly (e.g., from Flask server)
  server.on("/update", HTTP_POST,
    []() { // This is the onEnd handler for the direct upload
      if (Update.hasError()) {
        String errorMsg = "Update Error: " + String(Update.getError());
        Serial.println("❌ " + errorMsg);
        server.send(500, "text/plain", errorMsg); // Send error response
        otaStatus = "Update Error (Direct)";
        resetOTAProgress(); // Reset OTA state
      } else {
        Serial.println("✅ Flask OTA Update completed successfully!");
        otaStatus = "Completed (Direct)";
        server.send(200, "text/plain", "Update successful! Rebooting..."); // Send success response
        delay(1000); // Allow time for response
        ESP.restart(); // Reboot the device
      }
    },
    []() { // This is the onUpload handler (receives file data chunks)
      HTTPUpload& upload = server.upload(); // Get upload object

      if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("🔄 Flask OTA: Starting update with file: %s\n", upload.filename.c_str());
        String contentLengthHeader = server.header("Content-Length");
        size_t expectedSize = contentLengthHeader.toInt();
        Serial.printf("📊 Expected upload size (Flask): %u bytes\n", expectedSize);

        otaInProgress = true;
        otaActive = true;
        otaProgress = 0;
        otaTotal = expectedSize; // Set total size from header
        otaStatus = "Starting Flask OTA...";
        otaStartTime = millis();
        lastOTAActivity = millis();

        lcd.clear();
        lcd.print("Flask OTA Start");
        lcd.setCursor(0, 1);
        lcd.print("Receiving...");

        // Begin firmware update process
        if (expectedSize > 0) {
          if (!Update.begin(expectedSize)) { // Use expected size if available
            Serial.println("❌ Flask OTA: Not enough space or begin failed.");
            otaStatus = "Not enough space";
            resetOTAProgress();
          }
        } else {
          if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { // Fallback if size is unknown
            Serial.println("❌ Flask OTA: Update.begin() failed (size unknown).");
            otaStatus = "Update begin failed";
            resetOTAProgress();
          }
        }
      } else if (upload.status == UPLOAD_FILE_WRITE) { // Firmware data chunk received
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) { // Write chunk to flash
        Serial.println("❌ Flask OTA: Write failed.");
        otaStatus = "Write failed";
        resetOTAProgress();
    } else {
        otaProgress += upload.currentSize; // Update current progress
        lastOTAActivity = millis();
        
        // Fixed percentage calculation with explicit casting
        float percent = 0.0;
        if (otaTotal > 0) {
            percent = (float)otaProgress * 100.0 / (float)otaTotal;
        } else {
            // If total is unknown, show incremental progress
            static const unsigned int EST_FIRMWARE_SIZE = 1048576; // 1MB estimate
            percent = (float)otaProgress * 100.0 / (float)EST_FIRMWARE_SIZE;
            percent = min(percent, 99.0f); // Cap at 99% if we're estimating
        }
        
        // Only print progress to serial at defined intervals
        static unsigned long lastPrintTime = 0;
        static int lastPrintedPercent = -1;
        unsigned long currentTime = millis();
        int currentPercent = (int)(percent + 0.5); // Rounded percentage
        
        // Print only every 3 seconds OR when percentage changes by 5%
        if ((currentTime - lastPrintTime > 3000) || 
            (currentPercent >= lastPrintedPercent + 5) || 
            (currentPercent == 100 && lastPrintedPercent < 100)) {
            
            Serial.printf("📊 Flask OTA Progress: %.1f%% (%u/%u bytes)\n", percent, otaProgress, otaTotal);
            lastPrintTime = currentTime;
            lastPrintedPercent = currentPercent;
        }
        
        // Update status message regardless of printing
        otaStatus = "Uploading (Flask): " + String((int)(percent + 0.5)) + "%";

        // LCD update code (same as before)
        int currentDisplayPercent = (int)(percent + 0.5);
        static int lastDisplayPercent = -1;
        
        if (currentDisplayPercent != lastDisplayPercent && 
            (currentDisplayPercent % 10 == 0 || lastDisplayPercent == -1)) {
            lcd.clear();
            lcd.print("Flask OTA");
            lcd.setCursor(0, 1);
            if (otaTotal > 0) {
                lcd.print(String(currentDisplayPercent) + "% Received");
            } else {
                lcd.print(String(otaProgress/1024) + "KB Received");
            }
            lastDisplayPercent = currentDisplayPercent;
        }
      }
      } else if (upload.status == UPLOAD_FILE_END) { // Upload finished
        otaTotal = otaProgress; // Set final total size to actual progress
        if (Update.end(true)) { // Finalize the update (true to commit)
          Serial.println("✅ Flask OTA: Upload complete, verifying...");
          otaStatus = "Verifying (Flask)...";
          lcd.clear();
          lcd.print("Verifying...");
          lcd.setCursor(0, 1);
          lcd.print("Please wait...");
          // The onEnd handler (first lambda of server.on) will be called next
        } else {
          Serial.println("❌ Flask OTA: End failed.");
          Update.printError(Serial);
          otaStatus = "Verification failed (Flask)";
          resetOTAProgress();
        }
      }
    }
  );

  server.begin(); // Start the web server

  Serial.println("=======================================");
  Serial.println("🚀 Enhanced Slave OTA Server (WiFi) started");
  Serial.println("=======================================");
  Serial.printf("💾 Free heap: %d bytes\n", ESP.getFreeHeap());
  Serial.print("🌐 Access Control Panel at: http://"); Serial.println(WiFi.localIP());
  Serial.print("🔧 Access ElegantOTA at: http://"); Serial.println(WiFi.localIP().toString() + "/update");
  Serial.print("📊 Access OTA Progress at: http://"); Serial.println(WiFi.localIP().toString() + "/ota_progress");
  Serial.println("=======================================");
}

// Manages operations while the device is in WiFi OTA mode
void handleWiFiOTAMode() {
  server.handleClient(); // Handle incoming HTTP requests
  ElegantOTA.loop();     // Process ElegantOTA tasks

  // Check for WiFi OTA mode timeout
  if (millis() - wifiOtaTimeout > WIFI_OTA_TIMEOUT) {
    Serial.println("⏱️ WiFi OTA mode timed out. Returning to normal operation.");
    wifiOtaMode = false; // Exit WiFi OTA mode
    resetOTAProgress();    // Reset any OTA progress
    displaySystemStatus(); // Update display to normal status
  }
  delay(10); // Short delay
}

// Sends a heartbeat message via WiFi to the Flask server
void sendHeartbeat() {
  if (!isWifiAvailable || WiFi.status() != WL_CONNECTED) return; // Only send if WiFi is active

  HTTPClient http;
  http.begin(String(FLASK_SERVER) + "/api/device_heartbeat"); // API endpoint for heartbeats
  http.addHeader("Content-Type", "application/json");

  // Construct device name using IP for consistency with server expectations
  String deviceName = "ESP32-" + WiFi.localIP().toString();
  // Construct JSON payload
  String payload = "{";
  payload += "\"device_name\":\"" + deviceName + "\",";
  payload += "\"device_type\":\"slave\",";
  payload += "\"connection_type\":\"wifi_lora\","; // Indicate dual connectivity capability
  payload += "\"free_heap\":" + String(ESP.getFreeHeap()) + ",";
  payload += "\"people_count\":" + String(currentPeople);
  payload += "}";

  int responseCode = http.POST(payload);
  if (responseCode == HTTP_CODE_OK) { // Check for successful HTTP response
    Serial.printf("💓 WiFi heartbeat sent - Response: %d\n", responseCode);
  } else {
    Serial.printf("⚠️ WiFi heartbeat failed - HTTP Response: %d\n", responseCode);
  }
  http.end(); // Close HTTP connection
}

// Handles ACK messages received via LoRa (typically acknowledging a UID transmission)
void handleLoRaACK(String message) {
  // Expected format: ACK:<UID_that_was_acked>
  String ackedUID = message.substring(4); // Extract the UID from the ACK message
  if (ackedUID == lastSentUID) { // Check if it's an ACK for the last UID we sent
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ACK Received");
    lcd.setCursor(0, 1);
    lcd.print("UID: " + ackedUID.substring(0,16)); // Display acknowledged UID (truncated if long)
    delay(1000);
    // Potentially clear lastSentUID or mark as acknowledged if needed for retransmission logic
  } else {
    Serial.println("ℹ️ Received ACK for an older or unexpected UID: " + ackedUID);
  }
}

// Handles regular (non-OTA, non-ACK) messages received via LoRa
void handleRegularLoRaMessage(String message) {
  Serial.println("📨 Regular LoRa message received: " + message);
  // Example: Could be a command from master, like "OTA_MODE" to trigger WiFi OTA
  if (message == "OTA_MODE" && isWifiAvailable) {
      Serial.println("📶 Master requested WiFi OTA mode.");
      wifiOtaMode = true;
      wifiOtaTimeout = millis(); // Start timeout for WiFi OTA mode
      lcd.clear();
      lcd.print("WiFi OTA Mode");
      lcd.setCursor(0,1);
      lcd.print(WiFi.localIP().toString());
      // Send confirmation back to master via LoRa
      sendLoRaMessage("OTA_READY:" + WiFi.localIP().toString());
  }
  // Add other custom message handling here
}

void handlePeopleCounting() {
  if (loraOtaMode || wifiOtaMode) return; // Skip during OTA

  static float prevDistance1 = -1;
  static float prevDistance2 = -1;

  float distance1 = getFilteredDistance(TRIG_PIN_1, ECHO_PIN_1);
  float distance2 = getFilteredDistance(TRIG_PIN_2, ECHO_PIN_2);

  // Validate readings
  if (!isSensorReadingValid(distance1, prevDistance1) || !isSensorReadingValid(distance2, prevDistance2)) {
    prevDistance1 = distance1;
    prevDistance2 = distance2;
    return;
  }
  prevDistance1 = distance1;
  prevDistance2 = distance2;

  // Use dynamic threshold logic
  bool sensor1Active = isSensor1Active(distance1);
  bool sensor2Active = isSensor2Active(distance2);

  static bool prevSensor1State = false;
  static bool prevSensor2State = false;

  unsigned long currentTime = millis();

  if (currentState != DETECTION_IDLE && (currentTime - stateStartTime > SEQUENCE_TIMEOUT)) {
    Serial.println("⚠️ Detection sequence timeout");
    currentState = DETECTION_IDLE;
  }
  
  switch (currentState) {
    case DETECTION_IDLE:
      if (sensor1Active && !sensor2Active) {
        currentState = ENTRY_S1_ACTIVE;
        stateStartTime = currentTime;
        Serial.println("🔄 Entry sequence started: Sensor 1 triggered");
      } else if (!sensor1Active && sensor2Active) {
        currentState = EXIT_S2_ACTIVE;
        stateStartTime = currentTime;
        Serial.println("🔄 Exit sequence started: Sensor 2 triggered");
      }
      break;

    case ENTRY_S1_ACTIVE:
      if (!sensor1Active) {
        currentState = ENTRY_TRANSITION;
        stateStartTime = currentTime;
        Serial.println("🔄 Entry transition: Person between sensors");
      } else if (sensor2Active) {
        Serial.println("⚠️ Both sensors active during entry - possible crowding");
      }
      break;

    case ENTRY_TRANSITION:
      if (sensor2Active) {
        if (currentTime - stateStartTime >= MIN_TRANSITION_TIME) {
          currentPeople++;
          Serial.print("✅ Entry confirmed - People count: "); Serial.println(currentPeople);
          currentState = DETECTION_IDLE;
        } else {
          Serial.println("⚠️ Transition too fast - possible false detection");
          currentState = DETECTION_IDLE;
        }
      } else if (sensor1Active) {
        Serial.println("⚠️ Reversed direction during entry");
        currentState = ENTRY_S1_ACTIVE;
        stateStartTime = currentTime;
      }
      break;

    case EXIT_S2_ACTIVE:
      if (!sensor2Active) {
        currentState = EXIT_TRANSITION;
        stateStartTime = currentTime;
        Serial.println("🔄 Exit transition: Person between sensors");
      } else if (sensor1Active) {
        Serial.println("⚠️ Both sensors active during exit - possible crowding");
      }
      break;

    case EXIT_TRANSITION:
      if (sensor1Active) {
        if (currentTime - stateStartTime >= MIN_TRANSITION_TIME) {
          if (currentPeople > 0) {
            currentPeople--;
            Serial.print("✅ Exit confirmed - People count: "); Serial.println(currentPeople);
          } else {
            Serial.println("⚠️ Exit detected but count already zero");
          }
          currentState = DETECTION_IDLE;
        } else {
          Serial.println("⚠️ Transition too fast - possible false detection");
          currentState = DETECTION_IDLE;
        }
      } else if (sensor2Active) {
        Serial.println("⚠️ Reversed direction during exit");
        currentState = EXIT_S2_ACTIVE;
        stateStartTime = currentTime;
      }
      break;
  }

  prevSensor1State = sensor1Active;
  prevSensor2State = sensor2Active;
}


float getFilteredDistance(int trigPin, int echoPin) {
  // Get raw distance
  float rawDistance = getDistance(trigPin, echoPin);
  
  // Store in history array (circular buffer)
  if (trigPin == TRIG_PIN_1) {
    sensor1History[historyIndex] = rawDistance;
  } else {
    sensor2History[historyIndex] = rawDistance;
  }
  
  if (trigPin == TRIG_PIN_2) {  // Only increment index after both sensors
    historyIndex = (historyIndex + 1) % FILTER_SAMPLES;
  }
  
  // Calculate median or average (median is better for filtering outliers)
  float distances[FILTER_SAMPLES];
  if (trigPin == TRIG_PIN_1) {
    for (int i = 0; i < FILTER_SAMPLES; i++) distances[i] = sensor1History[i];
  } else {
    for (int i = 0; i < FILTER_SAMPLES; i++) distances[i] = sensor2History[i];
  }
  
  // Use median filtering (simple implementation for small array)
  for (int i = 0; i < FILTER_SAMPLES - 1; i++) {
    for (int j = i + 1; j < FILTER_SAMPLES; j++) {
      if (distances[i] > distances[j]) {
        float temp = distances[i];
        distances[i] = distances[j];
        distances[j] = temp;
      }
    }
  }
  
  // Return median value (middle of sorted array)
  return distances[FILTER_SAMPLES / 2];
}


// Measures distance using an ultrasonic sensor
float getDistance(int trigPin, int echoPin) {
  // Send ultrasonic pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure pulse duration from echo pin (timeout after 30ms for ~5m range)
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return MAX_DISTANCE + 10.0f; // Return out-of-range if timeout

  // Calculate distance in cm
  float distance = duration * 0.0343 / 2.0; // Speed of sound is ~343 m/s or 0.0343 cm/µs
  return (distance > MAX_DISTANCE || distance < 0) ? MAX_DISTANCE + 10.0f : distance; // Clamp to valid range
}

bool isSensorReadingValid(float currentDistance, float prevDistance) {
  // Check for sudden large changes (potential errors)
  if (prevDistance > 0 && abs(currentDistance - prevDistance) > 50) {
    return false;
  }
  
  // Check for out-of-range readings
  if (currentDistance < 2.0 || currentDistance > MAX_DISTANCE) {
    return false;
  }
  
  return true;
}

void calibrateSensors() {
  Serial.println("📏 Calibrating sensors (keep area clear)...");
  lcd.clear();
  lcd.print("Calibrating...");
  lcd.setCursor(0, 1);
  lcd.print("Keep area clear");
  
  const int CALIBRATION_SAMPLES = 20;
  float sum1 = 0, sum2 = 0;
  int validSamples1 = 0, validSamples2 = 0;
  
  // Collect multiple readings
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    float d1 = getDistance(TRIG_PIN_1, ECHO_PIN_1);
    float d2 = getDistance(TRIG_PIN_2, ECHO_PIN_2);
    
    // Only include reasonable readings
    if (d1 > 5 && d1 < MAX_DISTANCE) {
      sum1 += d1;
      validSamples1++;
    }
    if (d2 > 5 && d2 < MAX_DISTANCE) {
      sum2 += d2;
      validSamples2++;
    }
    delay(100);
  }
  
  // Calculate baselines if we got enough valid samples
  if (validSamples1 >= CALIBRATION_SAMPLES/2) {
    baseline1 = sum1 / validSamples1;
  }
  
  if (validSamples2 >= CALIBRATION_SAMPLES/2) {
    baseline2 = sum2 / validSamples2;
  }
  
  Serial.println("✅ Calibration complete");
  Serial.print("📊 Baseline 1: "); Serial.print(baseline1); 
  Serial.print(" cm, Baseline 2: "); Serial.print(baseline2); Serial.println(" cm");
  
  lcd.clear();
  lcd.print("Calibration done");
  delay(2000);
}

bool isSensor1Active(float distance) {
  if (baseline1 > 0) {
    // Use dynamic threshold if baseline is available
    return (distance > 0 && distance < baseline1 * DETECTION_PERCENTAGE);
  } else {
    // Fall back to fixed threshold
    return (distance > 0 && distance < MIN_DISTANCE);
  }
}

bool isSensor2Active(float distance) {
  if (baseline2 > 0) {
    return (distance > 0 && distance < baseline2 * DETECTION_PERCENTAGE);
  } else {
    return (distance > 0 && distance < MIN_DISTANCE);
  }
}

// Reset counter at specific times (e.g., after class hours)
void checkTimeBasedReset() {
  // If WiFi is available, could get network time
  // If not, use elapsed time since boot
  unsigned long currentTime = millis();
  static unsigned long lastResetTime = 0;
  
  // Reset counter every 24 hours (adjust interval as needed)
  const unsigned long RESET_INTERVAL = 24 * 60 * 60 * 1000; // 24 hours in ms
  
  if (currentTime - lastResetTime >= RESET_INTERVAL) {
    Serial.println("🔄 Performing scheduled counter reset");
    currentPeople = 0;
    lastResetTime = currentTime;
  }
}

void checkSensorHealth() {
  float d1 = getDistance(TRIG_PIN_1, ECHO_PIN_1);
  float d2 = getDistance(TRIG_PIN_2, ECHO_PIN_2);
  
  bool sensor1Healthy = (d1 >= 0 && d1 <= MAX_DISTANCE + 10);
  bool sensor2Healthy = (d2 >= 0 && d2 <= MAX_DISTANCE + 10);
  
  if (!sensor1Healthy || !sensor2Healthy) {
    static unsigned long lastWarningTime = 0;
    if (millis() - lastWarningTime > 30000) { // Show warning every 30 seconds
      Serial.println("⚠️ SENSOR HEALTH WARNING:");
      if (!sensor1Healthy) Serial.println("❌ Sensor 1 may be disconnected or faulty");
      if (!sensor2Healthy) Serial.println("❌ Sensor 2 may be disconnected or faulty");
      lastWarningTime = millis();
      
      // Optionally display on LCD
      lcd.clear();
      lcd.print("Sensor Warning!");
      lcd.setCursor(0, 1);
      lcd.print(sensor1Healthy ? "S2 Error" : "S1 Error");
      delay(2000);
    }
  }
}
// Updates the LCD display with current information
void updateDisplay() {
  if (loraOtaMode || wifiOtaMode || isEnteringID) return; // Don't update display during OTA or ID entry

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("People:" + String(currentPeople));
  if (isWifiAvailable) {
    lcd.print(" W+L"); // Indicate WiFi + LoRa connectivity
  } else {
    lcd.print(" L");   // Indicate LoRa only connectivity
  }

  lcd.setCursor(0, 1);
  // Display distances from sensors (or "---" if out of range)
  float d1 = getDistance(TRIG_PIN_1, ECHO_PIN_1);
  float d2 = getDistance(TRIG_PIN_2, ECHO_PIN_2);
  String s1Dist = (d1 > MAX_DISTANCE) ? "---" : String((int)d1);
  String s2Dist = (d2 > MAX_DISTANCE) ? "---" : String((int)d2);
  lcd.print("S1:" + s1Dist + " S2:" + s2Dist);
}

// Handles RFID card scanning and processing
void handleRFID() {
  if (loraOtaMode || wifiOtaMode || isEnteringID) return; // Pause RFID during OTA or ID entry

  // Check if a new RFID card is present and readable
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    String uid = ""; // String to store the card's UID
    // Read UID byte by byte and convert to hex string
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      char hex[3];
      sprintf(hex, "%02X", mfrc522.uid.uidByte[i]); // Format byte as 2-digit hex
      uid += hex;
    }

    if (studentCount < 70) { // Check if there's space in the student list
      students[studentCount] = uid; // Add UID to the list
      studentCount++;
      sendUIDViaLoRa(uid); // Send the scanned UID via LoRa
      lcd.clear();
      lcd.print("Card Saved:");
      lcd.setCursor(0, 1);
      lcd.print(uid.substring(0,16)); // Display UID (truncated)
      delay(2000);
    } else { // Student list is full
      lcd.clear();
      lcd.print("List Full!");
      delay(2000);
    }
    mfrc522.PICC_HaltA(); // Halt the card to allow reading new cards
    mfrc522.PCD_StopCrypto1(); // Stop encryption on PCD
  }
}

// Handles keypad input for student ID entry and admin menu access
void handleKeypad() {
  if (loraOtaMode || wifiOtaMode) return; // Pause keypad during OTA

  char key = keypad.getKey(); // Get pressed key

  if (key && !isEnteringID) { // If a key is pressed and not already entering ID
    isEnteringID = true; // Set flag to start ID entry mode
    studentID = "";      // Clear previous student ID
    lcd.clear();
    lcd.print("Enter Student ID:");
    lcd.setCursor(0, 1); // Move cursor to second line for ID input
  }

  while (isEnteringID) { // Loop while in ID entry mode
    key = keypad.getKey(); // Continuously check for key presses
    if (key >= '0' && key <= '9') { // If a digit is pressed
      if (studentID.length() < 8) { // Limit ID length to 8 digits
        studentID += key;
        lcd.print(key); // Display pressed digit
      }
    } else if (key == '*') { // '*' key for confirm/enter
      if (studentID.length() == 8) { // Check if ID length is valid
        if (studentID == "10111213") { // Check for admin ID
          lcd.clear();
          lcd.print("Admin Mode");
          delay(1000);
          adminMenu(); // Enter admin menu
        } else if (studentCount < 70) { // If not admin, save as student ID
          students[studentCount] = studentID;
          studentCount++;
          sendUIDViaLoRa(studentID); // Send entered ID via LoRa
          lcd.clear();
          lcd.print("Saved!");
          delay(2000);
        } else { // Student list is full
          lcd.clear();
          lcd.print("List Full!");
          delay(2000);
        }
      } else { // Invalid ID length
        lcd.clear();
        lcd.print("Invalid ID!");
        delay(2000);
      }
      isEnteringID = false; // Exit ID entry mode
    } else if (key == '#') { // '#' key for cancel
      isEnteringID = false; // Exit ID entry mode
      lcd.clear();
      lcd.print("Cancelled");
      delay(2000);
    }
    delay(50); // Short delay for keypad debouncing/ responsiveness
  }
}

// Sends a UID (student ID or card UID) via LoRa to the master device
void sendUIDViaLoRa(String uid) {
  // Avoid sending the same UID too frequently
  if (uid == lastSentUID && (millis() - lastSendTime < SEND_INTERVAL)) {
    Serial.println("ℹ️ UID " + uid + " already sent recently. Skipping.");
    return;
  }

  String message = "UID:" + uid; // Construct LoRa message
  sendLoRaMessage(message);      // Send the message

  Serial.print("📤 Sent UID via LoRa: "); Serial.println(uid);
  lastSentUID = uid;        // Update last sent UID
  lastSendTime = millis(); // Update timestamp of last send

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sending UID:");
  lcd.setCursor(0, 1);
  lcd.print(uid.substring(0,16)); // Display UID (truncated)
  delay(1000); // Display message for a short duration
}

// Displays and handles the admin menu options
void adminMenu() {
  String options[] = {
    "1. Send Data",
    "2. Clear List",
    "3. View Count",
    "4. Reset Counter",
    "5. ConnectionInfo",
    "6. Sensor Test"
  };
  int currentOption = 0; // Index of the currently selected option

  while(true) { // Loop until an option is chosen or cancelled
    lcd.clear();
    lcd.print(options[currentOption]); // Display current option

    char key = keypad.getKey(); // Get keypad input
    if(key == '2') currentOption = (currentOption + 1) % 6; // Changed from %5 to %6 for all options
    else if(key == '8') currentOption = (currentOption + 5) % 6; // Changed from %5 to %6 for all options
    else if(key == '*') { // '*' to select option
      executeAdminCommand(currentOption);
      break; // Exit admin menu after executing command
    }
    else if(key == '#') break; // '#' to cancel and exit admin menu

    delay(200); // Debounce delay for keypad
  }
  displaySystemStatus(); // Return to normal display after exiting admin menu
}

// Executes the selected admin command
void executeAdminCommand(int option) {
  lcd.clear();
  switch(option) {
    case 0: // Send Data
      lcd.print("Sending Data...");
      if (studentCount == 0) {
        lcd.setCursor(0, 1);
        lcd.print("No data to send");
      } else {
        int sent = 0;
        for (int i = 0; i < studentCount; i++) {
          // Update progress on second line
          lcd.setCursor(0, 1);
          lcd.print(String(i+1) + "/" + String(studentCount));
          
          sendUIDViaLoRa(students[i]);
          delay(1000);
          sent++;
        }
        lcd.clear();
        lcd.print("All Data Sent!");
        lcd.setCursor(0, 1);
        lcd.print("Total: " + String(sent));
      }
      break;
      
    case 1: // Clear List
      studentCount = 0;
      lcd.print("List Cleared");
      lcd.setCursor(0, 1);
      lcd.print("Count: 0");
      break;
      
    case 2: // View Count
      lcd.print("People Count:");
      lcd.setCursor(0, 1);
      lcd.print(String(currentPeople));
      break;
      
    case 3: // Reset Counter
      currentPeople = 0;
      lcd.print("Counter Reset");
      lcd.setCursor(0, 1);
      lcd.print("New Count: 0");
      break;
      
    case 4: // Connection Info
      lcd.print("ID:");
      lcd.setCursor(4, 0);
      // Show first 12 chars on first line
      lcd.print(deviceId.substring(0, 12));
      lcd.setCursor(0, 1);
      lcd.print(isWifiAvailable ? "WiFi+LoRa" : "LoRa Only");
      break;
      
    case 5: // Sensor Test
      calibrateSensors();
      
      // Continuous sensor display for testing
      lcd.clear();
      lcd.print("Sensor Test");
      lcd.setCursor(0, 1);
      lcd.print("Running...");
      delay(1000);
      
      unsigned long testEnd = millis() + 10000; // Run test for 10 seconds
      while (millis() < testEnd) {
        float d1 = getFilteredDistance(TRIG_PIN_1, ECHO_PIN_1);
        float d2 = getFilteredDistance(TRIG_PIN_2, ECHO_PIN_2);
        
        // Display readings on LCD
        lcd.clear();
        lcd.print("S1:" + String((int)d1) + "cm");
        lcd.setCursor(9, 0);
        lcd.print("B:" + String((int)baseline1));
        lcd.setCursor(0, 1);
        lcd.print("S2:" + String((int)d2) + "cm");
        lcd.setCursor(9, 1);
        lcd.print("B:" + String((int)baseline2));
        
        // Show sensor active status on serial
        bool s1Active = isSensor1Active(d1);
        bool s2Active = isSensor2Active(d2);
        Serial.printf("S1: %d cm (Active: %d), S2: %d cm (Active: %d)\n", 
                     (int)d1, s1Active, (int)d2, s2Active);
                     
        delay(300); // Update 3 times per second
      }
      
      lcd.clear();
      lcd.print("Test Complete");
      break;
  }
  delay(2000); // Show result for 2 seconds
}

// ============================================
// OTA Progress Management (primarily for WiFi OTA)
// ============================================

// Resets variables related to OTA update progress
void resetOTAProgress() {
  otaInProgress = false;
  otaProgress = 0;
  otaTotal = 0;
  otaActive = false;
  otaStatus = "Ready";
  lastOTAActivity = 0;
  Serial.println("🔄 Slave OTA progress variables reset.");
}