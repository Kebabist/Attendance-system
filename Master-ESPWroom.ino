/*
 * ESP32 Master Device - Enhanced Attendance System with LoRa OTA Support
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
 *    a) LoRa Library:
 *       - Search: "LoRa"
 *       - Install: "LoRa by Sandeep Mistry" (latest version)
 *    
 *    b) ElegantOTA Library:
 *       - Search: "ElegantOTA"
 *       - Install: "ElegantOTA by Ayush Sharma" (latest version)
 *    
 *    c) ArduinoJson Library:
 *       - Search: "ArduinoJson"
 *       - Install: "ArduinoJson by Benoit Blanchon" (version 6.x recommended)
 * 
 * 3. BUILT-IN LIBRARIES (No installation needed):
 *    - WiFi (ESP32 Core)
 *    - HTTPClient (ESP32 Core)
 *    - SPI (Arduino Core)
 *    - WebServer (ESP32 Core)
 *    - Update (ESP32 Core)
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
 *    LoRa Module (SX1276/SX1278) connections:
 *    - VCC → 3.3V
 *    - GND → GND
 *    - SCK → GPIO 18 (SPI SCK)
 *    - MISO → GPIO 19 (SPI MISO)
 *    - MOSI → GPIO 23 (SPI MOSI)
 *    - NSS/CS → GPIO 5 (LORA_SS_PIN)
 *    - RST → GPIO 14 (LORA_RST_PIN)
 *    - DIO0 → GPIO 2 (LORA_DIO0_PIN)
 * 
 * 6. BEFORE UPLOADING:
 *    - Update WiFi credentials (WIFI_SSID and WIFI_PASSWORD)
 *    - Update server URLs (SERVER_URL and FLASK_SERVER)
 *    - Ensure Flask server is running and accessible
 *    - Verify hardware connections
 * 
 * 7. FEATURES:
 *    - WiFi connectivity and web interface
 *    - LoRa communication with slave devices
 *    - Over-The-Air (OTA) firmware updates (WiFi and LoRa)
 *    - Attendance data collection and forwarding
 *    - Real-time device monitoring and management
 *    - Web-based control panel
 * 
 * 8. TROUBLESHOOTING:
 *    - If compilation fails, ensure all libraries are installed correctly
 *    - If upload fails, check COM port and board selection
 *    - If WiFi connection fails, verify credentials and network availability
 *    - If LoRa communication fails, check wiring and antenna connections
 * 
 * Author: Kebabist
 * Date: June 2025
 * Version: 1.0
 */
#include <WiFi.h>
#include <HTTPClient.h>
#include <SPI.h>
#include <LoRa.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <map>
#include <vector>

// LoRa pin definitions
#define LORA_SS_PIN 5
#define LORA_RST_PIN 14
#define LORA_DIO0_PIN 2

// WiFi network credentials
const char* WIFI_SSID = "Your Wifi SSID";
const char* WIFI_PASSWORD = "Your Wifi Password";

// Server URLs for communication
const char* SERVER_URL = "http://Replace with server IP:5001/register_attendance"; // Endpoint for registering attendance
const char* FLASK_SERVER = "http://Replace with server IP:5001"; // Base URL for the Flask server

// Web server instance for OTA updates and web interface
WebServer server(80);

// Global variables for device operation
String receivedUID = ""; // Stores the last UID received via LoRa
unsigned long lastSendTime = 0; // Timestamp of the last data transmission to the server
const long SEND_INTERVAL = 5000; // Interval for sending data to the server (milliseconds)
unsigned long lastHeartbeat = 0; // Timestamp of the last heartbeat sent
const unsigned long HEARTBEAT_INTERVAL = 60000; // Interval for sending heartbeats (milliseconds)

// Variables for tracking Over-The-Air (OTA) update progress
volatile bool otaInProgress = false; // Flag indicating if an OTA update is currently in progress
volatile size_t otaProgress = 0; // Current progress of the OTA update (bytes downloaded)
volatile size_t otaTotal = 0; // Total size of the OTA update (bytes)
volatile bool otaActive = false; // Flag indicating if OTA functionality is active
String otaStatus = "Ready"; // Current status of the OTA process
unsigned long otaStartTime = 0; // Timestamp when the OTA update started
unsigned long lastOTAActivity = 0; // Timestamp of the last OTA activity

// Variables for OTA command tracking
unsigned long lastOTACheck = 0; // Timestamp of the last check for OTA commands
const unsigned long OTA_CHECK_INTERVAL = 60000; // Interval for checking OTA commands (milliseconds)

// Variables for LoRa-based OTA updates
bool loraOtaInProgress = false; // Flag indicating if a LoRa OTA update is in progress
String loraOtaTargetSlave = ""; // ID of the slave device targeted for LoRa OTA
String loraOtaStatus = "Ready"; // Current status of the LoRa OTA process
size_t loraOtaProgress = 0; // Current progress of the LoRa OTA update (bytes sent/received)
size_t loraOtaTotal = 0; // Total size of the LoRa OTA firmware (bytes)
size_t loraOtaTotalChunks = 0; // Total number of chunks for the LoRa OTA firmware
size_t loraOtaSentChunks = 0; // Number of chunks already sent during LoRa OTA
size_t loraOtaCurrentChunk = 0; // Index of the current chunk being processed for LoRa OTA
uint32_t loraOtaCRC = 0; // CRC32 checksum of the LoRa OTA firmware
unsigned long loraOtaStartTime = 0; // Timestamp when the LoRa OTA update started
unsigned long loraOtaLastActivity = 0; // Timestamp of the last LoRa OTA activity
unsigned long loraOtaChunkTimeout = 0; // Timestamp for LoRa OTA chunk timeout
const unsigned long LORA_OTA_CHUNK_TIMEOUT = 10000; // Timeout for receiving a chunk ACK during LoRa OTA (milliseconds)
const unsigned long LORA_OTA_TOTAL_TIMEOUT = 600000; // Total timeout for the LoRa OTA process (milliseconds)

// Buffers for LoRa OTA chunk management
const size_t LORA_CHUNK_SIZE = 200; // Maximum size of a LoRa payload chunk
uint8_t* loraOtaFirmwareData = nullptr; // Buffer to store the firmware data for LoRa OTA
String* loraOtaChunks = nullptr; // Array to store Base64 encoded firmware chunks for LoRa OTA
bool* loraOtaChunkAcked = nullptr; // Array to track acknowledgment of sent chunks during LoRa OTA

// Structure to store information about connected slave devices
struct SlaveDevice {
  String id; // Unique identifier of the slave device
  String lastSeen; // Timestamp of the last communication from the slave
  int peopleCount; // People count reported by the slave
  uint32_t freeHeap; // Free heap memory reported by the slave
  int signalStrength; // LoRa signal strength (RSSI) of the slave
};

// Array to store connected slave devices
const int MAX_SLAVES = 10; // Maximum number of slave devices that can be tracked
SlaveDevice slaves[MAX_SLAVES]; // Array of SlaveDevice structures
int slaveCount = 0; // Current number of connected slave devices

unsigned long lastSlaveCleanup = 0; // Timestamp of the last inactive slave cleanup
const unsigned long SLAVE_CLEANUP_INTERVAL = 120000; // Interval for cleaning up inactive slaves (milliseconds)

// Queue for storing UIDs received via LoRa
const int MAX_QUEUE_SIZE = 10; // Maximum number of UIDs to store
String uidQueue[MAX_QUEUE_SIZE]; // Array to store UIDs
int queueHead = 0; // Index for the next UID to process
int queueTail = 0; // Index for adding new UIDs
int queueCount = 0; // Current number of UIDs in the queue

// Forward declarations of functions
uint8_t* base64_decode(const char* input, size_t input_length, size_t* output_length);
int findSlaveIndex(String slaveId);
void addOrUpdateSlave(String slaveId, int peopleCount, uint32_t freeHeap, int signalStrength);
void removeInactiveSlaves();

// Setup function: initializes the device and peripherals
void setup() {
  Serial.begin(115200); // Initialize serial communication at 115200 baud
  while (!Serial); // Wait for the serial port to connect

  Serial.println("\n=======================================");
  Serial.println("🚀 Enhanced Master with LoRa OTA Support");
  Serial.println("=======================================");

  connectToWiFi(); // Connect to the WiFi network
  initializeLoRa(); // Initialize the LoRa module
  sendHeartbeat(); // Send an initial heartbeat to the server
  setupOTA(); // Setup the OTA update server and handlers
}

// Main loop function: runs repeatedly to handle device operations
void loop() {
  server.handleClient(); // Handle incoming HTTP client requests for the web server
  ElegantOTA.loop(); // Handle ElegantOTA background tasks

  // Check for master OTA timeout
  if (otaInProgress && (millis() - lastOTAActivity > 30000)) {
    Serial.println("⚠️ Master OTA timeout detected, resetting progress tracking");
    resetOTAProgress(); // Reset OTA progress variables
  }
/*
  // Check for LoRa OTA timeouts
  if (loraOtaInProgress) {
    if (millis() - loraOtaLastActivity > LORA_OTA_TOTAL_TIMEOUT) {
      Serial.println("⏱️ LoRa OTA total timeout, aborting");
      abortLoRaOTA(); // Abort the LoRa OTA process
    } else if (millis() - loraOtaChunkTimeout > LORA_OTA_CHUNK_TIMEOUT && loraOtaCurrentChunk < loraOtaTotalChunks) {
      Serial.printf("⏱️ Chunk %u timeout, retrying...\n", loraOtaCurrentChunk);
      retryCurrentChunk(); // Retry sending the current LoRa OTA chunk
    }
  }
*/
  yield(); // Yield to allow background system tasks (e.g., WiFi)

  // Reconnect to WiFi if disconnected
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }

  receiveLoRaData(); // Check for and process incoming LoRa messages

  // Send received UID to server if interval has passed
  if (queueCount > 0 && millis() - lastSendTime > SEND_INTERVAL) {
    String uid = uidQueue[queueHead]; // Get the oldest UID from the queue
    Serial.println("🔄 Processing queued UID: " + uid);
    
    sendToServer(uid); // Send the UID to the backend server
    
    // Remove the processed UID from the queue
    queueHead = (queueHead + 1) % MAX_QUEUE_SIZE;
    queueCount--;
    Serial.printf("📊 Updated queue status: %d UIDs remaining\n", queueCount);
    
    lastSendTime = millis(); // Update timestamp of last send
  }

  // Periodically check for OTA commands
  if (millis() - lastOTACheck > OTA_CHECK_INTERVAL) {
    checkForOTACommand(); // Check for any pending OTA commands
    lastOTACheck = millis();
  }

  // Periodically send heartbeat to the server
  if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
    sendHeartbeat();
    lastHeartbeat = millis();
  }
/*
  // Periodically remove inactive slave devices
  if (millis() - lastSlaveCleanup > SLAVE_CLEANUP_INTERVAL) {
    removeInactiveSlaves();
    lastSlaveCleanup = millis();
  }
*/
  delay(10); // Small delay to prevent watchdog timer issues and allow other tasks
}

// ============================================
// Slave Management Functions
// ============================================

// Finds the index of a slave device in the 'slaves' array by its ID
int findSlaveIndex(String slaveId) {
  for (int i = 0; i < slaveCount; i++) {
    if (slaves[i].id == slaveId) {
      return i; // Return the index if found
    }
  }
  return -1; // Return -1 if not found
}

// Adds a new slave device or updates an existing one
void addOrUpdateSlave(String slaveId, int peopleCount, uint32_t freeHeap, int signalStrength) {
  int index = findSlaveIndex(slaveId); // Check if the slave already exists

  if (index >= 0) {
    // Update details of an existing slave
    slaves[index].lastSeen = String(millis());
    slaves[index].peopleCount = peopleCount;
    slaves[index].freeHeap = freeHeap;
    slaves[index].signalStrength = signalStrength;
  } else if (slaveCount < MAX_SLAVES) {
    // Add a new slave if there is space
    slaves[slaveCount].id = slaveId;
    slaves[slaveCount].lastSeen = String(millis());
    slaves[slaveCount].peopleCount = peopleCount;
    slaves[slaveCount].freeHeap = freeHeap;
    slaves[slaveCount].signalStrength = signalStrength;
    slaveCount++; // Increment the count of connected slaves
    Serial.printf("➕ Added new slave: %s (Total: %d)\n", slaveId.c_str(), slaveCount);
  } else {
    Serial.println("⚠️ Maximum slaves reached, cannot add new slave");
  }
}

// Removes slave devices that have been inactive for a defined timeout period
void removeInactiveSlaves() {
  unsigned long currentTime = millis();
  const unsigned long SLAVE_TIMEOUT = 180000; // 3 minutes timeout for slave inactivity

  for (int i = slaveCount - 1; i >= 0; i--) { // Iterate backwards to allow safe removal
    unsigned long lastSeenTime = slaves[i].lastSeen.toInt();
    if (currentTime - lastSeenTime > SLAVE_TIMEOUT) {
      Serial.printf("🧹 Removing inactive slave: %s\n", slaves[i].id.c_str());

      // Shift remaining slaves down in the array to fill the gap
      for (int j = i; j < slaveCount - 1; j++) {
        slaves[j] = slaves[j + 1];
      }
      slaveCount--; // Decrement the slave count
    }
  }
}

// Returns the current number of connected slave devices
int getConnectedSlavesCount() {
  return slaveCount;
}

// ============================================
// LoRa OTA Implementation
// ============================================

// Handles the initiation of a LoRa OTA push to a slave device
void handleLoRaOTAPush(String slaveId, String firmwareB64, size_t firmwareSize, uint32_t firmwareCRC) {
  Serial.println("=======================================");
  Serial.println("🔄 Starting LoRa OTA Push");
  Serial.println("=======================================");
  Serial.printf("📱 Target Slave: %s\n", slaveId.c_str());
  Serial.printf("📦 Firmware Size: %u bytes\n", firmwareSize);
  Serial.printf("🔐 CRC32: 0x%08X\n", firmwareCRC);

  if (loraOtaInProgress) {
    Serial.println("❌ LoRa OTA already in progress");
    return;
  }

  // Decode the Base64 encoded firmware data
  size_t decodedLength = 0;
  loraOtaFirmwareData = base64_decode(firmwareB64.c_str(), firmwareB64.length(), &decodedLength);

  if (loraOtaFirmwareData == nullptr || decodedLength != firmwareSize) {
    Serial.println("❌ Failed to decode firmware data");
    if (loraOtaFirmwareData) free(loraOtaFirmwareData); // Free memory if allocated
    return;
  }

  // Calculate the total number of chunks required
  loraOtaTotalChunks = (firmwareSize + LORA_CHUNK_SIZE - 1) / LORA_CHUNK_SIZE;

  // Allocate memory for chunk management arrays
  loraOtaChunks = new String[loraOtaTotalChunks];
  loraOtaChunkAcked = new bool[loraOtaTotalChunks];

  if (loraOtaChunks == nullptr || loraOtaChunkAcked == nullptr) {
    Serial.println("❌ Failed to allocate chunk management memory");
    cleanupLoRaOTA(); // Clean up any allocated resources
    return;
  }

  // Prepare all firmware chunks by encoding them in Base64
  Serial.println("📦 Preparing firmware chunks...");
  for (size_t i = 0; i < loraOtaTotalChunks; i++) {
    size_t chunkOffset = i * LORA_CHUNK_SIZE;
    size_t chunkSize = min((size_t)LORA_CHUNK_SIZE, firmwareSize - chunkOffset);

    String chunkB64 = encodeBase64(loraOtaFirmwareData + chunkOffset, chunkSize); // Encode binary chunk to Base64
    loraOtaChunks[i] = chunkB64;
    loraOtaChunkAcked[i] = false; // Initialize ACK status to false

    if (i % 50 == 0) { // Print progress and yield periodically
      Serial.printf("📊 Prepared chunk %u/%u\n", i + 1, loraOtaTotalChunks);
      yield();
    }
  }

  // Initialize LoRa OTA state variables
  loraOtaInProgress = true;
  loraOtaTargetSlave = slaveId;
  loraOtaTotal = firmwareSize;
  loraOtaCRC = firmwareCRC;
  loraOtaProgress = 0;
  loraOtaSentChunks = 0;
  loraOtaCurrentChunk = 0;
  loraOtaStartTime = millis();
  loraOtaLastActivity = millis();
  loraOtaStatus = "Sending OTA_START command...";

  Serial.printf("✅ LoRa OTA initialized - %u chunks prepared\n", loraOtaTotalChunks);

  sendLoRaOTAStart(); // Send the initial OTA_START command to the slave
}

// Encodes binary data to a Base64 string
String encodeBase64(uint8_t* data, size_t length) {
  const char base64_chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  String encoded = "";
  encoded.reserve((length * 4 / 3) + 4); // Pre-allocate string memory for efficiency

  for (size_t i = 0; i < length; i += 3) {
    uint32_t value = 0;
    int valb = 0; // Number of valid bytes in the current 3-byte group

    for (int j = 0; j < 3; j++) {
      value <<= 8;
      if (i + j < length) {
        value |= data[i + j];
        valb++;
      }
    }

    value <<= (3 - valb) * 8; // Shift to align for 4 Base64 characters

    for (int j = 0; j < 4; j++) {
      if (j <= valb) { // valb is 0-indexed, j is 0-indexed for 4 chars
        encoded += base64_chars[(value >> (6 * (3 - j))) & 0x3F];
      } else {
        encoded += '='; // Add padding if necessary
      }
    }
  }
  return encoded;
}

// Sends the OTA_START command to the target slave device
void sendLoRaOTAStart() {
  String command = "OTA_START:" + String(loraOtaTotal) + ":" + String(loraOtaTotalChunks) + ":" + String(loraOtaCRC, HEX);
  sendLoRaMessage(command); // Send the command via LoRa

  loraOtaChunkTimeout = millis(); // Set timeout for the first ACK
  loraOtaStatus = "Waiting for slave to confirm OTA_START...";

  Serial.printf("📤 Sent OTA_START command to %s\n", loraOtaTargetSlave.c_str());
  Serial.printf("📊 Command: %s\n", command.c_str());
}

// Sends a message via the LoRa module
void sendLoRaMessage(String message) {
  LoRa.beginPacket(); // Start LoRa packet
  LoRa.print(message); // Add message payload
  LoRa.endPacket(); // Send packet

  Serial.printf("📡 LoRa TX: %s\n", message.c_str());
}

// Sends the next firmware chunk during LoRa OTA
void sendNextChunk() {
  if (loraOtaCurrentChunk >= loraOtaTotalChunks) {
    sendLoRaOTAEnd(); // All chunks sent, send OTA_END command
    return;
  }

  // Construct and send the OTA_CHUNK message
  String chunkMessage = "OTA_CHUNK:" + String(loraOtaCurrentChunk) + ":" + loraOtaChunks[loraOtaCurrentChunk];
  sendLoRaMessage(chunkMessage);

  loraOtaChunkTimeout = millis(); // Reset chunk timeout
  loraOtaLastActivity = millis(); // Update last activity timestamp

  float progressPercent = (float)loraOtaCurrentChunk / loraOtaTotalChunks * 100.0;
  loraOtaStatus = "Sending chunk " + String(loraOtaCurrentChunk + 1) + "/" + String(loraOtaTotalChunks) + " (" + String((int)progressPercent) + "%)";

  Serial.printf("📤 Sent chunk %u/%u (%.1f%%) to %s\n",
                loraOtaCurrentChunk + 1, loraOtaTotalChunks, progressPercent, loraOtaTargetSlave.c_str());
}

// Retries sending the current LoRa OTA chunk if it timed out or NACKed
void retryCurrentChunk() {
  if (loraOtaCurrentChunk < loraOtaTotalChunks) {
    Serial.printf("🔄 Retrying chunk %u\n", loraOtaCurrentChunk + 1);
    sendNextChunk(); // Resend the current chunk
  }
}

// Sends the OTA_END command to the slave device after all chunks are sent
void sendLoRaOTAEnd() {
  sendLoRaMessage("OTA_END:"); // Send OTA_END command
  loraOtaChunkTimeout = millis(); // Set timeout for final ACK/NACK
  loraOtaStatus = "Waiting for slave to verify and flash firmware...";

  Serial.println("📤 Sent OTA_END command, waiting for slave to complete update");
}

// Aborts the current LoRa OTA process
void abortLoRaOTA() {
  Serial.println("❌ Aborting LoRa OTA");

  if (loraOtaInProgress) {
    sendLoRaMessage("OTA_ABORT"); // Notify the slave about the abortion
    loraOtaStatus = "OTA Aborted";
  }

  cleanupLoRaOTA(); // Clean up resources used by LoRa OTA
}

// Cleans up resources allocated for LoRa OTA
void cleanupLoRaOTA() {
  loraOtaInProgress = false;
  loraOtaTargetSlave = "";
  loraOtaStatus = "Ready";
  loraOtaProgress = 0;
  loraOtaTotal = 0;
  loraOtaTotalChunks = 0;
  loraOtaSentChunks = 0;
  loraOtaCurrentChunk = 0;
  loraOtaCRC = 0;

  // Free dynamically allocated memory
  if (loraOtaFirmwareData != nullptr) {
    free(loraOtaFirmwareData);
    loraOtaFirmwareData = nullptr;
  }
  if (loraOtaChunks != nullptr) {
    delete[] loraOtaChunks;
    loraOtaChunks = nullptr;
  }
  if (loraOtaChunkAcked != nullptr) {
    delete[] loraOtaChunkAcked;
    loraOtaChunkAcked = nullptr;
  }

  Serial.println("🧹 LoRa OTA cleanup completed");
}

// Processes acknowledgment (ACK) or negative acknowledgment (NACK) messages from the slave during LoRa OTA
void processLoRaOTAAck(String message) {
  loraOtaLastActivity = millis(); // Update last activity timestamp

  if (message.startsWith("OTA_ACK:")) {
    String ackType = message.substring(8); // Extract ACK type

    if (ackType == "READY") { // Slave is ready to receive firmware
      Serial.println("✅ Slave confirmed ready for OTA, starting chunk transmission");
      loraOtaCurrentChunk = 0; // Start with the first chunk
      sendNextChunk();
    } else if (ackType == "SUCCESS") { // Slave successfully updated
      Serial.println("🎉 LoRa OTA completed successfully!");
      loraOtaStatus = "OTA Completed Successfully!";
      reportLoRaOTAResult(true, "OTA completed successfully"); // Report success to the server
      delay(3000); // Wait a bit before cleaning up
      cleanupLoRaOTA();
    } else if (ackType == "ALL_CHUNKS_RECEIVED") { // Slave received all chunks
      Serial.println("✅ Slave confirmed all chunks received, sending OTA_END");
      sendLoRaOTAEnd(); // Send command to finalize update
    } else if (ackType.length() > 0 && ackType.toInt() >= 0) { // ACK for a specific chunk
      size_t chunkId = ackType.toInt();

      if (chunkId == loraOtaCurrentChunk && !loraOtaChunkAcked[chunkId]) { // Check if it's the expected chunk and not already ACKed
        loraOtaChunkAcked[chunkId] = true;
        loraOtaSentChunks++;
        loraOtaProgress = (loraOtaSentChunks * LORA_CHUNK_SIZE); // Update overall progress
        loraOtaCurrentChunk++; // Move to the next chunk

        Serial.printf("✅ Chunk %u ACKed, progress: %u/%u chunks\n",
                      chunkId + 1, loraOtaSentChunks, loraOtaTotalChunks);

        if (loraOtaCurrentChunk < loraOtaTotalChunks) {
          delay(100); // Small delay before sending the next chunk
          sendNextChunk();
        } else {
          // All chunks ACKed, this case should ideally be handled by ALL_CHUNKS_RECEIVED
          Serial.println("ℹ️ All individual chunks ACKed.");
          sendLoRaOTAEnd();
        }
      } else if (loraOtaChunkAcked[chunkId]) {
        Serial.printf("⚠️ Received duplicate ACK for chunk %u\n", chunkId + 1);
      } else {
         Serial.printf("⚠️ Received ACK for unexpected chunk %u (expected %u)\n", chunkId + 1, loraOtaCurrentChunk + 1);
      }
    }
  } else if (message.startsWith("OTA_NACK:")) { // Negative acknowledgment from slave
    String error = message.substring(9); // Extract error message
    Serial.println("❌ Slave NACK: " + error);

    if (error.startsWith("MISSING_CHUNKS") || error.startsWith("CRC_MISMATCH") || error.startsWith("UPDATE_FAILED")) {
      Serial.println("❌ LoRa OTA failed - " + error);
      loraOtaStatus = "OTA Failed: " + error;
      reportLoRaOTAResult(false, "OTA failed: " + error); // Report failure to the server
      cleanupLoRaOTA();
    } else {
      // For other NACKs (e.g., chunk decode error), retry the current chunk
      retryCurrentChunk();
    }
  }
}

// Reports the result of a LoRa OTA update to the Flask server
void reportLoRaOTAResult(bool success, String message) {
  if (WiFi.status() != WL_CONNECTED) return; // Don't report if not connected to WiFi

  HTTPClient http;
  http.begin(String(FLASK_SERVER) + "/api/lora_ota_result"); // API endpoint for reporting
  http.addHeader("Content-Type", "application/json");

  // Construct JSON payload
  String payload = "{";
  payload += "\"slave_id\":\"" + loraOtaTargetSlave + "\",";
  payload += "\"master_ip\":\"" + WiFi.localIP().toString() + "\",";
  payload += "\"success\":" + String(success ? "true" : "false") + ",";
  payload += "\"message\":\"" + message + "\",";
  payload += "\"duration\":" + String(millis() - loraOtaStartTime); // Duration of the OTA process
  payload += "}";

  int responseCode = http.POST(payload);
  if (responseCode > 0) {
    Serial.printf("📡 LoRa OTA result reported to server - Response: %d\n", responseCode);
  } else {
    Serial.printf("⚠️ Failed to report LoRa OTA result - HTTP Error: %d\n", responseCode);
  }
  http.end();
}

// ============================================
// Enhanced Server Routes for LoRa OTA
// ============================================

// Resets the progress variables for master's OTA update
void resetOTAProgress() {
  otaInProgress = false;
  otaProgress = 0;
  otaTotal = 0;
  otaActive = false;
  otaStatus = "Ready";
  lastOTAActivity = 0;

  // Reinitialize LoRa if it was stopped during master OTA
  if (!LoRa.begin(433E6)) { // Attempt to reinitialize LoRa
    Serial.println("⚠️ LoRa reinitialization failed during OTA reset");
  } else {
    Serial.println("✅ LoRa reinitialized after OTA reset");
    // Re-apply LoRa settings if necessary
    LoRa.setSyncWord(0xF3);
    LoRa.setSpreadingFactor(12);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
  }
}

// Sets up the web server for OTA updates and status display
void setupOTA() {
  server.enableCORS(true); // Enable Cross-Origin Resource Sharing

  // Route for the main web interface page
  server.on("/", []() {
    String html = "<!DOCTYPE html><html><head><title>Enhanced Master Device Control</title>";
    html += "<meta charset='UTF-8'>";
    html += "<meta http-equiv='refresh' content='3'>"; // Auto-refresh page every 3 seconds
    html += "<style>body{font-family:Arial;margin:40px;} button{padding:10px 20px;margin:10px;font-size:16px;}";
    html += ".progress{background:#f0f0f0;border-radius:5px;padding:3px;margin:10px 0;} ";
    html += ".progress-bar{background:#4CAF50;height:25px;border-radius:3px;transition:width 0.3s;text-align:center;line-height:25px;color:white;font-weight:bold;}";
    html += ".status{padding:10px;margin:10px 0;border-radius:5px;} .uploading{background:#fff3cd;border:1px solid #ffeaa7;} .ready{background:#d4edda;border:1px solid #c3e6cb;}";
    html += ".debug{font-size:10px;color:#666;font-family:monospace;} .slave-list{background:#f8f9fa;padding:10px;border-radius:5px;margin:10px 0;}";
    html += "</style></head>";
    html += "<body><h1>Enhanced Master Device Control Panel</h1>";

    // Display device status
    html += "<div class='status " + String((otaInProgress || loraOtaInProgress) ? "uploading" : "ready") + "'>";
    html += "<p><strong>Master Status:</strong> " + otaStatus + "</p>";
    html += "<p><strong>LoRa OTA Status:</strong> " + loraOtaStatus + "</p>";
    html += "<p><strong>WiFi:</strong> Connected</p>";
    html += "<p><strong>IP Address:</strong> " + WiFi.localIP().toString() + "</p>";
    html += "<p><strong>Free Heap:</strong> " + String(ESP.getFreeHeap()) + " bytes</p>";

    // Display debug information
    html += "<div class='debug'>";
    html += "Master OTA: InProgress=" + String(otaInProgress ? "true" : "false");
    html += ", Progress=" + String(otaProgress) + "/" + String(otaTotal) + "<br>";
    html += "LoRa OTA: InProgress=" + String(loraOtaInProgress ? "true" : "false");
    html += ", Target=" + loraOtaTargetSlave;
    html += ", Chunks=" + String(loraOtaSentChunks) + "/" + String(loraOtaTotalChunks);
    html += "</div>";
    html += "</div>";

    // Display list of connected slaves
    html += "<div class='slave-list'>";
    html += "<h3>📡 Connected LoRa Slaves (" + String(slaveCount) + ")</h3>";
    if (slaveCount > 0) {
      for (int i = 0; i < slaveCount; i++) {
        html += "<p><strong>" + slaves[i].id + "</strong> - ";
        html += "People: " + String(slaves[i].peopleCount) + ", ";
        html += "Heap: " + String(slaves[i].freeHeap) + " bytes, ";
        unsigned long seenAgo = (millis() - slaves[i].lastSeen.toInt()) / 1000;
        html += "Last Seen: " + String(seenAgo) + "s ago (RSSI: " + String(slaves[i].signalStrength) + ")</p>";
      }
    } else {
      html += "<p>No LoRa slaves currently connected</p>";
    }
    html += "</div>";

    // Display progress for master OTA update
    if (otaInProgress || otaActive) {
      int progressPercent = otaTotal > 0 ? (otaProgress * 100) / otaTotal : 0;
      html += "<hr><h3 style='color:#ff6600;'>🔄 Master OTA Update in Progress</h3>";
      html += "<div class='progress'>";
      html += "<div class='progress-bar' style='width:" + String(progressPercent) + "%'>";
      html += String(progressPercent) + "%";
      html += "</div></div>";
      html += "<p style='color:red;'><strong>⚠️ DO NOT POWER OFF THE DEVICE</strong></p>";
    }

    // Display progress for LoRa OTA update
    if (loraOtaInProgress) {
      int loraProgressPercent = loraOtaTotalChunks > 0 ? (loraOtaSentChunks * 100) / loraOtaTotalChunks : 0;
      html += "<hr><h3 style='color:#ff6600;'>📡 LoRa OTA to " + loraOtaTargetSlave + "</h3>";
      html += "<div class='progress'>";
      html += "<div class='progress-bar' style='width:" + String(loraProgressPercent) + "%'>";
      html += String(loraProgressPercent) + "%";
      html += "</div></div>";
      html += "<p><strong>Status:</strong> " + loraOtaStatus + "</p>";
      html += "<p><strong>Progress:</strong> " + String(loraOtaSentChunks) + "/" + String(loraOtaTotalChunks) + " chunks</p>";
      html += "<button onclick='abortLoRaOTA()' style='background:red;color:white;'>Abort LoRa OTA</button>"; // Button to abort LoRa OTA
    }

    // Display OTA update buttons if no OTA is in progress
    if (!otaInProgress && !loraOtaInProgress) {
      html += "<hr>";
      html += "<h2>OTA Updates</h2>";
      html += "<p><a href='/update'><button>Update Master Firmware</button></a></p>"; // Link to ElegantOTA page
      html += "<p><a href='/trigger_slave_ota'><button>Trigger Slave WiFi OTA</button></a></p>"; // Button to trigger slave OTA
    }

    html += "<hr>";
    html += "<h2>System Status</h2>";
    html += "<p>Last UID Received: <span id='lastUID'>" + receivedUID + "</span></p>"; // Display last received UID

    // JavaScript for abort button and dynamic UID update
    if (!otaInProgress && !loraOtaInProgress) {
      html += "<script>";
      html += "function abortLoRaOTA() {";
      html += "  fetch('/abort_lora_ota', {method: 'POST'}).then(() => location.reload());"; // Abort LoRa OTA via POST request
      html += "}";
      html += "setInterval(function(){"; // Periodically fetch status
      html += "fetch('/status').then(r=>r.json()).then(d=>{";
      html += "document.getElementById('lastUID').innerText=d.lastUID;"; // Update last UID display
      html += "});}, 3000);";
      html += "</script>";
    }

    html += "</body></html>";

    server.sendHeader("Cache-Control", "no-cache"); // Prevent caching of the page
    server.send(200, "text/html", html);
  });

  // Route to provide system status as JSON
  server.on("/status", []() {
    String json = "{";
    json += "\"lastUID\":\"" + receivedUID + "\",";
    json += "\"uptime\":" + String(millis()) + ",";
    json += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";
    json += "\"otaInProgress\":" + String(otaInProgress ? "true" : "false") + ",";
    json += "\"otaActive\":" + String(otaActive ? "true" : "false") + ",";
    json += "\"otaProgress\":" + String(otaProgress) + ",";
    json += "\"otaTotal\":" + String(otaTotal) + ",";
    json += "\"otaPercent\":" + String(otaTotal > 0 ? (otaProgress * 100) / otaTotal : 0) + ",";
    json += "\"otaStatus\":\"" + otaStatus + "\",";
    json += "\"loraOtaInProgress\":" + String(loraOtaInProgress ? "true" : "false") + ",";
    json += "\"loraOtaStatus\":\"" + loraOtaStatus + "\",";
    json += "\"loraOtaTarget\":\"" + loraOtaTargetSlave + "\",";
    json += "\"loraOtaProgress\":" + String(loraOtaSentChunks) + ",";
    json += "\"loraOtaTotal\":" + String(loraOtaTotalChunks) + ",";
    json += "\"loraOtaPercent\":" + String(loraOtaTotalChunks > 0 ? (loraOtaSentChunks * 100) / loraOtaTotalChunks : 0) + ",";
    json += "\"connectedSlaves\":" + String(slaveCount);
    json += "}";

    server.sendHeader("Access-Control-Allow-Origin", "*"); // Allow cross-origin requests
    server.sendHeader("Cache-Control", "no-cache");
    server.send(200, "application/json", json);
  });

  // Endpoint for Flask server to push LoRa OTA firmware
  server.on("/lora_ota_push", HTTP_POST, []() {
    if (loraOtaInProgress) {
      server.send(409, "application/json", "{\"error\":\"LoRa OTA already in progress\"}"); // Conflict if OTA already running
      return;
    }

    String body = server.arg("plain"); // Get JSON payload from request body
    DynamicJsonDocument doc(8192); // Allocate JSON document (adjust size as needed)

    if (deserializeJson(doc, body) != DeserializationError::Ok) {
      server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}"); // Bad request if JSON is invalid
      return;
    }

    // Extract parameters from JSON
    String slaveId = doc["slave_id"];
    String firmwareB64 = doc["firmware_data"];
    size_t firmwareSize = doc["firmware_size"];
    uint32_t firmwareCRC = doc["firmware_crc32"];

    if (slaveId.length() == 0 || firmwareB64.length() == 0 || firmwareSize == 0) {
      server.send(400, "application/json", "{\"error\":\"Missing required parameters\"}"); // Bad request if parameters are missing
      return;
    }

    Serial.printf("📥 Received LoRa OTA request for slave %s (%u bytes)\n", slaveId.c_str(), firmwareSize);

    handleLoRaOTAPush(slaveId, firmwareB64, firmwareSize, firmwareCRC); // Start the LoRa OTA process

    // Send response confirming initiation
    String response = "{";
    response += "\"message\":\"LoRa OTA initiated\",";
    response += "\"slave_id\":\"" + slaveId + "\",";
    response += "\"firmware_size\":" + String(firmwareSize) + ",";
    response += "\"total_chunks\":" + String(loraOtaTotalChunks) + ",";
    response += "\"status\":\"started\"";
    response += "}";

    server.send(200, "application/json", response);
  });

  // Endpoint to abort an ongoing LoRa OTA update
  server.on("/abort_lora_ota", HTTP_POST, []() {
    if (loraOtaInProgress) {
      abortLoRaOTA();
      server.send(200, "text/plain", "LoRa OTA aborted");
    } else {
      server.send(200, "text/plain", "No LoRa OTA in progress");
    }
  });

  // Endpoint to initialize LoRa OTA by fetching firmware from a URL (e.g., Flask server)
  server.on("/init_lora_ota", HTTP_POST, []() {
    if (loraOtaInProgress) {
      server.send(409, "application/json", "{\"error\":\"LoRa OTA already in progress\"}");
      return;
    }

    String body = server.arg("plain");
    DynamicJsonDocument doc(4096); // JSON document for request parameters

    if (deserializeJson(doc, body) != DeserializationError::Ok) {
      server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
      return;
    }

    // Extract parameters from JSON
    String slaveId = doc["slave_id"];
    // String slaveIp = doc["slave_ip"]; // Not currently used but available
    size_t firmwareSize = doc["firmware_size"];
    uint32_t firmwareCRC = doc["firmware_crc32"];
    // String filename = doc["filename"]; // Not currently used
    String flaskServerUrl = doc["flask_server"]; // Base URL of the server hosting the firmware
    String firmwarePath = doc["firmware_path"]; // Path to the firmware file on the server

    if (slaveId.length() == 0 || firmwareSize == 0 || flaskServerUrl.length() == 0 || firmwarePath.length() == 0) {
      server.send(400, "application/json", "{\"error\":\"Missing required parameters\"}");
      return;
    }

    Serial.printf("📥 Received LoRa OTA init request for slave %s (%u bytes)\n", slaveId.c_str(), firmwareSize);
    Serial.printf("📡 Will fetch firmware from: %s%s\n", flaskServerUrl.c_str(), firmwarePath.c_str());

    HTTPClient http;
    String firmwareUrl = flaskServerUrl + firmwarePath; // Full URL to the firmware file
    http.begin(firmwareUrl);
    http.setTimeout(15000); // 15-second timeout for the HTTP request

    Serial.printf("📡 Attempting to download firmware from: %s\n", firmwareUrl.c_str());
    Serial.printf("📊 WiFi status: %s\n", WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
    Serial.printf("📊 Free heap before download: %u bytes\n", ESP.getFreeHeap());

    int httpCode = http.GET(); // Perform GET request
    Serial.printf("📡 HTTP response code: %d\n", httpCode);

    if (httpCode == HTTP_CODE_OK) { // Check for successful response
      int contentLength = http.getSize(); // Get firmware size from HTTP headers
      if (contentLength > 0 && (size_t)contentLength != firmwareSize) {
        Serial.printf("⚠️ Size mismatch: expected %u, got %d from HTTP header. Using expected size.\n", firmwareSize, contentLength);
      } else if (contentLength <= 0) {
        Serial.printf("⚠️ Could not get content length from HTTP header, using provided size: %u\n", firmwareSize);
      }


      // Estimate memory needed: firmwareSize for binary, ~1.33*firmwareSize for Base64, plus buffer
      size_t estimatedMemoryNeeded = firmwareSize + (firmwareSize * 4 / 3) + 10240; // Add 10KB buffer
      size_t freeHeap = ESP.getFreeHeap();
      Serial.printf("📊 Available heap: %u bytes, estimated needed: %u bytes\n", freeHeap, estimatedMemoryNeeded);

      if (freeHeap < estimatedMemoryNeeded) {
        Serial.println("❌ Not enough memory for firmware processing");
        server.send(500, "application/json", "{\"error\":\"Insufficient memory for firmware processing\"}");
        http.end();
        return;
      }

      WiFiClient* stream = http.getStreamPtr(); // Get stream to read response body
      String firmwareB64 = "";

      uint8_t* buffer = (uint8_t*)malloc(firmwareSize); // Allocate buffer for firmware binary
      if (buffer != nullptr) {
        size_t bytesRead = stream->readBytes(buffer, firmwareSize); // Read firmware into buffer

        if (bytesRead == firmwareSize) {
          firmwareB64 = encodeBase64(buffer, firmwareSize); // Encode binary to Base64
          free(buffer); // Free the binary buffer

          Serial.printf("✅ Firmware downloaded and encoded (%u bytes -> %u B64 chars)\n",
                        firmwareSize, firmwareB64.length());

          handleLoRaOTAPush(slaveId, firmwareB64, firmwareSize, firmwareCRC); // Start LoRa OTA

          String response = "{";
          response += "\"message\":\"LoRa OTA initiated with downloaded firmware\",";
          response += "\"slave_id\":\"" + slaveId + "\",";
          response += "\"firmware_size\":" + String(firmwareSize) + ",";
          response += "\"total_chunks\":" + String(loraOtaTotalChunks) + ",";
          response += "\"status\":\"started\"";
          response += "}";
          server.send(200, "application/json", response);
        } else {
          free(buffer);
          Serial.printf("❌ Failed to download complete firmware. Read %u bytes, expected %u.\n", bytesRead, firmwareSize);
          server.send(500, "application/json", "{\"error\":\"Failed to download complete firmware\"}");
        }
      } else {
        Serial.println("❌ Failed to allocate memory for firmware buffer");
        server.send(500, "application/json", "{\"error\":\"Failed to allocate memory for firmware\"}");
      }
    } else {
      Serial.printf("❌ Failed to download firmware: HTTP %d\n", httpCode);
      String errorPayload = http.getString();
      Serial.printf("📄 Server error response: %s\n", errorPayload.c_str());
      server.send(500, "application/json", "{\"error\":\"Failed to download firmware from Flask server\", \"http_code\":" + String(httpCode) + "}");
    }
    http.end(); // Close HTTP connection
  });

  // Endpoint to provide OTA progress information as JSON
  server.on("/ota_progress", []() {
    lastOTAActivity = millis(); // Update last OTA activity timestamp

    int progressPercent = otaTotal > 0 ? (otaProgress * 100) / otaTotal : 0;
    String json = "{";
    json += "\"inProgress\":" + String(otaInProgress ? "true" : "false") + ",";
    json += "\"active\":" + String(otaActive ? "true" : "false") + ",";
    json += "\"progress\":" + String(otaProgress) + ",";
    json += "\"total\":" + String(otaTotal) + ",";
    json += "\"percent\":" + String(progressPercent) + ",";
    json += "\"status\":\"" + otaStatus + "\",";
    json += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";
    json += "\"startTime\":" + String(otaStartTime) + ",";
    json += "\"runtime\":" + String(millis() - otaStartTime) + ",";
    json += "\"loraOtaInProgress\":" + String(loraOtaInProgress ? "true" : "false") + ",";
    json += "\"loraOtaStatus\":\"" + loraOtaStatus + "\"";
    json += "}";

    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Cache-Control", "no-cache");
    server.send(200, "application/json", json);

    // Serial.printf("📊 Progress Query - Master: %d%%, LoRa: %s\n",
    // progressPercent, loraOtaInProgress ? "Active" : "Idle");
  });

  // Route to reboot the master device
  server.on("/reboot", HTTP_GET, []() {
    server.send(200, "text/plain", "Rebooting...");
    delay(100); // Allow time for response to be sent
    ESP.restart(); // Restart the ESP32
  });

  // Route to trigger WiFi OTA mode on slave devices via LoRa
  server.on("/trigger_slave_ota", HTTP_GET, []() {
    triggerSlaveOTA();
    server.send(200, "text/plain", "Slave OTA triggered");
  });

  // Initialize ElegantOTA library for web-based firmware updates
  ElegantOTA.begin(&server); // Pass the WebServer instance to ElegantOTA
  ElegantOTA.setAutoReboot(true); // Automatically reboot after successful OTA

  // Callback function when ElegantOTA starts
  ElegantOTA.onStart([]() {
    Serial.println("=======================================");
    Serial.println("🔄 ElegantOTA Update Started (Web Interface)");
    Serial.println("=======================================");

    otaInProgress = true;
    otaActive = true;
    otaProgress = 0;
    otaTotal = 0; // Total size will be known later
    otaStatus = "ElegantOTA Starting...";
    otaStartTime = millis();
    lastOTAActivity = millis();

    Serial.printf("📊 Free heap before update: %d bytes\n", ESP.getFreeHeap());
    Serial.println("⚠️  Stopping LoRa to free memory for OTA...");

    LoRa.end(); // End LoRa communication to free up SPI pins and memory
    Serial.println("✅ Ready for firmware upload");
  });

  // Callback function for ElegantOTA progress
  ElegantOTA.onProgress([](size_t current, size_t final_val) { // Renamed 'final' to 'final_val' to avoid conflict
    otaInProgress = true; // Ensure flag is set
    otaActive = true;     // Ensure flag is set
    otaProgress = current;
    lastOTAActivity = millis();

    if (otaTotal == 0 && final_val > 0) { // Set total size if not already set
      otaTotal = final_val;
    } else if (final_val > 0) { // Update total size if it changes (should not happen often)
        otaTotal = final_val;
    }


    static unsigned long lastPrint = 0;
    static int lastPercent = -1;
    unsigned long now = millis();

    int currentPercent = final_val > 0 ? (current * 100) / final_val : 0;
    otaStatus = "Uploading firmware... " + String(currentPercent) + "%";

    // Print progress to serial periodically or on significant change
    if ((now - lastPrint > 1000) || (currentPercent >= lastPercent + 5)) {
      if (final_val > 0) {
        Serial.printf("📤 ElegantOTA Progress: %d%% (%u/%u bytes)\n",
                     currentPercent, (unsigned int)current, (unsigned int)final_val);
      }
      lastPrint = now;
      lastPercent = currentPercent;
    }
  });

  // Callback function when ElegantOTA ends
  ElegantOTA.onEnd([](bool success) {
    if (success) {
      otaProgress = otaTotal; // Mark progress as complete
      otaStatus = "Update Successful! Rebooting...";
      Serial.println("✅ ElegantOTA Update Successful!");
      delay(3000); // Short delay before auto-reboot
      // ElegantOTA handles reboot if setAutoReboot(true)
    } else {
      Serial.println("❌ ElegantOTA Update Failed!");
      resetOTAProgress(); // Reset OTA state
      delay(2000);
      initializeLoRa(); // Re-initialize LoRa as it was stopped
      otaStatus = "Ready (Previous update failed)";
    }
    // otaInProgress and otaActive will be reset by resetOTAProgress or implicitly by reboot
  });

  // Endpoint for firmware updates pushed from Flask server (alternative to ElegantOTA)
  server.on("/update", HTTP_POST,
    []() { // This is the onEnd handler for the upload
      server.sendHeader("Connection", "close"); // Close connection after response

      if (Update.hasError()) {
        resetOTAProgress(); // Reset OTA state on error
        otaStatus = "Flask Update Failed!";
        server.send(500, "text/plain", "Update Failed!");
        Serial.println("❌ Flask OTA Update Failed!");
        Update.printError(Serial); // Print detailed error to serial
      } else {
        otaStatus = "Flask Update Successful! Rebooting...";
        server.send(200, "text/plain", "Update Success!");
        Serial.println("✅ Flask OTA Update Successful!");
        delay(1000); // Allow time for response
        ESP.restart(); // Reboot the device
      }
    },
    []() { // This is the onUpload handler (receives file data)
      HTTPUpload& upload = server.upload(); // Get upload object

      if (upload.status == UPLOAD_FILE_START) {
        Serial.println("=======================================");
        Serial.printf("🔄 Flask firmware update started: %s\n", upload.filename.c_str());
        Serial.println("=======================================");

        otaInProgress = true;
        otaActive = true;
        otaProgress = 0;

        if (upload.totalSize > 0) { // Get total size from HTTP headers if available
          otaTotal = upload.totalSize;
          Serial.printf("📦 Total size from upload header: %u bytes\n", upload.totalSize);
        } else {
          otaTotal = 0; // Total size unknown, will estimate
          Serial.println("⚠️ Total size not provided in upload header, will estimate during upload");
        }

        otaStatus = "Starting Flask Upload...";
        otaStartTime = millis();
        lastOTAActivity = millis();

        Serial.printf("💾 Free heap: %d bytes\n", ESP.getFreeHeap());

        LoRa.end(); // Stop LoRa to free resources
        Serial.println("⚠️ LoRa stopped to free memory");

        // Begin firmware update process
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { // Use UPDATE_SIZE_UNKNOWN if total size is not definite
          Update.printError(Serial);
          Serial.println("❌ Update.begin() failed!");
          otaStatus = "Update initialization failed!";
        } else {
          Serial.println("✅ Update initialized successfully");
          otaStatus = "Receiving firmware data from Flask...";
        }

      } else if (upload.status == UPLOAD_FILE_WRITE) { // Firmware data chunk received
        otaProgress += upload.currentSize; // Update current progress
        lastOTAActivity = millis();

        // Estimate total size if not known, for progress display
        if (otaTotal == 0) {
          if (otaProgress > 1000000) otaTotal = otaProgress + 200000; // Rough estimation
          else if (otaProgress > 500000) otaTotal = otaProgress + 500000;
          else otaTotal = 1200000; // Default guess
          Serial.printf("📊 Estimated total size: %u bytes\n", otaTotal);
        }
        // Adjust estimate if progress gets too close to current total
        if (otaTotal > 0 && otaProgress > (otaTotal * 0.9)) {
            otaTotal = otaProgress + 100000; // Add a buffer to the estimate
            Serial.printf("📈 Adjusted total size estimate: %u bytes\n", otaTotal);
        }


        // Write received chunk to flash
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
          Serial.println("❌ Update.write() failed!");
          otaStatus = "Write operation failed!";
        } else {
          int percent = otaTotal > 0 ? (otaProgress * 100) / otaTotal : 0;
          if (percent > 99 && upload.status == UPLOAD_FILE_WRITE) percent = 99; // Don't show 100% until UPLOAD_FILE_END

          otaStatus = "Writing from Flask... " + String(percent) + "%";

          // Print progress periodically
          static unsigned long lastProgressPrint = 0;
          if (millis() - lastProgressPrint > 1000) {
            Serial.printf("📝 Flask Writing: %d%% (%u/%u bytes)\n", percent, (unsigned int)otaProgress, (unsigned int)otaTotal);
            lastProgressPrint = millis();
          }
        }

      } else if (upload.status == UPLOAD_FILE_END) { // Upload finished
        // Update progress and total with actual final size
        otaProgress = upload.totalSize > 0 ? upload.totalSize : otaProgress;
        otaTotal = otaProgress;
        lastOTAActivity = millis();

        Serial.printf("📦 Final upload size: %u bytes\n", (unsigned int)otaProgress);

        // Finalize the update
        if (Update.end(true)) { // true to commit the update
          Serial.printf("✅ Flask firmware update successful: %u bytes\n", (unsigned int)otaProgress);
          otaStatus = "Flask update complete! Rebooting...";
        } else {
          Update.printError(Serial);
          Serial.println("❌ Update.end() failed!");
          otaStatus = "Update finalization failed!";
          resetOTAProgress(); // Reset OTA state on failure
        }
      }
    }
  );

  server.begin(); // Start the web server

  Serial.println("=======================================");
  Serial.println("🚀 Enhanced Master with LoRa OTA Server started");
  Serial.println("=======================================");
  Serial.printf("💾 Free heap: %d bytes\n", ESP.getFreeHeap());
  Serial.print("🌐 Access Control Panel at: http://");
  Serial.println(WiFi.localIP());
  Serial.print("🔧 Access ElegantOTA at: http://");
  Serial.println(WiFi.localIP().toString() + "/update");
  Serial.print("📊 Access OTA Progress at: http://");
  Serial.println(WiFi.localIP().toString() + "/ota_progress");
  Serial.println("=======================================");
}

// ============================================
// Enhanced LoRa Communication
// ============================================

// Receives and processes data packets from LoRa
void receiveLoRaData() {
  int packetSize = LoRa.parsePacket();

  if (packetSize) {
    String receivedData = "";
    while (LoRa.available()) {
      receivedData += (char)LoRa.read();
    }

    Serial.printf("📨 LoRa RX (%d bytes, RSSI: %d): %s\n", packetSize, LoRa.packetRssi(), receivedData.c_str());

    // Process different types of LoRa messages based on prefix
    if (receivedData.startsWith("HEARTBEAT:")) {
      processSlaveHeartbeat(receivedData);
    } else if (receivedData.startsWith("OTA_ACK:") || receivedData.startsWith("OTA_NACK:")) {
      if (loraOtaInProgress) {
        processLoRaOTAAck(receivedData);
      }
    } else if (receivedData.startsWith("OTA_READY:")) {
      String slaveIP = receivedData.substring(10);
      Serial.println("✅ Slave OTA Ready at IP: " + slaveIP);
      Serial.println("🔗 Access slave OTA at: http://" + slaveIP + "/update");
    } else if (receivedData.startsWith("UID:")) {
      // Extract UID and add to queue
      String uid = receivedData.substring(4);
      Serial.println("🏷️ UID Message Received: " + uid);
      addToQueue(uid);
      sendAckToSender(uid);
    } else {
      // Handle legacy UID format (direct UID without prefix)
      if (receivedData.length() > 2 && receivedData.length() < 20) {
        Serial.println("🏷️ Legacy UID Format Detected: " + receivedData);
        addToQueue(receivedData);
        sendAckToSender(receivedData);
      } else {
        Serial.println("⚠️ Unknown LoRa message format: " + receivedData);
      }
    }
  }
}

void addToQueue(String uid) {
  Serial.println("➕ Adding to UID queue: " + uid);
  
  // Check if the queue is full
  if (queueCount >= MAX_QUEUE_SIZE) {
    Serial.println("⚠️ UID Queue full! Discarding oldest UID");
    // Remove the oldest UID by advancing the head
    queueHead = (queueHead + 1) % MAX_QUEUE_SIZE;
    queueCount--;
  }
  
  // Add the new UID to the queue
  uidQueue[queueTail] = uid;
  queueTail = (queueTail + 1) % MAX_QUEUE_SIZE;
  queueCount++;
  
  Serial.printf("📊 Queue status: %d UIDs pending (head: %d, tail: %d)\n", 
                queueCount, queueHead, queueTail);
}

// Processes heartbeat messages received from slave devices
void processSlaveHeartbeat(String heartbeat) {
  // Expected format: HEARTBEAT:SLAVE-ID:peopleCount:freeHeap
  int firstColon = heartbeat.indexOf(':', 10); // Find first colon after "HEARTBEAT:"
  int secondColon = heartbeat.indexOf(':', firstColon + 1); // Find second colon
  int thirdColon = heartbeat.indexOf(':', secondColon + 1); // Find third colon (optional, for backward compatibility)

  if (firstColon == -1 || secondColon == -1) {
      Serial.println("⚠️ Invalid heartbeat format: " + heartbeat);
      return;
  }

  String slaveId = heartbeat.substring(10, firstColon);
  int peopleCount = heartbeat.substring(firstColon + 1, secondColon).toInt();
  uint32_t freeHeap;

  if (thirdColon != -1) { // New format with freeHeap
    freeHeap = heartbeat.substring(secondColon + 1, thirdColon).toInt();
  } else { // Old format without freeHeap (or if freeHeap is the last part)
    freeHeap = heartbeat.substring(secondColon + 1).toInt();
  }
  
  addOrUpdateSlave(slaveId, peopleCount, freeHeap, LoRa.packetRssi()); // Update slave list

  Serial.printf("💓 Slave heartbeat: %s (People: %d, Heap: %u, RSSI: %d)\n",
                slaveId.c_str(), peopleCount, freeHeap, LoRa.packetRssi());

  reportSlaveHeartbeat(slaveId, peopleCount, freeHeap, LoRa.packetRssi()); // Report to Flask server
}

// Reports slave heartbeat data to the Flask server with consistent naming
void reportSlaveHeartbeat(String slaveId, int peopleCount, uint32_t freeHeap, int signalStrength) {
  if (WiFi.status() != WL_CONNECTED) return; // Don't report if not connected to WiFi

  HTTPClient http;
  http.begin(String(FLASK_SERVER) + "/api/lora_slave_heartbeat"); 
  http.addHeader("Content-Type", "application/json");
  
  // Extract an index from the slaveId to create consistent device names
  int slaveIndex = 0;
  String slaveIP = "unknown";
  
  // Parse the slaveId to extract a meaningful index
  if (slaveId.startsWith("SLAVE-")) {
    // Extract the part after "SLAVE-"
    String identifier = slaveId.substring(6);
    
    // Check if this is an IP-based ID (numeric)
    if (identifier.length() >= 8 && isDigit(identifier[0])) {
      // For IP-based IDs (SLAVE-192168xxx), use the last octet as index
      slaveIndex = identifier.substring(identifier.length() - 3).toInt();
      
      // Try to reconstruct the IP from the ID (if it follows expected format)
      if (identifier.length() >= 12) {
        String ip1 = identifier.substring(0, 3);
        String ip2 = identifier.substring(3, 6);
        String ip3 = identifier.substring(6, 9);
        String ip4 = identifier.substring(9);
        
        // Only reconstruct if parts look valid
        if (ip1.toInt() <= 255 && ip2.toInt() <= 255 && 
            ip3.toInt() <= 255 && ip4.toInt() <= 255) {
          slaveIP = ip1 + "." + ip2 + "." + ip3 + "." + ip4;
        }
      }
    } 
    // Check if it's a MAC-based ID (hex)
    else {
      // For MAC-based IDs, use a hash of the identifier as index
      for (unsigned int i = 0; i < identifier.length(); i++) {
        slaveIndex = (slaveIndex * 31 + identifier[i]) % 100;
      }
      slaveIndex = max(1, slaveIndex); // Ensure non-zero index
    }
  }
  
  // If we couldn't determine an index, use a hash of the entire slaveId
  if (slaveIndex == 0) {
    for (unsigned int i = 0; i < slaveId.length(); i++) {
      slaveIndex = (slaveIndex * 31 + slaveId[i]) % 100;
    }
    slaveIndex = max(1, slaveIndex); // Ensure non-zero index
  }

  // Create consistent device name with index
  String deviceName = "ESP32-Slave-" + String(slaveIndex);

  // Construct JSON payload with consistent naming
  String payload = "{";
  payload += "\"slave_data\":{";
  payload += "\"slave_id\":\"" + deviceName + "\",";
  payload += "\"original_id\":\"" + slaveId + "\",";
  payload += "\"slave_ip\":\"" + slaveIP + "\",";
  payload += "\"slave_index\":" + String(slaveIndex) + ",";
  payload += "\"people_count\":" + String(peopleCount) + ",";
  payload += "\"free_heap\":" + String(freeHeap) + ",";
  payload += "\"signal_strength\":" + String(signalStrength) + ",";
  payload += "\"firmware_version\":\"v1.0\"";
  payload += "}";
  payload += "}";

  int responseCode = http.POST(payload);
  if (responseCode == HTTP_CODE_OK) {
    Serial.printf("📡 Slave heartbeat reported to server: %s (Index: %d)\n", 
                deviceName.c_str(), slaveIndex);
  } else {
    Serial.printf("⚠️ Failed to report slave heartbeat - HTTP Response: %d\n", responseCode);
  }
  http.end();
}

// ============================================
// Utility Functions (Base64 decoder)
// ============================================

// Decodes a Base64 encoded string into binary data
uint8_t* base64_decode(const char* input, size_t input_length, size_t* output_length) {
  const char base64_chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  if (input_length % 4 != 0) { // Base64 string length must be a multiple of 4
      *output_length = 0;
      return nullptr;
  }

  // Calculate output length
  *output_length = input_length / 4 * 3;
  if (input[input_length - 1] == '=') (*output_length)--; // Adjust for padding
  if (input[input_length - 2] == '=') (*output_length)--;

  uint8_t* output = (uint8_t*)malloc(*output_length); // Allocate memory for decoded data
  if (output == nullptr) {
      *output_length = 0;
      return nullptr; // Memory allocation failed
  }

  for (size_t i = 0, j = 0; i < input_length;) {
    // Get values of 4 Base64 characters
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
// Original Functions (Enhanced)
// ============================================

// Sends a heartbeat message from the master device to the Flask server
void sendHeartbeat() {
  if (WiFi.status() != WL_CONNECTED) return; // Don't send if not connected to WiFi

  HTTPClient http;
  http.begin(String(FLASK_SERVER) + "/api/device_heartbeat"); // API endpoint for heartbeat
  http.addHeader("Content-Type", "application/json");

  // Calculate total people count from all connected slaves
  int totalPeopleCount = 0;
  if (slaveCount > 0) {
    for (int i = 0; i < slaveCount; i++) {
      totalPeopleCount += slaves[i].peopleCount;
    }
    if (totalPeopleCount == 0 && slaveCount > 0) totalPeopleCount = 1; // Ensure at least 1 if slaves are present but count is 0
  } else {
    totalPeopleCount = 2;  // Default for testing if no slaves
  }

  String deviceName = "ESP32-Master"; // Identifier for the master device
  // Construct JSON payload
  String payload = "{";
  payload += "\"device_name\":\"" + deviceName + "\",";
  payload += "\"device_type\":\"master\",";
  payload += "\"free_heap\":" + String(ESP.getFreeHeap()) + ",";
  payload += "\"connected_slaves\":" + String(slaveCount) + ",";
  payload += "\"peopleCount\":" + String(totalPeopleCount) + ",";
  payload += "\"lora_ota_active\":" + String(loraOtaInProgress ? "true" : "false");
  payload += "}";

  int responseCode = http.POST(payload);
  if (responseCode == HTTP_CODE_OK) {
    Serial.println("💓 Master heartbeat sent to server");
  } else {
    Serial.printf("⚠️ Master heartbeat failed - HTTP Response: %d\n", responseCode);
  }
  http.end();
}

// Connects the ESP32 to the configured WiFi network
void connectToWiFi() {
  Serial.print("📡 Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // Start WiFi connection

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) { // Retry for up to 30 seconds
    delay(1000);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi connected!");
    Serial.print("📡 IP Address: "); Serial.println(WiFi.localIP());
    Serial.print("🌐 Gateway: "); Serial.println(WiFi.gatewayIP());
    Serial.print("🔍 DNS: "); Serial.println(WiFi.dnsIP());
  } else {
    Serial.println("\n❌ WiFi connection failed! Restarting...");
    delay(1000);
    ESP.restart(); // Restart if connection fails persistently
  }
}

// Initializes the LoRa module with specified parameters
void initializeLoRa() {
  LoRa.setPins(LORA_SS_PIN, LORA_RST_PIN, LORA_DIO0_PIN); // Set LoRa pins

  if (!LoRa.begin(433E6)) { // Initialize LoRa at 433MHz
    Serial.println("❌ LoRa initialization failed! Halting.");
    while (1); // Halt execution if LoRa fails
  }

  // Configure LoRa parameters
  LoRa.setSyncWord(0xF3); // Set custom sync word
  LoRa.setSpreadingFactor(12); // Set spreading factor (higher for longer range, lower for faster data rate)
  LoRa.setSignalBandwidth(125E3); // Set signal bandwidth (125kHz)
  LoRa.setCodingRate4(5); // Set coding rate (4/5)

  Serial.println("✅ LoRa initialized successfully!");
}

// Sends an acknowledgment (ACK) message back to the sender of a UID via LoRa
void sendAckToSender(String uid) {
  LoRa.beginPacket();
  LoRa.print("ACK:" + uid); // ACK message format: ACK:<UID>
  LoRa.endPacket();

  Serial.println("📤 Sent ACK to sender node for UID: " + uid);
}

// Triggers WiFi OTA mode on slave devices by sending a LoRa command
void triggerSlaveOTA() {
  Serial.println("📡 Triggering slave WiFi OTA mode...");

  LoRa.beginPacket();
  LoRa.print("OTA_MODE"); // Command to instruct slaves to enter WiFi OTA mode
  LoRa.endPacket();

  Serial.println("📤 OTA_MODE command sent to slaves via LoRa");
}

// Placeholder function to check for OTA commands from an external source (e.g., server)
void checkForOTACommand() {
  // This function can be expanded to fetch commands from the Flask server
  // or listen for specific triggers to initiate OTA updates.
  // Currently, LoRa OTA is primarily initiated via the /lora_ota_push or /init_lora_ota HTTP endpoints.
  // Serial.println("ℹ️ Checking for external OTA commands (placeholder)...");
}

// Sends a received UID and other data to the backend server
void sendToServer(String uid) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ WiFi not connected! Cannot send UID to server.");
    return;
  }

  HTTPClient http;
  http.begin(SERVER_URL); // API endpoint for registering attendance
  http.addHeader("Content-Type", "application/json");

  // Calculate total people count from connected slaves
  int totalPeopleCount = 0;
  if (slaveCount > 0) {
    for (int i = 0; i < slaveCount; i++) {
      totalPeopleCount += slaves[i].peopleCount;
    }
    if (totalPeopleCount == 0 && slaveCount > 0) totalPeopleCount = 1; // Default if slaves exist but count is 0
  } else {
    totalPeopleCount = 2; // Default for testing if no slaves
  }

  // Construct JSON payload
  String jsonPayload = "{\"uid\":\"" + uid +
                       "\",\"device_name\":\"ESP32-Master\"," +
                       "\"device_type\":\"master\"," +
                       "\"peopleCount\":" + String(totalPeopleCount) + "}";

  Serial.print("📤 Sending to server: ");
  Serial.println(jsonPayload);

  int httpResponseCode = http.POST(jsonPayload); // Send POST request

  if (httpResponseCode > 0) {
    Serial.print("✅ Server response code: ");
    Serial.println(httpResponseCode);
    String response = http.getString(); // Get server response
    Serial.print("📨 Server response: ");
    Serial.println(response);
  } else {
    Serial.print("❌ Error in HTTP request to server: ");
    Serial.println(httpResponseCode);
  }

  http.end(); // Close HTTP connection
  lastSendTime = millis(); // Update timestamp of last send
}
