/*
 * ============================================================================
 * Complete Arduino CAN to 4G LTE Cloud System
 * ============================================================================
 * 
 * SYSTEM ARCHITECTURE:
 * 
 * Vehicle CAN Bus → MCP2515 → Arduino → 4G LTE Module → Cellular Network 
 *                                           ↓
 *                                      Internet
 *                                           ↓
 *                                    Cloud Server
 *                                           ↓
 *                                       Database
 * 
 * Hardware Required:
 * - Arduino Uno/Mega/Nano
 * - MCP2515 CAN Bus Module
 * - SIM7600/SIM800 4G LTE Module
 * - SIM card with data plan
 * - Power supply (12V/5V converter for vehicle)
 * 
 * Pin Connections:
 * 
 * MCP2515 CAN Module:
 *   VCC  → 5V
 *   GND  → GND
 *   CS   → Pin 10
 *   SO   → Pin 12 (MISO)
 *   SI   → Pin 11 (MOSI)
 *   SCK  → Pin 13
 *   INT  → Pin 2 (optional)
 * 
 * 4G LTE Module (SIM7600):
 *   VCC  → 5V (needs 2A+ power supply)
 *   GND  → GND
 *   TXD  → Pin 3 (Arduino RX via SoftwareSerial)
 *   RXD  → Pin 4 (Arduino TX via SoftwareSerial)
 *   PWR  → Connect to power control if needed
 * 
 * ============================================================================
 */

#include <SPI.h>
#include <mcp2515.h>
#include <SoftwareSerial.h>

// ============================================================================
// CONFIGURATION - EDIT THESE VALUES
// ============================================================================

// CAN Bus Settings
#define CAN_SPEED CAN_500KBPS
#define CRYSTAL_FREQ MCP_8MHZ
#define CAN_CS_PIN 10

// 4G Module Settings
#define LTE_RX_PIN 3        // Arduino RX ← 4G TX
#define LTE_TX_PIN 4        // Arduino TX → 4G RX
#define LTE_BAUD 9600

// Network Settings
#define APN "internet"      // Your carrier's APN
#define SERVER_URL "your-server.com"
#define SERVER_PORT "80"
#define API_ENDPOINT "/api/can-data"
#define API_KEY "your-api-key"

// Data Transmission Settings
#define BATCH_SIZE 10       // Number of messages to batch before sending
#define SEND_INTERVAL 10000 // Send interval in milliseconds (10 seconds)

// Debug Settings
#define DEBUG_SERIAL true   // Set to false to disable debug output
#define LED_PIN 13

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

MCP2515 mcp2515(CAN_CS_PIN);
SoftwareSerial lteSerial(LTE_RX_PIN, LTE_TX_PIN);

struct can_frame canMsg;

// Message buffer
struct CANMessage {
  unsigned long timestamp;
  uint16_t can_id;
  uint8_t data[8];
  uint8_t dlc;
};

CANMessage messageBuffer[BATCH_SIZE];
uint8_t bufferIndex = 0;
unsigned long lastSendTime = 0;
bool lteConnected = false;
bool networkRegistered = false;

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  while (!Serial && millis() < 3000);  // Wait 3 seconds max
  
  Serial.println(F("========================================"));
  Serial.println(F("  Arduino CAN to 4G Cloud System"));
  Serial.println(F("========================================"));
  Serial.println();
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize MCP2515 CAN
  Serial.println(F("1. Initializing CAN Bus..."));
  if (initCAN()) {
    Serial.println(F("   ✓ CAN Bus ready"));
    blinkLED(2, 100);
  } else {
    Serial.println(F("   ✗ CAN initialization failed!"));
    while (1) blinkLED(5, 100);  // Error indicator
  }
  
  // Initialize 4G Module
  Serial.println(F("2. Initializing 4G LTE Module..."));
  lteSerial.begin(LTE_BAUD);
  delay(2000);  // Wait for module to boot
  
  if (init4G()) {
    Serial.println(F("   ✓ 4G Module ready"));
    blinkLED(3, 100);
  } else {
    Serial.println(F("   ✗ 4G initialization failed!"));
    Serial.println(F("   System will continue in offline mode"));
  }
  
  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F("  System Running"));
  Serial.println(F("========================================"));
  Serial.println();
  
  lastSendTime = millis();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // 1. Read CAN messages
  readCANMessages();
  
  // 2. Check if it's time to send data
  unsigned long currentTime = millis();
  
  if ((bufferIndex >= BATCH_SIZE) || 
      ((currentTime - lastSendTime >= SEND_INTERVAL) && bufferIndex > 0)) {
    
    if (lteConnected && networkRegistered) {
      sendDataToCloud();
    } else {
      Serial.println(F("⚠ Network not ready, skipping send"));
      bufferIndex = 0;  // Clear buffer to prevent overflow
    }
    
    lastSendTime = currentTime;
  }
  
  // 3. Monitor 4G connection periodically
  static unsigned long lastConnectionCheck = 0;
  if (currentTime - lastConnectionCheck > 30000) {  // Every 30 seconds
    checkNetworkStatus();
    lastConnectionCheck = currentTime;
  }
  
  // Small delay to prevent overwhelming the system
  delay(10);
}

// ============================================================================
// CAN BUS FUNCTIONS
// ============================================================================

bool initCAN() {
  mcp2515.reset();
  
  MCP2515::ERROR result = mcp2515.setBitrate(CAN_SPEED, CRYSTAL_FREQ);
  if (result != MCP2515::ERROR_OK) {
    return false;
  }
  
  mcp2515.setNormalMode();
  return true;
}

void readCANMessages() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    // LED indicator for CAN activity
    digitalWrite(LED_PIN, HIGH);
    
    // Store message in buffer
    if (bufferIndex < BATCH_SIZE) {
      messageBuffer[bufferIndex].timestamp = millis();
      messageBuffer[bufferIndex].can_id = canMsg.can_id;
      messageBuffer[bufferIndex].dlc = canMsg.can_dlc;
      
      for (int i = 0; i < canMsg.can_dlc; i++) {
        messageBuffer[bufferIndex].data[i] = canMsg.data[i];
      }
      
      bufferIndex++;
      
      if (DEBUG_SERIAL) {
        Serial.print(F("CAN RX: ID=0x"));
        Serial.print(canMsg.can_id, HEX);
        Serial.print(F(" Data="));
        for (int i = 0; i < canMsg.can_dlc; i++) {
          if (canMsg.data[i] < 0x10) Serial.print(F("0"));
          Serial.print(canMsg.data[i], HEX);
        }
        Serial.print(F(" ["));
        Serial.print(bufferIndex);
        Serial.println(F("/10]"));
      }
    }
    
    digitalWrite(LED_PIN, LOW);
  }
}

// ============================================================================
// 4G LTE FUNCTIONS
// ============================================================================

bool init4G() {
  Serial.println(F("   Testing 4G module..."));
  
  // Test AT command
  if (!sendATCommand("AT", "OK", 2000)) {
    Serial.println(F("   Module not responding"));
    return false;
  }
  Serial.println(F("   ✓ Module responding"));
  
  // Disable echo
  sendATCommand("ATE0", "OK", 1000);
  
  // Check SIM card
  Serial.println(F("   Checking SIM card..."));
  if (!sendATCommand("AT+CPIN?", "READY", 3000)) {
    Serial.println(F("   ✗ SIM card not ready"));
    return false;
  }
  Serial.println(F("   ✓ SIM card ready"));
  
  // Wait for network registration
  Serial.println(F("   Registering on network..."));
  for (int i = 0; i < 30; i++) {
    if (sendATCommand("AT+CREG?", "+CREG: 0,1", 2000) || 
        sendATCommand("AT+CREG?", "+CREG: 0,5", 2000)) {
      networkRegistered = true;
      Serial.println(F("   ✓ Network registered"));
      break;
    }
    Serial.print(F("   Waiting... "));
    Serial.println(i + 1);
    delay(1000);
  }
  
  if (!networkRegistered) {
    Serial.println(F("   ✗ Network registration timeout"));
    return false;
  }
  
  // Check signal strength
  sendATCommand("AT+CSQ", "OK", 2000);
  
  // Configure APN
  Serial.println(F("   Configuring APN..."));
  String apnCmd = "AT+CGDCONT=1,\"IP\",\"";
  apnCmd += APN;
  apnCmd += "\"";
  sendATCommand(apnCmd, "OK", 3000);
  
  // Activate PDP context
  sendATCommand("AT+CGACT=1,1", "OK", 5000);
  delay(2000);
  
  lteConnected = true;
  Serial.println(F("   ✓ 4G connection established"));
  
  return true;
}

bool sendATCommand(String command, String expectedResponse, unsigned long timeout) {
  lteSerial.println(command);
  
  if (DEBUG_SERIAL) {
    Serial.print(F("AT> "));
    Serial.println(command);
  }
  
  String response = "";
  unsigned long startTime = millis();
  
  while (millis() - startTime < timeout) {
    while (lteSerial.available()) {
      char c = lteSerial.read();
      response += c;
      
      if (response.indexOf(expectedResponse) != -1) {
        if (DEBUG_SERIAL) {
          Serial.print(F("AT< "));
          Serial.println(response);
        }
        return true;
      }
      
      if (response.indexOf("ERROR") != -1) {
        if (DEBUG_SERIAL) {
          Serial.print(F("AT< ERROR: "));
          Serial.println(response);
        }
        return false;
      }
    }
    delay(10);
  }
  
  if (DEBUG_SERIAL) {
    Serial.println(F("AT< TIMEOUT"));
  }
  
  return false;
}

void checkNetworkStatus() {
  Serial.println(F("📶 Checking network status..."));
  
  // Check registration
  String response = sendATCommandWithResponse("AT+CREG?", 2000);
  if (response.indexOf(",1") != -1 || response.indexOf(",5") != -1) {
    networkRegistered = true;
    Serial.println(F("   ✓ Network OK"));
  } else {
    networkRegistered = false;
    Serial.println(F("   ✗ Network lost"));
  }
  
  // Check signal
  sendATCommand("AT+CSQ", "OK", 2000);
}

String sendATCommandWithResponse(String command, unsigned long timeout) {
  lteSerial.println(command);
  
  String response = "";
  unsigned long startTime = millis();
  
  while (millis() - startTime < timeout) {
    while (lteSerial.available()) {
      response += (char)lteSerial.read();
    }
    delay(10);
  }
  
  return response;
}

// ============================================================================
// DATA TRANSMISSION FUNCTIONS
// ============================================================================

void sendDataToCloud() {
  Serial.println();
  Serial.println(F("========================================"));
  Serial.print(F("📤 Sending "));
  Serial.print(bufferIndex);
  Serial.println(F(" messages to cloud..."));
  Serial.println(F("========================================"));
  
  // Build JSON payload
  String jsonPayload = buildJSONPayload();
  
  if (DEBUG_SERIAL) {
    Serial.println(F("Payload:"));
    Serial.println(jsonPayload);
    Serial.println();
  }
  
  // Send via HTTP POST
  if (sendHTTPPost(jsonPayload)) {
    Serial.println(F("✓ Data sent successfully!"));
    blinkLED(1, 200);
    bufferIndex = 0;  // Clear buffer
  } else {
    Serial.println(F("✗ Failed to send data"));
    blinkLED(3, 100);
    // Keep data in buffer for retry
  }
  
  Serial.println(F("========================================"));
  Serial.println();
}

String buildJSONPayload() {
  String json = "{";
  json += "\"device_id\":\"arduino_001\",";
  json += "\"timestamp\":\"" + String(millis()) + "\",";
  json += "\"message_count\":" + String(bufferIndex) + ",";
  json += "\"messages\":[";
  
  for (int i = 0; i < bufferIndex; i++) {
    json += "{";
    json += "\"ts\":" + String(messageBuffer[i].timestamp) + ",";
    json += "\"id\":\"0x" + String(messageBuffer[i].can_id, HEX) + "\",";
    json += "\"data\":\"";
    
    for (int j = 0; j < messageBuffer[i].dlc; j++) {
      if (messageBuffer[i].data[j] < 0x10) json += "0";
      json += String(messageBuffer[i].data[j], HEX);
    }
    
    json += "\",";
    json += "\"dlc\":" + String(messageBuffer[i].dlc);
    json += "}";
    
    if (i < bufferIndex - 1) json += ",";
  }
  
  json += "]}";
  
  return json;
}

bool sendHTTPPost(String payload) {
  // Initialize HTTP
  if (!sendATCommand("AT+HTTPINIT", "OK", 5000)) {
    Serial.println(F("HTTP init failed"));
    return false;
  }
  
  // Set parameters
  sendATCommand("AT+HTTPPARA=\"CID\",1", "OK", 2000);
  
  String urlCmd = "AT+HTTPPARA=\"URL\",\"http://";
  urlCmd += SERVER_URL;
  urlCmd += API_ENDPOINT;
  urlCmd += "\"";
  sendATCommand(urlCmd, "OK", 2000);
  
  sendATCommand("AT+HTTPPARA=\"CONTENT\",\"application/json\"", "OK", 2000);
  
  // Add API key header if configured
  if (String(API_KEY).length() > 0) {
    String headerCmd = "AT+HTTPPARA=\"USERDATA\",\"Authorization: Bearer ";
    headerCmd += API_KEY;
    headerCmd += "\"";
    sendATCommand(headerCmd, "OK", 2000);
  }
  
  // Set POST data
  String dataCmd = "AT+HTTPDATA=" + String(payload.length()) + ",10000";
  lteSerial.println(dataCmd);
  delay(1000);
  
  // Send payload
  lteSerial.println(payload);
  delay(2000);
  
  // Execute POST
  bool success = sendATCommand("AT+HTTPACTION=1", "OK", 30000);
  
  delay(3000);  // Wait for response
  
  // Read response
  sendATCommand("AT+HTTPREAD", "OK", 5000);
  
  // Terminate HTTP
  sendATCommand("AT+HTTPTERM", "OK", 2000);
  
  return success;
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

void blinkLED(int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(delayMs);
    digitalWrite(LED_PIN, LOW);
    delay(delayMs);
  }
}

// ============================================================================
// HELPER FUNCTIONS FOR TESTING
// ============================================================================

void printSystemInfo() {
  Serial.println(F("\n=== System Information ==="));
  Serial.print(F("CAN Status: "));
  Serial.println(F("OK"));
  Serial.print(F("4G Status: "));
  Serial.println(lteConnected ? F("Connected") : F("Disconnected"));
  Serial.print(F("Network: "));
  Serial.println(networkRegistered ? F("Registered") : F("Not registered"));
  Serial.print(F("Buffer: "));
  Serial.print(bufferIndex);
  Serial.print(F("/"));
  Serial.println(BATCH_SIZE);
  Serial.println(F("=========================\n"));
}

/*
 * ============================================================================
 * SYSTEM FLOW DIAGRAM:
 * ============================================================================
 * 
 * [Vehicle CAN Bus]
 *        ↓
 *   CAN_H, CAN_L (twisted pair)
 *        ↓
 * [MCP2515 CAN Controller]
 *   - Converts CAN to SPI
 *   - Hardware filtering
 *   - Error detection
 *        ↓
 *   SPI Protocol (MISO, MOSI, SCK, CS)
 *        ↓
 * [Arduino Uno/Mega]
 *   - Reads CAN messages via SPI
 *   - Buffers messages (10 at a time)
 *   - Builds JSON payload
 *   - Sends AT commands to 4G module
 *        ↓
 *   Serial/UART (TX, RX)
 *        ↓
 * [SIM7600 4G LTE Module]
 *   - Manages cellular connection
 *   - Handles TCP/IP stack
 *   - HTTP client
 *        ↓
 *   RF Signal (850/900/1800/1900 MHz)
 *        ↓
 * [Cellular Tower]
 *        ↓
 * [Cellular Network Core]
 *   - Authentication (SIM)
 *   - Routing
 *   - APN gateway
 *        ↓
 * [Internet]
 *        ↓
 * [Cloud Server] (your-server.com)
 *   - Receives HTTP POST
 *   - Validates API key
 *   - Parses JSON
 *        ↓
 * [Database] (MySQL/PostgreSQL/MongoDB)
 *   - Stores CAN messages
 *   - Indexes by timestamp, device_id, can_id
 *   - Available for analytics
 * 
 * ============================================================================
 * DATA FLOW EXAMPLE:
 * ============================================================================
 * 
 * 1. Vehicle ECU sends CAN message: ID=0x123, Data=00112233445566 77
 * 2. MCP2515 receives via CAN bus, converts to SPI
 * 3. Arduino reads via mcp2515.readMessage()
 * 4. Arduino stores in buffer: messageBuffer[0] = {0x123, [00,11,22,33...]}
 * 5. After 10 messages or 10 seconds:
 * 6. Arduino builds JSON: {"device_id":"arduino_001","messages":[...]}
 * 7. Arduino sends to 4G: AT+HTTPINIT, AT+HTTPDATA=..., AT+HTTPACTION=1
 * 8. 4G module connects to cellular network using APN
 * 9. HTTP POST sent to server: POST /api/can-data
 * 10. Server receives, validates, stores in database
 * 11. Dashboard queries database, displays live CAN data
 * 
 * ============================================================================
 */