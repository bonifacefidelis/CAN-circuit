#include <SPI.h>
#include <mcp2515.h>
#include <SoftwareSerial.h>

MCP2515 mcp2515(10);                // MCP2515 CS pin
SoftwareSerial sim7600Serial(7, 8); // RX, TX pins for SIM7600E

const char *apn = "internet";
const char *server_ip = "2.16.134.20";
const int   server_port = 5000;

// ----------------------------
// SETUP
// ----------------------------
void setup() {
  Serial.begin(115200);
  sim7600Serial.begin(115200);
  SPI.begin();

  // Initialize CAN
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  Serial.println("CAN Simulator + SIM7600 TCP Sender Starting...");

  setupSIM7600E();
  connectTCP();
}


// ----------------------------
// MAIN LOOP
// ----------------------------
void loop() {

  // 1. Simulate CAN traffic
  sendEngineRPM();
  sendCoolantTemp();
  sendVehicleSpeed();
  sendRandomFrame();
  delay(150);

  // 2. Read CAN frames and forward via TCP
  struct can_frame rxMsg;
  if (mcp2515.readMessage(&rxMsg) == MCP2515::ERROR_OK) {

    String canData = "ID:" + String(rxMsg.can_id, HEX) + ",";

    for (int i = 0; i < rxMsg.can_dlc; i++) {
      if (rxMsg.data[i] < 0x10) canData += "0";
      canData += String(rxMsg.data[i], HEX);
      if (i < rxMsg.can_dlc - 1) canData += " ";
    }

    Serial.print("Sending CAN data: ");
    Serial.println(canData);

    sendTCPData(canData);
  }
}


// ------------------------------------------------------
// SIMULATED CAN MESSAGE GENERATORS
// ------------------------------------------------------

// Engine RPM (ID 0x0CFF0500)
void sendEngineRPM() {
  struct can_frame msg;
  msg.can_id = 0x0CFF0500;
  msg.can_dlc = 2;

  int rpm = random(700, 3500);
  msg.data[0] = rpm & 0xFF;
  msg.data[1] = rpm >> 8;

  mcp2515.sendMessage(&msg);

  Serial.print("[SIM] RPM: ");
  Serial.println(rpm);
}

// Coolant Temperature (ID 0x18FEEE00)
void sendCoolantTemp() {
  struct can_frame msg;
  msg.can_id = 0x18FEEE00;
  msg.can_dlc = 1;

  int temp = random(70, 105);
  msg.data[0] = temp;

  mcp2515.sendMessage(&msg);

  Serial.print("[SIM] Coolant Temp: ");
  Serial.println(temp);
}

// Vehicle Speed (ID 0x0CF00400)
void sendVehicleSpeed() {
  struct can_frame msg;
  msg.can_id = 0x0CF00400;
  msg.can_dlc = 1;

  int speed = random(0, 120);
  msg.data[0] = speed;

  mcp2515.sendMessage(&msg);

  Serial.print("[SIM] Speed: ");
  Serial.println(speed);
}

// Random CAN frame for bus noise testing
void sendRandomFrame() {
  struct can_frame msg;
  msg.can_id = random(0x100, 0x700);
  msg.can_dlc = random(3, 8);

  for (int i = 0; i < msg.can_dlc; i++) {
    msg.data[i] = random(0, 255);
  }

  mcp2515.sendMessage(&msg);

  Serial.print("[SIM] Random ID: 0x");
  Serial.println(msg.can_id, HEX);
}


// ------------------------------------------------------
// SIM7600 FUNCTIONS
// ------------------------------------------------------

void setupSIM7600E() {
  sendATCommand("AT");
  sendATCommand("AT+CFUN=1");
  sendATCommand("AT+CGDCONT=1,\"IP\",\"" + String(apn) + "\"");
  sendATCommand("AT+CGACT=1,1");
  delay(3000);
  flushSIM7600E();
}

void connectTCP() {
  sendATCommand("AT+QIOPEN=1,0,\"TCP\",\"" + String(server_ip) + "\"," + String(server_port) + ",0,1");
  delay(5000);
  flushSIM7600E();
}

void sendTCPData(String data) {
  data += "\r\n";
  String len = String(data.length());

  sim7600Serial.print("AT+QISEND=0," + len + "\r\n");
  delay(100);

  if (waitForResponse(">", 2000)) {
    sim7600Serial.print(data);
    delay(300);
  } else {
    Serial.println("Error: No '>' prompt");
  }

  flushSIM7600E();
}

void sendATCommand(String cmd) {
  sim7600Serial.print(cmd + "\r\n");
  delay(500);

  while (sim7600Serial.available()) {
    Serial.write(sim7600Serial.read());
  }
}

bool waitForResponse(String target, unsigned long timeout) {
  unsigned long start = millis();
  String resp = "";

  while (millis() - start < timeout) {
    while (sim7600Serial.available()) {
      char c = sim7600Serial.read();
      resp += c;

      if (resp.indexOf(target) != -1) return true;
    }
  }
  return false;
}

void flushSIM7600E() {
  while (sim7600Serial.available()) {
    Serial.write(sim7600Serial.read());
  }
}
