/************************************************************
 * ESP32 + Modbus Relay 12CH + Sensors + Blynk Legacy
 * TTL to RS485 (no DE/RE)
 *
 * node1 = Relay 12CH
 * node2 = 3IN1
 * node3 = Soil 4-in-1
 * node4 = NPK
 ************************************************************/

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ModbusMaster.h>
#include <WiFiManager.h>

// =========================
// USER CONFIG
// =========================
char auth[] = ""; //แก้ Blynk Token ตรงนี้นะจ๊ะ
char blynkServer[] = "192.168.0.253";
uint16_t blynkPort = 8080;

// =========================
// RS485 UART
// =========================
#define TXD 17
#define RXD 18

#define MODBUS_BAUDRATE 9600

HardwareSerial RS485Serial(2);

// =========================
// Modbus Nodes
// =========================
ModbusMaster node1;   // Relay 12CH
ModbusMaster node2;   // 3IN1
ModbusMaster node3;   // Soil 4-in-1
ModbusMaster node4;   // NPK

BlynkTimer timer;

// =========================
// Relay Virtual pins
// =========================
#define VPIN_SW1   V1
#define VPIN_SW2   V2
#define VPIN_SW3   V3
#define VPIN_SW4   V4
#define VPIN_SW5   V5
#define VPIN_SW6   V6
#define VPIN_SW7   V7
#define VPIN_SW8   V8
#define VPIN_SW9   V9
#define VPIN_SW10  V10
#define VPIN_SW11  V11
#define VPIN_SW12  V12
#define VPIN_ALL   V13

// =========================
// Sensor Virtual pins
// =========================
#define VPIN_NODE2_HUMI   V20
#define VPIN_NODE2_TEMP   V21
#define VPIN_NODE2_LUX    V22

#define VPIN_NODE3_SOIL   V23
#define VPIN_NODE3_TEMP   V24
#define VPIN_NODE3_EC     V25
#define VPIN_NODE3_PH     V26

#define VPIN_NODE4_N      V27
#define VPIN_NODE4_P      V28
#define VPIN_NODE4_K      V29

// =========================
// Relay addresses
// =========================
const uint16_t relayCoils[12] = {
  0x0000, 0x0001, 0x0002, 0x0003,
  0x0004, 0x0005, 0x0006, 0x0007,
  0x0008, 0x0009, 0x000A, 0x000B
};

bool relayState[12] = {0};
bool isSyncingUI = false;

// =========================
// Helper
// =========================
int16_t toSigned16(uint16_t val) {
  return (val > 0x7FFF) ? (int16_t)(val - 0x10000) : (int16_t)val;
}

// =========================
// Forward declarations
// =========================
bool writeSingleRelay(uint8_t relayIndex, bool state);
bool writeAllRelays(bool state);
bool readAllRelayStatesFromBoard();
void syncAllToBlynk();
void checkConnections();
void pollRelayStates();
void printRelayStates();

void readAllSensors();
void readNode2();
void readNode3();
void readNode4();
void connectWiFiWithManager();

// =========================
// WiFiManager connect
// =========================
void connectWiFiWithManager() {
  WiFiManager wm;
  wm.setConfigPortalTimeout(180);

  bool res = wm.autoConnect("ESP32-SmartFarm-ชื่อภาษาอังกฤษ");//แก้ชื่อตรงนี้ด้วยนะจ๊ะ

  if (res) {
    Serial.println("WiFi connected via WiFiManager");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFiManager connect failed or timeout");
  }
}

// =========================
// Sync relay state to Blynk
// =========================
void syncAllToBlynk() {
  if (!Blynk.connected()) return;

  isSyncingUI = true;

  Blynk.virtualWrite(VPIN_SW1,  relayState[0]);
  Blynk.virtualWrite(VPIN_SW2,  relayState[1]);
  Blynk.virtualWrite(VPIN_SW3,  relayState[2]);
  Blynk.virtualWrite(VPIN_SW4,  relayState[3]);
  Blynk.virtualWrite(VPIN_SW5,  relayState[4]);
  Blynk.virtualWrite(VPIN_SW6,  relayState[5]);
  Blynk.virtualWrite(VPIN_SW7,  relayState[6]);
  Blynk.virtualWrite(VPIN_SW8,  relayState[7]);
  Blynk.virtualWrite(VPIN_SW9,  relayState[8]);
  Blynk.virtualWrite(VPIN_SW10, relayState[9]);
  Blynk.virtualWrite(VPIN_SW11, relayState[10]);
  Blynk.virtualWrite(VPIN_SW12, relayState[11]);

  bool allOn = true;
  for (int i = 0; i < 12; i++) {
    if (!relayState[i]) {
      allOn = false;
      break;
    }
  }
  Blynk.virtualWrite(VPIN_ALL, allOn ? 1 : 0);

  isSyncingUI = false;
}

// =========================
// Relay: write single
// =========================
bool writeSingleRelay(uint8_t relayIndex, bool state) {
  if (relayIndex >= 12) return false;

  uint8_t result;
  uint16_t coilAddr = relayCoils[relayIndex];

  if (state) {
    result = node1.writeSingleCoil(coilAddr, 0x00FF);
  } else {
    result = node1.writeSingleCoil(coilAddr, 0x0000);
  }

  if (result == node1.ku8MBSuccess) {
    Serial.print("Write Relay ");
    Serial.print(relayIndex + 1);
    Serial.print(" -> ");
    Serial.println(state ? "ON" : "OFF");

    readAllRelayStatesFromBoard();
    syncAllToBlynk();
    return true;
  } else {
    Serial.print("Write single failed, relay ");
    Serial.print(relayIndex + 1);
    Serial.print(", err=");
    Serial.println(result);
    return false;
  }
}

// =========================
// Relay: write all
// =========================
bool writeAllRelays(bool state) {
  uint8_t result;

  node1.clearTransmitBuffer();

  if (state) {
    node1.setTransmitBuffer(0, 0x00FF);
    node1.setTransmitBuffer(1, 0x000F);
  } else {
    node1.setTransmitBuffer(0, 0x0000);
    node1.setTransmitBuffer(1, 0x0000);
  }

  result = node1.writeMultipleCoils(0x0000, 12);

  if (result == node1.ku8MBSuccess) {
    Serial.println(state ? "ALL RELAYS -> ON" : "ALL RELAYS -> OFF");

    readAllRelayStatesFromBoard();
    syncAllToBlynk();
    return true;
  } else {
    Serial.print("Write all failed, err=");
    Serial.println(result);
    return false;
  }
}

// =========================
// Relay: read all states
// =========================
bool readAllRelayStatesFromBoard() {
  uint8_t result = node1.readCoils(0x0000, 12);

  if (result != node1.ku8MBSuccess) {
    Serial.print("Read coils failed, err=");
    Serial.println(result);
    return false;
  }

  uint16_t packed = node1.getResponseBuffer(0);

  for (int i = 0; i < 12; i++) {
    relayState[i] = (packed >> i) & 0x01;
  }

  return true;
}

// =========================
// Relay: print states
// =========================
void printRelayStates() {
  Serial.print("Relay states: ");
  for (int i = 0; i < 12; i++) {
    Serial.print(relayState[i] ? "1" : "0");
    if (i < 11) Serial.print(" ");
  }
  Serial.println();
}

// =========================
// Relay: polling
// =========================
void pollRelayStates() {
  static bool firstRead = true;
  bool oldState[12];

  for (int i = 0; i < 12; i++) {
    oldState[i] = relayState[i];
  }

  if (!readAllRelayStatesFromBoard()) {
    return;
  }

  bool changed = firstRead;
  for (int i = 0; i < 12; i++) {
    if (relayState[i] != oldState[i]) {
      changed = true;
      Serial.print("Detected change: Relay ");
      Serial.print(i + 1);
      Serial.print(" = ");
      Serial.println(relayState[i] ? "ON" : "OFF");
    }
  }

  if (changed) {
    printRelayStates();
    syncAllToBlynk();
    firstRead = false;
  }
}

// =========================
// WiFi/Blynk reconnect
// =========================
void checkConnections() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, opening WiFiManager...");
    connectWiFiWithManager();
  }

  if (WiFi.status() == WL_CONNECTED && !Blynk.connected()) {
    Serial.println("Blynk disconnected, reconnecting...");
    Blynk.connect(5000);
  }
}

// =========================
// Sensor: read all
// =========================
void readAllSensors() {
  readNode2();
  delay(100);
  readNode3();
  delay(100);
  readNode4();
}

// =========================
// node2 : 3IN1
// =========================
void readNode2() {
  uint8_t result = node2.readHoldingRegisters(0x0000, 3);

  if (result == node2.ku8MBSuccess) {
    float humidity = node2.getResponseBuffer(0) / 10.0;
    float tempC    = node2.getResponseBuffer(1) / 10.0;
    float lux      = node2.getResponseBuffer(2);

    Serial.println("----- node2 : 3IN1 -----");
    Serial.print("Humidity    : ");
    Serial.print(humidity, 1);
    Serial.println(" %RH");

    Serial.print("Temperature : ");
    Serial.print(tempC, 1);
    Serial.println(" C");

    Serial.print("Lux         : ");
    Serial.print(lux);
    Serial.println(" lux");

    if (Blynk.connected()) {
      Blynk.virtualWrite(VPIN_NODE2_HUMI, humidity);
      Blynk.virtualWrite(VPIN_NODE2_TEMP, tempC);
      Blynk.virtualWrite(VPIN_NODE2_LUX, lux);
    }
  } else {
    Serial.print("node2 ERROR = ");
    Serial.println(result);
  }
}

// =========================
// node3 : Soil 4-in-1
// =========================
void readNode3() {
  uint8_t result = node3.readHoldingRegisters(0x0000, 4);

  if (result == node3.ku8MBSuccess) {
    float moisture = node3.getResponseBuffer(0) / 10.0;
    float tempC    = toSigned16(node3.getResponseBuffer(1)) / 10.0;
    float ec       = node3.getResponseBuffer(2);
    float ph       = node3.getResponseBuffer(3) / 10.0;

    Serial.println("----- node3 : Soil 4-in-1 -----");
    Serial.print("Soil Moisture    : ");
    Serial.print(moisture, 1);
    Serial.println(" %");

    Serial.print("Soil Temperature : ");
    Serial.print(tempC, 1);
    Serial.println(" C");

    Serial.print("EC               : ");
    Serial.print(ec);
    Serial.println(" us/cm");

    Serial.print("pH               : ");
    Serial.println(ph, 1);

    if (Blynk.connected()) {
      Blynk.virtualWrite(VPIN_NODE3_SOIL, moisture);
      Blynk.virtualWrite(VPIN_NODE3_TEMP, tempC);
      Blynk.virtualWrite(VPIN_NODE3_EC, ec);
      Blynk.virtualWrite(VPIN_NODE3_PH, ph);
    }
  } else {
    Serial.print("node3 ERROR = ");
    Serial.println(result);
  }
}

// =========================
// node4 : NPK
// =========================
void readNode4() {
  uint8_t result = node4.readHoldingRegisters(0x001E, 3);

  if (result == node4.ku8MBSuccess) {
    uint16_t nitrogen   = node4.getResponseBuffer(0);
    uint16_t phosphorus = node4.getResponseBuffer(1);
    uint16_t potassium  = node4.getResponseBuffer(2);

    Serial.println("----- node4 : NPK -----");
    Serial.print("Nitrogen   : ");
    Serial.print(nitrogen);
    Serial.println(" mg/kg");

    Serial.print("Phosphorus : ");
    Serial.print(phosphorus);
    Serial.println(" mg/kg");

    Serial.print("Potassium  : ");
    Serial.print(potassium);
    Serial.println(" mg/kg");

    if (Blynk.connected()) {
      Blynk.virtualWrite(VPIN_NODE4_N, nitrogen);
      Blynk.virtualWrite(VPIN_NODE4_P, phosphorus);
      Blynk.virtualWrite(VPIN_NODE4_K, potassium);
    }
  } else {
    Serial.print("node4 ERROR = ");
    Serial.println(result);
  }
}

// =========================
// Blynk connected
// =========================
BLYNK_CONNECTED() {
  Serial.println("Blynk connected");

  if (readAllRelayStatesFromBoard()) {
    printRelayStates();
    syncAllToBlynk();
  }

  readAllSensors();
}

// =========================
// Blynk handlers
// =========================
BLYNK_WRITE(VPIN_SW1)  { if (!isSyncingUI) writeSingleRelay(0,  param.asInt()); }
BLYNK_WRITE(VPIN_SW2)  { if (!isSyncingUI) writeSingleRelay(1,  param.asInt()); }
BLYNK_WRITE(VPIN_SW3)  { if (!isSyncingUI) writeSingleRelay(2,  param.asInt()); }
BLYNK_WRITE(VPIN_SW4)  { if (!isSyncingUI) writeSingleRelay(3,  param.asInt()); }
BLYNK_WRITE(VPIN_SW5)  { if (!isSyncingUI) writeSingleRelay(4,  param.asInt()); }
BLYNK_WRITE(VPIN_SW6)  { if (!isSyncingUI) writeSingleRelay(5,  param.asInt()); }
BLYNK_WRITE(VPIN_SW7)  { if (!isSyncingUI) writeSingleRelay(6,  param.asInt()); }
BLYNK_WRITE(VPIN_SW8)  { if (!isSyncingUI) writeSingleRelay(7,  param.asInt()); }
BLYNK_WRITE(VPIN_SW9)  { if (!isSyncingUI) writeSingleRelay(8,  param.asInt()); }
BLYNK_WRITE(VPIN_SW10) { if (!isSyncingUI) writeSingleRelay(9,  param.asInt()); }
BLYNK_WRITE(VPIN_SW11) { if (!isSyncingUI) writeSingleRelay(10, param.asInt()); }
BLYNK_WRITE(VPIN_SW12) { if (!isSyncingUI) writeSingleRelay(11, param.asInt()); }

BLYNK_WRITE(VPIN_ALL) {
  if (isSyncingUI) return;
  writeAllRelays(param.asInt() == 1);
}

// =========================
// Setup
// =========================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Starting ESP32 Relay + Sensors + Blynk + WiFiManager");

  RS485Serial.begin(MODBUS_BAUDRATE, SERIAL_8N1, RXD, TXD);
  RS485Serial.setTimeout(200);

  node1.begin(1, RS485Serial);  // Relay
  node2.begin(2, RS485Serial);  // 3IN1
  node3.begin(3, RS485Serial);  // Soil 4-in-1
  node4.begin(4, RS485Serial);  // NPK

  Serial.println("All nodes initialized");

  connectWiFiWithManager();

  Blynk.config(auth, blynkServer, blynkPort);
  if (WiFi.status() == WL_CONNECTED) {
    if (Blynk.connect(10000)) {
      Serial.println("Blynk connected successfully");
    } else {
      Serial.println("Initial Blynk connect failed");
    }
  } else {
    Serial.println("Skip Blynk connect because WiFi is not connected");
  }

  timer.setInterval(300L, pollRelayStates);
  timer.setInterval(5000L, readAllSensors);
  timer.setInterval(10000L, checkConnections);

  if (readAllRelayStatesFromBoard()) {
    printRelayStates();
  }

  readAllSensors();
}

// =========================
// Loop
// =========================
void loop() {
  Blynk.run();
  timer.run();
}
