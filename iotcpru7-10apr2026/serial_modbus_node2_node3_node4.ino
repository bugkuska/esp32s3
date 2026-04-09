#include <ModbusMaster.h>

// =========================
// ESP32 RS485 Serial2
// =========================
#define RXD2 18
#define TXD2 17

HardwareSerial RS485Serial(2);

// =========================
// Modbus Nodes
// =========================
ModbusMaster node2;   // Slave ID2 = 3IN1
ModbusMaster node3;   // Slave ID3 = Soil 4-in-1
ModbusMaster node4;   // Slave ID4 = NPK

// =========================
// Helper function
// =========================
int16_t toSigned16(uint16_t val) {
  return (val > 0x7FFF) ? (int16_t)(val - 0x10000) : (int16_t)val;
}

// =========================
// Setup
// =========================
void setup() {
  Serial.begin(115200);
  delay(3000);

  Serial.println("======================================");
  Serial.println("ESP32 Modbus ALL NODE START");
  Serial.println("======================================");

  RS485Serial.begin(9600, SERIAL_8N1, RXD2, TXD2);
  RS485Serial.setTimeout(200);

  node2.begin(2, RS485Serial);
  node3.begin(3, RS485Serial);
  node4.begin(4, RS485Serial);

  Serial.println("All nodes initialized");
}

// =========================
// LOOP
// =========================
void loop() {
  readNode2();
  delay(1000);

  readNode3();
  delay(1000);

  readNode4();
  delay(1000);

  Serial.println("======================================");
  delay(3000);
}

// =====================================================
// node2 : 3IN1 (Humidity / Temp / Lux)
// =====================================================
void readNode2() {
  uint8_t result = node2.readHoldingRegisters(0x0000, 3);

  Serial.println("----- node2 : 3IN1 -----");

  if (result == node2.ku8MBSuccess) {
    float humidity = node2.getResponseBuffer(0) / 10.0;
    float tempC    = node2.getResponseBuffer(1) / 10.0;
    float lux      = node2.getResponseBuffer(2);

    Serial.print("Humidity    : ");
    Serial.print(humidity, 1);
    Serial.println(" %RH");

    Serial.print("Temperature : ");
    Serial.print(tempC, 1);
    Serial.println(" C");

    Serial.print("Lux         : ");
    Serial.print(lux);
    Serial.println(" lux");
  } else {
    Serial.print("node2 ERROR = ");
    Serial.println(result);
  }
}

// =====================================================
// node3 : Soil 4-in-1 (Moisture / Temp / EC / pH)
// =====================================================
void readNode3() {
  uint8_t result = node3.readHoldingRegisters(0x0000, 4);

  Serial.println("----- node3 : Soil 4-in-1 -----");

  if (result == node3.ku8MBSuccess) {
    float moisture = node3.getResponseBuffer(0) / 10.0;
    float tempC    = toSigned16(node3.getResponseBuffer(1)) / 10.0;
    float ec       = node3.getResponseBuffer(2);
    float ph       = node3.getResponseBuffer(3) / 10.0;

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
  } else {
    Serial.print("node3 ERROR = ");
    Serial.println(result);
  }
}

// =====================================================
// node4 : NPK
// =====================================================
void readNode4() {
  uint8_t result = node4.readHoldingRegisters(0x001E, 3);

  Serial.println("----- node4 : NPK -----");

  if (result == node4.ku8MBSuccess) {
    uint16_t nitrogen   = node4.getResponseBuffer(0);
    uint16_t phosphorus = node4.getResponseBuffer(1);
    uint16_t potassium  = node4.getResponseBuffer(2);

    Serial.print("Nitrogen   : ");
    Serial.print(nitrogen);
    Serial.println(" mg/kg");

    Serial.print("Phosphorus : ");
    Serial.print(phosphorus);
    Serial.println(" mg/kg");

    Serial.print("Potassium  : ");
    Serial.print(potassium);
    Serial.println(" mg/kg");
  } else {
    Serial.print("node4 ERROR = ");
    Serial.println(result);
  }
}
