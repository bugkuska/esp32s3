#include <ModbusMaster.h>

#define RXD2 18
#define TXD2 17

HardwareSerial RS485Serial(2);
ModbusMaster node2;

void setup() {
  Serial.begin(115200);
  delay(3000);

  Serial.println("================================");
  Serial.println("Modbus node2 debug start");
  Serial.println("================================");

  RS485Serial.begin(9600, SERIAL_8N1, RXD2, TXD2);
  RS485Serial.setTimeout(200);
  delay(500);

  node2.begin(2, RS485Serial);

  Serial.println("RS485 init done");
}

void loop() {
  Serial.println("Try read node2 ...");

  uint8_t result = node2.readHoldingRegisters(0x0000, 3);

  Serial.print("Result = ");
  Serial.println(result);

  if (result == node2.ku8MBSuccess) {
    Serial.print("R0 = ");
    Serial.println(node2.getResponseBuffer(0));

    Serial.print("R1 = ");
    Serial.println(node2.getResponseBuffer(1));

    Serial.print("R2 = ");
    Serial.println(node2.getResponseBuffer(2));
  }

  Serial.println("----------------------------");
  delay(2000);
}
