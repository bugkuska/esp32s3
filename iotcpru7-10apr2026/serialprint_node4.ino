#include <ModbusMaster.h>

#define RXD2 18
#define TXD2 17

HardwareSerial RS485Serial(2);
ModbusMaster node4;

void setup() {
  Serial.begin(115200);
  delay(3000);

  Serial.println("Start test node4");

  RS485Serial.begin(9600, SERIAL_8N1, RXD2, TXD2);
  RS485Serial.setTimeout(200);
  delay(500);

  node4.begin(4, RS485Serial);

  Serial.println("node4 begin done");
}

void loop() {
  Serial.println("Before read node4...");

  uint8_t result = node4.readHoldingRegisters(0x001E, 3);

  Serial.print("After read node4, result = ");
  Serial.println(result);

  if (result == node4.ku8MBSuccess) {
    uint16_t nitrogen   = node4.getResponseBuffer(0);
    uint16_t phosphorus = node4.getResponseBuffer(1);
    uint16_t potassium  = node4.getResponseBuffer(2);

    Serial.print("Reg30 Nitrogen   = ");
    Serial.println(nitrogen);

    Serial.print("Reg31 Phosphorus = ");
    Serial.println(phosphorus);

    Serial.print("Reg32 Potassium  = ");
    Serial.println(potassium);

    Serial.println("--- Converted ---");
    Serial.print("Nitrogen   : ");
    Serial.print(nitrogen);
    Serial.println(" mg/kg");

    Serial.print("Phosphorus : ");
    Serial.print(phosphorus);
    Serial.println(" mg/kg");

    Serial.print("Potassium  : ");
    Serial.print(potassium);
    Serial.println(" mg/kg");
  }

  Serial.println("----------------------------");
  delay(2000);
}
