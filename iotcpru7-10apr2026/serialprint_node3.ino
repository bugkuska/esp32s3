#include <ModbusMaster.h>

#define RXD2 18
#define TXD2 17

HardwareSerial RS485Serial(2);
ModbusMaster node3;

int16_t toSigned16(uint16_t val) {
  return (val > 0x7FFF) ? (int16_t)(val - 0x10000) : (int16_t)val;
}

void setup() {
  Serial.begin(115200);
  delay(3000);

  Serial.println("Start test node3");

  RS485Serial.begin(9600, SERIAL_8N1, RXD2, TXD2);
  RS485Serial.setTimeout(200);
  delay(500);

  node3.begin(3, RS485Serial);

  Serial.println("node3 begin done");
}

void loop() {
  Serial.println("Before read node3...");

  uint8_t result = node3.readHoldingRegisters(0x0000, 4);

  Serial.print("After read node3, result = ");
  Serial.println(result);

  if (result == node3.ku8MBSuccess) {
    uint16_t rawMoisture = node3.getResponseBuffer(0);
    uint16_t rawTemp     = node3.getResponseBuffer(1);
    uint16_t rawEC       = node3.getResponseBuffer(2);
    uint16_t rawPH       = node3.getResponseBuffer(3);

    float moisture = rawMoisture / 10.0;
    float tempC    = toSigned16(rawTemp) / 10.0;
    float ec       = rawEC;
    float ph       = rawPH / 10.0;

    Serial.print("Reg0 Moisture = ");
    Serial.println(rawMoisture);

    Serial.print("Reg1 Temp     = ");
    Serial.println(rawTemp);

    Serial.print("Reg2 EC       = ");
    Serial.println(rawEC);

    Serial.print("Reg3 pH       = ");
    Serial.println(rawPH);

    Serial.println("--- Converted ---");
    Serial.print("Soil Moisture    : ");
    Serial.print(moisture, 1);
    Serial.println(" %");

    Serial.print("Soil Temperature : ");
    Serial.print(tempC, 1);
    Serial.println(" C");

    Serial.print("EC               : ");
    Serial.print(ec, 0);
    Serial.println(" us/cm");

    Serial.print("pH               : ");
    Serial.println(ph, 1);
  }

  Serial.println("----------------------------");
  delay(2000);
}
