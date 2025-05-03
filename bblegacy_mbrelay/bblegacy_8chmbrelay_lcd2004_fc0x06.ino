#include <Wire.h>              // ไลบรารีสำหรับ I2C
#include <WiFi.h>              // ไลบรารีสำหรับการเชื่อมต่อ Wi-Fi
#include <WiFiClient.h>        // ไลบรารีสำหรับการจัดการ Wi-Fi client
#include <BlynkSimpleEsp32.h>  // ไลบรารีสำหรับการเชื่อมต่อ Blynk กับ ESP32
#include <ModbusMaster.h>      // ไลบรารีสำหรับการใช้งาน Modbus Master

//================LCD 2004=================//
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);
//================LCD 2004=================//

// Wi-Fi and Blynk credentials
#include <WiFiManager.h>                                 // ✅ เพิ่ม WiFiManager
const char auth[] = "";  // โทเคน Blynk

// Modbus settings
#define RXD 18       // กำหนดขา RX ของ ESP32 (เชื่อมต่อกับ TX ของอุปกรณ์ Modbus)
#define TXD 17       // กำหนดขา TX ของ ESP32 (เชื่อมต่อกับ RX ของอุปกรณ์ Modbus)
ModbusMaster node1;  // สร้างออบเจกต์สำหรับใช้งาน Modbus

uint8_t relayState[8] = {0}; 

// Timer for periodic tasks
BlynkTimer timer;  // ตัวจับเวลา Blynk สำหรับเรียกใช้ฟังก์ชันตามรอบเวลาที่กำหนด

WiFiManager wm;  // ✅ สร้างอ็อบเจกต์ WiFiManager

// Function prototypes
void checkConnections();  // ฟังก์ชันสำหรับตรวจสอบการเชื่อมต่อ Wi-Fi และ Blynk
void debugModbus();       // ฟังก์ชันสำหรับ Debug การเชื่อมต่อ Modbus

// Define Blynk virtual pins
#define Widget_Btn_SW1 V1  // กำหนดปุ่ม Virtual Pin V1 สำหรับ Relay 1
#define Widget_Btn_SW2 V2  // กำหนดปุ่ม Virtual Pin V2 สำหรับ Relay 2
#define Widget_Btn_SW3 V3  // กำหนดปุ่ม Virtual Pin V3 สำหรับ Relay 3
#define Widget_Btn_SW4 V4  // กำหนดปุ่ม Virtual Pin V4 สำหรับ Relay 4
#define Widget_Btn_SW5 V5  // กำหนดปุ่ม Virtual Pin V5 สำหรับ Relay 5
#define Widget_Btn_SW6 V6  // กำหนดปุ่ม Virtual Pin V6 สำหรับ Relay 6
#define Widget_Btn_SW7 V7  // กำหนดปุ่ม Virtual Pin V7 สำหรับ Relay 7
#define Widget_Btn_SW8 V8  // กำหนดปุ่ม Virtual Pin V8 สำหรับ Relay 8

void setup() {
  // Initialize serial communication
  Serial.begin(115200);                       // เริ่มการสื่อสาร Serial (ความเร็ว 115200 bps) สำหรับ Debugging
  Serial2.begin(9600, SERIAL_8N1, RXD, TXD);  // เริ่มการสื่อสาร Serial2 สำหรับ Modbus (9600 bps, 8-bit data, no parity, 1 stop bit)

  // Initialize Modbus
  node1.begin(1, Serial2);  // ตั้งค่า Modbus Address 1 และใช้ Serial2 เป็นการสื่อสาร

  Wire.begin(8, 9); // SDA = GPIO08, SCL = GPIO9
  lcd.begin();      // แทน lcd.init();
  lcd.backlight();  // เปิด backlight
  updateRelayStatusLCD();  // <== เพิ่มบรรทัดนี้เพื่อแสดงข้อมูลตั้งแต่เริ่มต้น

  // ✅ WiFiManager: ถ้าเชื่อมต่อ Wi-Fi ไม่ได้ ให้เปิด AP
  if (!wm.autoConnect("ESP32S3-Modbus")) {
    Serial.println("Failed to connect. Restarting...");
    delay(3000);
    ESP.restart();
  }

  Serial.println("Wi-Fi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // ✅ ใช้ Wi-Fi ที่เชื่อมต่อสำเร็จ มาสมัครเข้า Blynk
  Blynk.begin(auth, WiFi.SSID().c_str(), WiFi.psk().c_str(), "blynk-local-server", port); //แก้ blynk-local-server และ port
  // Set timers
  timer.setInterval(5000L, checkConnections);  // ตั้งตัวจับเวลา 5 วินาทีเพื่อเรียกใช้ฟังก์ชัน `checkConnections`
  //timer.setInterval(2000L, debugModbus);  // ตัวจับเวลาสำหรับ Debug Modbus (ปิดการใช้งานไว้)
}

// ฟังก์ชัน BLYNK_CONNECTED()
// ฟังก์ชันนี้จะถูกเรียกโดยอัตโนมัติเมื่อ ESP32 เชื่อมต่อกับเซิร์ฟเวอร์ Blynk สำเร็จ
BLYNK_CONNECTED() {
  // แสดงข้อความใน Serial Monitor เพื่อยืนยันการเชื่อมต่อสำเร็จ
  Serial.println("Blynk connected! Synchronizing virtual pins...");

  // ซิงโครไนซ์สถานะของ Virtual Pins บน Blynk กับ ESP32
  Blynk.syncVirtual(Widget_Btn_SW1);  // ซิงโครไนซ์สถานะของปุ่ม Virtual Pin V1
  Blynk.syncVirtual(Widget_Btn_SW2);  // ซิงโครไนซ์สถานะของปุ่ม Virtual Pin V2
  Blynk.syncVirtual(Widget_Btn_SW3);  // ซิงโครไนซ์สถานะของปุ่ม Virtual Pin V3
  Blynk.syncVirtual(Widget_Btn_SW4);  // ซิงโครไนซ์สถานะของปุ่ม Virtual Pin V4
  Blynk.syncVirtual(Widget_Btn_SW5);  // ซิงโครไนซ์สถานะของปุ่ม Virtual Pin V5
  Blynk.syncVirtual(Widget_Btn_SW6);  // ซิงโครไนซ์สถานะของปุ่ม Virtual Pin V6
  Blynk.syncVirtual(Widget_Btn_SW7);  // ซิงโครไนซ์สถานะของปุ่ม Virtual Pin V7
  Blynk.syncVirtual(Widget_Btn_SW8);  // ซิงโครไนซ์สถานะของปุ่ม Virtual Pin V8
}

void checkConnections() {
  // ตรวจสอบสถานะการเชื่อมต่อ Wi-Fi
  if (WiFi.status() != WL_CONNECTED) {  // หาก Wi-Fi หลุด
    Serial.println("Wi-Fi disconnected! Restarting WiFiManager...");
    WiFi.disconnect();                 // ตัดการเชื่อมต่อปัจจุบันก่อน
    wm.autoConnect("ESP32S3-Modbus");  // ใช้ WiFiManager เชื่อมต่อใหม่
  }

  // ตรวจสอบสถานะการเชื่อมต่อ Blynk
  if (!Blynk.connected()) {  // หาก Blynk ไม่ได้เชื่อมต่อ
    Serial.println("Blynk disconnected! Reconnecting...");
    Blynk.connect();  // พยายามเชื่อมต่อ Blynk ใหม่
  }
}

BLYNK_WRITE(V1) {
  int value = param.asInt();
  relayState[0] = value;  // บันทึกสถานะ Relay 1
  uint16_t regValue = (value == 1) ? 0x0100 : 0x0200;
  node1.writeSingleRegister(0x0001, regValue);
  updateRelayStatusLCD();  // อัปเดต LCD
}
BLYNK_WRITE(V2) {
  int value = param.asInt();
  relayState[1] = value;  // บันทึกสถานะ Relay 2
  uint16_t regValue = (value == 1) ? 0x0100 : 0x0200;
  node1.writeSingleRegister(0x0002, regValue);
  updateRelayStatusLCD();  // อัปเดต LCD
}
BLYNK_WRITE(V3) {
  int value = param.asInt();
  relayState[2] = value;  // บันทึกสถานะ Relay 3
  uint16_t regValue = (value == 1) ? 0x0100 : 0x0200;
  node1.writeSingleRegister(0x0003, regValue);
  updateRelayStatusLCD();  // อัปเดต LCD
}
BLYNK_WRITE(V4) {
  int value = param.asInt();
  relayState[3] = value;  // บันทึกสถานะ Relay 4
  uint16_t regValue = (value == 1) ? 0x0100 : 0x0200;
  node1.writeSingleRegister(0x0004, regValue);
  updateRelayStatusLCD();  // อัปเดต LCD
}
BLYNK_WRITE(V5) {
  int value = param.asInt();
  relayState[4] = value;  // บันทึกสถานะ Relay 5
  uint16_t regValue = (value == 1) ? 0x0100 : 0x0200;
  node1.writeSingleRegister(0x0005, regValue);
  updateRelayStatusLCD();  // อัปเดต LCD
}
BLYNK_WRITE(V6) {
  int value = param.asInt();
  relayState[5] = value;  // บันทึกสถานะ Relay 6
  uint16_t regValue = (value == 1) ? 0x0100 : 0x0200;
  node1.writeSingleRegister(0x0006, regValue);
  updateRelayStatusLCD();  // อัปเดต LCD
}
BLYNK_WRITE(V7) {
  int value = param.asInt();
  relayState[6] = value;  // บันทึกสถานะ Relay 7
  uint16_t regValue = (value == 1) ? 0x0100 : 0x0200;
  node1.writeSingleRegister(0x0007, regValue);
  updateRelayStatusLCD();  // อัปเดต LCD
}
BLYNK_WRITE(V8) {
  int value = param.asInt();
  relayState[7] = value;  // บันทึกสถานะ Relay 8
  uint16_t regValue = (value == 1) ? 0x0100 : 0x0200;
  node1.writeSingleRegister(0x0008, regValue);
  updateRelayStatusLCD();  // อัปเดต LCD
}
void updateRelayStatusLCD() {
  lcd.clear();
  
  // บรรทัดที่ 1: ข้อความคงที่
  lcd.setCursor(0, 0);
  lcd.print("#SMF 081-411-1142#");

  // บรรทัดที่ 2: Relay 1-3
  lcd.setCursor(0, 1);
  lcd.print("R1:");
  lcd.print(relayState[0] ? "ON " : "OFF");
  lcd.print(" R2:");
  lcd.print(relayState[1] ? "ON " : "OFF");
  lcd.print(" R3:");
  lcd.print(relayState[2] ? "ON " : "OFF");

  // บรรทัดที่ 3: Relay 4-6
  lcd.setCursor(0, 2);
  lcd.print("R4:");
  lcd.print(relayState[3] ? "ON " : "OFF");
  lcd.print(" R5:");
  lcd.print(relayState[4] ? "ON " : "OFF");
  lcd.print(" R6:");
  lcd.print(relayState[5] ? "ON " : "OFF");

  // บรรทัดที่ 4: Relay 7-8
  lcd.setCursor(0, 3);
  lcd.print("R7:");
  lcd.print(relayState[6] ? "ON " : "OFF");
  lcd.print(" R8:");
  lcd.print(relayState[7] ? "ON " : "OFF");
}

// ฟังก์ชัน loop() ทำงานในลูปอย่างต่อเนื่อง
void loop() {
  // ตรวจสอบว่าการเชื่อมต่อ Blynk ยังเปิดอยู่หรือไม่
  if (Blynk.connected()) {
    Blynk.run();  // หากเชื่อมต่ออยู่ ให้รันกระบวนการของ Blynk (จัดการการสื่อสารและอีเวนต์ต่าง ๆ)
  }

  // รันงานที่กำหนดไว้ในตัวจับเวลา BlynkTimer
  timer.run();  // ตรวจสอบและเรียกใช้ฟังก์ชันตามรอบเวลาที่กำหนดในตัวจับเวลา

  // เพิ่มดีเลย์เล็กน้อยเพื่อป้องกัน Watchdog Timeout
  delay(1);  // หน่วงเวลา 1 มิลลิวินาที เพื่อให้ระบบ ESP32 ทำงานพื้นฐานอื่น ๆ เช่น Wi-Fi
}
