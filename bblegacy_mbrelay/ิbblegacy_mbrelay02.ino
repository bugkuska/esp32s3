#include <Wire.h>                  // ไลบรารีสำหรับ I2C
#include <WiFi.h>                  // ไลบรารีสำหรับการเชื่อมต่อ Wi-Fi
#include <WiFiClient.h>            // ไลบรารีสำหรับการจัดการ Wi-Fi client
#include <BlynkSimpleEsp32.h>      // ไลบรารีสำหรับการเชื่อมต่อ Blynk กับ ESP32
#include <ModbusMaster.h>          // ไลบรารีสำหรับการใช้งาน Modbus Master

// Wi-Fi and Blynk credentials
const char ssid[] = "579smf001";   // ชื่อ Wi-Fi SSID
const char pass[] = "0814111142";  // รหัสผ่าน Wi-Fi
const char auth[] = "RANBbnsmWwvEmoi3HZQEVAe7bF0UJteU";  // โทเคน Blynk

// Modbus settings
#define RXD 18  // กำหนดขา RX ของ ESP32 (เชื่อมต่อกับ TX ของอุปกรณ์ Modbus)
#define TXD 17  // กำหนดขา TX ของ ESP32 (เชื่อมต่อกับ RX ของอุปกรณ์ Modbus)
ModbusMaster node1;  // สร้างออบเจกต์สำหรับใช้งาน Modbus

// Timer for periodic tasks
BlynkTimer timer;  // ตัวจับเวลา Blynk สำหรับเรียกใช้ฟังก์ชันตามรอบเวลาที่กำหนด

// Debugging LED (optional, GPIO2 is onboard LED on ESP32)
#define DEBUG_LED 2  // กำหนดขา GPIO2 เป็น LED สำหรับแสดงสถานะ Debug (ถ้าต้องการ)

// Function prototypes
void checkConnections();  // ฟังก์ชันสำหรับตรวจสอบการเชื่อมต่อ Wi-Fi และ Blynk
void debugModbus();       // ฟังก์ชันสำหรับ Debug การเชื่อมต่อ Modbus

// Define Blynk virtual pins
#define Widget_Btn_SW1 V1   // กำหนดปุ่ม Virtual Pin V1 สำหรับ Relay 1
#define Widget_Btn_SW2 V2   // กำหนดปุ่ม Virtual Pin V2 สำหรับ Relay 2
#define Widget_Btn_SW3 V3   // กำหนดปุ่ม Virtual Pin V3 สำหรับ Relay 3
#define Widget_Btn_SW4 V4   // กำหนดปุ่ม Virtual Pin V4 สำหรับ Relay 4
#define Widget_Btn_SW5 V5   // กำหนดปุ่ม Virtual Pin V5 สำหรับ Relay 5
#define Widget_Btn_SW6 V6   // กำหนดปุ่ม Virtual Pin V6 สำหรับ Relay 6
#define Widget_Btn_SW7 V7   // กำหนดปุ่ม Virtual Pin V7 สำหรับ Relay 7
#define Widget_Btn_SW8 V8   // กำหนดปุ่ม Virtual Pin V8 สำหรับ Relay 8
#define Widget_Btn_SW9 V9   // กำหนดปุ่ม Virtual Pin V9 สำหรับ Relay 9
#define Widget_Btn_SW10 V10 // กำหนดปุ่ม Virtual Pin V10 สำหรับ Relay 10
#define Widget_Btn_SW11 V11 // กำหนดปุ่ม Virtual Pin V11 สำหรับ Relay 11
#define Widget_Btn_SW12 V12 // กำหนดปุ่ม Virtual Pin V12 สำหรับ Relay 12

// Array to track if relays are being updated from the app
bool isUpdatingFromApp[12] = { false };  
// ใช้เก็บสถานะว่า Relay แต่ละตัวถูกควบคุมจากแอปหรือไม่


void setup() {
  // Initialize serial communication
  Serial.begin(115200);  // เริ่มการสื่อสาร Serial (ความเร็ว 115200 bps) สำหรับ Debugging
  Serial2.begin(9600, SERIAL_8N1, RXD, TXD);  // เริ่มการสื่อสาร Serial2 สำหรับ Modbus (9600 bps, 8-bit data, no parity, 1 stop bit)

  // Initialize Modbus
  node1.begin(1, Serial2);  // ตั้งค่า Modbus Address 1 และใช้ Serial2 เป็นการสื่อสาร

  // Debugging LED setup
  pinMode(DEBUG_LED, OUTPUT);  // กำหนดขา DEBUG_LED เป็นขา Output
  digitalWrite(DEBUG_LED, LOW);  // ปิด LED Debug เริ่มต้น

  // Connect to Wi-Fi
  Serial.println("Connecting to Wi-Fi...");  // แสดงข้อความเริ่มเชื่อมต่อ Wi-Fi
  WiFi.begin(ssid, pass);  // เชื่อมต่อ Wi-Fi ด้วย SSID และรหัสผ่านที่กำหนด
  unsigned long startAttemptTime = millis();  // บันทึกเวลาที่เริ่มต้นการพยายามเชื่อมต่อ

  // รอจนกว่า Wi-Fi จะเชื่อมต่อสำเร็จ หรือครบ 10 วินาที
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(500);  // รอ 500 มิลลิวินาที
    Serial.print(".");  // แสดงจุดเพื่อบอกสถานะการเชื่อมต่อ
  }

  // ตรวจสอบว่า Wi-Fi เชื่อมต่อสำเร็จหรือไม่
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWi-Fi connected");  // แสดงข้อความเมื่อเชื่อมต่อสำเร็จ
    Serial.print("IP Address: ");  // แสดงข้อความ IP Address
    Serial.println(WiFi.localIP());  // แสดง IP Address ของ ESP32
  } else {
    Serial.println("\nFailed to connect to Wi-Fi");  // แสดงข้อความเมื่อเชื่อมต่อไม่สำเร็จ
  }

  // Enable Wi-Fi auto-reconnect
  WiFi.setAutoReconnect(true);  // เปิดการเชื่อมต่อ Wi-Fi อัตโนมัติเมื่อหลุด
  WiFi.persistent(true);  // บันทึกการตั้งค่า Wi-Fi ลงใน Flash Memory

  // Connect to Blynk server
  Serial.println("Connecting to Blynk...");  // แสดงข้อความเริ่มเชื่อมต่อ Blynk
  Blynk.begin(auth, ssid, pass, "iotservices.thddns.net", 5535);  
  // เชื่อมต่อ Blynk Server ด้วย Token, SSID, Password และระบุเซิร์ฟเวอร์กับพอร์ต

  // Set timers
  timer.setInterval(5000L, checkConnections);  // ตั้งตัวจับเวลา 5 วินาทีเพื่อเรียกใช้ฟังก์ชัน `checkConnections`
  //timer.setInterval(2000L, debugModbus);  // ตัวจับเวลาสำหรับ Debug Modbus (ปิดการใช้งานไว้)
  timer.setInterval(5000L, updateRelayStatus);  // ตั้งตัวจับเวลา 5 วินาทีเพื่ออัปเดตสถานะ Relay
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
  Blynk.syncVirtual(Widget_Btn_SW9);  // ซิงโครไนซ์สถานะของปุ่ม Virtual Pin V9
  Blynk.syncVirtual(Widget_Btn_SW10); // ซิงโครไนซ์สถานะของปุ่ม Virtual Pin V10
  Blynk.syncVirtual(Widget_Btn_SW11); // ซิงโครไนซ์สถานะของปุ่ม Virtual Pin V11
  Blynk.syncVirtual(Widget_Btn_SW12); // ซิงโครไนซ์สถานะของปุ่ม Virtual Pin V12
}

// ฟังก์ชันสำหรับตรวจสอบการเชื่อมต่อ Wi-Fi และ Blynk
void checkConnections() {
  // ตรวจสอบสถานะการเชื่อมต่อ Wi-Fi
  if (WiFi.status() != WL_CONNECTED) {  // หาก Wi-Fi ไม่ได้เชื่อมต่อ
    Serial.println("Wi-Fi disconnected! Reconnecting...");  // แสดงข้อความใน Serial Monitor
    WiFi.begin(ssid, pass);  // พยายามเชื่อมต่อ Wi-Fi ใหม่
  }

  // ตรวจสอบสถานะการเชื่อมต่อ Blynk
  if (!Blynk.connected()) {  // หาก Blynk ไม่ได้เชื่อมต่อ
    Serial.println("Blynk disconnected! Reconnecting...");  // แสดงข้อความใน Serial Monitor
    Blynk.connect();  // พยายามเชื่อมต่อ Blynk ใหม่
  }
}

/*
// ฟังก์ชันสำหรับ Debug การสื่อสาร Modbus
void debugModbus() {
  // ตัวอย่าง: อ่านค่าจาก Modbus Register Address 0x0000
  uint8_t result = node1.readHoldingRegisters(0x0000, 1);  
  // ส่งคำสั่งอ่านค่า Holding Register ที่ Address 0x0000 จำนวน 1 ค่า

  yield();  // ป้องกัน Watchdog Timeout (ช่วยให้ ESP32 สามารถดำเนินงานอื่น ๆ ได้ในระหว่างนี้)

  // ตรวจสอบผลลัพธ์ของคำสั่ง Modbus
  if (result == node1.ku8MBSuccess) {  // หากคำสั่งสำเร็จ
    Serial.print("Modbus Data: ");  // แสดงข้อความ "Modbus Data" ใน Serial Monitor
    Serial.println(node1.getResponseBuffer(0));  // แสดงค่าที่อ่านได้จาก Register Address 0x0000
  } else {  // หากคำสั่งล้มเหลว
    Serial.print("Modbus Error: ");  // แสดงข้อความ "Modbus Error" ใน Serial Monitor
    Serial.println(result);  // แสดงรหัสข้อผิดพลาด
  }
}

*/
// ฟังก์ชันสำหรับอัปเดตสถานะของรีเลย์จาก Modbus และส่งสถานะไปยัง Blynk
void updateRelayStatus() {
  // วนลูปเพื่อตรวจสอบรีเลย์ทั้งหมด 12 ตัว (index 0 ถึง 11)
  for (int i = 0; i < 12; i++) {
    // ตรวจสอบว่ารีเลย์ตัวนี้ไม่ได้ถูกควบคุมจากแอป
    if (!isUpdatingFromApp[i]) {  // ถ้ารีเลย์ตัวนี้ไม่อยู่ในสถานะ "Updating from App"
      uint8_t result = node1.readCoils(i, 1);  // อ่านสถานะของคอยล์ Modbus ทีละ 1 ตัว

      // ตรวจสอบว่าการอ่านสำเร็จหรือไม่
      if (result == node1.ku8MBSuccess) {
        // ดึงค่าของรีเลย์จาก Response Buffer
        bool relayState = node1.getResponseBuffer(0);

        // แสดงสถานะของรีเลย์ใน Serial Monitor
        Serial.print("Relay ");
        Serial.print(i + 1);  // ตัวเลขรีเลย์ (เริ่มที่ 1)
        Serial.print(": ");
        Serial.println(relayState);  // แสดงสถานะ (0 หรือ 1)

        // อัปเดตสถานะของรีเลย์ใน Blynk Virtual Pin
        Blynk.virtualWrite(V1 + i, relayState);  // Virtual Pin เริ่มจาก V1 ถึง V12
      } else {
        // หากการอ่านล้มเหลว แสดงข้อความข้อผิดพลาดใน Serial Monitor
        Serial.print("Failed to read Relay ");
        Serial.print(i + 1);  // ตัวเลขรีเลย์ (เริ่มที่ 1)
        Serial.print(": Error Code ");
        Serial.println(result);  // แสดงรหัสข้อผิดพลาด
      }
    }
  }
}

// ฟังก์ชัน BLYNK_WRITE สำหรับ Relay 1
// ฟังก์ชันนี้จะถูกเรียกเมื่อมีการเปลี่ยนแปลงสถานะของ Widget_Btn_SW1 ในแอป Blynk
BLYNK_WRITE(Widget_Btn_SW1) {
  // อ่านค่าจาก Widget_Btn_SW1 (ค่าที่ส่งมาจะเป็น 0 หรือ 1) และเก็บไว้ในตัวแปร valueSW1
  int valueSW1 = param.asInt();

  // กำหนดสถานะว่า Relay 1 กำลังถูกควบคุมจากแอป โดยตั้งค่า isUpdatingFromApp[0] เป็น true
  isUpdatingFromApp[0] = true;

  // อ่านสถานะปัจจุบันของคอยล์ใน Modbus Address 0x0000
  uint8_t result = node1.readCoils(0x0000, 1);

  // ตรวจสอบว่าการอ่านสถานะสำเร็จหรือไม่
  if (result == node1.ku8MBSuccess) {
    // หากการอ่านสำเร็จ ดึงสถานะปัจจุบันของคอยล์จาก buffer และเก็บไว้ในตัวแปร currentStatus
    bool currentStatus = node1.getResponseBuffer(0);

    // เปรียบเทียบสถานะใหม่ (valueSW1) กับสถานะปัจจุบัน (currentStatus)
    if (valueSW1 != currentStatus) {
      // หากสถานะใหม่ไม่ตรงกับสถานะปัจจุบัน เขียนสถานะใหม่ลงในคอยล์ที่ Modbus Address 0x0000
      if (node1.writeSingleCoil(0x0000, valueSW1) == node1.ku8MBSuccess) {
        // หากเขียนสำเร็จ แสดงข้อความยืนยันใน Serial Monitor
        Serial.println(valueSW1 ? "SW1 ON success" : "SW1 OFF success");

        // อัปเดตสถานะของ Widget_Btn_SW1 ในแอป Blynk ให้ตรงกับสถานะใหม่
        Blynk.virtualWrite(Widget_Btn_SW1, valueSW1);
      } else {
        // หากการเขียนล้มเหลว แสดงข้อความข้อผิดพลาดใน Serial Monitor
        Serial.println(valueSW1 ? "SW1 ON failed" : "SW1 OFF failed");
      }
    } else {
      // หากสถานะใหม่เหมือนกับสถานะปัจจุบัน ไม่ต้องทำอะไรเพิ่มเติม และแสดงข้อความใน Serial Monitor
      Serial.println("No change in SW1 status.");
    }
  } else {
    // หากการอ่านสถานะคอยล์ล้มเหลว แสดงรหัสข้อผิดพลาดใน Serial Monitor
    Serial.print("Failed to read coil status: ");
    Serial.println(result);
  }

  // รีเซ็ตตัวแปร isUpdatingFromApp[0] เป็น false
  // เพื่อระบุว่า Relay 1 ไม่ได้ถูกควบคุมจากแอปอีกต่อไป
  isUpdatingFromApp[0] = false;
}

// BLYNK_WRITE สำหรับ Relay 2
// ฟังก์ชันนี้จะทำงานเมื่อแอป Blynk มีการเปลี่ยนแปลงสถานะของ Widget_Btn_SW2
BLYNK_WRITE(Widget_Btn_SW2) {
  int valueSW2 = param.asInt();  // อ่านค่าจาก Widget_Btn_SW2 (0 หรือ 1)

  isUpdatingFromApp[1] = true;  // กำหนดให้สถานะบอกว่า Relay 2 ถูกควบคุมจากแอป

  // อ่านสถานะปัจจุบันของคอยล์ที่อยู่ใน Modbus Address 0x0001
  uint8_t result = node1.readCoils(0x0001, 1);
  if (result == node1.ku8MBSuccess) {  // ตรวจสอบว่าการอ่านสำเร็จหรือไม่
    bool currentStatus = node1.getResponseBuffer(0);  // ดึงสถานะปัจจุบันของคอยล์

    // ตรวจสอบว่าสถานะใหม่ (valueSW2) แตกต่างจากสถานะปัจจุบันหรือไม่
    if (valueSW2 != currentStatus) {
      // เขียนสถานะใหม่ไปยังคอยล์ใน Modbus Address 0x0001
      if (node1.writeSingleCoil(0x0001, valueSW2) == node1.ku8MBSuccess) {
        // หากเขียนสำเร็จ แสดงข้อความใน Serial Monitor และอัปเดต Widget
        Serial.println(valueSW2 ? "SW2 ON success" : "SW2 OFF success");
        Blynk.virtualWrite(Widget_Btn_SW2, valueSW2);  // อัปเดตสถานะในแอป Blynk
      } else {
        // หากเขียนล้มเหลว แสดงข้อความใน Serial Monitor
        Serial.println(valueSW2 ? "SW2 ON failed" : "SW2 OFF failed");
      }
    } else {
      // หากสถานะไม่เปลี่ยน แสดงข้อความใน Serial Monitor
      Serial.println("No change in SW2 status.");
    }
  } else {
    // หากการอ่านสถานะคอยล์ล้มเหลว แสดงรหัสข้อผิดพลาดใน Serial Monitor
    Serial.print("Failed to read coil status: ");
    Serial.println(result);
  }

  isUpdatingFromApp[1] = false;  // รีเซ็ตสถานะว่าไม่ได้ควบคุมจากแอปอีกต่อไป
}
// ฟังก์ชัน BLYNK_WRITE สำหรับ Relay 3
// ฟังก์ชันนี้จะถูกเรียกเมื่อมีการเปลี่ยนแปลงสถานะของ Widget_Btn_SW3 ในแอป Blynk
BLYNK_WRITE(Widget_Btn_SW3) {
  // อ่านค่าจาก Widget_Btn_SW3 (ค่าที่ได้รับจะเป็น 0 หรือ 1) และเก็บในตัวแปร valueSW3
  int valueSW3 = param.asInt();

  // กำหนดตัวแปร isUpdatingFromApp[2] ให้เป็น true
  // เพื่อระบุว่า Relay 3 กำลังถูกควบคุมจากแอป
  isUpdatingFromApp[2] = true;

  // เรียกใช้คำสั่ง Modbus เพื่ออ่านสถานะของคอยล์ที่ Modbus Address 0x0002
  uint8_t result = node1.readCoils(0x0002, 1);

  // ตรวจสอบว่าการอ่านสถานะสำเร็จหรือไม่
  if (result == node1.ku8MBSuccess) {
    // หากการอ่านสำเร็จ ให้ดึงสถานะปัจจุบันของคอยล์จาก buffer และเก็บในตัวแปร currentStatus
    bool currentStatus = node1.getResponseBuffer(0);

    // เปรียบเทียบสถานะใหม่ (valueSW3) กับสถานะปัจจุบัน (currentStatus)
    if (valueSW3 != currentStatus) {
      // หากสถานะใหม่ไม่ตรงกับสถานะปัจจุบัน เขียนสถานะใหม่ลงในคอยล์
      if (node1.writeSingleCoil(0x0002, valueSW3) == node1.ku8MBSuccess) {
        // หากเขียนสำเร็จ แสดงข้อความยืนยันใน Serial Monitor
        Serial.println(valueSW3 ? "SW3 ON success" : "SW3 OFF success");

        // อัปเดตสถานะใน Widget_Btn_SW3 ในแอป Blynk ให้ตรงกับสถานะใหม่
        Blynk.virtualWrite(Widget_Btn_SW3, valueSW3);
      } else {
        // หากการเขียนล้มเหลว แสดงข้อความข้อผิดพลาดใน Serial Monitor
        Serial.println(valueSW3 ? "SW3 ON failed" : "SW3 OFF failed");
      }
    } else {
      // หากสถานะใหม่เหมือนกับสถานะปัจจุบัน ไม่ต้องทำอะไรเพิ่มเติม
      Serial.println("No change in SW3 status.");
    }
  } else {
    // หากการอ่านสถานะคอยล์ล้มเหลว แสดงรหัสข้อผิดพลาดใน Serial Monitor
    Serial.print("Failed to read coil status: ");
    Serial.println(result);
  }

  // รีเซ็ตตัวแปร isUpdatingFromApp[2] เป็น false
  // เพื่อระบุว่า Relay 3 ไม่ได้ถูกควบคุมจากแอปอีกต่อไป
  isUpdatingFromApp[2] = false;
}

// ฟังก์ชัน BLYNK_WRITE สำหรับ Relay 4
// ฟังก์ชันนี้จะถูกเรียกเมื่อมีการเปลี่ยนแปลงสถานะของ Widget_Btn_SW4 ในแอป Blynk
BLYNK_WRITE(Widget_Btn_SW4) {
  // อ่านค่าจาก Widget_Btn_SW4 (ค่าที่ส่งมาจะเป็น 0 หรือ 1) และเก็บไว้ในตัวแปร valueSW4
  int valueSW4 = param.asInt();

  // กำหนดสถานะว่า Relay 4 กำลังถูกควบคุมจากแอป โดยตั้งค่า isUpdatingFromApp[3] เป็น true
  isUpdatingFromApp[3] = true;

  // อ่านสถานะปัจจุบันของคอยล์ใน Modbus Address 0x0003
  uint8_t result = node1.readCoils(0x0003, 1);

  // ตรวจสอบว่าการอ่านสถานะสำเร็จหรือไม่
  if (result == node1.ku8MBSuccess) {
    // หากการอ่านสำเร็จ ดึงสถานะปัจจุบันของคอยล์จาก buffer และเก็บไว้ในตัวแปร currentStatus
    bool currentStatus = node1.getResponseBuffer(0);

    // เปรียบเทียบสถานะใหม่ (valueSW4) กับสถานะปัจจุบัน (currentStatus)
    if (valueSW4 != currentStatus) {
      // หากสถานะใหม่ไม่ตรงกับสถานะปัจจุบัน เขียนสถานะใหม่ลงในคอยล์ที่ Modbus Address 0x0003
      if (node1.writeSingleCoil(0x0003, valueSW4) == node1.ku8MBSuccess) {
        // หากเขียนสำเร็จ แสดงข้อความยืนยันใน Serial Monitor
        Serial.println(valueSW4 ? "SW4 ON success" : "SW4 OFF success");

        // อัปเดตสถานะของ Widget_Btn_SW4 ในแอป Blynk ให้ตรงกับสถานะใหม่
        Blynk.virtualWrite(Widget_Btn_SW4, valueSW4);
      } else {
        // หากการเขียนล้มเหลว แสดงข้อความข้อผิดพลาดใน Serial Monitor
        Serial.println(valueSW4 ? "SW4 ON failed" : "SW4 OFF failed");
      }
    } else {
      // หากสถานะใหม่เหมือนกับสถานะปัจจุบัน ไม่ต้องทำอะไรเพิ่มเติม และแสดงข้อความใน Serial Monitor
      Serial.println("No change in SW4 status.");
    }
  } else {
    // หากการอ่านสถานะคอยล์ล้มเหลว แสดงรหัสข้อผิดพลาดใน Serial Monitor
    Serial.print("Failed to read coil status: ");
    Serial.println(result);
  }

  // รีเซ็ตตัวแปร isUpdatingFromApp[3] เป็น false
  // เพื่อระบุว่า Relay 4 ไม่ได้ถูกควบคุมจากแอปอีกต่อไป
  isUpdatingFromApp[3] = false;
}

// ฟังก์ชัน BLYNK_WRITE สำหรับ Relay 5
// ฟังก์ชันนี้จะถูกเรียกเมื่อมีการเปลี่ยนแปลงสถานะของ Widget_Btn_SW5 ในแอป Blynk
BLYNK_WRITE(Widget_Btn_SW5) {
  // อ่านค่าจาก Widget_Btn_SW5 (ค่าที่ส่งมาจะเป็น 0 หรือ 1) และเก็บไว้ในตัวแปร valueSW5
  int valueSW5 = param.asInt();

  // กำหนดสถานะว่า Relay 5 กำลังถูกควบคุมจากแอป โดยตั้งค่า isUpdatingFromApp[4] เป็น true
  isUpdatingFromApp[4] = true;

  // อ่านสถานะปัจจุบันของคอยล์ใน Modbus Address 0x0004
  uint8_t result = node1.readCoils(0x0004, 1);

  // ตรวจสอบว่าการอ่านสถานะสำเร็จหรือไม่
  if (result == node1.ku8MBSuccess) {
    // หากการอ่านสำเร็จ ดึงสถานะปัจจุบันของคอยล์จาก buffer และเก็บไว้ในตัวแปร currentStatus
    bool currentStatus = node1.getResponseBuffer(0);

    // เปรียบเทียบสถานะใหม่ (valueSW5) กับสถานะปัจจุบัน (currentStatus)
    if (valueSW5 != currentStatus) {
      // หากสถานะใหม่ไม่ตรงกับสถานะปัจจุบัน เขียนสถานะใหม่ลงในคอยล์ที่ Modbus Address 0x0004
      if (node1.writeSingleCoil(0x0004, valueSW5) == node1.ku8MBSuccess) {
        // หากเขียนสำเร็จ แสดงข้อความยืนยันใน Serial Monitor
        Serial.println(valueSW5 ? "SW5 ON success" : "SW5 OFF success");

        // อัปเดตสถานะของ Widget_Btn_SW5 ในแอป Blynk ให้ตรงกับสถานะใหม่
        Blynk.virtualWrite(Widget_Btn_SW5, valueSW5);
      } else {
        // หากการเขียนล้มเหลว แสดงข้อความข้อผิดพลาดใน Serial Monitor
        Serial.println(valueSW5 ? "SW5 ON failed" : "SW5 OFF failed");
      }
    } else {
      // หากสถานะใหม่เหมือนกับสถานะปัจจุบัน ไม่ต้องทำอะไรเพิ่มเติม และแสดงข้อความใน Serial Monitor
      Serial.println("No change in SW5 status.");
    }
  } else {
    // หากการอ่านสถานะคอยล์ล้มเหลว แสดงรหัสข้อผิดพลาดใน Serial Monitor
    Serial.print("Failed to read coil status: ");
    Serial.println(result);
  }

  // รีเซ็ตตัวแปร isUpdatingFromApp[4] เป็น false
  // เพื่อระบุว่า Relay 5 ไม่ได้ถูกควบคุมจากแอปอีกต่อไป
  isUpdatingFromApp[4] = false;
}

// ฟังก์ชัน BLYNK_WRITE สำหรับ Relay 6
// ฟังก์ชันนี้จะถูกเรียกเมื่อมีการเปลี่ยนแปลงสถานะของ Widget_Btn_SW6 ในแอป Blynk
BLYNK_WRITE(Widget_Btn_SW6) {
  // อ่านค่าจาก Widget_Btn_SW6 (ค่าที่ส่งมาจะเป็น 0 หรือ 1) และเก็บไว้ในตัวแปร valueSW6
  int valueSW6 = param.asInt();

  // กำหนดสถานะว่า Relay 6 กำลังถูกควบคุมจากแอป โดยตั้งค่า isUpdatingFromApp[5] เป็น true
  isUpdatingFromApp[5] = true;

  // อ่านสถานะปัจจุบันของคอยล์ใน Modbus Address 0x0005
  uint8_t result = node1.readCoils(0x0005, 1);

  // ตรวจสอบว่าการอ่านสถานะสำเร็จหรือไม่
  if (result == node1.ku8MBSuccess) {
    // หากการอ่านสำเร็จ ดึงสถานะปัจจุบันของคอยล์จาก buffer และเก็บไว้ในตัวแปร currentStatus
    bool currentStatus = node1.getResponseBuffer(0);

    // เปรียบเทียบสถานะใหม่ (valueSW6) กับสถานะปัจจุบัน (currentStatus)
    if (valueSW6 != currentStatus) {
      // หากสถานะใหม่ไม่ตรงกับสถานะปัจจุบัน เขียนสถานะใหม่ลงในคอยล์ที่ Modbus Address 0x0005
      if (node1.writeSingleCoil(0x0005, valueSW6) == node1.ku8MBSuccess) {
        // หากเขียนสำเร็จ แสดงข้อความยืนยันใน Serial Monitor
        Serial.println(valueSW6 ? "SW6 ON success" : "SW6 OFF success");

        // อัปเดตสถานะของ Widget_Btn_SW6 ในแอป Blynk ให้ตรงกับสถานะใหม่
        Blynk.virtualWrite(Widget_Btn_SW6, valueSW6);
      } else {
        // หากการเขียนล้มเหลว แสดงข้อความข้อผิดพลาดใน Serial Monitor
        Serial.println(valueSW6 ? "SW6 ON failed" : "SW6 OFF failed");
      }
    } else {
      // หากสถานะใหม่เหมือนกับสถานะปัจจุบัน ไม่ต้องทำอะไรเพิ่มเติม และแสดงข้อความใน Serial Monitor
      Serial.println("No change in SW6 status.");
    }
  } else {
    // หากการอ่านสถานะคอยล์ล้มเหลว แสดงรหัสข้อผิดพลาดใน Serial Monitor
    Serial.print("Failed to read coil status: ");
    Serial.println(result);
  }

  // รีเซ็ตตัวแปร isUpdatingFromApp[5] เป็น false
  // เพื่อระบุว่า Relay 6 ไม่ได้ถูกควบคุมจากแอปอีกต่อไป
  isUpdatingFromApp[5] = false;
}

// ฟังก์ชัน BLYNK_WRITE สำหรับ Relay 7
// ฟังก์ชันนี้จะถูกเรียกเมื่อมีการเปลี่ยนแปลงสถานะของ Widget_Btn_SW7 ในแอป Blynk
BLYNK_WRITE(Widget_Btn_SW7) {
  // อ่านค่าจาก Widget_Btn_SW7 (ค่าที่ส่งมาจะเป็น 0 หรือ 1) และเก็บไว้ในตัวแปร valueSW7
  int valueSW7 = param.asInt();

  // กำหนดสถานะว่า Relay 7 กำลังถูกควบคุมจากแอป โดยตั้งค่า isUpdatingFromApp[6] เป็น true
  isUpdatingFromApp[6] = true;

  // อ่านสถานะปัจจุบันของคอยล์ใน Modbus Address 0x0006
  uint8_t result = node1.readCoils(0x0006, 1);

  // ตรวจสอบว่าการอ่านสถานะสำเร็จหรือไม่
  if (result == node1.ku8MBSuccess) {
    // หากการอ่านสำเร็จ ดึงสถานะปัจจุบันของคอยล์จาก buffer และเก็บไว้ในตัวแปร currentStatus
    bool currentStatus = node1.getResponseBuffer(0);

    // เปรียบเทียบสถานะใหม่ (valueSW7) กับสถานะปัจจุบัน (currentStatus)
    if (valueSW7 != currentStatus) {
      // หากสถานะใหม่ไม่ตรงกับสถานะปัจจุบัน เขียนสถานะใหม่ลงในคอยล์ที่ Modbus Address 0x0006
      if (node1.writeSingleCoil(0x0006, valueSW7) == node1.ku8MBSuccess) {
        // หากเขียนสำเร็จ แสดงข้อความยืนยันใน Serial Monitor
        Serial.println(valueSW7 ? "SW7 ON success" : "SW7 OFF success");

        // อัปเดตสถานะของ Widget_Btn_SW7 ในแอป Blynk ให้ตรงกับสถานะใหม่
        Blynk.virtualWrite(Widget_Btn_SW7, valueSW7);
      } else {
        // หากการเขียนล้มเหลว แสดงข้อความข้อผิดพลาดใน Serial Monitor
        Serial.println(valueSW7 ? "SW7 ON failed" : "SW7 OFF failed");
      }
    } else {
      // หากสถานะใหม่เหมือนกับสถานะปัจจุบัน ไม่ต้องทำอะไรเพิ่มเติม และแสดงข้อความใน Serial Monitor
      Serial.println("No change in SW7 status.");
    }
  } else {
    // หากการอ่านสถานะคอยล์ล้มเหลว แสดงรหัสข้อผิดพลาดใน Serial Monitor
    Serial.print("Failed to read coil status: ");
    Serial.println(result);
  }

  // รีเซ็ตตัวแปร isUpdatingFromApp[6] เป็น false
  // เพื่อระบุว่า Relay 7 ไม่ได้ถูกควบคุมจากแอปอีกต่อไป
  isUpdatingFromApp[6] = false;
}

// ฟังก์ชัน BLYNK_WRITE สำหรับ Relay 8
// ฟังก์ชันนี้จะถูกเรียกเมื่อมีการเปลี่ยนแปลงสถานะของ Widget_Btn_SW8 ในแอป Blynk
BLYNK_WRITE(Widget_Btn_SW8) {
  // อ่านค่าจาก Widget_Btn_SW8 (ค่าที่ส่งมาจะเป็น 0 หรือ 1) และเก็บไว้ในตัวแปร valueSW8
  int valueSW8 = param.asInt();

  // กำหนดสถานะว่า Relay 8 กำลังถูกควบคุมจากแอป โดยตั้งค่า isUpdatingFromApp[7] เป็น true
  isUpdatingFromApp[7] = true;

  // อ่านสถานะปัจจุบันของคอยล์ใน Modbus Address 0x0007
  uint8_t result = node1.readCoils(0x0007, 1);

  // ตรวจสอบว่าการอ่านสถานะสำเร็จหรือไม่
  if (result == node1.ku8MBSuccess) {
    // หากการอ่านสำเร็จ ดึงสถานะปัจจุบันของคอยล์จาก buffer และเก็บไว้ในตัวแปร currentStatus
    bool currentStatus = node1.getResponseBuffer(0);

    // เปรียบเทียบสถานะใหม่ (valueSW8) กับสถานะปัจจุบัน (currentStatus)
    if (valueSW8 != currentStatus) {
      // หากสถานะใหม่ไม่ตรงกับสถานะปัจจุบัน เขียนสถานะใหม่ลงในคอยล์ที่ Modbus Address 0x0007
      if (node1.writeSingleCoil(0x0007, valueSW8) == node1.ku8MBSuccess) {
        // หากเขียนสำเร็จ แสดงข้อความยืนยันใน Serial Monitor
        Serial.println(valueSW8 ? "SW8 ON success" : "SW8 OFF success");

        // อัปเดตสถานะของ Widget_Btn_SW8 ในแอป Blynk ให้ตรงกับสถานะใหม่
        Blynk.virtualWrite(Widget_Btn_SW8, valueSW8);
      } else {
        // หากการเขียนล้มเหลว แสดงข้อความข้อผิดพลาดใน Serial Monitor
        Serial.println(valueSW8 ? "SW8 ON failed" : "SW8 OFF failed");
      }
    } else {
      // หากสถานะใหม่เหมือนกับสถานะปัจจุบัน ไม่ต้องทำอะไรเพิ่มเติม และแสดงข้อความใน Serial Monitor
      Serial.println("No change in SW8 status.");
    }
  } else {
    // หากการอ่านสถานะคอยล์ล้มเหลว แสดงรหัสข้อผิดพลาดใน Serial Monitor
    Serial.print("Failed to read coil status: ");
    Serial.println(result);
  }

  // รีเซ็ตตัวแปร isUpdatingFromApp[7] เป็น false
  // เพื่อระบุว่า Relay 8 ไม่ได้ถูกควบคุมจากแอปอีกต่อไป
  isUpdatingFromApp[7] = false;
}

// ฟังก์ชัน BLYNK_WRITE สำหรับ Relay 9
// ฟังก์ชันนี้จะถูกเรียกเมื่อมีการเปลี่ยนแปลงสถานะของ Widget_Btn_SW9 ในแอป Blynk
BLYNK_WRITE(Widget_Btn_SW9) {
  // อ่านค่าจาก Widget_Btn_SW9 (ค่าที่ส่งมาจะเป็น 0 หรือ 1) และเก็บไว้ในตัวแปร valueSW9
  int valueSW9 = param.asInt();

  // กำหนดสถานะว่า Relay 9 กำลังถูกควบคุมจากแอป โดยตั้งค่า isUpdatingFromApp[8] เป็น true
  isUpdatingFromApp[8] = true;

  // อ่านสถานะปัจจุบันของคอยล์ใน Modbus Address 0x0008
  uint8_t result = node1.readCoils(0x0008, 1);

  // ตรวจสอบว่าการอ่านสถานะสำเร็จหรือไม่
  if (result == node1.ku8MBSuccess) {
    // หากการอ่านสำเร็จ ดึงสถานะปัจจุบันของคอยล์จาก buffer และเก็บไว้ในตัวแปร currentStatus
    bool currentStatus = node1.getResponseBuffer(0);

    // เปรียบเทียบสถานะใหม่ (valueSW9) กับสถานะปัจจุบัน (currentStatus)
    if (valueSW9 != currentStatus) {
      // หากสถานะใหม่ไม่ตรงกับสถานะปัจจุบัน เขียนสถานะใหม่ลงในคอยล์ที่ Modbus Address 0x0008
      if (node1.writeSingleCoil(0x0008, valueSW9) == node1.ku8MBSuccess) {
        // หากเขียนสำเร็จ แสดงข้อความยืนยันใน Serial Monitor
        Serial.println(valueSW9 ? "SW9 ON success" : "SW9 OFF success");

        // อัปเดตสถานะของ Widget_Btn_SW9 ในแอป Blynk ให้ตรงกับสถานะใหม่
        Blynk.virtualWrite(Widget_Btn_SW9, valueSW9);
      } else {
        // หากการเขียนล้มเหลว แสดงข้อความข้อผิดพลาดใน Serial Monitor
        Serial.println(valueSW9 ? "SW9 ON failed" : "SW9 OFF failed");
      }
    } else {
      // หากสถานะใหม่เหมือนกับสถานะปัจจุบัน ไม่ต้องทำอะไรเพิ่มเติม และแสดงข้อความใน Serial Monitor
      Serial.println("No change in SW9 status.");
    }
  } else {
    // หากการอ่านสถานะคอยล์ล้มเหลว แสดงรหัสข้อผิดพลาดใน Serial Monitor
    Serial.print("Failed to read coil status: ");
    Serial.println(result);
  }

  // รีเซ็ตตัวแปร isUpdatingFromApp[8] เป็น false
  // เพื่อระบุว่า Relay 9 ไม่ได้ถูกควบคุมจากแอปอีกต่อไป
  isUpdatingFromApp[8] = false;
}

// ฟังก์ชัน BLYNK_WRITE สำหรับ Relay 10
// ฟังก์ชันนี้จะถูกเรียกเมื่อมีการเปลี่ยนแปลงสถานะของ Widget_Btn_SW10 ในแอป Blynk
BLYNK_WRITE(Widget_Btn_SW10) {
  // อ่านค่าจาก Widget_Btn_SW10 (ค่าที่ส่งมาจะเป็น 0 หรือ 1) และเก็บไว้ในตัวแปร valueSW10
  int valueSW10 = param.asInt();

  // กำหนดสถานะว่า Relay 10 กำลังถูกควบคุมจากแอป โดยตั้งค่า isUpdatingFromApp[9] เป็น true
  isUpdatingFromApp[9] = true;

  // อ่านสถานะปัจจุบันของคอยล์ใน Modbus Address 0x0009
  uint8_t result = node1.readCoils(0x0009, 1);

  // ตรวจสอบว่าการอ่านสถานะสำเร็จหรือไม่
  if (result == node1.ku8MBSuccess) {
    // หากการอ่านสำเร็จ ดึงสถานะปัจจุบันของคอยล์จาก buffer และเก็บไว้ในตัวแปร currentStatus
    bool currentStatus = node1.getResponseBuffer(0);

    // เปรียบเทียบสถานะใหม่ (valueSW10) กับสถานะปัจจุบัน (currentStatus)
    if (valueSW10 != currentStatus) {
      // หากสถานะใหม่ไม่ตรงกับสถานะปัจจุบัน เขียนสถานะใหม่ลงในคอยล์ที่ Modbus Address 0x0009
      if (node1.writeSingleCoil(0x0009, valueSW10) == node1.ku8MBSuccess) {
        // หากเขียนสำเร็จ แสดงข้อความยืนยันใน Serial Monitor
        Serial.println(valueSW10 ? "SW10 ON success" : "SW10 OFF success");

        // อัปเดตสถานะของ Widget_Btn_SW10 ในแอป Blynk ให้ตรงกับสถานะใหม่
        Blynk.virtualWrite(Widget_Btn_SW10, valueSW10);
      } else {
        // หากการเขียนล้มเหลว แสดงข้อความข้อผิดพลาดใน Serial Monitor
        Serial.println(valueSW10 ? "SW10 ON failed" : "SW10 OFF failed");
      }
    } else {
      // หากสถานะใหม่เหมือนกับสถานะปัจจุบัน ไม่ต้องทำอะไรเพิ่มเติม และแสดงข้อความใน Serial Monitor
      Serial.println("No change in SW10 status.");
    }
  } else {
    // หากการอ่านสถานะคอยล์ล้มเหลว แสดงรหัสข้อผิดพลาดใน Serial Monitor
    Serial.print("Failed to read coil status: ");
    Serial.println(result);
  }

  // รีเซ็ตตัวแปร isUpdatingFromApp[9] เป็น false
  // เพื่อระบุว่า Relay 10 ไม่ได้ถูกควบคุมจากแอปอีกต่อไป
  isUpdatingFromApp[9] = false;
}

// ฟังก์ชัน BLYNK_WRITE สำหรับ Relay 11
// ฟังก์ชันนี้จะถูกเรียกเมื่อมีการเปลี่ยนแปลงสถานะของ Widget_Btn_SW11 ในแอป Blynk
BLYNK_WRITE(Widget_Btn_SW11) {
  // อ่านค่าจาก Widget_Btn_SW11 (ค่าที่ส่งมาจะเป็น 0 หรือ 1) และเก็บไว้ในตัวแปร valueSW11
  int valueSW11 = param.asInt();

  // กำหนดสถานะว่า Relay 11 กำลังถูกควบคุมจากแอป โดยตั้งค่า isUpdatingFromApp[10] เป็น true
  isUpdatingFromApp[10] = true;

  // อ่านสถานะปัจจุบันของคอยล์ใน Modbus Address 0x000A
  uint8_t result = node1.readCoils(0x000A, 1);

  // ตรวจสอบว่าการอ่านสถานะสำเร็จหรือไม่
  if (result == node1.ku8MBSuccess) {
    // หากการอ่านสำเร็จ ดึงสถานะปัจจุบันของคอยล์จาก buffer และเก็บไว้ในตัวแปร currentStatus
    bool currentStatus = node1.getResponseBuffer(0);

    // เปรียบเทียบสถานะใหม่ (valueSW11) กับสถานะปัจจุบัน (currentStatus)
    if (valueSW11 != currentStatus) {
      // หากสถานะใหม่ไม่ตรงกับสถานะปัจจุบัน เขียนสถานะใหม่ลงในคอยล์ที่ Modbus Address 0x000A
      if (node1.writeSingleCoil(0x000A, valueSW11) == node1.ku8MBSuccess) {
        // หากเขียนสำเร็จ แสดงข้อความยืนยันใน Serial Monitor
        Serial.println(valueSW11 ? "SW11 ON success" : "SW11 OFF success");

        // อัปเดตสถานะของ Widget_Btn_SW11 ในแอป Blynk ให้ตรงกับสถานะใหม่
        Blynk.virtualWrite(Widget_Btn_SW11, valueSW11);
      } else {
        // หากการเขียนล้มเหลว แสดงข้อความข้อผิดพลาดใน Serial Monitor
        Serial.println(valueSW11 ? "SW11 ON failed" : "SW11 OFF failed");
      }
    } else {
      // หากสถานะใหม่เหมือนกับสถานะปัจจุบัน ไม่ต้องทำอะไรเพิ่มเติม และแสดงข้อความใน Serial Monitor
      Serial.println("No change in SW11 status.");
    }
  } else {
    // หากการอ่านสถานะคอยล์ล้มเหลว แสดงรหัสข้อผิดพลาดใน Serial Monitor
    Serial.print("Failed to read coil status: ");
    Serial.println(result);
  }

  // รีเซ็ตตัวแปร isUpdatingFromApp[10] เป็น false
  // เพื่อระบุว่า Relay 11 ไม่ได้ถูกควบคุมจากแอปอีกต่อไป
  isUpdatingFromApp[10] = false;
}

// ฟังก์ชัน BLYNK_WRITE สำหรับ Relay 12
// ฟังก์ชันนี้จะถูกเรียกเมื่อมีการเปลี่ยนแปลงสถานะของ Widget_Btn_SW12 ในแอป Blynk
BLYNK_WRITE(Widget_Btn_SW12) {
  // อ่านค่าจาก Widget_Btn_SW12 (ค่าที่ส่งมาจะเป็น 0 หรือ 1) และเก็บไว้ในตัวแปร valueSW12
  int valueSW12 = param.asInt();

  // กำหนดสถานะว่า Relay 12 กำลังถูกควบคุมจากแอป โดยตั้งค่า isUpdatingFromApp[11] เป็น true
  isUpdatingFromApp[11] = true;

  // อ่านสถานะปัจจุบันของคอยล์ใน Modbus Address 0x000B
  uint8_t result = node1.readCoils(0x000B, 1);

  // ตรวจสอบว่าการอ่านสถานะสำเร็จหรือไม่
  if (result == node1.ku8MBSuccess) {
    // หากการอ่านสำเร็จ ดึงสถานะปัจจุบันของคอยล์จาก buffer และเก็บไว้ในตัวแปร currentStatus
    bool currentStatus = node1.getResponseBuffer(0);

    // เปรียบเทียบสถานะใหม่ (valueSW12) กับสถานะปัจจุบัน (currentStatus)
    if (valueSW12 != currentStatus) {
      // หากสถานะใหม่ไม่ตรงกับสถานะปัจจุบัน เขียนสถานะใหม่ลงในคอยล์ที่ Modbus Address 0x000B
      if (node1.writeSingleCoil(0x000B, valueSW12) == node1.ku8MBSuccess) {
        // หากเขียนสำเร็จ แสดงข้อความยืนยันใน Serial Monitor
        Serial.println(valueSW12 ? "SW12 ON success" : "SW12 OFF success");

        // อัปเดตสถานะของ Widget_Btn_SW12 ในแอป Blynk ให้ตรงกับสถานะใหม่
        Blynk.virtualWrite(Widget_Btn_SW12, valueSW12);
      } else {
        // หากการเขียนล้มเหลว แสดงข้อความข้อผิดพลาดใน Serial Monitor
        Serial.println(valueSW12 ? "SW12 ON failed" : "SW12 OFF failed");
      }
    } else {
      // หากสถานะใหม่เหมือนกับสถานะปัจจุบัน ไม่ต้องทำอะไรเพิ่มเติม และแสดงข้อความใน Serial Monitor
      Serial.println("No change in SW12 status.");
    }
  } else {
    // หากการอ่านสถานะคอยล์ล้มเหลว แสดงรหัสข้อผิดพลาดใน Serial Monitor
    Serial.print("Failed to read coil status: ");
    Serial.println(result);
  }

  // รีเซ็ตตัวแปร isUpdatingFromApp[11] เป็น false
  // เพื่อระบุว่า Relay 12 ไม่ได้ถูกควบคุมจากแอปอีกต่อไป
  isUpdatingFromApp[11] = false;
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
