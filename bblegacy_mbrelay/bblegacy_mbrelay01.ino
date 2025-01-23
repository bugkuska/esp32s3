#include <Wire.h>              // ไลบรารีสำหรับการใช้งาน I2C (ไม่ได้ใช้งานในโค้ดนี้ แต่สามารถใช้สำหรับอุปกรณ์อื่นในอนาคต)
#include <WiFi.h>              // ไลบรารีสำหรับการเชื่อมต่อ Wi-Fi
#include <WiFiClient.h>        // ไลบรารีสำหรับการจัดการการเชื่อมต่อ Wi-Fi Client
#include <BlynkSimpleEsp32.h>  // ไลบรารีสำหรับเชื่อมต่อ ESP32 กับ Blynk Legacy
#include <ModbusMaster.h>      // ไลบรารีสำหรับการสื่อสารผ่านโปรโตคอล Modbus ในโหมด Master

// Wi-Fi and Blynk credentials
const char ssid[] = "";                         // ชื่อ Wi-Fi SSID
const char pass[] = "";                        // รหัสผ่าน Wi-Fi
const char auth[] = "";  // โทเคน Blynk

// Modbus settings
#define RXD 18       // กำหนด GPIO 18 ของ ESP32 เป็น RX (รับข้อมูล) สำหรับ Modbus RTU
#define TXD 17       // กำหนด GPIO 17 ของ ESP32 เป็น TX (ส่งข้อมูล) สำหรับ Modbus RTU
ModbusMaster node1;  // สร้างออบเจกต์ node1 เพื่อใช้ Modbus Master ในการสื่อสารกับอุปกรณ์ Modbus

// Timer for periodic tasks
BlynkTimer timer;  // สร้างออบเจกต์ timer สำหรับจัดการฟังก์ชันที่ต้องทำงานซ้ำตามรอบเวลา

// Debugging LED (optional, GPIO2 is onboard LED on ESP32)
#define DEBUG_LED 2  // กำหนด GPIO 2 สำหรับ Debug LED (หรือใช้ LED บนบอร์ด ESP32)

// Function prototypes
void checkConnections();  // ฟังก์ชันสำหรับตรวจสอบการเชื่อมต่อ Wi-Fi และ Blynk
//void debugModbus();       // ฟังก์ชันสำหรับ Debug การสื่อสาร Modbus

// กำหนด Virtual Pins สำหรับปุ่มควบคุมรีเลย์ใน Blynk
#define Widget_Btn_SW1 V1    // ปุ่มควบคุมรีเลย์ 1 (Virtual Pin V1)
#define Widget_Btn_SW2 V2    // ปุ่มควบคุมรีเลย์ 2 (Virtual Pin V2)
#define Widget_Btn_SW3 V3    // ปุ่มควบคุมรีเลย์ 3 (Virtual Pin V3)
#define Widget_Btn_SW4 V4    // ปุ่มควบคุมรีเลย์ 4 (Virtual Pin V4)
#define Widget_Btn_SW5 V5    // ปุ่มควบคุมรีเลย์ 5 (Virtual Pin V5)
#define Widget_Btn_SW6 V6    // ปุ่มควบคุมรีเลย์ 6 (Virtual Pin V6)
#define Widget_Btn_SW7 V7    // ปุ่มควบคุมรีเลย์ 7 (Virtual Pin V7)
#define Widget_Btn_SW8 V8    // ปุ่มควบคุมรีเลย์ 8 (Virtual Pin V8)
#define Widget_Btn_SW9 V9    // ปุ่มควบคุมรีเลย์ 9 (Virtual Pin V9)
#define Widget_Btn_SW10 V10  // ปุ่มควบคุมรีเลย์ 10 (Virtual Pin V10)
#define Widget_Btn_SW11 V11  // ปุ่มควบคุมรีเลย์ 11 (Virtual Pin V11)
#define Widget_Btn_SW12 V12  // ปุ่มควบคุมรีเลย์ 12 (Virtual Pin V12)

void setup() {
  // Initialize serial communication
  Serial.begin(115200);                       // เริ่มต้นการสื่อสาร Serial เพื่อ Debugging ด้วยความเร็ว 115200 bps
  Serial2.begin(9600, SERIAL_8N1, RXD, TXD);  // เริ่มต้นการสื่อสาร Serial2 สำหรับ Modbus ด้วยความเร็ว 9600 bps

  // Initialize Modbus
  node1.begin(1, Serial2);  // ตั้งค่า Modbus Address เป็น 1 และใช้ Serial2 สำหรับการสื่อสาร

  // Debugging LED setup
  pinMode(DEBUG_LED, OUTPUT);    // กำหนด GPIO 2 (DEBUG_LED) เป็นขา Output
  digitalWrite(DEBUG_LED, LOW);  // ปิด LED Debug เริ่มต้น

  // Connect to Wi-Fi
  Serial.println("Connecting to Wi-Fi...");  // แสดงข้อความใน Serial Monitor ว่ากำลังเชื่อมต่อ Wi-Fi
  WiFi.begin(ssid, pass);                    // เชื่อมต่อ Wi-Fi ด้วย SSID และรหัสผ่านที่กำหนด

  unsigned long startAttemptTime = millis();  // บันทึกเวลาที่เริ่มการพยายามเชื่อมต่อ Wi-Fi
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    // รอจนกว่า Wi-Fi จะเชื่อมต่อสำเร็จ หรือครบเวลา 10 วินาที
    delay(500);         // หน่วงเวลา 500 มิลลิวินาที
    Serial.print(".");  // แสดงจุดเพื่อบอกสถานะการพยายามเชื่อมต่อ
  }

  if (WiFi.status() == WL_CONNECTED) {
    // แสดงข้อความเมื่อ Wi-Fi เชื่อมต่อสำเร็จ
    Serial.println("\nWi-Fi connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());  // แสดง IP Address ของ ESP32
  } else {
    // แสดงข้อความเมื่อ Wi-Fi เชื่อมต่อไม่สำเร็จ
    Serial.println("\nFailed to connect to Wi-Fi");
  }

  // Enable Wi-Fi auto-reconnect
  WiFi.setAutoReconnect(true);  // เปิดใช้งานการเชื่อมต่อ Wi-Fi อัตโนมัติเมื่อสัญญาณหลุด
  WiFi.persistent(true);        // บันทึกการตั้งค่า Wi-Fi ลงใน Flash Memory เพื่อให้เชื่อมต่อใหม่อัตโนมัติ

  // Connect to Blynk server
  Serial.println("Connecting to Blynk...");  // แสดงข้อความว่าเริ่มเชื่อมต่อกับเซิร์ฟเวอร์ Blynk
  Blynk.begin(auth, ssid, pass, "iotservices.thddns.net", 5535);
  // เชื่อมต่อกับ Blynk Server โดยระบุ Auth Token, SSID, Password, IP Address ของ Blynk Server และพอร์ต 8080

  // Set timers
  timer.setInterval(5000L, checkConnections);  // ตั้งตัวจับเวลา 5 วินาทีสำหรับตรวจสอบการเชื่อมต่อ Wi-Fi และ Blynk
  //timer.setInterval(2000L, debugModbus);       // ตั้งตัวจับเวลา 2 วินาทีสำหรับ Debug การสื่อสาร Modbus
}

BLYNK_CONNECTED() {
  // ฟังก์ชันนี้จะถูกเรียกโดยอัตโนมัติเมื่อ ESP32 เชื่อมต่อกับ Blynk Server สำเร็จ
  Serial.println("Blynk connected! Synchronizing virtual pins...");
  // แสดงข้อความใน Serial Monitor เพื่อแจ้งว่าการเชื่อมต่อกับ Blynk สำเร็จและกำลังเริ่มการซิงโครไนซ์

  // ซิงโครไนซ์สถานะของ Virtual Pins กับเซิร์ฟเวอร์ Blynk
  Blynk.syncVirtual(Widget_Btn_SW1);   // ซิงโครไนซ์สถานะของ Virtual Pin V1
  Blynk.syncVirtual(Widget_Btn_SW2);   // ซิงโครไนซ์สถานะของ Virtual Pin V2
  Blynk.syncVirtual(Widget_Btn_SW3);   // ซิงโครไนซ์สถานะของ Virtual Pin V3
  Blynk.syncVirtual(Widget_Btn_SW4);   // ซิงโครไนซ์สถานะของ Virtual Pin V4
  Blynk.syncVirtual(Widget_Btn_SW5);   // ซิงโครไนซ์สถานะของ Virtual Pin V5
  Blynk.syncVirtual(Widget_Btn_SW6);   // ซิงโครไนซ์สถานะของ Virtual Pin V6
  Blynk.syncVirtual(Widget_Btn_SW7);   // ซิงโครไนซ์สถานะของ Virtual Pin V7
  Blynk.syncVirtual(Widget_Btn_SW8);   // ซิงโครไนซ์สถานะของ Virtual Pin V8
  Blynk.syncVirtual(Widget_Btn_SW9);   // ซิงโครไนซ์สถานะของ Virtual Pin V9
  Blynk.syncVirtual(Widget_Btn_SW10);  // ซิงโครไนซ์สถานะของ Virtual Pin V10
  Blynk.syncVirtual(Widget_Btn_SW11);  // ซิงโครไนซ์สถานะของ Virtual Pin V11
  Blynk.syncVirtual(Widget_Btn_SW12);  // ซิงโครไนซ์สถานะของ Virtual Pin V12
}

void checkConnections() {
  // ฟังก์ชันสำหรับตรวจสอบการเชื่อมต่อ Wi-Fi และ Blynk
  // ตรวจสอบการเชื่อมต่อ Wi-Fi
  if (WiFi.status() != WL_CONNECTED) {
    // หาก Wi-Fi ไม่ได้เชื่อมต่อ (สถานะไม่เท่ากับ WL_CONNECTED)
    Serial.println("Wi-Fi disconnected! Reconnecting...");
    // แสดงข้อความใน Serial Monitor เพื่อแจ้งเตือนว่ากำลังพยายามเชื่อมต่อ Wi-Fi
    WiFi.begin(ssid, pass);
    // เริ่มกระบวนการเชื่อมต่อ Wi-Fi ใหม่โดยใช้ SSID และรหัสผ่านที่กำหนด
  }

  // ตรวจสอบการเชื่อมต่อ Blynk
  if (!Blynk.connected()) {
    // หาก Blynk ไม่ได้เชื่อมต่อ
    Serial.println("Blynk disconnected! Reconnecting...");
    // แสดงข้อความใน Serial Monitor เพื่อแจ้งเตือนว่ากำลังพยายามเชื่อมต่อ Blynk
    Blynk.connect();
    // พยายามเชื่อมต่อกับเซิร์ฟเวอร์ Blynk ใหม่
  }
}
/*
void debugModbus() {
  // ฟังก์ชันสำหรับ Debug การสื่อสารกับอุปกรณ์ Modbus

  // ตัวอย่าง: อ่านค่าจาก Modbus Holding Register Address 0x0000
  uint8_t result = node1.readHoldingRegisters(0x0000, 1);
  // ส่งคำสั่งไปยังอุปกรณ์ Modbus เพื่ออ่านค่าจาก Holding Register Address 0x0000 จำนวน 1 ค่า

  yield();  // ป้องกัน Watchdog Timeout โดยอนุญาตให้ ESP32 ทำงานพื้นฐานระหว่างคำสั่งที่ใช้เวลานาน

  // ตรวจสอบผลลัพธ์ของการอ่าน Modbus
  if (result == node1.ku8MBSuccess) {
    // หากการอ่านสำเร็จ (ผลลัพธ์คือ ku8MBSuccess)
    Serial.print("Modbus Data: ");  // แสดงข้อความ "Modbus Data" ใน Serial Monitor
    Serial.println(node1.getResponseBuffer(0));
    // แสดงค่าที่อ่านได้จาก Holding Register Address 0x0000
  } else {
    // หากการอ่านล้มเหลว
    Serial.print("Modbus Error: ");  // แสดงข้อความ "Modbus Error" ใน Serial Monitor
    Serial.println(result);
    // แสดงรหัสข้อผิดพลาด (Error Code) ที่ส่งกลับมา
  }
}
*/
// Blynk write handler for button controlling SW1
BLYNK_WRITE(Widget_Btn_SW1) {
  // รับค่าจากแอป Blynk เมื่อปุ่มใน Virtual Pin V1 ถูกกดหรือเปลี่ยนสถานะ
  int valueSW1 = param.asInt();  // อ่านค่าจาก Virtual Pin V1 และแปลงเป็น int (0 หรือ 1)

  if (valueSW1 == 1) {  // หากค่าที่รับมาคือ 1 (เปิดรีเลย์)
    if (node1.writeSingleCoil(0x0000, 1) == node1.ku8MBSuccess) {
      // เขียนค่า 1 (ON) ไปยัง Modbus Coil Address 0x0000
      Serial.println("SW1 ON success");  // แสดงข้อความใน Serial Monitor ว่าการเปิดรีเลย์สำเร็จ
    } else {
      Serial.println("SW1 ON failed");  // แสดงข้อความใน Serial Monitor ว่าการเปิดรีเลย์ล้มเหลว
    }
  } else {  // หากค่าที่รับมาคือ 0 (ปิดรีเลย์)
    if (node1.writeSingleCoil(0x0000, 0) == node1.ku8MBSuccess) {
      // เขียนค่า 0 (OFF) ไปยัง Modbus Coil Address 0x0000
      Serial.println("SW1 OFF success");  // แสดงข้อความใน Serial Monitor ว่าการปิดรีเลย์สำเร็จ
    } else {
      Serial.println("SW1 OFF failed");  // แสดงข้อความใน Serial Monitor ว่าการปิดรีเลย์ล้มเหลว
    }
  }
}
// Blynk write handler for button controlling SW2
BLYNK_WRITE(Widget_Btn_SW2) {
  // รับค่าจากแอป Blynk เมื่อปุ่มใน Virtual Pin V2 ถูกกดหรือเปลี่ยนสถานะ
  int valueSW2 = param.asInt();  // อ่านค่าจาก Virtual Pin V2 และแปลงเป็น int (0 หรือ 1)

  if (valueSW2 == 1) {  // หากค่าที่รับมาคือ 1 (เปิดรีเลย์)
    if (node1.writeSingleCoil(0x0001, 1) == node1.ku8MBSuccess) {
      // เขียนค่า 1 (ON) ไปยัง Modbus Coil Address 0x0001
      Serial.println("SW2 ON success");  // แสดงข้อความใน Serial Monitor ว่าการเปิดรีเลย์สำเร็จ
    } else {
      Serial.println("SW2 ON failed");  // แสดงข้อความใน Serial Monitor ว่าการเปิดรีเลย์ล้มเหลว
    }
  } else {  // หากค่าที่รับมาคือ 0 (ปิดรีเลย์)
    if (node1.writeSingleCoil(0x0001, 0) == node1.ku8MBSuccess) {
      // เขียนค่า 0 (OFF) ไปยัง Modbus Coil Address 0x0001
      Serial.println("SW2 OFF success");  // แสดงข้อความใน Serial Monitor ว่าการปิดรีเลย์สำเร็จ
    } else {
      Serial.println("SW2 OFF failed");  // แสดงข้อความใน Serial Monitor ว่าการปิดรีเลย์ล้มเหลว
    }
  }
}

// Blynk write handler for button controlling SW3
BLYNK_WRITE(Widget_Btn_SW3) {
  // รับค่าจากแอป Blynk เมื่อปุ่มใน Virtual Pin V3 ถูกกดหรือเปลี่ยนสถานะ
  int valueSW3 = param.asInt();  // อ่านค่าจาก Virtual Pin V3 และแปลงเป็น int (0 หรือ 1)

  if (valueSW3 == 1) {  // หากค่าที่รับมาคือ 1 (เปิดรีเลย์)
    if (node1.writeSingleCoil(0x0002, 1) == node1.ku8MBSuccess) {
      // เขียนค่า 1 (ON) ไปยัง Modbus Coil Address 0x0002
      Serial.println("SW3 ON success");  // แสดงข้อความใน Serial Monitor ว่าการเปิดรีเลย์สำเร็จ
    } else {
      Serial.println("SW3 ON failed");  // แสดงข้อความใน Serial Monitor ว่าการเปิดรีเลย์ล้มเหลว
    }
  } else {  // หากค่าที่รับมาคือ 0 (ปิดรีเลย์)
    if (node1.writeSingleCoil(0x0002, 0) == node1.ku8MBSuccess) {
      // เขียนค่า 0 (OFF) ไปยัง Modbus Coil Address 0x0002
      Serial.println("SW3 OFF success");  // แสดงข้อความใน Serial Monitor ว่าการปิดรีเลย์สำเร็จ
    } else {
      Serial.println("SW3 OFF failed");  // แสดงข้อความใน Serial Monitor ว่าการปิดรีเลย์ล้มเหลว
    }
  }
}

// Blynk write handler for button controlling SW4
BLYNK_WRITE(Widget_Btn_SW4) {
  // รับค่าจากแอป Blynk เมื่อปุ่มใน Virtual Pin V4 ถูกกดหรือเปลี่ยนสถานะ
  int valueSW4 = param.asInt();  // อ่านค่าจาก Virtual Pin V4 และแปลงเป็น int (0 หรือ 1)

  if (valueSW4 == 1) {  // หากค่าที่รับมาคือ 1 (เปิดรีเลย์)
    if (node1.writeSingleCoil(0x0003, 1) == node1.ku8MBSuccess) {
      // เขียนค่า 1 (ON) ไปยัง Modbus Coil Address 0x0003
      Serial.println("SW4 ON success");  // แสดงข้อความใน Serial Monitor ว่าการเปิดรีเลย์สำเร็จ
    } else {
      Serial.println("SW4 ON failed");  // แสดงข้อความใน Serial Monitor ว่าการเปิดรีเลย์ล้มเหลว
    }
  } else {  // หากค่าที่รับมาคือ 0 (ปิดรีเลย์)
    if (node1.writeSingleCoil(0x0003, 0) == node1.ku8MBSuccess) {
      // เขียนค่า 0 (OFF) ไปยัง Modbus Coil Address 0x0003
      Serial.println("SW4 OFF success");  // แสดงข้อความใน Serial Monitor ว่าการปิดรีเลย์สำเร็จ
    } else {
      Serial.println("SW4 OFF failed");  // แสดงข้อความใน Serial Monitor ว่าการปิดรีเลย์ล้มเหลว
    }
  }
}

// Blynk write handler for button controlling SW5
BLYNK_WRITE(Widget_Btn_SW5) {
  // รับค่าจากแอป Blynk เมื่อปุ่มใน Virtual Pin V5 ถูกกดหรือเปลี่ยนสถานะ
  int valueSW5 = param.asInt();  // อ่านค่าจาก Virtual Pin V5 และแปลงเป็น int (0 หรือ 1)

  if (valueSW5 == 1) {  // หากค่าที่รับมาคือ 1 (เปิดรีเลย์)
    if (node1.writeSingleCoil(0x0004, 1) == node1.ku8MBSuccess) {
      // เขียนค่า 1 (ON) ไปยัง Modbus Coil Address 0x0004
      Serial.println("SW5 ON success");  // แสดงข้อความใน Serial Monitor ว่าการเปิดรีเลย์สำเร็จ
    } else {
      Serial.println("SW5 ON failed");  // แสดงข้อความใน Serial Monitor ว่าการเปิดรีเลย์ล้มเหลว
    }
  } else {  // หากค่าที่รับมาคือ 0 (ปิดรีเลย์)
    if (node1.writeSingleCoil(0x0004, 0) == node1.ku8MBSuccess) {
      // เขียนค่า 0 (OFF) ไปยัง Modbus Coil Address 0x0004
      Serial.println("SW5 OFF success");  // แสดงข้อความใน Serial Monitor ว่าการปิดรีเลย์สำเร็จ
    } else {
      Serial.println("SW5 OFF failed");  // แสดงข้อความใน Serial Monitor ว่าการปิดรีเลย์ล้มเหลว
    }
  }
}

// Blynk write handler for button controlling SW6
BLYNK_WRITE(Widget_Btn_SW6) {
  // รับค่าจากแอป Blynk เมื่อปุ่มใน Virtual Pin V6 ถูกกดหรือเปลี่ยนสถานะ
  int valueSW6 = param.asInt();  // อ่านค่าจาก Virtual Pin V6 และแปลงเป็น int (0 หรือ 1)

  if (valueSW6 == 1) {  // หากค่าที่รับมาคือ 1 (เปิดรีเลย์)
    if (node1.writeSingleCoil(0x0005, 1) == node1.ku8MBSuccess) {
      // เขียนค่า 1 (ON) ไปยัง Modbus Coil Address 0x0005
      Serial.println("SW6 ON success");  // แสดงข้อความใน Serial Monitor ว่าการเปิดรีเลย์สำเร็จ
    } else {
      Serial.println("SW6 ON failed");  // แสดงข้อความใน Serial Monitor ว่าการเปิดรีเลย์ล้มเหลว
    }
  } else {  // หากค่าที่รับมาคือ 0 (ปิดรีเลย์)
    if (node1.writeSingleCoil(0x0005, 0) == node1.ku8MBSuccess) {
      // เขียนค่า 0 (OFF) ไปยัง Modbus Coil Address 0x0005
      Serial.println("SW6 OFF success");  // แสดงข้อความใน Serial Monitor ว่าการปิดรีเลย์สำเร็จ
    } else {
      Serial.println("SW6 OFF failed");  // แสดงข้อความใน Serial Monitor ว่าการปิดรีเลย์ล้มเหลว
    }
  }
}

// Blynk write handler for button controlling SW7
BLYNK_WRITE(Widget_Btn_SW7) {
  // รับค่าจากแอป Blynk เมื่อปุ่มใน Virtual Pin V7 ถูกกดหรือเปลี่ยนสถานะ
  int valueSW7 = param.asInt();  // อ่านค่าจาก Virtual Pin V7 และแปลงเป็น int (0 หรือ 1)

  if (valueSW7 == 1) {  // หากค่าที่รับมาคือ 1 (เปิดรีเลย์)
    if (node1.writeSingleCoil(0x0006, 1) == node1.ku8MBSuccess) {
      // เขียนค่า 1 (ON) ไปยัง Modbus Coil Address 0x0006
      Serial.println("SW7 ON success");  // แสดงข้อความใน Serial Monitor ว่าการเปิดรีเลย์สำเร็จ
    } else {
      Serial.println("SW7 ON failed");  // แสดงข้อความใน Serial Monitor ว่าการเปิดรีเลย์ล้มเหลว
    }
  } else {  // หากค่าที่รับมาคือ 0 (ปิดรีเลย์)
    if (node1.writeSingleCoil(0x0006, 0) == node1.ku8MBSuccess) {
      // เขียนค่า 0 (OFF) ไปยัง Modbus Coil Address 0x0006
      Serial.println("SW7 OFF success");  // แสดงข้อความใน Serial Monitor ว่าการปิดรีเลย์สำเร็จ
    } else {
      Serial.println("SW7 OFF failed");  // แสดงข้อความใน Serial Monitor ว่าการปิดรีเลย์ล้มเหลว
    }
  }
}

// Blynk write handler for button controlling SW8
BLYNK_WRITE(Widget_Btn_SW8) {
  // รับค่าจากแอป Blynk เมื่อปุ่มใน Virtual Pin V8 ถูกกดหรือเปลี่ยนสถานะ
  int valueSW8 = param.asInt();  // อ่านค่าจาก Virtual Pin V8 และแปลงเป็น int (0 หรือ 1)

  if (valueSW8 == 1) {  // หากค่าที่รับมาคือ 1 (เปิดรีเลย์)
    if (node1.writeSingleCoil(0x0007, 1) == node1.ku8MBSuccess) {
      // เขียนค่า 1 (ON) ไปยัง Modbus Coil Address 0x0007
      Serial.println("SW8 ON success");  // แสดงข้อความใน Serial Monitor ว่าการเปิดรีเลย์สำเร็จ
    } else {
      Serial.println("SW8 ON failed");  // แสดงข้อความใน Serial Monitor ว่าการเปิดรีเลย์ล้มเหลว
    }
  } else {  // หากค่าที่รับมาคือ 0 (ปิดรีเลย์)
    if (node1.writeSingleCoil(0x0007, 0) == node1.ku8MBSuccess) {
      // เขียนค่า 0 (OFF) ไปยัง Modbus Coil Address 0x0007
      Serial.println("SW8 OFF success");  // แสดงข้อความใน Serial Monitor ว่าการปิดรีเลย์สำเร็จ
    } else {
      Serial.println("SW8 OFF failed");  // แสดงข้อความใน Serial Monitor ว่าการปิดรีเลย์ล้มเหลว
    }
  }
}

// Blynk write handler for button controlling SW9
BLYNK_WRITE(Widget_Btn_SW9) {
  // รับค่าจากแอป Blynk เมื่อปุ่มใน Virtual Pin V9 ถูกกดหรือเปลี่ยนสถานะ
  int valueSW9 = param.asInt();  // อ่านค่าจาก Virtual Pin V9 และแปลงเป็น int (0 หรือ 1)

  if (valueSW9 == 1) {  // หากค่าที่รับมาคือ 1 (เปิดรีเลย์)
    if (node1.writeSingleCoil(0x0008, 1) == node1.ku8MBSuccess) {
      // เขียนค่า 1 (ON) ไปยัง Modbus Coil Address 0x0008
      Serial.println("SW9 ON success");  // แสดงข้อความใน Serial Monitor ว่าการเปิดรีเลย์สำเร็จ
    } else {
      Serial.println("SW9 ON failed");  // แสดงข้อความใน Serial Monitor ว่าการเปิดรีเลย์ล้มเหลว
    }
  } else {  // หากค่าที่รับมาคือ 0 (ปิดรีเลย์)
    if (node1.writeSingleCoil(0x0008, 0) == node1.ku8MBSuccess) {
      // เขียนค่า 0 (OFF) ไปยัง Modbus Coil Address 0x0008
      Serial.println("SW9 OFF success");  // แสดงข้อความใน Serial Monitor ว่าการปิดรีเลย์สำเร็จ
    } else {
      Serial.println("SW9 OFF failed");  // แสดงข้อความใน Serial Monitor ว่าการปิดรีเลย์ล้มเหลว
    }
  }
}

// Blynk write handler for button controlling SW10
BLYNK_WRITE(Widget_Btn_SW10) {
  // รับค่าจากแอป Blynk เมื่อปุ่มใน Virtual Pin V10 ถูกกดหรือเปลี่ยนสถานะ
  int valueSW10 = param.asInt();  // อ่านค่าจาก Virtual Pin V10 และแปลงเป็น int (0 หรือ 1)

  if (valueSW10 == 1) {  // หากค่าที่รับมาคือ 1 (เปิดรีเลย์)
    if (node1.writeSingleCoil(0x0009, 1) == node1.ku8MBSuccess) {
      // เขียนค่า 1 (ON) ไปยัง Modbus Coil Address 0x0009
      Serial.println("SW10 ON success");  // แสดงข้อความใน Serial Monitor ว่าการเปิดรีเลย์สำเร็จ
    } else {
      Serial.println("SW10 ON failed");  // แสดงข้อความใน Serial Monitor ว่าการเปิดรีเลย์ล้มเหลว
    }
  } else {  // หากค่าที่รับมาคือ 0 (ปิดรีเลย์)
    if (node1.writeSingleCoil(0x0009, 0) == node1.ku8MBSuccess) {
      // เขียนค่า 0 (OFF) ไปยัง Modbus Coil Address 0x0009
      Serial.println("SW10 OFF success");  // แสดงข้อความใน Serial Monitor ว่าการปิดรีเลย์สำเร็จ
    } else {
      Serial.println("SW10 OFF failed");  // แสดงข้อความใน Serial Monitor ว่าการปิดรีเลย์ล้มเหลว
    }
  }
}

// Blynk write handler for button controlling SW11
BLYNK_WRITE(Widget_Btn_SW11) {
  // รับค่าจากแอป Blynk เมื่อปุ่มใน Virtual Pin V11 ถูกกดหรือเปลี่ยนสถานะ
  int valueSW11 = param.asInt();  // อ่านค่าจาก Virtual Pin V11 และแปลงเป็น int (0 หรือ 1)

  if (valueSW11 == 1) {  // หากค่าที่รับมาคือ 1 (เปิดรีเลย์)
    if (node1.writeSingleCoil(0x000A, 1) == node1.ku8MBSuccess) {
      // เขียนค่า 1 (ON) ไปยัง Modbus Coil Address 0x000A
      Serial.println("SW11 ON success");  // แสดงข้อความใน Serial Monitor ว่าการเปิดรีเลย์สำเร็จ
    } else {
      Serial.println("SW11 ON failed");  // แสดงข้อความใน Serial Monitor ว่าการเปิดรีเลย์ล้มเหลว
    }
  } else {  // หากค่าที่รับมาคือ 0 (ปิดรีเลย์)
    if (node1.writeSingleCoil(0x000A, 0) == node1.ku8MBSuccess) {
      // เขียนค่า 0 (OFF) ไปยัง Modbus Coil Address 0x000A
      Serial.println("SW11 OFF success");  // แสดงข้อความใน Serial Monitor ว่าการปิดรีเลย์สำเร็จ
    } else {
      Serial.println("SW11 OFF failed");  // แสดงข้อความใน Serial Monitor ว่าการปิดรีเลย์ล้มเหลว
    }
  }
}

// Blynk write handler for button controlling SW12
BLYNK_WRITE(Widget_Btn_SW12) {
  // รับค่าจากแอป Blynk เมื่อปุ่มใน Virtual Pin V12 ถูกกดหรือเปลี่ยนสถานะ
  int valueSW12 = param.asInt();  // อ่านค่าจาก Virtual Pin V12 และแปลงเป็น int (0 หรือ 1)

  if (valueSW12 == 1) {  // หากค่าที่รับมาคือ 1 (เปิดรีเลย์)
    if (node1.writeSingleCoil(0x000B, 1) == node1.ku8MBSuccess) {
      // เขียนค่า 1 (ON) ไปยัง Modbus Coil Address 0x000B
      Serial.println("SW12 ON success");  // แสดงข้อความใน Serial Monitor ว่าการเปิดรีเลย์สำเร็จ
    } else {
      Serial.println("SW12 ON failed");  // แสดงข้อความใน Serial Monitor ว่าการเปิดรีเลย์ล้มเหลว
    }
  } else {  // หากค่าที่รับมาคือ 0 (ปิดรีเลย์)
    if (node1.writeSingleCoil(0x000B, 0) == node1.ku8MBSuccess) {
      // เขียนค่า 0 (OFF) ไปยัง Modbus Coil Address 0x000B
      Serial.println("SW12 OFF success");  // แสดงข้อความใน Serial Monitor ว่าการปิดรีเลย์สำเร็จ
    } else {
      Serial.println("SW12 OFF failed");  // แสดงข้อความใน Serial Monitor ว่าการปิดรีเลย์ล้มเหลว
    }
  }
}

//Loop Function
void loop() {
  // ฟังก์ชันหลักที่ทำงานวนลูปตลอดเวลา
  // ตรวจสอบว่า Blynk ยังเชื่อมต่ออยู่หรือไม่
  if (Blynk.connected()) {
    // หากเชื่อมต่อกับ Blynk Server สำเร็จ
    Blynk.run();
    // ดำเนินการกระบวนการของ Blynk เช่น การจัดการอีเวนต์จากแอป
  }
  // เรียกใช้ฟังก์ชันที่ตั้งค่าไว้ใน BlynkTimer
  timer.run();
  // ตัวอย่างเช่น การตรวจสอบการเชื่อมต่อ Wi-Fi, Blynk หรืออัปเดตสถานะรีเลย์
  // เพิ่มดีเลย์เล็กน้อยเพื่อป้องกัน Watchdog Timeout
  delay(1);
  // หน่วงเวลา 1 มิลลิวินาที เพื่อให้ระบบ ESP32 มีเวลาจัดการงานพื้นฐาน
}
