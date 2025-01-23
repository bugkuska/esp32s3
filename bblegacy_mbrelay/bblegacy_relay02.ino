#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ModbusMaster.h>
// Wi-Fi and Blynk credentials
const char ssid[] = "579smf001";
const char pass[] = "0814111142";
const char auth[] = "RANBbnsmWwvEmoi3HZQEVAe7bF0UJteU";
// Modbus settings
#define RXD 18  //RX ของบอร์ด esp32s3
#define TXD 17  //TX ของบอร์ด esp32s3
ModbusMaster node1;
// Timer for periodic tasks
BlynkTimer timer;
// Debugging LED (optional, GPIO2 is onboard LED on ESP32)
#define DEBUG_LED 2
// Function prototypes
void checkConnections();
void debugModbus();
// Define Blynk virtual pins
#define Widget_Btn_SW1 V1
#define Widget_Btn_SW2 V2
#define Widget_Btn_SW3 V3
#define Widget_Btn_SW4 V4
#define Widget_Btn_SW5 V5
#define Widget_Btn_SW6 V6
#define Widget_Btn_SW7 V7
#define Widget_Btn_SW8 V8
#define Widget_Btn_SW9 V9
#define Widget_Btn_SW10 V10
#define Widget_Btn_SW11 V11
#define Widget_Btn_SW12 V12
bool isUpdatingFromApp[12] = { false };  // สถานะว่า Relay ถูกควบคุมจากแอปหรือไม่

//Setup Function
void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD, TXD);
  // Initialize Modbus
  node1.begin(1, Serial2);
  // Debugging LED setup
  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(DEBUG_LED, LOW);
  // Connect to Wi-Fi
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, pass);
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWi-Fi connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to Wi-Fi");
  }
  // Enable Wi-Fi auto-reconnect
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  // Connect to Blynk server
  Serial.println("Connecting to Blynk...");
  Blynk.begin(auth, ssid, pass, "iotservices.thddns.net", 5535);
  // Set timers
  timer.setInterval(5000L, checkConnections);
  //timer.setInterval(2000L, debugModbus);
  timer.setInterval(5000L, updateRelayStatus);
}

//Blynk connected
BLYNK_CONNECTED() {
  Serial.println("Blynk connected! Synchronizing virtual pins...");
  Blynk.syncVirtual(Widget_Btn_SW1);
  Blynk.syncVirtual(Widget_Btn_SW2);
  Blynk.syncVirtual(Widget_Btn_SW3);
  Blynk.syncVirtual(Widget_Btn_SW4);
  Blynk.syncVirtual(Widget_Btn_SW5);
  Blynk.syncVirtual(Widget_Btn_SW6);
  Blynk.syncVirtual(Widget_Btn_SW7);
  Blynk.syncVirtual(Widget_Btn_SW8);
  Blynk.syncVirtual(Widget_Btn_SW9);
  Blynk.syncVirtual(Widget_Btn_SW10);
  Blynk.syncVirtual(Widget_Btn_SW11);
  Blynk.syncVirtual(Widget_Btn_SW12);
}
//Check connection
void checkConnections() {
  // Check Wi-Fi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi disconnected! Reconnecting...");
    WiFi.begin(ssid, pass);
  }
  // Check Blynk connection
  if (!Blynk.connected()) {
    Serial.println("Blynk disconnected! Reconnecting...");
    Blynk.connect();
  }
}
/*
void debugModbus() {
  // Example: Read Modbus register 0x0000
  uint8_t result = node1.readHoldingRegisters(0x0000, 1);
  yield();  // Prevent watchdog timeout
  if (result == node1.ku8MBSuccess) {
    Serial.print("Modbus Data: ");
    Serial.println(node1.getResponseBuffer(0));
  } else {
    Serial.print("Modbus Error: ");
    Serial.println(result);
  }
}
*/
//Update relay status
void updateRelayStatus() {
  for (int i = 0; i < 12; i++) {
    if (!isUpdatingFromApp[i]) {               // หากรีเลย์นี้ไม่ได้ถูกควบคุมจากแอป
      uint8_t result = node1.readCoils(i, 1);  // อ่าน Coil ทีละ 1 ตัว
      if (result == node1.ku8MBSuccess) {
        bool relayState = node1.getResponseBuffer(0);  // อ่านค่าของรีเลย์
        Serial.print("Relay ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(relayState);
        // อัปเดตสถานะใน Blynk
        Blynk.virtualWrite(V1 + i, relayState);  // Virtual Pin V1 ถึง V12
      } else {
        Serial.print("Failed to read Relay ");
        Serial.print(i + 1);
        Serial.print(": Error Code ");
        Serial.println(result);
      }
    }
  }
}
// BLYNK_WRITE สำหรับ Relay 1
BLYNK_WRITE(Widget_Btn_SW1) {
  int valueSW1 = param.asInt();                                             // อ่านค่าจากแอป Blynk เมื่อผู้ใช้กดปุ่ม Widget_Btn_SW1 (ON หรือ OFF)
  isUpdatingFromApp[0] = true;                                              // ตั้งค่าสถานะ isUpdatingFromApp[0] เป็น true เพื่อบอกว่ากำลังควบคุม Relay 1 จากแอป Blynk
  uint8_t result = node1.readCoils(0x0000, 1);                              // อ่านสถานะปัจจุบันของ Relay 1 (Coil 0x0000)
  if (result == node1.ku8MBSuccess) {                                       // ตรวจสอบว่าการอ่าน Coil สำเร็จหรือไม่
    bool currentStatus = node1.getResponseBuffer(0);                        // ถ้าอ่านสำเร็จ ให้ดึงสถานะปัจจุบันของ Relay 1 จากบัฟเฟอร์
    if (valueSW1 != currentStatus) {                                        // ตรวจสอบว่ามีการเปลี่ยนสถานะหรือไม่
      if (node1.writeSingleCoil(0x0000, valueSW1) == node1.ku8MBSuccess) {  // ถ้าสถานะที่ส่งจากแอปไม่ตรงกับสถานะปัจจุบัน ให้เขียนสถานะใหม่ไปยัง Relay
        Serial.println(valueSW1 ? "SW1 ON success" : "SW1 OFF success");    // ถ้าเขียนสำเร็จ แสดงข้อความใน Serial Monitor
        Blynk.virtualWrite(Widget_Btn_SW1, valueSW1);                       // อัปเดตสถานะในแอป Blynk ให้ตรงกับสถานะใหม่
      } else {
        // ถ้าเขียนไม่สำเร็จ แสดงข้อความ Error ใน Serial Monitor
        Serial.println(valueSW1 ? "SW1 ON failed" : "SW1 OFF failed");
      }
    } else {
      Serial.println("No change in SW1 status.");  // ถ้าสถานะที่ส่งจากแอปเหมือนกับสถานะปัจจุบัน ไม่ต้องทำอะไร
    }
  } else {
    Serial.print("Failed to read coil status: ");  // ถ้าอ่าน Coil ไม่สำเร็จ แสดง Error Code ใน Serial Monitor
    Serial.println(result);
  }
  isUpdatingFromApp[0] = false;  // รีเซ็ตสถานะ isUpdatingFromApp[0] เป็น false เพื่อบอกว่า Relay 1 ไม่ได้ถูกควบคุมจากแอป Blynk อีกต่อไป
}
// BLYNK_WRITE สำหรับ Relay 2
BLYNK_WRITE(Widget_Btn_SW2) {
  int valueSW2 = param.asInt();

  isUpdatingFromApp[1] = true;  // บอกว่า Relay 2 ถูกควบคุมจากแอป

  uint8_t result = node1.readCoils(0x0001, 1);
  if (result == node1.ku8MBSuccess) {
    bool currentStatus = node1.getResponseBuffer(0);

    if (valueSW2 != currentStatus) {
      if (node1.writeSingleCoil(0x0001, valueSW2) == node1.ku8MBSuccess) {
        Serial.println(valueSW2 ? "SW2 ON success" : "SW2 OFF success");
        Blynk.virtualWrite(Widget_Btn_SW2, valueSW2);
      } else {
        Serial.println(valueSW2 ? "SW2 ON failed" : "SW2 OFF failed");
      }
    } else {
      Serial.println("No change in SW2 status.");
    }
  } else {
    Serial.print("Failed to read coil status: ");
    Serial.println(result);
  }

  isUpdatingFromApp[1] = false;  // รีเซ็ตสถานะ
}
// BLYNK_WRITE สำหรับ Relay 3
BLYNK_WRITE(Widget_Btn_SW3) {
  int valueSW3 = param.asInt();

  isUpdatingFromApp[2] = true;  // บอกว่า Relay 3 ถูกควบคุมจากแอป

  uint8_t result = node1.readCoils(0x0002, 1);
  if (result == node1.ku8MBSuccess) {
    bool currentStatus = node1.getResponseBuffer(0);

    if (valueSW3 != currentStatus) {
      if (node1.writeSingleCoil(0x0002, valueSW3) == node1.ku8MBSuccess) {
        Serial.println(valueSW3 ? "SW3 ON success" : "SW3 OFF success");
        Blynk.virtualWrite(Widget_Btn_SW3, valueSW3);
      } else {
        Serial.println(valueSW3 ? "SW3 ON failed" : "SW3 OFF failed");
      }
    } else {
      Serial.println("No change in SW3 status.");
    }
  } else {
    Serial.print("Failed to read coil status: ");
    Serial.println(result);
  }

  isUpdatingFromApp[2] = false;  // รีเซ็ตสถานะ
}
// BLYNK_WRITE สำหรับ Relay 4
BLYNK_WRITE(Widget_Btn_SW4) {
  int valueSW4 = param.asInt();

  isUpdatingFromApp[3] = true;  // บอกว่า Relay 4 ถูกควบคุมจากแอป

  uint8_t result = node1.readCoils(0x0003, 1);
  if (result == node1.ku8MBSuccess) {
    bool currentStatus = node1.getResponseBuffer(0);

    if (valueSW4 != currentStatus) {
      if (node1.writeSingleCoil(0x0003, valueSW4) == node1.ku8MBSuccess) {
        Serial.println(valueSW4 ? "SW4 ON success" : "SW4 OFF success");
        Blynk.virtualWrite(Widget_Btn_SW4, valueSW4);
      } else {
        Serial.println(valueSW4 ? "SW4 ON failed" : "SW4 OFF failed");
      }
    } else {
      Serial.println("No change in SW4 status.");
    }
  } else {
    Serial.print("Failed to read coil status: ");
    Serial.println(result);
  }

  isUpdatingFromApp[3] = false;  // รีเซ็ตสถานะ
}
// BLYNK_WRITE สำหรับ Relay 5
BLYNK_WRITE(Widget_Btn_SW5) {
  int valueSW5 = param.asInt();

  isUpdatingFromApp[4] = true;  // บอกว่า Relay 5 ถูกควบคุมจากแอป

  uint8_t result = node1.readCoils(0x0004, 1);
  if (result == node1.ku8MBSuccess) {
    bool currentStatus = node1.getResponseBuffer(0);

    if (valueSW5 != currentStatus) {
      if (node1.writeSingleCoil(0x0004, valueSW5) == node1.ku8MBSuccess) {
        Serial.println(valueSW5 ? "SW5 ON success" : "SW5 OFF success");
        Blynk.virtualWrite(Widget_Btn_SW5, valueSW5);
      } else {
        Serial.println(valueSW5 ? "SW5 ON failed" : "SW5 OFF failed");
      }
    } else {
      Serial.println("No change in SW5 status.");
    }
  } else {
    Serial.print("Failed to read coil status: ");
    Serial.println(result);
  }

  isUpdatingFromApp[4] = false;  // รีเซ็ตสถานะ
}
// BLYNK_WRITE สำหรับ Relay 6
BLYNK_WRITE(Widget_Btn_SW6) {
  int valueSW6 = param.asInt();

  isUpdatingFromApp[5] = true;  // บอกว่า Relay 6 ถูกควบคุมจากแอป

  uint8_t result = node1.readCoils(0x0005, 1);
  if (result == node1.ku8MBSuccess) {
    bool currentStatus = node1.getResponseBuffer(0);

    if (valueSW6 != currentStatus) {
      if (node1.writeSingleCoil(0x0005, valueSW6) == node1.ku8MBSuccess) {
        Serial.println(valueSW6 ? "SW6 ON success" : "SW6 OFF success");
        Blynk.virtualWrite(Widget_Btn_SW6, valueSW6);
      } else {
        Serial.println(valueSW6 ? "SW6 ON failed" : "SW6 OFF failed");
      }
    } else {
      Serial.println("No change in SW6 status.");
    }
  } else {
    Serial.print("Failed to read coil status: ");
    Serial.println(result);
  }

  isUpdatingFromApp[5] = false;  // รีเซ็ตสถานะ
}
// BLYNK_WRITE สำหรับ Relay 7
BLYNK_WRITE(Widget_Btn_SW7) {
  int valueSW7 = param.asInt();

  isUpdatingFromApp[6] = true;  // บอกว่า Relay 7 ถูกควบคุมจากแอป

  uint8_t result = node1.readCoils(0x0006, 1);
  if (result == node1.ku8MBSuccess) {
    bool currentStatus = node1.getResponseBuffer(0);

    if (valueSW7 != currentStatus) {
      if (node1.writeSingleCoil(0x0006, valueSW7) == node1.ku8MBSuccess) {
        Serial.println(valueSW7 ? "SW7 ON success" : "SW7 OFF success");
        Blynk.virtualWrite(Widget_Btn_SW7, valueSW7);
      } else {
        Serial.println(valueSW7 ? "SW7 ON failed" : "SW7 OFF failed");
      }
    } else {
      Serial.println("No change in SW7 status.");
    }
  } else {
    Serial.print("Failed to read coil status: ");
    Serial.println(result);
  }

  isUpdatingFromApp[6] = false;  // รีเซ็ตสถานะ
}
// BLYNK_WRITE สำหรับ Relay 8
BLYNK_WRITE(Widget_Btn_SW8) {
  int valueSW8 = param.asInt();

  isUpdatingFromApp[7] = true;  // บอกว่า Relay 8 ถูกควบคุมจากแอป

  uint8_t result = node1.readCoils(0x0007, 1);
  if (result == node1.ku8MBSuccess) {
    bool currentStatus = node1.getResponseBuffer(0);

    if (valueSW8 != currentStatus) {
      if (node1.writeSingleCoil(0x0007, valueSW8) == node1.ku8MBSuccess) {
        Serial.println(valueSW8 ? "SW8 ON success" : "SW8 OFF success");
        Blynk.virtualWrite(Widget_Btn_SW8, valueSW8);
      } else {
        Serial.println(valueSW8 ? "SW8 ON failed" : "SW8 OFF failed");
      }
    } else {
      Serial.println("No change in SW8 status.");
    }
  } else {
    Serial.print("Failed to read coil status: ");
    Serial.println(result);
  }

  isUpdatingFromApp[7] = false;  // รีเซ็ตสถานะ
}
// BLYNK_WRITE สำหรับ Relay 9
BLYNK_WRITE(Widget_Btn_SW9) {
  int valueSW9 = param.asInt();

  isUpdatingFromApp[8] = true;  // บอกว่า Relay 9 ถูกควบคุมจากแอป

  uint8_t result = node1.readCoils(0x0008, 1);
  if (result == node1.ku8MBSuccess) {
    bool currentStatus = node1.getResponseBuffer(0);

    if (valueSW9 != currentStatus) {
      if (node1.writeSingleCoil(0x0008, valueSW9) == node1.ku8MBSuccess) {
        Serial.println(valueSW9 ? "SW9 ON success" : "SW9 OFF success");
        Blynk.virtualWrite(Widget_Btn_SW9, valueSW9);
      } else {
        Serial.println(valueSW9 ? "SW9 ON failed" : "SW9 OFF failed");
      }
    } else {
      Serial.println("No change in SW9 status.");
    }
  } else {
    Serial.print("Failed to read coil status: ");
    Serial.println(result);
  }

  isUpdatingFromApp[8] = false;  // รีเซ็ตสถานะ
}
// BLYNK_WRITE สำหรับ Relay 10
BLYNK_WRITE(Widget_Btn_SW10) {
  int valueSW10 = param.asInt();

  isUpdatingFromApp[9] = true;  // บอกว่า Relay 10 ถูกควบคุมจากแอป

  uint8_t result = node1.readCoils(0x0009, 1);
  if (result == node1.ku8MBSuccess) {
    bool currentStatus = node1.getResponseBuffer(0);

    if (valueSW10 != currentStatus) {
      if (node1.writeSingleCoil(0x0009, valueSW10) == node1.ku8MBSuccess) {
        Serial.println(valueSW10 ? "SW10 ON success" : "SW10 OFF success");
        Blynk.virtualWrite(Widget_Btn_SW10, valueSW10);
      } else {
        Serial.println(valueSW10 ? "SW10 ON failed" : "SW10 OFF failed");
      }
    } else {
      Serial.println("No change in SW10 status.");
    }
  } else {
    Serial.print("Failed to read coil status: ");
    Serial.println(result);
  }

  isUpdatingFromApp[9] = false;  // รีเซ็ตสถานะ
}
// BLYNK_WRITE สำหรับ Relay 11
BLYNK_WRITE(Widget_Btn_SW11) {
  int valueSW11 = param.asInt();

  isUpdatingFromApp[10] = true;  // บอกว่า Relay 11 ถูกควบคุมจากแอป

  uint8_t result = node1.readCoils(0x000A, 1);
  if (result == node1.ku8MBSuccess) {
    bool currentStatus = node1.getResponseBuffer(0);

    if (valueSW11 != currentStatus) {
      if (node1.writeSingleCoil(0x000A, valueSW11) == node1.ku8MBSuccess) {
        Serial.println(valueSW11 ? "SW11 ON success" : "SW11 OFF success");
        Blynk.virtualWrite(Widget_Btn_SW11, valueSW11);
      } else {
        Serial.println(valueSW11 ? "SW11 ON failed" : "SW11 OFF failed");
      }
    } else {
      Serial.println("No change in SW11 status.");
    }
  } else {
    Serial.print("Failed to read coil status: ");
    Serial.println(result);
  }

  isUpdatingFromApp[10] = false;  // รีเซ็ตสถานะ
}
// BLYNK_WRITE สำหรับ Relay12
BLYNK_WRITE(Widget_Btn_SW12) {
  int valueSW12 = param.asInt();

  isUpdatingFromApp[11] = true;  // บอกว่า Relay 12 ถูกควบคุมจากแอป

  uint8_t result = node1.readCoils(0x000B, 1);
  if (result == node1.ku8MBSuccess) {
    bool currentStatus = node1.getResponseBuffer(0);

    if (valueSW12 != currentStatus) {
      if (node1.writeSingleCoil(0x000B, valueSW12) == node1.ku8MBSuccess) {
        Serial.println(valueSW12 ? "SW12 ON success" : "SW12 OFF success");
        Blynk.virtualWrite(Widget_Btn_SW12, valueSW12);
      } else {
        Serial.println(valueSW12 ? "SW12 ON failed" : "SW12 OFF failed");
      }
    } else {
      Serial.println("No change in SW12 status.");
    }
  } else {
    Serial.print("Failed to read coil status: ");
    Serial.println(result);
  }

  isUpdatingFromApp[11] = false;  // รีเซ็ตสถานะ
}

//Loop function
void loop() {
  // Run Blynk processes if connected
  if (Blynk.connected()) {
    Blynk.run();
  }
  // Run timer tasks
  timer.run();
  // Add a small delay to prevent watchdog timeout
  delay(1);
}
