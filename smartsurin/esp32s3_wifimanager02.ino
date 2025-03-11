#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ModbusMaster.h>
#include <WiFiManager.h>
#include <Preferences.h>

#include <TimeLib.h>    // ✅ ไลบรารีจัดการเวลา
#include <WidgetRTC.h>  // ✅ ไลบรารี RTC ของ Blynk
WidgetRTC rtc;          // ✅ สร้างอ็อบเจ็กต์ RTC
// ✅ เพิ่ม Virtual Pin สำหรับแสดงเวลาและวันที่
#define Time_VPin V7  // เวลา (HH:MM:SS)
#define Date_VPin V8  // วันที่ (DD/MM/YYYY)

// Wi-Fi and Blynk credentials
const char auth[] = "JD_zF70EX3EPJ56MCjzhFFXCXI0CXX-x";  // Blynk Token

// GPIO Pins
#define TXD 17
#define RXD 18
#define RELAY1 1   // ปั๊มน้ำ
#define RELAY2 2   // เช็คสถานะ Blynk
#define RELAY3 41  // วาล์ว 1
#define RELAY4 42  // วาล์ว 2

// Blynk Virtual Pins
#define Widget_Btn_SW1 V1
#define Widget_Btn_SW2 V2
#define Widget_Btn_SW3 V3
#define SoilMoisture_VPin V4
#define AutoManual_VPin V5     // ปุ่มเลือก Auto/Manual
#define SoilThreshold_VPin V6  // Slider ตั้งค่าความชื้นที่ต้องการ

// Modbus settings
ModbusMaster node1;
WiFiManager wm;
BlynkTimer timer;
Preferences preferences;

// ตัวแปรควบคุม Auto/Manual และค่าความชื้น
bool isAutoMode = false;
float soilThreshold = 50.0;  // ค่าเริ่มต้นของค่าความชื้นที่ต้องการ (%)
float soilMoisture = 0.0;    // ค่าความชื้นที่อ่านได้จากเซ็นเซอร์

void checkConnections();
void readSoilMoisture();

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD, TXD);

  // ตั้งค่าขา GPIO สำหรับ Relay
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  pinMode(RELAY4, OUTPUT);

  digitalWrite(RELAY1, LOW);
  digitalWrite(RELAY2, LOW);
  digitalWrite(RELAY3, LOW);
  digitalWrite(RELAY4, LOW);

  // ✅ โหลดค่าที่บันทึกไว้จาก Flash Storage
  preferences.begin("sensor_data", false);
  isAutoMode = preferences.getBool("auto_mode", false);
  soilThreshold = preferences.getFloat("soil_threshold", 50.0);
  soilMoisture = preferences.getFloat("soil_moisture", 0.0);

  Serial.print("Loaded Auto Mode: ");
  Serial.println(isAutoMode ? "ON" : "OFF");
  Serial.print("Loaded Soil Threshold: ");
  Serial.println(soilThreshold);
  Serial.print("Loaded Soil Moisture: ");
  Serial.println(soilMoisture);

  // ✅ ใช้ WiFiManager
  if (!wm.autoConnect("ESP32S3-Smartfarm01")) {
    Serial.println("Failed to connect. Restarting...");
    delay(3000);
    ESP.restart();
  }

  Serial.println("Wi-Fi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // ✅ เชื่อมต่อ Blynk
  Blynk.begin(auth, WiFi.SSID().c_str(), WiFi.psk().c_str(), "iotservices.thddns.net", 5535);

  rtc.begin();  // เรียกใช้ Blynk RTC

  // ✅ ตั้งค่า Modbus Address สำหรับ Soil Moisture Sensor (ID1)
  node1.begin(1, Serial2);

  // ตั้งค่าตัวจับเวลา
  timer.setInterval(10000L, checkConnections);     // เช็ค Wi-Fi และ Blynk ทุก 5 วินาที
  timer.setInterval(15000L, readSoilMoisture);     // อ่านค่าความชื้นทุก 5 วินาที
  timer.setInterval(1000L, sendDateTimeToBlynk);  // ✅ อัปเดตเวลาและวันที่ทุก 1 วินาที
}

// ✅ เมื่อเชื่อมต่อ Blynk ได้สำเร็จ
BLYNK_CONNECTED() {
  Serial.println("Blynk connected! Synchronizing virtual pins...");

  // ✅ ซิงค์ค่าทั้งหมดจากเซิร์ฟเวอร์ Blynk
  Blynk.syncVirtual(AutoManual_VPin);
  Blynk.syncVirtual(SoilThreshold_VPin);
  Blynk.syncVirtual(Widget_Btn_SW1);
  Blynk.syncVirtual(Widget_Btn_SW2);
  Blynk.syncVirtual(Widget_Btn_SW3);
  Blynk.syncVirtual(SoilMoisture_VPin);

  digitalWrite(RELAY2, HIGH);  // แสดงสถานะว่า Blynk เชื่อมต่อสำเร็จ

  // ✅ โหลดค่าโหมด Auto/Manual ที่บันทึกไว้ใน Flash Storage
  isAutoMode = preferences.getBool("auto_mode", false);

  // ✅ อัปเดตสถานะของปุ่ม Auto/Manual ใน Blynk
  Blynk.virtualWrite(AutoManual_VPin, isAutoMode);

  // ✅ อัปเดตสถานะปุ่มปั๊มน้ำและวาล์ว ตามค่า Relay จริง
  Blynk.virtualWrite(Widget_Btn_SW1, digitalRead(RELAY1));
  Blynk.virtualWrite(Widget_Btn_SW2, digitalRead(RELAY3));
  Blynk.virtualWrite(Widget_Btn_SW3, digitalRead(RELAY4));

  // ✅ แสดงค่าทั้งหมดใน Serial Monitor
  Serial.print("Loaded Auto Mode: ");
  Serial.println(isAutoMode ? "Auto" : "Manual");

  Serial.print("Relay1 (Pump): ");
  Serial.println(digitalRead(RELAY1) ? "ON" : "OFF");

  Serial.print("Relay3 (Valve 1): ");
  Serial.println(digitalRead(RELAY3) ? "ON" : "OFF");

  Serial.print("Relay4 (Valve 2): ");
  Serial.println(digitalRead(RELAY4) ? "ON" : "OFF");

  Serial.println("Blynk connected! Synchronizing RTC...");
  rtc.begin();                // ✅ เรียกใช้ RTC ของ Blynk
  Blynk.syncVirtual(V7, V8);  // ✅ ซิงค์ค่าเวลาและวันที่จาก Blynk
  setSyncInterval(600);       // ✅ ตั้งให้ซิงค์เวลาใหม่ทุก 10 นาที
}

// ✅ ตรวจสอบ Wi-Fi และ Blynk ทุก 5 วินาที
void checkConnections() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi disconnected! Restarting WiFiManager...");
    WiFi.disconnect();
    wm.autoConnect("ESP32S3-Modbus");
  }

  if (!Blynk.connected()) {
    Serial.println("Blynk disconnected! Reconnecting...");
    Blynk.connect();
    digitalWrite(RELAY2, LOW);
  } else {
    digitalWrite(RELAY2, HIGH);
  }
}

// ✅ ควบคุม Relay1 (ปั๊มน้ำ) ใน Manual Mode
BLYNK_WRITE(Widget_Btn_SW1) {
  if (!isAutoMode) {
    int state = param.asInt();
    digitalWrite(RELAY1, state);
  }
}

// ✅ ควบคุม Relay3 (วาล์ว 1) ใน Manual Mode
BLYNK_WRITE(Widget_Btn_SW2) {
  if (!isAutoMode) {
    int state = param.asInt();
    digitalWrite(RELAY3, state);
  }
}

// ✅ ควบคุม Relay4 (วาล์ว 2) ใน Manual Mode
BLYNK_WRITE(Widget_Btn_SW3) {
  if (!isAutoMode) {
    int state = param.asInt();
    digitalWrite(RELAY4, state);
  }
}

// ✅ เปลี่ยนโหมด Auto/Manual จาก Eventor หรือจากปุ่ม Blynk
// ✅ เปลี่ยนโหมด Auto/Manual
BLYNK_WRITE(AutoManual_VPin) {
    isAutoMode = param.asInt(); // รับค่าจาก Eventor หรือปุ่ม Blynk
    Serial.print("Mode changed to: ");
    Serial.println(isAutoMode ? "Auto" : "Manual");

    // ✅ บันทึกค่าโหมดลง Flash Storage
    preferences.putBool("auto_mode", isAutoMode);

    // ✅ ถ้าเปลี่ยนเป็น Manual Mode → ปิดปั๊มและวาล์วทั้งหมด
    if (!isAutoMode) {
        digitalWrite(RELAY1, LOW); // ปิดปั๊มน้ำ
        digitalWrite(RELAY3, LOW); // ปิดวาล์ว 1
        digitalWrite(RELAY4, LOW); // ปิดวาล์ว 2

        // ✅ อัปเดตสถานะใน Blynk ให้ตรงกับรีเลย์
        Blynk.virtualWrite(Widget_Btn_SW1, 0);
        Blynk.virtualWrite(Widget_Btn_SW2, 0);
        Blynk.virtualWrite(Widget_Btn_SW3, 0);

        Serial.println("Switched to Manual Mode: All pumps and valves turned OFF.");
    }

    // ✅ ส่งค่ากลับไปที่ Blynk เพื่ออัปเดตสถานะปุ่ม
    Blynk.virtualWrite(AutoManual_VPin, isAutoMode);
}


// ✅ รับค่าจาก Slider (ค่าความชื้นที่ต้องการ)
BLYNK_WRITE(SoilThreshold_VPin) {
  soilThreshold = param.asFloat();
  Serial.print("Soil Moisture Threshold set to: ");
  Serial.print(soilThreshold);
  Serial.println(" %");

  // ✅ บันทึกค่าความชื้นที่ต้องการลง Flash Storage
  preferences.putFloat("soil_threshold", soilThreshold);
}

// ✅ ฟังก์ชันอ่านค่าความชื้นในดิน
void readSoilMoisture() {
  uint8_t result;
  Serial.println("Reading Soil Moisture...");

  result = node1.readHoldingRegisters(0x0000, 3);
  if (result == node1.ku8MBSuccess) {
    float newSoilMoisture = node1.getResponseBuffer(2) / 10.0f;

    if (newSoilMoisture >= 0.0 && newSoilMoisture <= 100.0) {
      soilMoisture = newSoilMoisture;

      Serial.print("Soil Moisture: ");
      Serial.print(soilMoisture);
      Serial.println(" %");

      Blynk.virtualWrite(SoilMoisture_VPin, soilMoisture);

      // ✅ บันทึกค่าความชื้นลง Flash Storage
      preferences.putFloat("soil_moisture", soilMoisture);

      // ✅ ควบคุมปั๊มน้ำและวาล์วใน Auto Mode
      if (isAutoMode) {
        if (soilMoisture < soilThreshold) {
          Serial.println("Soil moisture is LOW, turning ON pump and valve.");
          digitalWrite(RELAY1, HIGH);
          digitalWrite(RELAY3, HIGH);

          // ✅ อัปเดตสถานะของปุ่มใน Blynk
          Blynk.virtualWrite(Widget_Btn_SW1, 1);  // ปั๊มน้ำ: เปิด
          Blynk.virtualWrite(Widget_Btn_SW2, 1);  // วาล์ว 1: เปิด
        } else {
          Serial.println("Soil moisture is OK, turning OFF pump and valve.");
          digitalWrite(RELAY1, LOW);
          digitalWrite(RELAY3, LOW);

          // ✅ อัปเดตสถานะของปุ่มใน Blynk
          Blynk.virtualWrite(Widget_Btn_SW1, 0);  // ปั๊มน้ำ: ปิด
          Blynk.virtualWrite(Widget_Btn_SW2, 0);  // วาล์ว 1: ปิด
        }
      }
    }
  } else {
    Serial.println("Failed to read Soil Moisture data!");
  }
}

// ✅ ฟังก์ชันส่งเวลาและวันที่ไปที่ Blynk
void sendDateTimeToBlynk() {
  if (Blynk.connected() && year() > 1970) {  // ✅ เช็คว่ามีการซิงค์เวลาแล้ว
    char timeBuffer[10], dateBuffer[15];

    // ✅ ดึงค่าจาก RTC
    int currentHour = hour();
    int currentMinute = minute();
    int currentSecond = second();
    int currentDay = day();
    int currentMonth = month();
    int currentYear = year();

    // ✅ จัดรูปแบบข้อความเวลา (HH:MM:SS)
    sprintf(timeBuffer, "%02d:%02d:%02d", currentHour, currentMinute, currentSecond);

    // ✅ จัดรูปแบบข้อความวันที่ (DD/MM/YYYY)
    sprintf(dateBuffer, "%02d/%02d/%04d", currentDay, currentMonth, currentYear);

    // ✅ ส่งค่าไปที่ Blynk
    Blynk.virtualWrite(Time_VPin, timeBuffer);
    Blynk.virtualWrite(Date_VPin, dateBuffer);

    // ✅ แสดงค่าบน Serial Monitor
    Serial.print("Time: ");
    Serial.println(timeBuffer);
    Serial.print("Date: ");
    Serial.println(dateBuffer);
  } else {
    Serial.println("RTC not synced yet...");
  }
}

// ✅ ฟั่งชั่นการทำงานวนรอบ
void loop() {
  if (Blynk.connected()) {
    Blynk.run();
  }
  timer.run();

  Serial.print("Current Mode: ");
  Serial.println(isAutoMode ? "Auto" : "Manual");

  delay(1000);
}
