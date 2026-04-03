#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ModbusMaster.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <TimeLib.h>
#include <WidgetRTC.h>

WidgetRTC rtc;

// =========================
// Blynk Auth
// =========================
const char auth[] = ""; //Token

// =========================
// GPIO Pins
// =========================
#define TXD 17
#define RXD 18

#define RELAY1 1    // GPIO1  = Blynk Connect Indicator
#define RELAY2 2    // GPIO2  = Fan
#define RELAY3 41   // GPIO41 = Valve1
#define RELAY4 42   // GPIO42 = Valve2
#define RELAY5 45   // GPIO45 = Valve3
#define RELAY6 46   // GPIO46 = Valve4

// =========================
// Relay Logic
// Active High
// ถ้าบอร์ดกลับด้านค่อยสลับเป็น LOW/HIGH
// =========================
#define RELAY_ON  HIGH
#define RELAY_OFF LOW

// =========================
// Blynk Virtual Pins
// =========================
// Common
#define VPIN_FAN              V1
#define VPIN_TIME             V7
#define VPIN_DATE             V8
#define VPIN_VALVE4           V10

// Zone 1
#define VPIN_VALVE1           V2
#define VPIN_SOIL1            V4
#define VPIN_AUTO1            V5
#define VPIN_THRESHOLD1       V6

// Zone 2
#define VPIN_VALVE2           V11
#define VPIN_SOIL2            V12
#define VPIN_AUTO2            V13
#define VPIN_THRESHOLD2       V14

// Zone 3
#define VPIN_VALVE3           V15
#define VPIN_SOIL3            V16
#define VPIN_AUTO3            V17
#define VPIN_THRESHOLD3       V18

// Fan Sensor (Slave ID4)
#define VPIN_HUM4             V19
#define VPIN_TEMP4            V20
#define VPIN_AUTO_FAN         V21
#define VPIN_TEMP_THRESHOLD   V22

// =========================
// Global Objects
// =========================
ModbusMaster node;
WiFiManager wm;
BlynkTimer timer;
Preferences preferences;

// =========================
// Global Variables
// =========================
unsigned long lastBlynkConnectTime = 0;

// Zone 1
bool  isAutoMode1 = false;
float soilThreshold1 = 50.0;
float soilMoisture1 = 0.0;

// Zone 2
bool  isAutoMode2 = false;
float soilThreshold2 = 50.0;
float soilMoisture2 = 0.0;

// Zone 3
bool  isAutoMode3 = false;
float soilThreshold3 = 50.0;
float soilMoisture3 = 0.0;

// Fan Auto
bool  isAutoFan = false;
float tempThreshold = 35.0;
float temperature4 = 0.0;
float humidity4 = 0.0;

// =========================
// Function Prototypes
// =========================
void checkConnections();
void readAllSensors();
void readZone1();
void readZone2();
void readZone3();
void readTempHumZone4();

bool readSoilBySlave(uint8_t slaveId, float &value);
bool readTempHumBySlave4(float &tempValue, float &humValue);

void controlZone1Auto();
void controlZone2Auto();
void controlZone3Auto();
void controlFanAuto();

void sendDateTimeToBlynk();
void syncAllToBlynk();

// =========================
// Setup
// =========================
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD, TXD);

  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  pinMode(RELAY4, OUTPUT);
  pinMode(RELAY5, OUTPUT);
  pinMode(RELAY6, OUTPUT);

  digitalWrite(RELAY1, RELAY_OFF);
  digitalWrite(RELAY2, RELAY_OFF);
  digitalWrite(RELAY3, RELAY_OFF);
  digitalWrite(RELAY4, RELAY_OFF);
  digitalWrite(RELAY5, RELAY_OFF);
  digitalWrite(RELAY6, RELAY_OFF);

  preferences.begin("sensor_data", false);

  isAutoMode1    = preferences.getBool("auto1", false);
  soilThreshold1 = preferences.getFloat("th1", 50.0);
  soilMoisture1  = preferences.getFloat("soil1", 0.0);

  isAutoMode2    = preferences.getBool("auto2", false);
  soilThreshold2 = preferences.getFloat("th2", 50.0);
  soilMoisture2  = preferences.getFloat("soil2", 0.0);

  isAutoMode3    = preferences.getBool("auto3", false);
  soilThreshold3 = preferences.getFloat("th3", 50.0);
  soilMoisture3  = preferences.getFloat("soil3", 0.0);

  isAutoFan      = preferences.getBool("autofan", false);
  tempThreshold  = preferences.getFloat("temp_th", 35.0);
  temperature4   = preferences.getFloat("temp4", 0.0);
  humidity4      = preferences.getFloat("hum4", 0.0);

  Serial.println("=== Loaded Preferences ===");
  Serial.printf("Zone1 -> Auto:%d Threshold:%.1f Soil:%.1f\n", isAutoMode1, soilThreshold1, soilMoisture1);
  Serial.printf("Zone2 -> Auto:%d Threshold:%.1f Soil:%.1f\n", isAutoMode2, soilThreshold2, soilMoisture2);
  Serial.printf("Zone3 -> Auto:%d Threshold:%.1f Soil:%.1f\n", isAutoMode3, soilThreshold3, soilMoisture3);
  Serial.printf("Fan   -> Auto:%d TempTH:%.1f Temp:%.1f Hum:%.1f\n", isAutoFan, tempThreshold, temperature4, humidity4);

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin();

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi not connected! Starting WiFiManager...");

    wm.setCleanConnect(true);
    wm.setEnableConfigPortal(true);
    wm.setConfigPortalTimeout(180);
    //wm.startConfigPortal("ESP32S3-Smartfarm01");
    wm.startConfigPortal("ESP32S3-Smartfarmkkk", "0814111142");

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFi connected via Config Portal!");
      WiFi.mode(WIFI_STA);
    } else {
      Serial.println("Still no WiFi. Restarting...");
      delay(3000);
      ESP.restart();
    }
  }

  Serial.println("\nWi-Fi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  WiFi.mode(WIFI_STA);
  lastBlynkConnectTime = millis();

  Blynk.config(auth, "ip-address-server", 8080); //แก้ ip-address-server
  Blynk.connect();

  rtc.begin();

  node.begin(1, Serial2);

  timer.setInterval(10000L, checkConnections);
  timer.setInterval(15000L, readAllSensors);
  timer.setInterval(10000L, sendDateTimeToBlynk);

  // อ่านค่าเร็วหลังบูต
  timer.setTimeout(3000L, readAllSensors);
}

// =========================
// Blynk Connected
// =========================
BLYNK_CONNECTED() {
  Serial.println("Blynk connected! Synchronizing virtual pins...");
  digitalWrite(RELAY1, RELAY_ON);

  // sync auto flags ก่อน
  Blynk.syncVirtual(VPIN_AUTO1, VPIN_THRESHOLD1);
  Blynk.syncVirtual(VPIN_AUTO2, VPIN_THRESHOLD2);
  Blynk.syncVirtual(VPIN_AUTO3, VPIN_THRESHOLD3);
  Blynk.syncVirtual(VPIN_AUTO_FAN, VPIN_TEMP_THRESHOLD);

  // sync manual controls เฉพาะตัวที่ไม่ชน Auto Fan
  Blynk.syncVirtual(VPIN_VALVE4);

  if (!isAutoMode1) Blynk.syncVirtual(VPIN_VALVE1);
  if (!isAutoMode2) Blynk.syncVirtual(VPIN_VALVE2);
  if (!isAutoMode3) Blynk.syncVirtual(VPIN_VALVE3);
  if (!isAutoFan)   Blynk.syncVirtual(VPIN_FAN);

  syncAllToBlynk();

  // อ่าน sensor ใหม่และคุมทันทีหลัง connect
  readAllSensors();

  rtc.begin();
  setSyncInterval(600);
}

// =========================
// Sync All Widgets
// =========================
void syncAllToBlynk() {
  Blynk.virtualWrite(VPIN_FAN, digitalRead(RELAY2) == RELAY_ON ? 1 : 0);
  Blynk.virtualWrite(VPIN_VALVE4, digitalRead(RELAY6) == RELAY_ON ? 1 : 0);

  Blynk.virtualWrite(VPIN_AUTO1, isAutoMode1);
  Blynk.virtualWrite(VPIN_THRESHOLD1, soilThreshold1);
  Blynk.virtualWrite(VPIN_SOIL1, soilMoisture1);
  Blynk.virtualWrite(VPIN_VALVE1, digitalRead(RELAY3) == RELAY_ON ? 1 : 0);

  Blynk.virtualWrite(VPIN_AUTO2, isAutoMode2);
  Blynk.virtualWrite(VPIN_THRESHOLD2, soilThreshold2);
  Blynk.virtualWrite(VPIN_SOIL2, soilMoisture2);
  Blynk.virtualWrite(VPIN_VALVE2, digitalRead(RELAY4) == RELAY_ON ? 1 : 0);

  Blynk.virtualWrite(VPIN_AUTO3, isAutoMode3);
  Blynk.virtualWrite(VPIN_THRESHOLD3, soilThreshold3);
  Blynk.virtualWrite(VPIN_SOIL3, soilMoisture3);
  Blynk.virtualWrite(VPIN_VALVE3, digitalRead(RELAY5) == RELAY_ON ? 1 : 0);

  Blynk.virtualWrite(VPIN_AUTO_FAN, isAutoFan);
  Blynk.virtualWrite(VPIN_TEMP_THRESHOLD, tempThreshold);
  Blynk.virtualWrite(VPIN_HUM4, humidity4);
  Blynk.virtualWrite(VPIN_TEMP4, temperature4);
}

// =========================
// Check Connections
// =========================
void checkConnections() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi disconnected! Attempting reconnect...");

    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    WiFi.begin();

    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 5000) {
      delay(500);
      Serial.print(".");
    }

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("\nReconnect failed. Launching Config Portal...");

      wm.setEnableConfigPortal(true);
      wm.setConfigPortalTimeout(120);
      wm.startConfigPortal("ESP32S3-Smartfarm01");

      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("WiFi connected via Config Portal!");
        WiFi.mode(WIFI_STA);
      } else {
        Serial.println("WiFi still not connected.");
      }
    } else {
      Serial.println("\nWi-Fi Reconnected!");
      WiFi.mode(WIFI_STA);
    }
  }

  if (!Blynk.connected()) {
    Serial.println("Blynk disconnected! Attempting to reconnect...");
    Blynk.connect();

    if (millis() - lastBlynkConnectTime > 120000) {
      Serial.println("Blynk reconnect timeout. Restarting device...");
      ESP.restart();
    }

    digitalWrite(RELAY1, RELAY_OFF);
  } else {
    digitalWrite(RELAY1, RELAY_ON);
    lastBlynkConnectTime = millis();
  }
}

// =========================
// Manual Controls
// =========================
BLYNK_WRITE(VPIN_FAN) {
  if (!isAutoFan) {
    int state = param.asInt();
    digitalWrite(RELAY2, state ? RELAY_ON : RELAY_OFF);
    Serial.print("Fan Manual = ");
    Serial.println(state ? "ON" : "OFF");
  } else {
    Serial.println("Fan is in AUTO mode, manual command ignored.");
    Blynk.virtualWrite(VPIN_FAN, digitalRead(RELAY2) == RELAY_ON ? 1 : 0);
  }
}

BLYNK_WRITE(VPIN_VALVE4) {
  int state = param.asInt();
  digitalWrite(RELAY6, state ? RELAY_ON : RELAY_OFF);
  Serial.print("Valve4 = ");
  Serial.println(state ? "ON" : "OFF");
}

// Zone 1
BLYNK_WRITE(VPIN_VALVE1) {
  if (!isAutoMode1) {
    int state = param.asInt();
    digitalWrite(RELAY3, state ? RELAY_ON : RELAY_OFF);
    Serial.print("Valve1 Manual = ");
    Serial.println(state ? "ON" : "OFF");
  }
}

BLYNK_WRITE(VPIN_AUTO1) {
  isAutoMode1 = param.asInt();
  preferences.putBool("auto1", isAutoMode1);

  Serial.print("Zone1 Mode = ");
  Serial.println(isAutoMode1 ? "Auto" : "Manual");

  if (!isAutoMode1) {
    digitalWrite(RELAY3, RELAY_OFF);
    Blynk.virtualWrite(VPIN_VALVE1, 0);
  } else {
    controlZone1Auto();
  }
}

BLYNK_WRITE(VPIN_THRESHOLD1) {
  soilThreshold1 = param.asFloat();
  preferences.putFloat("th1", soilThreshold1);

  Serial.print("Zone1 Threshold = ");
  Serial.println(soilThreshold1);

  if (isAutoMode1) {
    controlZone1Auto();
  }
}

// Zone 2
BLYNK_WRITE(VPIN_VALVE2) {
  if (!isAutoMode2) {
    int state = param.asInt();
    digitalWrite(RELAY4, state ? RELAY_ON : RELAY_OFF);
    Serial.print("Valve2 Manual = ");
    Serial.println(state ? "ON" : "OFF");
  }
}

BLYNK_WRITE(VPIN_AUTO2) {
  isAutoMode2 = param.asInt();
  preferences.putBool("auto2", isAutoMode2);

  Serial.print("Zone2 Mode = ");
  Serial.println(isAutoMode2 ? "Auto" : "Manual");

  if (!isAutoMode2) {
    digitalWrite(RELAY4, RELAY_OFF);
    Blynk.virtualWrite(VPIN_VALVE2, 0);
  } else {
    controlZone2Auto();
  }
}

BLYNK_WRITE(VPIN_THRESHOLD2) {
  soilThreshold2 = param.asFloat();
  preferences.putFloat("th2", soilThreshold2);

  Serial.print("Zone2 Threshold = ");
  Serial.println(soilThreshold2);

  if (isAutoMode2) {
    controlZone2Auto();
  }
}

// Zone 3
BLYNK_WRITE(VPIN_VALVE3) {
  if (!isAutoMode3) {
    int state = param.asInt();
    digitalWrite(RELAY5, state ? RELAY_ON : RELAY_OFF);
    Serial.print("Valve3 Manual = ");
    Serial.println(state ? "ON" : "OFF");
  }
}

BLYNK_WRITE(VPIN_AUTO3) {
  isAutoMode3 = param.asInt();
  preferences.putBool("auto3", isAutoMode3);

  Serial.print("Zone3 Mode = ");
  Serial.println(isAutoMode3 ? "Auto" : "Manual");

  if (!isAutoMode3) {
    digitalWrite(RELAY5, RELAY_OFF);
    Blynk.virtualWrite(VPIN_VALVE3, 0);
  } else {
    controlZone3Auto();
  }
}

BLYNK_WRITE(VPIN_THRESHOLD3) {
  soilThreshold3 = param.asFloat();
  preferences.putFloat("th3", soilThreshold3);

  Serial.print("Zone3 Threshold = ");
  Serial.println(soilThreshold3);

  if (isAutoMode3) {
    controlZone3Auto();
  }
}

// Fan Auto
BLYNK_WRITE(VPIN_AUTO_FAN) {
  isAutoFan = param.asInt();
  preferences.putBool("autofan", isAutoFan);

  Serial.print("Fan Mode = ");
  Serial.println(isAutoFan ? "Auto" : "Manual");

  if (!isAutoFan) {
    digitalWrite(RELAY2, RELAY_OFF);
    Blynk.virtualWrite(VPIN_FAN, 0);
  } else {
    // เปิด Auto แล้วอ่านค่าใหม่ทันที
    readTempHumZone4();
  }
}

BLYNK_WRITE(VPIN_TEMP_THRESHOLD) {
  tempThreshold = param.asFloat();
  preferences.putFloat("temp_th", tempThreshold);

  Serial.print("Fan Temperature Threshold = ");
  Serial.println(tempThreshold);

  if (isAutoFan) {
    // เปลี่ยน threshold แล้วอ่านใหม่ทันที
    readTempHumZone4();
  }
}

// =========================
// Read All Sensors
// =========================
void readAllSensors() {
  Serial.println("=== Reading All Sensors ===");
  readZone1();
  delay(300);
  readZone2();
  delay(300);
  readZone3();
  delay(300);
  readTempHumZone4();
}

// =========================
// Soil Read
// =========================
bool readSoilBySlave(uint8_t slaveId, float &value) {
  node.begin(slaveId, Serial2);
  uint8_t result = node.readHoldingRegisters(0x0000, 3);

  if (result == node.ku8MBSuccess) {
    float tempValue = node.getResponseBuffer(2) / 10.0f;

    if (tempValue >= 0.0 && tempValue <= 100.0) {
      value = tempValue;
      return true;
    }
  }

  Serial.print("Read Soil failed, Slave ID = ");
  Serial.println(slaveId);
  return false;
}

// =========================
// Temp/Hum Read Slave 4
// Mapping used now:
// reg0 = Humidity x10
// reg1 = Temperature x10
// =========================
bool readTempHumBySlave4(float &tempValue, float &humValue) {
  node.begin(4, Serial2);
  uint8_t result = node.readHoldingRegisters(0x0000, 2);

  if (result == node.ku8MBSuccess) {
    uint16_t raw0 = node.getResponseBuffer(0);
    uint16_t raw1 = node.getResponseBuffer(1);

    Serial.print("Slave4 Raw Reg0 = ");
    Serial.println(raw0);
    Serial.print("Slave4 Raw Reg1 = ");
    Serial.println(raw1);

    humValue  = raw0 / 10.0f;
    tempValue = raw1 / 10.0f;

    if (tempValue >= -40.0 && tempValue <= 100.0 &&
        humValue >= 0.0 && humValue <= 100.0) {
      return true;
    }

    Serial.println("Slave4 value out of range!");
    Serial.print("Parsed Humidity = ");
    Serial.println(humValue);
    Serial.print("Parsed Temperature = ");
    Serial.println(tempValue);
  } else {
    Serial.println("Read Temp/Humidity Slave4 failed!");
  }

  return false;
}

// =========================
// Zone 1
// =========================
void readZone1() {
  float newValue = 0.0;

  if (readSoilBySlave(1, newValue)) {
    soilMoisture1 = newValue;
    preferences.putFloat("soil1", soilMoisture1);
    Blynk.virtualWrite(VPIN_SOIL1, soilMoisture1);

    Serial.print("Zone1 Soil Moisture = ");
    Serial.println(soilMoisture1);

    controlZone1Auto();
  } else {
    Serial.println("Zone1 read failed!");
  }
}

void controlZone1Auto() {
  if (isAutoMode1) {
    Serial.print("Zone1 AUTO Check -> Soil = ");
    Serial.print(soilMoisture1);
    Serial.print(" Threshold = ");
    Serial.println(soilThreshold1);

    if (soilMoisture1 < soilThreshold1) {
      digitalWrite(RELAY3, RELAY_ON);
      Blynk.virtualWrite(VPIN_VALVE1, 1);
      Serial.println("Zone1 AUTO -> Valve1 ON");
    } else {
      digitalWrite(RELAY3, RELAY_OFF);
      Blynk.virtualWrite(VPIN_VALVE1, 0);
      Serial.println("Zone1 AUTO -> Valve1 OFF");
    }
  }
}

// =========================
// Zone 2
// =========================
void readZone2() {
  float newValue = 0.0;

  if (readSoilBySlave(2, newValue)) {
    soilMoisture2 = newValue;
    preferences.putFloat("soil2", soilMoisture2);
    Blynk.virtualWrite(VPIN_SOIL2, soilMoisture2);

    Serial.print("Zone2 Soil Moisture = ");
    Serial.println(soilMoisture2);

    controlZone2Auto();
  } else {
    Serial.println("Zone2 read failed!");
  }
}

void controlZone2Auto() {
  if (isAutoMode2) {
    Serial.print("Zone2 AUTO Check -> Soil = ");
    Serial.print(soilMoisture2);
    Serial.print(" Threshold = ");
    Serial.println(soilThreshold2);

    if (soilMoisture2 < soilThreshold2) {
      digitalWrite(RELAY4, RELAY_ON);
      Blynk.virtualWrite(VPIN_VALVE2, 1);
      Serial.println("Zone2 AUTO -> Valve2 ON");
    } else {
      digitalWrite(RELAY4, RELAY_OFF);
      Blynk.virtualWrite(VPIN_VALVE2, 0);
      Serial.println("Zone2 AUTO -> Valve2 OFF");
    }
  }
}

// =========================
// Zone 3
// =========================
void readZone3() {
  float newValue = 0.0;

  if (readSoilBySlave(3, newValue)) {
    soilMoisture3 = newValue;
    preferences.putFloat("soil3", soilMoisture3);
    Blynk.virtualWrite(VPIN_SOIL3, soilMoisture3);

    Serial.print("Zone3 Soil Moisture = ");
    Serial.println(soilMoisture3);

    controlZone3Auto();
  } else {
    Serial.println("Zone3 read failed!");
  }
}

void controlZone3Auto() {
  if (isAutoMode3) {
    Serial.print("Zone3 AUTO Check -> Soil = ");
    Serial.print(soilMoisture3);
    Serial.print(" Threshold = ");
    Serial.println(soilThreshold3);

    if (soilMoisture3 < soilThreshold3) {
      digitalWrite(RELAY5, RELAY_ON);
      Blynk.virtualWrite(VPIN_VALVE3, 1);
      Serial.println("Zone3 AUTO -> Valve3 ON");
    } else {
      digitalWrite(RELAY5, RELAY_OFF);
      Blynk.virtualWrite(VPIN_VALVE3, 0);
      Serial.println("Zone3 AUTO -> Valve3 OFF");
    }
  }
}

// =========================
// Zone 4 = Temp/Humidity
// =========================
void readTempHumZone4() {
  float newTemp = 0.0;
  float newHum = 0.0;

  if (readTempHumBySlave4(newTemp, newHum)) {
    temperature4 = newTemp;
    humidity4 = newHum;

    preferences.putFloat("temp4", temperature4);
    preferences.putFloat("hum4", humidity4);

    Blynk.virtualWrite(VPIN_HUM4, humidity4);
    Blynk.virtualWrite(VPIN_TEMP4, temperature4);

    Serial.print("Zone4 Humidity = ");
    Serial.print(humidity4);
    Serial.println(" %");

    Serial.print("Zone4 Temperature = ");
    Serial.print(temperature4);
    Serial.println(" C");

    controlFanAuto();
  } else {
    Serial.println("Zone4 Temp/Humidity read failed!");
  }
}

void controlFanAuto() {
  if (isAutoFan) {
    Serial.print("Fan AUTO Check -> Temp = ");
    Serial.print(temperature4);
    Serial.print(" C, Threshold = ");
    Serial.println(tempThreshold);

    if (temperature4 > tempThreshold) {
      digitalWrite(RELAY2, RELAY_ON);
      Blynk.virtualWrite(VPIN_FAN, 1);
      Serial.println("Fan AUTO -> ON");
    } else {
      digitalWrite(RELAY2, RELAY_OFF);
      Blynk.virtualWrite(VPIN_FAN, 0);
      Serial.println("Fan AUTO -> OFF");
    }
  } else {
    Serial.println("Fan Mode = Manual, AUTO skipped");
  }
}

// =========================
// Send Date/Time
// =========================
void sendDateTimeToBlynk() {
  if (Blynk.connected() && year() > 1970) {
    char timeBuffer[10];
    char dateBuffer[15];

    sprintf(timeBuffer, "%02d:%02d:%02d", hour(), minute(), second());
    sprintf(dateBuffer, "%02d/%02d/%04d", day(), month(), year());

    Blynk.virtualWrite(VPIN_TIME, timeBuffer);
    Blynk.virtualWrite(VPIN_DATE, dateBuffer);

    Serial.print("Time: ");
    Serial.println(timeBuffer);
    Serial.print("Date: ");
    Serial.println(dateBuffer);
  } else {
    Serial.println("RTC not synced yet...");
  }
}

// =========================
// Loop
// =========================
void loop() {
  if (Blynk.connected()) {
    Blynk.run();
  }
  timer.run();
}
