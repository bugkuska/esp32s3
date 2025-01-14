#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ModbusMaster.h>

// Wi-Fi and Blynk credentials
const char ssid[] = "แก้ชื่อ Wi-Fi";
const char pass[] = "รหัส Wi-Fi";
const char auth[] = "Auth token จาก App blynk legacy";

// Modbus settings
#define RXD 18   //RX ของบอร์ด esp32s3
#define TXD 17   //TX ของบอร์ด esp32s3
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
  Blynk.begin(auth, ssid, pass, "ip address blynk local-server", 8080);

  // Set timers
  timer.setInterval(5000L, checkConnections);
  timer.setInterval(2000L, debugModbus);
}

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
}

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

void debugModbus() {
  // Example: Read Modbus register 0x0000
  uint8_t result = node1.readHoldingRegisters(0x0000, 1);
  yield(); // Prevent watchdog timeout
  if (result == node1.ku8MBSuccess) {
    Serial.print("Modbus Data: ");
    Serial.println(node1.getResponseBuffer(0));
  } else {
    Serial.print("Modbus Error: ");
    Serial.println(result);
  }
}

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

// Blynk write handlers for buttons
BLYNK_WRITE(Widget_Btn_SW1) {
  int valueSW1 = param.asInt();
  if (valueSW1 == 1) {
    if (node1.writeSingleCoil(0x0000, 1) == node1.ku8MBSuccess) {
      Serial.println("SW1 ON success");
    } else {
      Serial.println("SW1 ON failed");
    }
  } else {
    if (node1.writeSingleCoil(0x0000, 0) == node1.ku8MBSuccess) {
      Serial.println("SW1 OFF success");
    } else {
      Serial.println("SW1 OFF failed");
    }
  }
}

BLYNK_WRITE(Widget_Btn_SW2) {
  int valueSW2 = param.asInt();
  if (valueSW2 == 1) {
    if (node1.writeSingleCoil(0x0001, 1) == node1.ku8MBSuccess) {
      Serial.println("SW2 ON success");
    } else {
      Serial.println("SW2 ON failed");
    }
  } else {
    if (node1.writeSingleCoil(0x0001, 0) == node1.ku8MBSuccess) {
      Serial.println("SW2 OFF success");
    } else {
      Serial.println("SW2 OFF failed");
    }
  }
}

BLYNK_WRITE(Widget_Btn_SW3) {
  int valueSW3 = param.asInt();
  if (valueSW3 == 1) {
    if (node1.writeSingleCoil(0x0002, 1) == node1.ku8MBSuccess) {
      Serial.println("SW3 ON success");
    } else {
      Serial.println("SW3 ON failed");
    }
  } else {
    if (node1.writeSingleCoil(0x0002, 0) == node1.ku8MBSuccess) {
      Serial.println("SW3 OFF success");
    } else {
      Serial.println("SW3 OFF failed");
    }
  }
}

BLYNK_WRITE(Widget_Btn_SW4) {
  int valueSW4 = param.asInt();
  if (valueSW4 == 1) {
    if (node1.writeSingleCoil(0x0003, 1) == node1.ku8MBSuccess) {
      Serial.println("SW4 ON success");
    } else {
      Serial.println("SW4 ON failed");
    }
  } else {
    if (node1.writeSingleCoil(0x0003, 0) == node1.ku8MBSuccess) {
      Serial.println("SW4 OFF success");
    } else {
      Serial.println("SW4 OFF failed");
    }
  }
}

BLYNK_WRITE(Widget_Btn_SW5) {
  int valueSW5 = param.asInt();
  if (valueSW5 == 1) {
    if (node1.writeSingleCoil(0x0004, 1) == node1.ku8MBSuccess) {
      Serial.println("SW5 ON success");
    } else {
      Serial.println("SW5 ON failed");
    }
  } else {
    if (node1.writeSingleCoil(0x0004, 0) == node1.ku8MBSuccess) {
      Serial.println("SW5 OFF success");
    } else {
      Serial.println("SW5 OFF failed");
    }
  }
}

BLYNK_WRITE(Widget_Btn_SW6) {
  int valueSW6 = param.asInt();
  if (valueSW6 == 1) {
    if (node1.writeSingleCoil(0x0005, 1) == node1.ku8MBSuccess) {
      Serial.println("SW6 ON success");
    } else {
      Serial.println("SW6 ON failed");
    }
  } else {
    if (node1.writeSingleCoil(0x0005, 0) == node1.ku8MBSuccess) {
      Serial.println("SW6 OFF success");
    } else {
      Serial.println("SW6 OFF failed");
    }
  }
}

BLYNK_WRITE(Widget_Btn_SW7) {
  int valueSW7 = param.asInt();
  if (valueSW7 == 1) {
    if (node1.writeSingleCoil(0x0006, 1) == node1.ku8MBSuccess) {
      Serial.println("SW7 ON success");
    } else {
      Serial.println("SW7 ON failed");
    }
  } else {
    if (node1.writeSingleCoil(0x0006, 0) == node1.ku8MBSuccess) {
      Serial.println("SW7 OFF success");
    } else {
      Serial.println("SW7 OFF failed");
    }
  }
}

BLYNK_WRITE(Widget_Btn_SW8) {
  int valueSW8 = param.asInt();
  if (valueSW8 == 1) {
    if (node1.writeSingleCoil(0x0007, 1) == node1.ku8MBSuccess) {
      Serial.println("SW8 ON success");
    } else {
      Serial.println("SW8 ON failed");
    }
  } else {
    if (node1.writeSingleCoil(0x0007, 0) == node1.ku8MBSuccess) {
      Serial.println("SW8 OFF success");
    } else {
      Serial.println("SW8 OFF failed");
    }
  }
}
