#include <LiquidCrystal.h>
#include <Adafruit_Fingerprint.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h> // لـ JSON serialization/deserialization
#include <time.h>        // لـ localtime و strftime لإنشاء Timestamp بتنسيق ISO 8601
#include <NTPClient.h>   // للحصول على الوقت الدقيق من NTP Server
#include <WiFiUdp.h>     // لـ NTPClient
#include <Base64.h>      // لإضافة تشفير Base64 (تأكد إنها مثبتة برضو)

// مكتبات Micro SD Card
#include <SPI.h> // مكتبة SPI للتواصل مع SD Card
#include <SD.h>  // مكتبة SD Card

// مكتبات RTC Module (DS3231)
#include <Wire.h> // مكتبة I2C للتواصل مع RTC
#include <RTClib.h> // مكتبة RTC

// تعريفات WiFi
#define WIFI_SSID ".."
#define WIFI_PASSWORD "ASAnos382023"

// معلومات السيرفر
const char* API_HOST = "192.168.1.6";
const int API_PORT = 7069;
const char* API_PATH = "/api/SensorData";

// ---
// ## إعدادات شاشة LCD - تم تحديث المداخل
// ---
const int rs = 27, en = 26, d4 = 25, d5 = 33, d6 = 32, d7 = 14;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// إعدادات حساس البصمة
HardwareSerial mySerial(2); // استخدم UART2 GPIO16 (RX), GPIO17 (TX)
Adafruit_Fingerprint finger(&mySerial);

// ---
// ## تعريف زر الإضافة - تم التأكد من المدخل
// ---
const int addButtonPin = 34;

// ---
// ## تعريفات مسامير SD Card - تم تحديث المداخل
// ---
#define SD_CS   5  // Chip Select (CS) for SD Card
#define SD_MOSI 23  // Master Out Slave In (MOSI)
#define SD_MISO 19  // Master In Slave Out (MISO)
#define SD_SCK  18  // Serial Clock (SCK)

SPIClass spiSD(VSPI);

RTC_DS3231 rtc;

bool sdCardReady = false;

struct FingerprintData {
  uint16_t id;
  String name;
  unsigned long timestamp_unix;
};

FingerprintData fingerprints[128];
uint16_t nextID_local = 0;

enum MenuState {
  MAIN_MENU,
  SHOW_MENU_CHOICES
};
MenuState menuState = MAIN_MENU;

unsigned long buttonPressStart = 0;
bool buttonHeld = false;
bool waitingForSecondPress = false;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 3 * 3600, 60000);

void setupWiFi();
void displayMainMenu();
bool addFingerprint();
void enrollWithRetry();
String generateFingerName(uint16_t id);
// ** تم إعادة توقيع الدالة لاستقبال قالب البصمة **
void sendToServer(uint16_t id, const byte* fingerprintModel, unsigned long timestamp_unix);
void scanFingerprint();
void showMenuChoices();
void printFingerprintsFromServer();
void clearServerData();
bool confirmPassword();
uint16_t findNextAvailableID_local();
uint16_t getNextAvailableIDFromServer();
void syncSensorIDsWithServer();
void logEventToSD(uint16_t id, const String& eventType, const String& timestamp);

bool syncedOnce = false;

void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);
  lcd.print("Initializing...");
  pinMode(addButtonPin, INPUT_PULLUP);

  mySerial.begin(57600, SERIAL_8N1, 16, 17);
  finger.begin(57600);

  if (!finger.verifyPassword()) {
    lcd.setCursor(0, 1);
    lcd.print("Sensor not found!");
    Serial.println("Fingerprint sensor not found or password incorrect!");
    while (1) delay(1);
  }

  Serial.print("Initializing SD card...");
  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS, spiSD)) {
    Serial.println("SD Card initialization failed!");
    lcd.setCursor(0, 1);
    lcd.print("SD Init Failed!");
    sdCardReady = false;
  } else {
    Serial.println("SD Card initialized.");
    lcd.setCursor(0, 1);
    lcd.print("SD OK!");
    sdCardReady = true;
  }
  delay(1000);

  Wire.begin();
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    lcd.clear();
    lcd.print("RTC Not Found!");
    lcd.setCursor(0,1);
    lcd.print("Using NTP only");
    delay(2000);
  } else {
    Serial.println("RTC found and initialized.");
    lcd.clear();
    lcd.print("RTC Ready!");
    lcd.setCursor(0,1);
    if (rtc.lostPower()) {
      Serial.println("RTC lost power, setting the time...");
    }
  }
  delay(1000);

  lcd.clear();
  setupWiFi();

  timeClient.begin();
  timeClient.update();
  Serial.print("Current UTC Time from NTP: ");
  Serial.println(timeClient.getFormattedTime());
  configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");

  if (rtc.lostPower()) {
      Serial.println("Setting RTC time from NTP...");
      rtc.adjust(DateTime(timeClient.getEpochTime()));
      Serial.println("RTC time set.");
  }

  nextID_local = getNextAvailableIDFromServer();
  displayMainMenu();
}

void loop() {
  static bool menuShown = false;
  int buttonState = digitalRead(addButtonPin);

  timeClient.update();

  if (WiFi.status() == WL_CONNECTED && !syncedOnce) {
    Serial.println("🔁 Syncing Sensor to Server (only IDs)...");
    syncSensorIDsWithServer();
    syncedOnce = true;
  } else if (WiFi.status() != WL_CONNECTED) {
    syncedOnce = false;
  }

  if (buttonState == LOW) {
    if (!buttonHeld) {
      buttonPressStart = millis();
      buttonHeld = true;
    } else {
      unsigned long heldTime = millis() - buttonPressStart;

      if (!menuShown && heldTime > 5000 && menuState == MAIN_MENU) {
        showMenuChoices();
        menuState = SHOW_MENU_CHOICES;
        menuShown = true;
        delay(500);
      }

      if (menuState == SHOW_MENU_CHOICES && heldTime > 25000) {
        if (confirmPassword()) {
          clearServerData();
          finger.emptyDatabase();
          for (int i = 0; i < 128; i++) {
            fingerprints[i].id = 0;
            fingerprints[i].name = "";
            fingerprints[i].timestamp_unix = 0;
          }
          nextID_local = getNextAvailableIDFromServer();
          lcd.clear();
          lcd.print("All Deleted");
        } else {
          lcd.clear();
          lcd.print("Wrong Password");
        }
        delay(2000);
        displayMainMenu();
        menuState = MAIN_MENU;
        menuShown = false;
        buttonHeld = false;
      }
    }
  } else {
    if (buttonHeld) {
      unsigned long heldTime = millis() - buttonPressStart;

      if (menuState == MAIN_MENU && heldTime < 5000) {
        enrollWithRetry();
        displayMainMenu();
      }
      else if (menuState == SHOW_MENU_CHOICES && heldTime < 5000) {
        printFingerprintsFromServer();
        displayMainMenu();
        menuState = MAIN_MENU;
        menuShown = false;
      }
      buttonHeld = false;
    }
  }
  scanFingerprint();
  delay(100);
}

void setupWiFi() {
  lcd.clear();
  lcd.print("Connecting WiFi");
  lcd.setCursor(0, 1);
  int dots = 0;
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    lcd.print(".");
    delay(300);
    dots++;
    if (dots > 14) {
      lcd.setCursor(0, 1);
      lcd.print("                ");
      lcd.setCursor(0, 1);
      dots = 0;
    }
  }
  lcd.clear();
  lcd.print("WiFi Connected");
  delay(1000);
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void displayMainMenu() {
  lcd.clear();
  lcd.print("Add: Press Button");
  lcd.setCursor(0, 1);
  lcd.print("Scan: Place Finger");
}

bool addFingerprint() {
  int p = -1;

  lcd.clear();
  lcd.print("Place Finger (1/3)");
  Serial.println("Place finger on sensor for first view (1/3)...");

  p = -1;
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    if (p == FINGERPRINT_NOFINGER) delay(50);
    else if (p == FINGERPRINT_OK) Serial.println("Image taken.");
    else { lcd.print("Error! Retrying..."); Serial.print("Error: "); Serial.println(p); delay(1000); }
  }

  p = finger.image2Tz(1);
  if (p != FINGERPRINT_OK) { lcd.print("Img 1 failed!"); Serial.print("Img to Template 1 failed: "); Serial.println(p); delay(2000); return false; }
  Serial.println("Image 1 converted.");

  lcd.clear();
  lcd.print("Lift Finger");
  lcd.setCursor(0, 1);
  lcd.print("Then Reposition");
  Serial.println("Lift your finger, then reposition for second view...");
  delay(1000);
  while (finger.getImage() != FINGERPRINT_NOFINGER) delay(50);
  delay(500);

  lcd.clear();
  lcd.print("Place Finger (2/3)");
  Serial.println("Place finger on sensor for second view (2/3)...");

  p = -1;
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    if (p == FINGERPRINT_NOFINGER) delay(50);
    else if (p == FINGERPRINT_OK) Serial.println("Image taken.");
    else { lcd.print("Error! Retrying..."); Serial.print("Error: "); Serial.println(p); delay(1000); }
  }

  p = finger.image2Tz(2);
  if (p != FINGERPRINT_OK) { lcd.print("Img 2 failed!"); Serial.print("Img to Template 2 failed: "); Serial.println(p); delay(2000); return false; }
  Serial.println("Image 2 converted.");

  p = finger.createModel();
  if (p == FINGERPRINT_OK) { Serial.println("Fingerprint models matched and merged."); }
  else if (p == FINGERPRINT_PACKETRECIEVEERR) { lcd.print("No Match! Try again"); Serial.println("Could not create model: no match."); delay(2000); return false; }
  else { lcd.print("Model creation fail"); Serial.print("Create model failed: "); Serial.println(p); delay(2000); return false; }

  lcd.clear();
  lcd.print("Lift Finger (Final)");
  lcd.setCursor(0,1);
  lcd.print("Then Reposition");
  Serial.println("Lift finger for final view (3/3)...");
  delay(1000);
  while (finger.getImage() != FINGERPRINT_NOFINGER) delay(50);
  delay(500);

  lcd.clear();
  lcd.print("Place Finger (3/3)");
  Serial.println("Place finger on sensor for final view (3/3)...");

  p = -1;
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    if (p == FINGERPRINT_NOFINGER) delay(50);
    else if (p == FINGERPRINT_OK) Serial.println("Image taken.");
    else { lcd.print("Error! Retrying..."); Serial.print("Error: "); Serial.println(p); delay(1000); }
  }
  p = finger.image2Tz(1);
  if (p != FINGERPRINT_OK) { lcd.print("Final Img fail"); Serial.print("Image to Template final failed: "); Serial.println(p); delay(2000); return false; }
  Serial.println("Final image converted.");

  // ** هنا الجزء الذي يسحب قالب البصمة باستخدام getTemplate() **
  byte fingerprintTemplate[256];
 // p = finger.getTemplate(1, fingerprintTemplate); // سحب القالب من Buffer1

  Adafruit_Fingerprint finger;
  p = finger.getModel(); // Transfers data to host
  // Now access raw template:

  if (p == FINGERPRINT_OK) {
      Serial.println("Fingerprint template read from sensor buffer.");
      for (int i=0; i<512; i++) {
        fingerprintTemplate[i] = finger.templateBuffer[i];
      }
  } else {
      Serial.print("Failed to read template from sensor buffer. Error: ");
      Serial.println(p);
      lcd.clear();
      lcd.print("Failed to get FP!");
      delay(2000);
      return false;
  }


  uint16_t id_from_server = getNextAvailableIDFromServer();
  if (id_from_server == 65535) {
      lcd.clear();
      lcd.print("Failed to get ID");
      lcd.setCursor(0, 1);
      lcd.print("from Server.");
      Serial.println("Failed to get a valid ID from the server. Aborting enrollment.");
      delay(3000);
      return false;
  }
  nextID_local = id_from_server;

  p = finger.storeModel(nextID_local);
  if (p == FINGERPRINT_OK) {
    Serial.println("Fingerprint stored successfully on sensor.");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    lcd.clear();
    lcd.print("Store Error!");
    Serial.println("Store model failed: Packet recieve error (sensor memory full or bad ID).");
    delay(2000);
    return false;
  } else {
    lcd.clear();
    lcd.print("Store Failed!");
    Serial.print("Store model failed: "); Serial.println(p);
    delay(2000);
    return false;
  }

  String generatedName = generateFingerName(nextID_local);

  DateTime now;
  if (rtc.lostPower()) {
      Serial.println("RTC not running or lost power, using NTP for timestamp.");
      timeClient.update();
      now = DateTime(timeClient.getEpochTime());
  } else {
      now = rtc.now();
      Serial.println("Using RTC for timestamp.");
  }
  unsigned long currentUnixTime = now.unixtime();

  char isoTime[25];
  sprintf(isoTime, "%04d-%02d-%02dT%02d:%02d:%02dZ",
          now.year(), now.month(), now.day(),
          now.hour(), now.minute(), now.second());

  fingerprints[nextID_local].id = nextID_local;
  fingerprints[nextID_local].name = generatedName;
  fingerprints[nextID_local].timestamp_unix = currentUnixTime;

  // ** استدعاء الدالة sendToServer مع الـ fingerprintTemplate الذي تم سحبه **
  sendToServer(nextID_local, fingerprintTemplate, currentUnixTime);

  logEventToSD(nextID_local, "Enrollment", isoTime);

  lcd.clear();
  lcd.print("Added ID: " + String(nextID_local));
  lcd.setCursor(0, 1);
  lcd.print("Name: " + generatedName);
  Serial.println("Added fingerprint ID: " + String(nextID_local) + ", Name: " + generatedName + ", Timestamp: " + String(isoTime));
  delay(3000);

  nextID_local = getNextAvailableIDFromServer();
  return true;
}

// ** وظيفة sendToServer معدلة لاستقبال قالب البصمة **
void sendToServer(uint16_t id, const byte* fingerprintModel, unsigned long timestamp_unix) {
  if (WiFi.status() != WL_CONNECTED) {
    lcd.clear();
    lcd.print("WiFi Not Conn");
    Serial.println("WiFi not connected, cannot send to server.");
    return;
  }

  HTTPClient http;
  String url = String("https://") + API_HOST + ":" + API_PORT + API_PATH;

  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  char isoTime[25];
  struct tm * ti;
  time_t t_unix = timestamp_unix;
  ti = gmtime(&t_unix);
  strftime(isoTime, sizeof(isoTime), "%Y-%m-%dT%H:%M:%SZ", ti);

  // ** تشفير Fingerprint Model إلى Base64 **
  // قالب البصمة حجمه 256 بايت، تشفيره Base64 يجعله أكبر (حوالي 344 بايت بعد التشفير)
  size_t encodedLen = Base64.encodedLength(256);
  char encodedModel[encodedLen + 1]; // +1 للنهاية الصفرية
  Base64.encode(encodedModel, (char*)fingerprintModel, 256);
  String encodedModelString = String(encodedModel);

  DynamicJsonDocument doc(512); // زودنا الحجم عشان قالب البصمة المشفر
  doc["id"] = id;
  doc["fingerprintModel"] = encodedModelString; // إضافة القالب المشفر
  doc["timestamp"] = isoTime;

  String jsonStr;
  serializeJson(doc, jsonStr);

  Serial.println("Sending JSON: " + jsonStr);

  int httpCode = http.POST(jsonStr);

  if (httpCode == 200 || httpCode == 201) {
    lcd.clear();
    lcd.print("Sent to Server");
    Serial.println("Data sent successfully.");
  } else if (httpCode == HTTP_CODE_CONFLICT) {
    lcd.clear();
    lcd.print("ID Exists (API)");
    Serial.println("Error: ID already exists on server (HTTP 409).");
  } else {
    lcd.clear();
    lcd.print("Send Fail");
    Serial.printf("Send error: %d - %s\n", httpCode, http.errorToString(httpCode).c_str());
  }

  http.end();
  delay(1000);
}


// ... (باقي الدوال بدون تغيير) ...

// دالة توليد الاسم تلقائيا
String generateFingerName(uint16_t id) {
  return "User_" + String(id);
}

uint16_t getNextAvailableIDFromServer() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, cannot get ID from server.");
    return findNextAvailableID_local();
  }

  HTTPClient http;
  String url = String("https://") + API_HOST + ":" + API_PORT + API_PATH + "/generate-id";

  http.begin(url);
  int httpCode = http.GET();

  if (httpCode == 200) {
    String payload = http.getString();
    http.end();
    uint16_t nextId = payload.toInt();
    Serial.println("Received next ID from server: " + String(nextId));
    return nextId;
  } else {
    Serial.printf("Failed to get next ID from server. HTTP code: %d\n", httpCode);
    Serial.println(http.errorToString(httpCode));
    http.end();
    return findNextAvailableID_local();
  }
}

void printFingerprintsFromServer() {
  if (WiFi.status() != WL_CONNECTED) {
    lcd.clear();
    lcd.print("WiFi Not Conn");
    delay(1500);
    return;
  }

  HTTPClient http;
  String url = String("https://") + API_HOST + ":" + API_PORT + API_PATH;
  http.begin(url);
  int httpCode = http.GET();

  if (httpCode == 200) {
    String payload = http.getString();
    Serial.println("Server Data:");
    Serial.println(payload);

    DynamicJsonDocument doc(4096);
    DeserializationError err = deserializeJson(doc, payload);
    if (!err) {
      lcd.clear();
      lcd.print("Data in Serial");
      lcd.setCursor(0, 1);
      lcd.print("Check Monitor");

      for (JsonObject item : doc.as<JsonArray>()) {
        int id = item["id"];
        const char* name = item["name"];
        const char* timestamp_str = item["timestamp"];

        Serial.printf("ID: %d, Name: %s, Timestamp: %s\n", id, name ? name : "N/A", timestamp_str ? timestamp_str : "N/A");
        delay(100);
      }
    } else {
      lcd.clear();
      lcd.print("JSON Error");
      Serial.println("JSON parsing failed: " + String(err.c_str()));
    }
  } else {
    lcd.clear();
    lcd.print("GET Error");
    Serial.printf("GET All Data error: %d - %s\n", httpCode, http.errorToString(httpCode).c_str());
  }
  http.end();
  delay(3000);
}

void clearServerData() {
  if (WiFi.status() != WL_CONNECTED) {
    lcd.clear();
    lcd.print("WiFi Not Conn");
    delay(1500);
    return;
  }
  HTTPClient http;
  String url = String("https://") + API_HOST + ":" + API_PORT + API_PATH + "/clear";
  http.begin(url);
  int httpCode = http.POST("");
  if (httpCode == 200) {
    lcd.clear();
    lcd.print("Data Cleared");
    Serial.println("Server data cleared");
  } else {
    lcd.clear();
    lcd.print("Clear Fail");
    Serial.printf("Clear error: %d - %s\n", httpCode, http.errorToString(httpCode).c_str());
  }
  http.end();
  delay(3000);
}

void scanFingerprint() {
  int p = finger.getImage();
  if (p == FINGERPRINT_OK) {
    if (finger.image2Tz() != FINGERPRINT_OK) return;
    p = finger.fingerFastSearch();
    if (p == FINGERPRINT_OK) {
      uint16_t id = finger.fingerID;
      String foundName = "Unknown";
      if (id < 128 && fingerprints[id].id == id) {
        foundName = fingerprints[id].name;
      }

      DateTime now;
      if (rtc.lostPower()) {
          Serial.println("RTC not running or lost power, using NTP for timestamp.");
          timeClient.update();
          now = DateTime(timeClient.getEpochTime());
      } else {
          now = rtc.now();
          Serial.println("Using RTC for timestamp.");
      }
      unsigned long currentUnixTime = now.unixtime();

      char isoTime[25];
      sprintf(isoTime, "%04d-%02d-%02dT%02d:%02d:%02dZ",
              now.year(), now.month(), now.day(),
              now.hour(), now.minute(), now.second());

      logEventToSD(id, "Access", isoTime);

      lcd.clear();
      lcd.print("ID: " + String(id));
      lcd.setCursor(0, 1);
      lcd.print("Name: " + foundName);
      Serial.println("Found ID: " + String(id) + ", Name: " + foundName + ", Timestamp: " + String(isoTime));
      delay(2000);
      displayMainMenu();
    } else {
      lcd.clear();
      lcd.print("Fingerprint Not");
      lcd.setCursor(5, 1);
      lcd.print("Found");
      Serial.println("Fingerprint Not Found!");
      delay(2000);
      displayMainMenu();
    }
  }
}

void showMenuChoices() {
  lcd.clear();
  lcd.print("1.Show All Data");
  lcd.setCursor(0, 1);
  lcd.print("2.Clear All:Hold");
}

bool confirmPassword() {
  lcd.clear();
  lcd.print("Enter Password:");
  lcd.setCursor(0, 1);
  lcd.print("Type in Serial");

  Serial.println("Enter admin password:");
  String input = "";
  unsigned long startTime = millis();

  while (millis() - startTime < 30000) {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        return input == "admin";
      } else {
        input += c;
      }
    }
    delay(10);
  }
  return false;
}

uint16_t findNextAvailableID_local() {
  Serial.println("Scanning fingerprint sensor for next available ID...");
  for (uint16_t id = 0; id < 128; id++) {
    if (finger.loadModel(id) != FINGERPRINT_OK) {
      Serial.print("Next local available ID is: ");
      Serial.println(id);
      return id;
    }
  }
  Serial.println("All local sensor IDs are full.");
  return 65535;
}

void syncSensorIDsWithServer() {
  bool serverIDs[128] = { false };
  bool sensorIDs[128] = { false };

  lcd.clear();
  lcd.print("Syncing IDs...");

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = String("https://") + API_HOST + ":" + API_PORT + API_PATH;
    http.begin(url);
    int httpCode = http.GET();
    if (httpCode == 200) {
      String payload = http.getString();
      DynamicJsonDocument doc(4096);
      DeserializationError err = deserializeJson(doc, payload);
      if (!err) {
        for (JsonObject item : doc.as<JsonArray>()) {
          int id = item["id"];
          if (id >= 0 && id < 128) {
            serverIDs[id] = true;
          }
        }
      } else {
        Serial.println("Failed to parse server JSON for sync.");
      }
    } else {
      Serial.printf("Failed to get server data for sync: %d\n", httpCode);
    }
    http.end();
  } else {
    Serial.println("WiFi not connected for sync with server.");
    return;
  }

  for (uint16_t id = 0; id < 128; id++) {
    if (finger.loadModel(id) == FINGERPRINT_OK) {
      sensorIDs[id] = true;
    }
  }

  for (uint16_t id = 0; id < 128; id++) {
    if (sensorIDs[id] && !serverIDs[id]) {
      Serial.printf("Deleting ID %d from sensor (not found on server)\n", id);
      lcd.setCursor(0, 1);
      lcd.print("Del. ID " + String(id) + " from Sens");
      if (finger.deleteModel(id) == FINGERPRINT_OK) {
        Serial.printf("✅ Deleted ID %d from Sensor\n", id);
      } else {
        Serial.printf("❌ Failed to delete ID %d from Sensor\n", id);
      }
      delay(500);
    }
    else if (!sensorIDs[id] && serverIDs[id]) {
      Serial.printf("⚠️ ID %d exists on server but not in sensor. Manual re-enrollment needed.\n", id);
    }
  }
  lcd.clear();
  lcd.print("Sync Done!");
  delay(1500);
}

void logEventToSD(uint16_t id, const String& eventType, const String& timestamp) {
  if (!sdCardReady) {
    Serial.println("SD card not initialized. Cannot log event.");
    return;
  }

  File dataFile = SD.open("/log.txt", FILE_APPEND);

  if (dataFile) {
    String logEntry = String(timestamp) + "," + String(id) + "," + eventType;
    dataFile.println(logEntry);
    dataFile.close();
    Serial.println("Logged to SD: " + logEntry);
    lcd.setCursor(0, 0);
    lcd.print("Logged to SD!");
    delay(500);
  } else {
    Serial.println("Error opening log.txt on SD card.");
    lcd.setCursor(0, 0);
    lcd.print("SD Write Fail!");
    delay(500);
  }
}