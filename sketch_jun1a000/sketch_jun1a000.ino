#include <LiquidCrystal.h>
#include <Adafruit_Fingerprint.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h> // لـ JSON serialization/deserialization
#include <time.h>        // لـ localtime و strftime لإنشاء Timestamp بتنسيق ISO 8601
#include <NTPClient.h>   // للحصول على الوقت الدقيق من NTP Server
#include <WiFiUdp.h>     // لـ NTPClient
//sfsf
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

// ** إضافة هذا السطر لتعريف كائن SPI مخصص لـ SD Card **
// ** ده بيحل مشكلة "cannot bind non-const lvalue reference" **
SPIClass spiSD(VSPI); // غالبًا VSPI هي الواجهة المستخدمة لكارت الـ SD على ESP32

// تهيئة وحدة RTC
RTC_DS3231 rtc;

// ** متغير جديد لتتبع حالة تهيئة SD Card **
bool sdCardReady = false; 

// هيكل بيانات لتخزين بيانات البصمة محليًا
struct FingerprintData {
  uint16_t id;
  String name;
  unsigned long timestamp_unix; // لتخزين Timestamp كتوقيت يونكس (محليًا)
};
// لتخزين بيانات البصمات (حتى 128 بصمة)
FingerprintData fingerprints[128];
uint16_t nextID_local = 0; // الـ ID التالي المتاح محليًا

// متغيرات حالة القائمة
enum MenuState {
  MAIN_MENU,
  SHOW_MENU_CHOICES
};
MenuState menuState = MAIN_MENU;

// متغيرات زر الضغط
unsigned long buttonPressStart = 0;
bool buttonHeld = false;
bool waitingForSecondPress = false;

// لإعداد وقت NTP
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 3 * 3600, 60000); // +3 ساعات (EEST)

// تعريفات الدوال
void setupWiFi();
void displayMainMenu();
bool addFingerprint();
void enrollWithRetry();
String generateFingerName(uint16_t id); // دالة لتوليد الاسم تلقائياً
void sendToServer(uint16_t id, unsigned long timestamp_unix); // تم تغيير الحمولة لتقبل Timestamp Unix
void scanFingerprint();
void showMenuChoices();
void printFingerprintsFromServer();
void clearServerData();
bool confirmPassword();
uint16_t findNextAvailableID_local(); // البحث عن الـ ID التالي المتاح محليًا في السنسور
uint16_t getNextAvailableIDFromServer(); // الحصول على الـ ID التالي من السيرفر
void syncSensorIDsWithServer();

// دالة لتسجيل حدث الدخول/الخروج إلى بطاقة SD
void logEventToSD(uint16_t id, const String& eventType, const String& timestamp);

// متغير للمزامنة الأولية
bool syncedOnce = false;


void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);
  lcd.print("Initializing...");
  pinMode(addButtonPin, INPUT_PULLUP); 

  // تهيئة اتصال حساس البصمة
  mySerial.begin(57600, SERIAL_8N1, 16, 17);
  finger.begin(57600);

  // التحقق من حساس البصمة
  if (!finger.verifyPassword()) {
    lcd.setCursor(0, 1);
    lcd.print("Sensor not found!");
    Serial.println("Fingerprint sensor not found or password incorrect!");
    while (1) delay(1); // توقف البرنامج
  }

  // تهيئة SD Card
  Serial.print("Initializing SD card...");
  // ** تهيئة SPI بكائن spiSD قبل استخدام SD.begin **
  // ** ده بيحدد أي واجهة SPI يستخدمها كارت الـ SD **
  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS); // SCK, MISO, MOSI, CS

  // ** تعديل استدعاء SD.begin() ليمرر كائن SPI المخصص **
  if (!SD.begin(SD_CS, spiSD)) { // هنا بنمرر spiSD كواجهة SPI
    Serial.println("SD Card initialization failed!");
    lcd.setCursor(0, 1);
    lcd.print("SD Init Failed!");
    sdCardReady = false; // تحديث المتغير هنا
  } else {
    Serial.println("SD Card initialized.");
    lcd.setCursor(0, 1);
    lcd.print("SD OK!");
    sdCardReady = true; // تحديث المتغير هنا
  }
  delay(1000);

  // تهيئة RTC Module
  Wire.begin(); // بدء اتصال I2C
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
    // ضبط وقت RTC من NTP إذا كان الوقت غير صالح (أو إذا كانت البطارية فارغة)
    if (rtc.lostPower()) {
      Serial.println("RTC lost power, setting the time...");
      lcd.print("RTC lost power!");
      // سيتم ضبط الوقت من NTP بعد الاتصال بالواي فاي
    }
  }
  delay(1000);


  // الاتصال بالواي فاي
  lcd.clear();
  setupWiFi();

  // تهيئة NTP للحصول على الوقت الدقيق
  timeClient.begin();
  timeClient.update();
  Serial.print("Current UTC Time from NTP: ");
  Serial.println(timeClient.getFormattedTime());
  // ضبط الوقت المحلي للنظام ليتم استخدامه بواسطة ctime functions
  // +3 ساعات للمنطقة الزمنية (EEST)
  configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");

  // إذا كان الـ RTC غير مضبوط أو فقد الطاقة، اضبطه من NTP
  if (rtc.lostPower()) {
      Serial.println("Setting RTC time from NTP...");
      rtc.adjust(DateTime(timeClient.getEpochTime()));
      Serial.println("RTC time set.");
  }


  // الحصول على الـ ID التالي من السيرفر
  nextID_local = getNextAvailableIDFromServer(); // نستخدم هذا ليكون ID قاعدة البيانات والخلاقة

  displayMainMenu(); // عرض القائمة الرئيسية
}

void loop() {
  static bool menuShown = false;
  int buttonState = digitalRead(addButtonPin);

  // تحديث الوقت من NTP بانتظام
  timeClient.update();

  // منطق مزامنة المستشعر مع السيرفر (خاصة حذف البصمات)
  if (WiFi.status() == WL_CONNECTED && !syncedOnce) {
    Serial.println("🔁 Syncing Sensor to Server (only IDs)...");
    syncSensorIDsWithServer();
    syncedOnce = true;
  } else if (WiFi.status() != WL_CONNECTED) {
    syncedOnce = false; // لما النت يفصل، نرجّع السماح بالمزامنة
  }

  if (buttonState == LOW) { // عند الضغط على الزر
    if (!buttonHeld) {
      buttonPressStart = millis();
      buttonHeld = true;
    } else {
      unsigned long heldTime = millis() - buttonPressStart;

      // الضغط المطول لعرض قائمة الخيارات
      if (!menuShown && heldTime > 5000 && menuState == MAIN_MENU) { 
        showMenuChoices();
        menuState = SHOW_MENU_CHOICES;
        menuShown = true;
        delay(500); // تأخير لتجنب تكرار الدخول
      }

      // الضغط المطول جداً لمسح كل شيء
      if (menuState == SHOW_MENU_CHOICES && heldTime > 25000) {
        if (confirmPassword()) {
          clearServerData();      // مسح بيانات السيرفر
          finger.emptyDatabase(); // مسح بيانات المستشعر
          // مسح البيانات المخزنة محلياً
          for (int i = 0; i < 128; i++) {
            fingerprints[i].id = 0;
            fingerprints[i].name = "";
            fingerprints[i].timestamp_unix = 0;
          }
          nextID_local = getNextAvailableIDFromServer(); // جلب ID جديد بعد المسح
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
  } else { // عند رفع الزر
    if (buttonHeld) {
      unsigned long heldTime = millis() - buttonPressStart;

      // ضغطة قصيرة لإضافة بصمة (عندما نكون في القائمة الرئيسية)
      if (menuState == MAIN_MENU && heldTime < 5000) {
        enrollWithRetry(); // محاولة تسجيل بصمة جديدة
        displayMainMenu();
      } 
      // ضغطة قصيرة لعرض البيانات من السيرفر (عندما نكون في قائمة الخيارات)
      else if (menuState == SHOW_MENU_CHOICES && heldTime < 5000) { 
        printFingerprintsFromServer();
        displayMainMenu();
        menuState = MAIN_MENU;
        menuShown = false;
      }

      buttonHeld = false; // إعادة ضبط حالة الزر
    }
  }

  scanFingerprint(); // المسح المستمر للبصمات الموجودة
  delay(100);
}

// *******************************************************************
// **************************** وظائف الواي فاي ********************
// *******************************************************************
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
      lcd.print("                "); // مسح النقاط
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

// *******************************************************************
// **************************** وظائف الشاشة ***********************
// *******************************************************************
void displayMainMenu() {
  lcd.clear();
  lcd.print("Add: Press Button");
  lcd.setCursor(0, 1);
  lcd.print("Scan: Place Finger");
}

// *******************************************************************
// **************************** وظائف تسجيل البصمة *****************
// *******************************************************************
bool addFingerprint() {
  int p = -1; // متغير لتخزين نتيجة وظائف المستشعر

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

  // الخطوة الثالثة لتحسين الجودة
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
  p = finger.image2Tz(1); // استخدام المخزن 1 مرة أخرى لتحديث القالب
  if (p != FINGERPRINT_OK) { lcd.print("Final Img fail"); Serial.print("Image to Template final failed: "); Serial.println(p); delay(2000); return false; }
  Serial.println("Final image converted.");

  // ** الجزء الأهم: استخدام الـ ID من السيرفر **
  uint16_t id_from_server = getNextAvailableIDFromServer();
  if (id_from_server == 65535) { // 65535 عادة ما تكون قيمة غير صالحة لـ uint16_t أو عندما يفشل الاستعلام
      lcd.clear();
      lcd.print("Failed to get ID");
      lcd.setCursor(0, 1);
      lcd.print("from Server.");
      Serial.println("Failed to get a valid ID from the server. Aborting enrollment.");
      delay(3000);
      return false;
  }
  // تحديث nextID_local بالـ ID الجديد الذي حصلنا عليه
  nextID_local = id_from_server;

  // تخزين النموذج النهائي في قاعدة بيانات المستشعر بالـ ID المتاح
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

  // ** توليد الاسم محليًا وتخزينه وعرضه **
  String generatedName = generateFingerName(nextID_local);

  // الحصول على الوقت من RTC أولاً، ثم NTP إذا كان RTC غير متاح أو غير مضبوط
  DateTime now;
  if (rtc.lostPower()) {
      // RTC غير مضبوط أو فقد الطاقة، استخدم NTP
      Serial.println("RTC not running or lost power, using NTP for timestamp.");
      timeClient.update(); // تأكد من تحديث NTP
      now = DateTime(timeClient.getEpochTime());
  } else {
      // RTC يعمل بشكل صحيح
      now = rtc.now();
      Serial.println("Using RTC for timestamp.");
  }
  unsigned long currentUnixTime = now.unixtime();

  // تحويل وقت Unix إلى سلسلة ISO 8601
  char isoTime[25];
  sprintf(isoTime, "%04d-%02d-%02dT%02d:%02d:%02dZ",
          now.year(), now.month(), now.day(),
          now.hour(), now.minute(), now.second());

  // تخزين البيانات محليًا (في الذاكرة العشوائية RAM)
  fingerprints[nextID_local].id = nextID_local;
  fingerprints[nextID_local].name = generatedName;
  fingerprints[nextID_local].timestamp_unix = currentUnixTime;

  // إرسال البيانات (ID و Timestamp) إلى السيرفر
  sendToServer(nextID_local, currentUnixTime);

  // تسجيل الحدث في SD Card
  logEventToSD(nextID_local, "Enrollment", isoTime);


  lcd.clear();
  lcd.print("Added ID: " + String(nextID_local));
  lcd.setCursor(0, 1);
  lcd.print("Name: " + generatedName); // عرض الاسم الذي تم توليده
  Serial.println("Added fingerprint ID: " + String(nextID_local) + ", Name: " + generatedName + ", Timestamp: " + String(isoTime));
  delay(3000);

  // تحديث nextID_local للحصول على الـ ID التالي للمرة القادمة
  nextID_local = getNextAvailableIDFromServer();  
  return true;
}

void enrollWithRetry() {
  lcd.clear();
  lcd.print("Place Finger");
  delay(1000);
  for (int i = 1; i <= 3; i++) { // محاولات تسجيل البصمة
    lcd.clear();
    lcd.print("Attempt " + String(i));
    if (addFingerprint()) {
      lcd.clear();
      lcd.print("Success!");
      delay(2000);
      return;
    }
    delay(1500);
  }
  lcd.clear();
  lcd.print("Failed to add!");
  delay(2000);
}


// دالة توليد الاسم تلقائيا
String generateFingerName(uint16_t id) {
  return "User_" + String(id);
}

// *******************************************************************
// **************************** وظائف API ***************************
// *******************************************************************

// إرسال الـ ID و Timestamp إلى السيرفر
void sendToServer(uint16_t id, unsigned long timestamp_unix) {
  if (WiFi.status() != WL_CONNECTED) {
    lcd.clear();
    lcd.print("WiFi Not Conn");
    Serial.println("WiFi not connected, cannot send to server.");
    return;
  }

  HTTPClient http;
  String url = String("https://") + API_HOST + ":" + API_PORT + API_PATH; 

  // شهادات SSL/TLS (مهمة للـ HTTPS)
  // في بيئة التطوير، قد تحتاج لتعطيل التحقق إذا كان السيرفر يستخدم شهادة ذاتية التوقيع
  // http.setInsecure(); // تم حذف هذا السطر
  
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  // تحويل توقيت يونكس إلى سلسلة ISO 8601
  char isoTime[25]; 
  struct tm * ti;
  time_t t_unix = timestamp_unix;
  ti = gmtime(&t_unix); 
  strftime(isoTime, sizeof(isoTime), "%Y-%m-%dT%H:%M:%SZ", ti);

  DynamicJsonDocument doc(256);
  doc["id"] = id;
  doc["timestamp"] = isoTime; 

  String jsonStr;
  serializeJson(doc, jsonStr);

  int httpCode = http.POST(jsonStr);

  if (httpCode == 200 || httpCode == 201) {
    lcd.clear();
    lcd.print("Sent to Server");
    Serial.println("Data sent: " + jsonStr);
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

// الحصول على الـ ID التالي من السيرفر باستخدام api/SensorData/generate-id
uint16_t getNextAvailableIDFromServer() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, cannot get ID from server.");
    // Fallback: ابحث عن ID متاح محليًا إذا لم يكن هناك اتصال بالإنترنت
    return findNextAvailableID_local();
  }

  HTTPClient http;
  String url = String("https://") + API_HOST + ":" + API_PORT + API_PATH + "/generate-id";
  
  // http.setInsecure(); // تم حذف هذا السطر
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
    // Fallback: ابحث عن ID متاح محليًا
    return findNextAvailableID_local();
  }
}

// عرض جميع البيانات من السيرفر
void printFingerprintsFromServer() {
  if (WiFi.status() != WL_CONNECTED) {
    lcd.clear();
    lcd.print("WiFi Not Conn");
    delay(1500);
    return;
  }

  HTTPClient http;
  String url = String("https://") + API_HOST + ":" + API_PORT + API_PATH;
  // http.setInsecure(); // تم حذف هذا السطر
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

// مسح جميع البيانات من السيرفر
void clearServerData() {
  if (WiFi.status() != WL_CONNECTED) {
    lcd.clear();
    lcd.print("WiFi Not Conn");
    delay(1500);
    return;
  }
  HTTPClient http;
  String url = String("https://") + API_HOST + ":" + API_PORT + API_PATH + "/clear";
  // http.setInsecure(); // تم حذف هذا السطر
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

// *******************************************************************
// **************************** وظائف المستشعر *********************
// *******************************************************************
void scanFingerprint() {
  int p = finger.getImage();
  if (p == FINGERPRINT_OK) {
    if (finger.image2Tz() != FINGERPRINT_OK) return;
    p = finger.fingerFastSearch();
    if (p == FINGERPRINT_OK) {
      uint16_t id = finger.fingerID;
      // البحث عن الاسم المخزن محلياً
      String foundName = "Unknown";
      if (id < 128 && fingerprints[id].id == id) {
        foundName = fingerprints[id].name;
      }

      // الحصول على الوقت من RTC أولاً، ثم NTP إذا كان RTC غير متاح أو غير مضبوط
      DateTime now;
      if (rtc.lostPower()) {
          Serial.println("RTC not running or lost power, using NTP for timestamp.");
          timeClient.update(); // تأكد من تحديث NTP
          now = DateTime(timeClient.getEpochTime());
      } else {
          now = rtc.now();
          Serial.println("Using RTC for timestamp.");
      }
      unsigned long currentUnixTime = now.unixtime();

      // تحويل وقت Unix إلى سلسلة ISO 8601 لتسجيله في SD Card
      char isoTime[25];
      sprintf(isoTime, "%04d-%02d-%02dT%02d:%02d:%02dZ",
              now.year(), now.month(), now.day(),
              now.hour(), now.minute(), now.second());

      // تسجيل حدث الدخول/الخروج إلى بطاقة SD
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

// عرض خيارات القائمة
void showMenuChoices() {
  lcd.clear();
  lcd.print("1.Show All Data");
  lcd.setCursor(0, 1);
  lcd.print("2.Clear All:Hold"); 
}

// *******************************************************************
// **************************** وظائف مساعدة ***********************
// *******************************************************************

// التحقق من كلمة المرور
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

// البحث عن الـ ID المتاح التالي في السنسور (محليًا)
// يستخدم كـ fallback إذا لم نتمكن من الاتصال بالسيرفر
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

// مزامنة ID's السنسور مع السيرفر
void syncSensorIDsWithServer() {
  bool serverIDs[128] = { false };
  bool sensorIDs[128] = { false };

  lcd.clear();
  lcd.print("Syncing IDs...");

  // 1. Get IDs from server (using GetAllData API)
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = String("https://") + API_HOST + ":" + API_PORT + API_PATH;
    // http.setInsecure(); // تم حذف هذا السطر
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

  // 2. Get IDs from fingerprint sensor
  for (uint16_t id = 0; id < 128; id++) {
    if (finger.loadModel(id) == FINGERPRINT_OK) {
      sensorIDs[id] = true;
    }
  }

  // 3. Perform synchronization logic
  for (uint16_t id = 0; id < 128; id++) {
    if (sensorIDs[id] && !serverIDs[id]) {
      // Fingerprint on sensor but not on server — remove it from sensor
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
    // إذا كان ID في السيرفر ولكن ليس في السنسور، لا يمكن لجهاز ESP32 إضافته
    // بل يجب إضافة البصمة يدويًا إلى السنسور باستخدام نفس الـ ID.
    else if (!sensorIDs[id] && serverIDs[id]) {
      Serial.printf("⚠️ ID %d exists on server but not in sensor. Manual re-enrollment needed.\n", id);
    }
  }
  lcd.clear();
  lcd.print("Sync Done!");
  delay(1500);
}

// دالة لتسجيل حدث الدخول/الخروج إلى بطاقة SD
void logEventToSD(uint16_t id, const String& eventType, const String& timestamp) {
  // ****** التعديل هنا ******
  if (!sdCardReady) { 
    Serial.println("SD card not initialized. Cannot log event.");
    return;
  }

  File dataFile = SD.open("/log.txt", FILE_APPEND); // افتح الملف في وضع الإلحاق

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