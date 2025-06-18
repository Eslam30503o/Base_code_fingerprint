

// --- LIBRARIES ---
#include <LiquidCrystal.h>        // Library for controlling LCD displays (e.g., 16x2 LCD)
#include <Adafruit_Fingerprint.h> // Library for interacting with Adafruit fingerprint sensors
#include <HardwareSerial.h>       // Standard Arduino library for serial communication (used for fingerprint sensor)
#include <WiFi.h>                 // Standard Arduino library for Wi-Fi connectivity
#include <WiFiManager.h>          // Library for captive portal Wi-Fi setup, simplifying initial connection without hardcoding credentials
#include <WiFiClientSecure.h>     // Library for secure client connections over SSL/TLS (HTTPS)
#include <HTTPClient.h>           // Library for making HTTP/HTTPS requests
#include <ArduinoJson.h>          // Library for parsing and generating JSON data (used for API communication)
#include <SPI.h>                  // Standard Arduino library for SPI communication (used by SD card module)
#include <SD.h>                   // Library for interacting with SD card modules (for offline logging)
#include <Wire.h>                 // Standard Arduino library for I2C communication (used by RTC module)
#include "RTClib.h"               // Library for Real-Time Clock (RTC) modules

// --- RTC OBJECT ---
// Use RTC_DS3231 for more accuracy, or RTC_DS1307 for the other common module.
RTC_DS3231 rtc; // Instantiates an RTC object using the DS3231 module

// --- SERVER AND TIME CONFIGURATION ---
// WIFI credentials are now handled by WiFiManager and are no longer hardcoded.
#define SERVER_HOST "https://192.168.1.12:7069" // Defines the base URL of the attendance server (HTTPS)
#define NTP_SERVER "pool.ntp.org"               // Defines the NTP server for time synchronization
#define GMT_OFFSET_SEC 3600 * 2                 // Defines the GMT offset for EET (Egypt Standard Time, UTC+2)
#define DAYLIGHT_OFFSET_SEC 0                   // Defines daylight saving offset (0 for no daylight saving)

// --- HARDWARE PIN DEFINITIONS ---
// LCD Pins
const int rs = 19, en = 23, d4 = 32, d5 = 33, d6 = 25, d7 = 26; // Defines pins for the Liquid Crystal Display
// Fingerprint Sensor RX/TX
const int FINGERPRINT_RX = 16;  // RX pin for the fingerprint sensor's serial communication
const int FINGERPRINT_TX = 17;  // TX pin for the fingerprint sensor's serial communication
// Button Pin
const int BUTTON_PIN = 18;      // Pin for the control button
// SD Card CS Pin
const int SD_CS_PIN = 5;        // Chip Select (CS) pin for the SD card module

// --- GLOBAL OBJECTS ---
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);                         // Creates an LCD object with specified pins
HardwareSerial fingerSerial(2);                                    // Creates a HardwareSerial object for the fingerprint sensor on UART2
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&fingerSerial); // Creates an Adafruit_Fingerprint object linked to the serial port
WiFiClientSecure client;                                           // Creates a WiFiClientSecure object for HTTPS connections

// --- STATE MANAGEMENT ---
enum class MenuState {
  MAIN_MENU,    // Represents the main menu state of the device
  OPTIONS_MENU  // Represents the options menu state of the device
};
MenuState currentMenuState = MenuState::MAIN_MENU; // Initial state set to main menu

// Structure to hold cached fingerprint data (ID and name)
struct FingerprintData {
  uint16_t id;    // Fingerprint ID
  String name;    // Associated name
};
FingerprintData fingerprintCache[128]; // Array to cache fingerprint data locally (sensor supports up to 128 templates)
const char* LOG_FILE = "/attendance_log.txt"; // Defines the filename for offline attendance logs on SD card

// --- FUNCTION PROTOTYPES ---
// These declarations allow functions to be called before their full definitions appear in the code.
void connectToWiFi();                                                  // Handles WiFi connection using WiFiManager (NEW: Replaces old hardcoded setupWiFi())
void displayMessage(String line1, String line2 = "", int delayMs = 0); // Displays messages on the LCD
void handleButton();                                                   // Manages button presses (short, long, very long)
void runMainMenuAction();                                              // Executes action when button is pressed in main menu
void runOptionsMenuAction();                                           // Executes action when button is pressed in options menu
void enrollNewFingerprint();                                           // Guides the user through enrolling a new fingerprint
int getFingerprintImage(int step);                                     // Captures a fingerprint image from the sensor
int createAndStoreModel(uint16_t id);                                  // Creates a fingerprint model and stores it on the sensor
String getFingerNameFromSerial(uint16_t id);                           // Prompts for and reads a name from Serial Monitor
void scanForFingerprint();                                             // Scans for a fingerprint and records attendance
void showOptionsMenu();                                                // Displays the options menu on the LCD
void syncAndDisplayServerData();                                       // Fetches and displays fingerprint data from the server, updates cache
void attemptToClearAllData();                                          // Initiates the process to clear all data (server & sensor)
bool confirmAdminPassword();                                           // Prompts for admin password confirmation via serial
void syncSensorWithServer();                                           // Synchronizes fingerprint templates between sensor and server (NEW: Enhanced logic)
bool sendFingerprintToServer(uint16_t id, const String& name);         // Sends new fingerprint data to the server
int getNextAvailableIDFromServer();                                    // Fetches the next available fingerprint ID from the server

// RTC and Offline Logging functions
void setupSDCard();                                        // Initializes the SD card
void setupRTC();                                           // Initializes the RTC module
void syncRtcWithNtp();                                     // Synchronizes RTC time with NTP server
void recordAttendance(uint16_t id);                        // Records attendance, attempting server log first, then offline
bool logAttendanceToServer(uint16_t id, time_t timestamp); // Sends attendance log to the server
void logAttendanceOffline(uint16_t id, time_t timestamp);  // Saves attendance log to SD card
void syncOfflineLogs();                                    // Attempts to upload offline logs from SD card to the server

// =================================================================================================
// SETUP FUNCTION
// =================================================================================================
void setup() {
  Serial.begin(115200);                                    // Initializes serial communication for debugging
  Wire.begin();                                            // Initializes I2C communication (for RTC)
  while (!Serial);                                         // Waits for serial port to connect (useful for debugging, especially on ESP32)

  lcd.begin(16, 2);                                        // Initializes the LCD with 16 columns and 2 rows
  displayMessage("System Starting", "Please wait...");     // Displays initial startup message

  // --- INITIALIZE HARDWARE ---
  setupSDCard();                                           // Initializes the SD card module
  pinMode(BUTTON_PIN, INPUT_PULLUP);                       // Sets the button pin as input with internal pull-up resistor

  // Initializes serial communication for the fingerprint sensor
  fingerSerial.begin(57600, SERIAL_8N1, FINGERPRINT_RX, FINGERPRINT_TX);
  if (finger.verifyPassword()) {                           // Attempts to connect to the fingerprint sensor
    Serial.println("Fingerprint sensor found!");           // Success message
  } else {
    displayMessage("Sensor Error", "Check wiring.", 10000); // Error message if sensor not found
    while (1) delay(1);                                     // Halts execution if sensor isn't found
  }
  
  // --- CONNECT TO WIFI USING WIFIMANAGER ---
  connectToWiFi(); // Attempts to connect to WiFi using WiFiManager (NEW: WiFiManager integration)

  // --- PERFORM ONLINE TASKS only if WiFi is connected ---
  if (WiFi.status() == WL_CONNECTED) { // Checks if WiFi successfully connected
    // We must initialize the RTC after connecting to WiFi to ensure it can be
    // synced with NTP on the first run if its power was lost.
    setupRTC(); // Initializes RTC (and syncs with NTP if needed and online)

    // IMPORTANT: Allow insecure HTTPS connections for local development server
    // (NEW: Explicitly noted for security awareness)
    client.setInsecure(); // Allows connections to servers with untrusted or self-signed SSL certificates.
                          // IMPORTANT: For production, this should be replaced with certificate pinning/validation for security.

    // Now, perform all server-related synchronization tasks
    syncSensorWithServer(); // Synchronizes fingerprint data with the server (NEW: Logic enhanced for orphaned sensor data)
    syncOfflineLogs();      // Uploads any unsynced attendance logs from SD card to server (NEW: Sync logic)
  } else {
    // If we are here, WiFiManager timed out or failed. We can't sync the clock via NTP immediately.
    // Initialize the RTC anyway; it will use its last known time.
    setupRTC(); // Initializes RTC even if offline (will use internal battery-backed time)
    Serial.println("\nCould not connect to WiFi. Starting in offline mode."); // Informs user of offline mode
    displayMessage("Offline Mode", "No WiFi", 2000); // Displays offline mode message
  }

  displayMessage("Add: Press Btn", "Scan: Place Finger"); // Initial prompt for user
}

// =================================================================================================
// MAIN LOOP
// =================================================================================================
void loop() {
  handleButton();       // Constantly checks for button presses and handles menu navigation/actions
  scanForFingerprint(); // Constantly scans for fingerprints and processes attendance

  static unsigned long lastSyncAttempt = 0; // Static variable to track last sync time
  // If WiFi connects later, this will start syncing logs.
  // (NEW: Periodic check for offline logs)
  if (WiFi.status() == WL_CONNECTED && (millis() - lastSyncAttempt > 300000)) { // Every 5 minutes (300000 ms)
      lastSyncAttempt = millis(); // Resets the timer
      Serial.println("Periodic check for offline logs to sync..."); // Debug message
      syncOfflineLogs(); // Initiates synchronization of offline logs
  }

  delay(50); // Short delay to prevent busy-waiting and allow other tasks to run
}

// =================================================================================================
// WIFI & DISPLAY FUNCTIONS
// =================================================================================================
/**
 * @brief Handles WiFi connection using the WiFiManager library.
 * (NEW: WiFiManager implementation)
 */
void connectToWiFi() {
    // Create a WiFiManager object
    WiFiManager wm; // Instantiates a WiFiManager object

    // Set a timeout for the configuration portal. If the user doesn't configure
    // WiFi within this time, the portal will close and the ESP will continue.
    wm.setConnectTimeout(180); // 3-minute timeout for the captive portal

    // Display a message on the LCD to inform the user.
    displayMessage("Connect to AP:", "FingerprintSetup"); // Instructions for connecting to the AP
    
    // Start the auto-connect process.
    // If the ESP32 has saved credentials, it will try to connect.
    // If not, it will start an Access Point with the name "FingerprintSetupAP".
    // The 'true' parameter means this will block until a connection is made or it times out.
    bool connected = wm.autoConnect("FingerprintSetupAP"); // Attempts to connect; if no saved creds, creates AP named "FingerprintSetupAP"

    if (!connected) {
        Serial.println("Failed to connect and hit timeout. Proceeding in offline mode."); // Debug message for timeout
        // The rest of the setup function will handle the offline state.
    } else {
        Serial.println("WiFi connected successfully!"); // Debug message for successful connection
        displayMessage("WiFi Connected", WiFi.localIP().toString(), 2000); // Displays connected status and IP
    }
}

// Displays messages on the LCD.
// line1: Text for the first line.
// line2: Text for the second line (optional, defaults to empty).
// delayMs: Duration to display the message (optional, defaults to 0 for no delay).
void displayMessage(String line1, String line2, int delayMs) {
  lcd.clear();        // Clears the LCD screen
  lcd.setCursor(0, 0);  // Sets cursor to the beginning of the first line
  lcd.print(line1);     // Prints the first line
  lcd.setCursor(0, 1);  // Sets cursor to the beginning of the second line
  lcd.print(line2);     // Prints the second line
  if (delayMs > 0) {    // If a delay is specified
    delay(delayMs);     // Waits for the specified duration
  }
}

// =================================================================================================
// BUTTON & MENU LOGIC (Unchanged from original logic, just commented)
// =================================================================================================
void handleButton() {
  static unsigned long buttonPressTime = 0; // Stores the time when the button was pressed down
  static bool buttonWasPressed = false;    // Flag to track if button was previously pressed

  bool isPressed = (digitalRead(BUTTON_PIN) == LOW); // Reads button state (LOW when pressed due to INPUT_PULLUP)

  if (isPressed && !buttonWasPressed) { // Button just pressed
    buttonPressTime = millis();     // Record the start time of the press
    buttonWasPressed = true;        // Set flag
  } else if (!isPressed && buttonWasPressed) { // Button just released
    unsigned long pressDuration = millis() - buttonPressTime; // Calculate duration of press
    if (pressDuration < 5000) { // Short Press (< 5 seconds)
      if (currentMenuState == MenuState::MAIN_MENU) {
        runMainMenuAction();    // Execute main menu action
      } else if (currentMenuState == MenuState::OPTIONS_MENU) {
        runOptionsMenuAction(); // Execute options menu action
      }
    }
    buttonWasPressed = false; // Reset flag
  } else if (isPressed && buttonWasPressed) { // Button is being held down
    unsigned long pressDuration = millis() - buttonPressTime; // Calculate current hold duration
    if (pressDuration > 5000 && pressDuration < 5100) { // Long press (triggered once around 5s mark)
      if (currentMenuState == MenuState::MAIN_MENU) {
        showOptionsMenu(); // Transition to options menu
      }
    } else if (pressDuration > 10000 && pressDuration < 10100) { // Very long press (triggered once around 10s mark)
      if (currentMenuState == MenuState::OPTIONS_MENU) {
        attemptToClearAllData(); // Attempt to clear all data
      }
    }
  }
}

// Executes the action for the main menu (enroll new fingerprint)
void runMainMenuAction() {
  enrollNewFingerprint(); // Calls the function to start fingerprint enrollment
  displayMessage("Add: Press Btn", "Scan: Place Finger"); // Restores default main menu message
}

// Executes the action for the options menu (sync and display server data)
void runOptionsMenuAction() {
  syncAndDisplayServerData(); // Calls the function to sync and display server data
  displayMessage("Add: Press Btn", "Scan: Place Finger"); // Restores default main menu message
  currentMenuState = MenuState::MAIN_MENU; // Returns to main menu after action
}

// =================================================================================================
// FINGERPRINT ENROLLMENT PROCESS (Unchanged from original logic, just commented)
// =================================================================================================
void enrollNewFingerprint() {
  if (WiFi.status() != WL_CONNECTED) { // Check if WiFi is connected
    displayMessage("Enroll Failed", "Need WiFi", 2000); // Enrollment requires WiFi connection
    return;
  }
  
  displayMessage("Getting ID...", "From server"); // Inform user that ID is being fetched
  int newId = getNextAvailableIDFromServer(); // Attempts to get a new available ID from the server

  if (newId < 0 || newId >= 128) { // Checks if a valid ID was received
    displayMessage("Enroll Failed", "No available ID", 2000); // Error message if no valid ID
    return;
  }
  
  Serial.printf("Starting enrollment for new ID: %d\n", newId); // Debug message
  displayMessage("Place finger", "On the sensor"); // Instruct user to place finger

  if (getFingerprintImage(1) != FINGERPRINT_OK) return; // Capture first image; return if error

  displayMessage("Place again", "Same finger"); // Instruct user for second image
  
  if (getFingerprintImage(2) != FINGERPRINT_OK) return; // Capture second image; return if error

  if (createAndStoreModel(newId) != FINGERPRINT_OK) return; // Create and store model; return if error

  String name = getFingerNameFromSerial(newId); // Get name for the fingerprint from serial monitor
  
  if (sendFingerprintToServer(newId, name)) { // Send fingerprint data (ID and name) to the server
    fingerprintCache[newId] = { (uint16_t)newId, name }; // Cache the new fingerprint data locally
    displayMessage("Enroll Success!", "ID: " + String(newId), 2000); // Success message
  } else {
    displayMessage("Enroll Failed", "Server error", 2000); // Error if server communication failed
    finger.deleteModel(newId); // Delete the model from the sensor if server sync fails
  }
}

// Captures a fingerprint image and converts it to a template.
// step: 1 for first image, 2 for second image (for model creation).
// Returns FINGERPRINT_OK on success, or an error code.
int getFingerprintImage(int step) {
  int p = -1; // Result variable
  while (p != FINGERPRINT_OK) { // Loop until an image is successfully captured
    p = finger.getImage(); // Attempt to get an image
    if (p != FINGERPRINT_OK && p != FINGERPRINT_NOFINGER) { // If it's an error but not just "no finger"
      displayMessage("Imaging Error", "", 1500); // Display error message
      return p; // Return the error code
    }
  }

  p = finger.image2Tz(step); // Convert image to template (TZ1 or TZ2 buffer)
  if (p != FINGERPRINT_OK) { // If template conversion fails
    displayMessage("Processing Error", "", 1500); // Display error
  }
  return p; // Return the result
}

// Creates a consolidated fingerprint model from two templates and stores it on the sensor.
// id: The ID to store the model under.
// Returns FINGERPRINT_OK on success, or an error code.
int createAndStoreModel(uint16_t id) {
  displayMessage("Creating model...", ""); // Inform user
  int p = finger.createModel(); // Combine TZ1 and TZ2 templates into a model
  if (p != FINGERPRINT_OK) { // If model creation fails (e.g., images don't match)
    displayMessage("Fingers do not", "match. Try again.", 2000); // Error message
    return p; // Return error
  }

  p = finger.storeModel(id); // Store the created model on the sensor at the specified ID
  if (p != FINGERPRINT_OK) { // If storage fails (e.g., ID slot taken or sensor memory full)
    displayMessage("Storage Error", "Slot may be full.", 2000); // Error message
  }
  return p; // Return result
}

// Prompts the user via Serial Monitor to enter a name for the enrolled fingerprint.
// id: The ID of the fingerprint being named.
// Returns the entered name as a String, or a default name if timeout occurs.
String getFingerNameFromSerial(uint16_t id) {
  displayMessage("Enter name in", "Serial Monitor"); // Instruct user to use serial monitor
  Serial.println("Please enter a name for ID " + String(id) + " and press Enter:"); // Prompt on serial
  
  unsigned long startTime = millis(); // Record start time for timeout
  String name = "";                   // Initialize empty name string
  while (millis() - startTime < 30000) { // Wait for up to 30 seconds for input
    if (Serial.available()) { // If data is available in serial buffer
      char c = Serial.read(); // Read a character
      if (c == '\n' || c == '\r') { // If Enter key is pressed
        if (name.length() > 0) break; // If name is not empty, break loop
      } else {
        name += c; // Append character to name
      }
    }
  }
  
  if (name.length() == 0) { // If no name was entered within timeout
    name = "User_" + String(id); // Assign a default name
    Serial.println("Timeout. Using default name: " + name); // Inform user
  }
  
  return name; // Return the name
}

// =================================================================================================
// CORE OPERATIONS (Unchanged from original logic, just commented)
// =================================================================================================
// Scans for a fingerprint, identifies it, displays a welcome message, and records attendance.
void scanForFingerprint() {
  if (finger.getImage() != FINGERPRINT_OK) return; // Get image; return if no finger or error
  if (finger.image2Tz() != FINGERPRINT_OK) return; // Convert image to template; return if error
  if (finger.fingerFastSearch() != FINGERPRINT_OK) return; // Search database for match; return if no match or error

  uint16_t id = finger.fingerID; // Get the ID of the matched fingerprint
  String name = fingerprintCache[id].name; // Retrieve name from local cache
  if (name.isEmpty()) { // If name not found in cache (e.g., after restart, before sync)
    name = "Name not synced"; // Default message
  }
  
  displayMessage("Welcome!", "ID: " + String(id) + " " + name, 2000); // Display welcome message
  
  recordAttendance(id); // Record attendance for the identified ID
  
  displayMessage("Add: Press Btn", "Scan: Place Finger"); // Restore main menu prompt
}

// Displays the options menu on the LCD.
void showOptionsMenu() {
  currentMenuState = MenuState::OPTIONS_MENU; // Set current state to options menu
  displayMessage("1.Show Data(short)", "2.Del All(hold 10s)"); // Display options
}

// Synchronizes fingerprint data with the server and displays it on the Serial Monitor.
void syncAndDisplayServerData() {
  if (WiFi.status() != WL_CONNECTED) { // Check WiFi status
    displayMessage("WiFi Error", "Not connected", 2000); // Error if not connected
    return;
  }

  displayMessage("Getting data...", "From server"); // Inform user data is being fetched

  HTTPClient http; // Create HTTPClient object
  String url = String(SERVER_HOST) + "/api/SensorData"; // Construct API URL
  http.begin(client, url); // Begin HTTP GET request with secure client

  int httpCode = http.GET(); // Send GET request
  if (httpCode == HTTP_CODE_OK) { // If request was successful (HTTP 200)
    String payload = http.getString(); // Get the response payload (JSON data)
    http.end(); // Close connection
    
    DynamicJsonDocument doc(4096); // Create a JSON document to parse the payload (size adjusted to 4KB)
    DeserializationError error = deserializeJson(doc, payload); // Deserialize JSON string

    if (error) { // If JSON parsing fails
      Serial.println("Failed to parse server data."); // Debug message
      displayMessage("JSON Error", "", 2000); // Display error
      return;
    }
    
    displayMessage("Data in Serial", "Monitor", 2000); // Inform user data is in serial monitor
    Serial.println("--- Fingerprint Data from Server ---"); // Header for serial output
    for (JsonObject item : doc.as<JsonArray>()) { // Iterate through JSON array of objects
      int id = item["id"]; // Get ID
      const char* name = item["name"]; // Get name
      Serial.printf("ID: %d, Name: %s\n", id, name); // Print to serial
      if(id >= 0 && id < 128) { // Update local cache if ID is valid
        fingerprintCache[id] = {(uint16_t)id, String(name)};
      }
    }
    Serial.println("------------------------------------"); // Footer for serial output

  } else {
    http.end(); // Close connection
    Serial.printf("Failed to get data. HTTP Error: %d\n", httpCode); // Debug error message
    displayMessage("Server Error", "Code: " + String(httpCode), 2000); // Display error on LCD
  }
}

// Attempts to clear all fingerprint data from both the server and the sensor.
// Requires admin password confirmation.
void attemptToClearAllData() {
  if (WiFi.status() != WL_CONNECTED){ // Check WiFi status
      displayMessage("Clear Failed", "Need WiFi", 2000); // Error if not connected
      return;
  }
  if (confirmAdminPassword()) { // Call function to confirm admin password
    displayMessage("Clearing Data...", ""); // Inform user
    
    HTTPClient http; // Create HTTPClient object
    String url = String(SERVER_HOST) + "/api/SensorData/clear"; // Construct URL for clear API
    http.begin(client, url); // Begin HTTP POST request
    int httpCode = http.POST(""); // Send POST request (empty body)
    http.end(); // Close connection

    if (httpCode == HTTP_CODE_OK) { // If server clearing was successful
      Serial.println("Server data cleared successfully."); // Debug message
      displayMessage("Server Cleared", "Clearing sensor..."); // Inform user
      
      if (finger.emptyDatabase() == FINGERPRINT_OK) { // Attempt to clear sensor database
        Serial.println("Fingerprint sensor cleared."); // Success message
        displayMessage("All Data Deleted", "", 2000); // Final success message
      } else {
        Serial.println("Failed to clear sensor."); // Debug error
        displayMessage("Sensor Clear Fail", "", 2000); // Display error
      }
      
      // Clear local fingerprint cache (OLD: part of clearing; NEW: good practice to clear local cache too)
      for (int i = 0; i < 128; i++) {
        fingerprintCache[i] = {0, ""};
      }
      SD.remove(LOG_FILE); // (NEW: Also remove the offline log file)

    } else {
      Serial.printf("Failed to clear server data. HTTP Error: %d\n", httpCode); // Debug error
      displayMessage("Server Clear Fail", "Code: " + String(httpCode), 2000); // Display error
    }

  } else {
    displayMessage("Wrong Password", "Operation Canceled", 2000); // Message for incorrect password
  }
  
  displayMessage("Add: Press Btn", "Scan: Place Finger"); // Restore main menu prompt
  currentMenuState = MenuState::MAIN_MENU; // Return to main menu
}

// Prompts for and validates an admin password entered via the Serial Monitor.
// Returns true if password is "admin" within timeout, false otherwise.
bool confirmAdminPassword() {
  displayMessage("Enter Password", "in Serial Monitor"); // Instruct user
  Serial.println("ADMIN ACTION: Enter password to confirm data deletion:"); // Prompt on serial
  
  String input = "";                  // String to store user input
  unsigned long startTime = millis(); // Record start time for timeout
  while (millis() - startTime < 30000) { // Wait for up to 30 seconds
    if (Serial.available()) { // If serial data is available
      char c = Serial.read(); // Read character
      if (c == '\n' || c == '\r') { // If Enter key is pressed
        if (input == "admin") { // Check if input matches "admin"
          Serial.println("Password correct."); // Debug message
          return true; // Return true for correct password
        } else {
          Serial.println("Incorrect password."); // Debug message
          return false; // Return false for incorrect password
        }
      } else {
        input += c; // Append character to input string
      }
    }
  }
  Serial.println("Password entry timed out."); // Debug message for timeout
  return false; // Return false if timeout occurs
}

// =================================================================================================
// RTC, SD CARD & OFFLINE LOGGING (NEW: Comprehensive comments for all these functions)
// =================================================================================================
/**
 * @brief Initializes the SD card module.
 */
void setupSDCard() {
    Serial.println("Initializing SD card..."); // Debug message
    if (!SD.begin(SD_CS_PIN)) { // Attempts to initialize SD card with CS pin
        Serial.println("SD Card initialization failed!"); // Error message if initialization fails
        displayMessage("SD Card Error!", "Check connection", 5000); // Display error on LCD
    } else {
        Serial.println("SD card initialized."); // Success message
    }
}

/**
 * @brief Initializes the RTC module and sets the time via NTP if required and if online.
 */
void setupRTC() {
    Serial.println("Initializing RTC..."); // Debug message
    if (!rtc.begin()) { // Attempts to initialize the RTC
        Serial.println("Couldn't find RTC module!"); // Error if RTC not found
        displayMessage("RTC Error!", "Check wiring", 5000); // Display error on LCD
        return;
    }

    // Only sync time if the RTC has lost power AND we are connected to WiFi
    if (rtc.lostPower()) { // Checks if the RTC has lost power (indicating time might be incorrect)
        Serial.println("RTC lost power."); // Debug message
        if (WiFi.status() == WL_CONNECTED) { // Check if WiFi is connected for NTP sync
            displayMessage("RTC power lost", "Syncing time...", 2000); // Inform user of sync
            syncRtcWithNtp(); // Sync RTC with NTP
        } else {
            Serial.println("Cannot sync RTC time, no WiFi connection."); // Debug message if no WiFi
            displayMessage("RTC Time Not Set", "No WiFi", 2000); // Display message for un-synced RTC
        }
    } else {
        Serial.println("RTC is running."); // Debug message if RTC is OK
        displayMessage("RTC OK", "", 1000); // Display success message
    }
}

/**
 * @brief Connects to NTP to set the RTC's time. Assumes WiFi is already connected.
 */
void syncRtcWithNtp() {
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER); // Configures time zone and NTP server
    struct tm timeinfo; // Structure to hold time information
    if (getLocalTime(&timeinfo, 10000)) { // Attempts to get time from NTP with a 10-second timeout
        // Adjust RTC to the obtained NTP time
        rtc.adjust(DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec));
        Serial.println("RTC has been set with NTP time."); // Debug message for successful sync
        displayMessage("RTC Time Synced", "", 2000); // Display success message
    } else {
        Serial.println("Failed to obtain NTP time."); // Debug message for NTP sync failure
        displayMessage("NTP Sync Failed", "", 2000); // Display failure message
    }
}

/**
 * @brief Records an attendance event. Attempts to log to the server first, then falls back to offline logging.
 * @param id The fingerprint ID to log.
 */
void recordAttendance(uint16_t id) {
    if (!rtc.now().isValid()){ // Checks if the RTC time is valid (i.e., set)
        Serial.println("Cannot log attendance: RTC not set."); // Debug message
        displayMessage("Log Failed", "Time not set", 2000); // Display error
        return;
    }

    time_t now = rtc.now().unixtime(); // Get current timestamp from RTC in Unix time format

    if (WiFi.status() == WL_CONNECTED) { // Check if WiFi is connected
        if (!logAttendanceToServer(id, now)) { // Attempt to send log to server
            Serial.println("Server log failed. Saving offline."); // Debug message if server log fails
            displayMessage("Server Error", "Logging offline", 1500); // Inform user
            logAttendanceOffline(id, now); // Save to SD card if server fails
        } else {
            Serial.println("Attendance logged to server."); // Success message
            displayMessage("Log Sent", "To server", 1500); // Display success
        }
    } else {
        Serial.println("No WiFi. Saving attendance offline."); // Debug message if no WiFi
        logAttendanceOffline(id, now); // Save to SD card directly
    }
}

/**
 * @brief Logs attendance data to the SD card for offline storage.
 * @param id The fingerprint ID.
 * @param timestamp The Unix timestamp of the attendance event.
 */
void logAttendanceOffline(uint16_t id, time_t timestamp) {
    // Open the log file in append mode (FILE_APPEND will create if it doesn't exist)
    File logFile = SD.open(LOG_FILE, FILE_APPEND);
    if (!logFile) { // Check if file opened successfully
        Serial.println("Failed to open log file for appending."); // Error message
        displayMessage("SD Write Error", "", 2000); // Display error
        return;
    }
    
    // Write the ID and timestamp to the file, followed by a newline
    if (logFile.printf("%u,%lu\n", id, timestamp)) {
        Serial.printf("Successfully wrote log to SD: ID %u, Time %lu\n", id, timestamp); // Debug success
        displayMessage("Log Saved", "Offline", 1500); // Display success
    } else {
        Serial.println("Failed to write to log file."); // Debug error
        displayMessage("SD Write Error", "", 2000); // Display error
    }
    logFile.close(); // Close the file
}

/**
 * @brief Attempts to synchronize offline attendance logs stored on the SD card with the server.
 * Reads logs line by line, attempts to send each to the server. Logs that fail are rewritten to a temporary file.
 * The original log file is then replaced with the temporary file.
 */
void syncOfflineLogs() {
    if (WiFi.status() != WL_CONNECTED) return; // Exit if not connected to WiFi

    Serial.println("Starting offline log sync process..."); // Debug message
    displayMessage("Syncing Logs...", ""); // Inform user

    // Open the main log file for reading
    File logFile = SD.open(LOG_FILE, FILE_READ);
    if (!logFile || logFile.size() == 0) { // Check if file exists and has content
        if (logFile) logFile.close(); // Close if opened but empty
        Serial.println("No offline logs to sync."); // Debug message
        displayMessage("No Offline Logs", "", 1500); // Display message
        return;
    }

    const char* tempLogFile = "/temp_log.txt"; // Define a temporary file name
    SD.remove(tempLogFile); // Ensure the temporary file is clean before starting
    File tempFile = SD.open(tempLogFile, FILE_WRITE); // Open temporary file for writing

    if (!tempFile) { // Check if temporary file opened successfully
        Serial.println("Failed to create temporary log file. Aborting sync."); // Error message
        logFile.close(); // Close original log file
        return;
    }

    int syncedCount = 0; // Counter for successfully synced logs
    int failedCount = 0; // Counter for logs that failed to sync

    while (logFile.available()) { // Read log file line by line
        String line = logFile.readStringUntil('\n'); // Read a line until newline
        line.trim(); // Remove leading/trailing whitespace (including carriage return)
        if (line.length() == 0) continue; // Skip empty lines

        int commaIndex = line.indexOf(','); // Find the comma separator
        if (commaIndex == -1) continue; // Skip malformed lines

        uint16_t id = line.substring(0, commaIndex).toInt(); // Parse ID from the line
        time_t timestamp = strtoul(line.substring(commaIndex + 1).c_str(), NULL, 10); // Parse timestamp

        if (logAttendanceToServer(id, timestamp)) { // Attempt to send the log to the server
            syncedCount++; // Increment synced count if successful
        } else {
            tempFile.println(line); // Rewrite the line to the temporary file if it fails
            failedCount++; // Increment failed count
        }
    }
    
    logFile.close(); // Close the original log file
    tempFile.close(); // Close the temporary log file

    SD.remove(LOG_FILE); // Delete the original log file
    SD.rename(tempLogFile, LOG_FILE); // Rename the temporary file to the original log file name
    
    Serial.printf("Log sync finished. Synced: %d, Failed (re-saved): %d\n", syncedCount, failedCount); // Debug summary
    if (syncedCount > 0) {
        displayMessage("Synced " + String(syncedCount) + " logs", "", 2000); // Display sync summary
    } else if (failedCount == 0) {
        displayMessage("Logs are up", "to date.", 2000); // Message if no logs needed syncing
    }
}

// =================================================================================================
// SERVER COMMUNICATION & SYNC (Unchanged from original logic, just commented)
// =================================================================================================
/**
 * @brief Synchronizes the fingerprint sensor's database with the server's database.
 * Fetches data from the server and deletes any fingerprint templates on the sensor
 * that are not present on the server (orphaned sensor data).
 * (NEW: Logic explicitly mentions handling of orphaned sensor data)
 */
void syncSensorWithServer() {
  if (WiFi.status() != WL_CONNECTED) { // Check WiFi status
    Serial.println("Sync failed: WiFi not connected."); // Debug message
    displayMessage("Sync Failed", "No WiFi", 2000); // Display error
    return;
  }

  Serial.println("Syncing sensor with server..."); // Debug message
  displayMessage("Syncing...", ""); // Inform user

  bool serverHasID[128] = {false}; // Array to track which IDs exist on the server
  HTTPClient http; // Create HTTPClient object
  String url = String(SERVER_HOST) + "/api/SensorData"; // Construct API URL
  http.begin(client, url); // Begin HTTP GET request
  int httpCode = http.GET(); // Send GET request
  if (httpCode == HTTP_CODE_OK) { // If server request is successful
    String payload = http.getString(); // Get response payload
    DynamicJsonDocument doc(4096); // Create JSON document
    if (deserializeJson(doc, payload).code() == DeserializationError::Ok) { // Parse JSON
      for (JsonObject item : doc.as<JsonArray>()) { // Iterate through server data
        int id = item["id"]; // Get ID
        if (id >= 0 && id < 128) { // If ID is valid
          serverHasID[id] = true; // Mark this ID as existing on the server
          fingerprintCache[id] = {(uint16_t)id, String(item["name"].as<const char*>())}; // Update local cache
        }
      }
    }
  }
  http.end(); // Close connection
  
  Serial.println("Checking for orphaned fingerprints on sensor..."); // Debug message
  // Iterate through all possible sensor IDs (0-127)
  for (int id = 0; id < 128; id++) {
    if (finger.loadModel(id) == FINGERPRINT_OK) { // Attempt to load a model from the sensor at this ID
      if (!serverHasID[id]) { // If a model exists on sensor but NOT on the server
        Serial.printf("ID %d is on sensor but not server. Deleting from sensor.\n", id); // Debug message
        finger.deleteModel(id); // Delete the orphaned model from the sensor
      }
    }
  }
  
  Serial.println("Sync complete."); // Debug message
  displayMessage("Sync Complete", "", 1500); // Display success message
}

/**
 * @brief Fetches the next available fingerprint ID from the server.
 * This ensures that new enrollments use IDs managed by the central system.
 * @return The next available ID (0-127) on success, or -1 on failure.
 */
int getNextAvailableIDFromServer() {
  if (WiFi.status() != WL_CONNECTED) return -1; // Return -1 if not connected to WiFi

  HTTPClient http; // Create HTTPClient object
  String url = String(SERVER_HOST) + "/api/SensorData/generate-id"; // Construct API URL for ID generation
  http.begin(client, url); // Begin HTTP GET request

  int httpCode = http.GET(); // Send GET request
  if (httpCode == HTTP_CODE_OK) { // If successful
    int id = http.getString().toInt(); // Parse the response (which should be the ID)
    http.end(); // Close connection
    return id; // Return the ID
  } else {
    Serial.printf("Failed to get next ID. HTTP Error: %d\n", httpCode); // Debug error
    http.end(); // Close connection
    return -1; // Return -1 on failure
  }
}

/**
 * @brief Sends new fingerprint data (ID and name) to the server.
 * This is called after a successful enrollment on the device.
 * @param id The ID of the newly enrolled fingerprint.
 * @param name The name associated with the fingerprint.
 * @return True on successful server communication (HTTP 200/201), false otherwise.
 */
bool sendFingerprintToServer(uint16_t id, const String& name) {
  if (WiFi.status() != WL_CONNECTED) return false; // Return false if not connected to WiFi

  HTTPClient http; // Create HTTPClient object
  String url = String(SERVER_HOST) + "/api/SensorData"; // Construct API URL for adding sensor data
  http.begin(client, url); // Begin HTTP POST request
  http.addHeader("Content-Type", "application/json"); // Set content type header

  DynamicJsonDocument doc(256); // Create JSON document for payload
  doc["id"] = id;         // Add ID to JSON
  doc["name"] = name;     // Add name to JSON
  String jsonPayload;     // String to hold serialized JSON
  serializeJson(doc, jsonPayload); // Serialize JSON document to string

  int httpCode = http.POST(jsonPayload); // Send POST request with JSON payload
  http.end(); // Close connection
  
  if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_CREATED) { // Check for success codes (200 OK or 201 Created)
    Serial.println("Fingerprint sent to server successfully."); // Debug message
    return true; // Return true on success
  } else {
    Serial.printf("Failed to send fingerprint to server. HTTP Error: %d\n", httpCode); // Debug error
    return false; // Return false on failure
  }
}

/**
 * @brief Sends an attendance log entry to the server.
 * @param id The fingerprint ID for the attendance.
 * @param timestamp The Unix timestamp of the attendance event.
 * @return True on successful server communication (HTTP 200/201), false otherwise.
 */
bool logAttendanceToServer(uint16_t id, time_t timestamp) {
    if (WiFi.status() != WL_CONNECTED) return false; // Return false if not connected to WiFi

    HTTPClient http; // Create HTTPClient object
    String url = String(SERVER_HOST) + "/api/AttendanceLogs"; // Construct API URL for attendance logs
    http.begin(client, url); // Begin HTTP POST request
    http.addHeader("Content-Type", "application/json"); // Set content type header

    DynamicJsonDocument doc(256); // Create JSON document for payload
    doc["fingerprintId"] = id; // Add fingerprint ID to JSON
    doc["timestamp"] = timestamp; // Add timestamp to JSON
    String jsonPayload; // String to hold serialized JSON
    serializeJson(doc, jsonPayload); // Serialize JSON document to string

    Serial.println("Sending log to server: " + jsonPayload); // Debug: show payload being sent
    int httpCode = http.POST(jsonPayload); // Send POST request with JSON payload
    http.end(); // Close connection
    
    if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_CREATED) { // Check for success codes
        Serial.printf("Server accepted log. Response: %d\n", httpCode); // Debug success
        return true; // Return true on success
    } else {
        Serial.printf("Failed to send log to server. HTTP Error: %d\n", httpCode); // Debug error
        return false; // Return false on failure
    }
}