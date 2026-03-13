#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <HTTPClient.h>
#include <HardwareSerial.h>
#include <NewPing.h>
#include <Preferences.h>
#include <RTClib.h>
#include <Update.h>
#include <WebServer.h>
#include <WebSocketsClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <Wire.h>

// ================= WIFI & SERVER CONFIG =================
// Default Server IP (will be overwritten by NVS or WiFiManager if entered)
char websockets_server_host[64] = "homepump.espserver.site";
const uint16_t websockets_server_port = 443; // Server Port (WSS)

// ================= OTA CONFIG =================
const char *firmwareUrl = "https://github.com/shohidmax/home_pump/releases/download/tank/Home_tank.ino.bin";
const char *versionUrl = "https://raw.githubusercontent.com/shohidmax/home_pump/refs/heads/main/version.txt";
const char *currentFirmwareVersion = "1.1.8";
const unsigned long updateCheckInterval = 5 * 60 * 1000; // 5 minutes
unsigned long lastUpdateCheck = 0;

// ================= NTP CONFIG =================
const long gmtOffset_sec = 21600; // +6 hours for Bangladesh
const int daylightOffset_sec = 0;
const char *ntpServer = "pool.ntp.org";

// ================= CONFIGURATION =================
// These defaults will be overwritten by values from Preferences if available
int TANK_HEIGHT_CM = 100;
const int SENSOR_GAP_CM = 5;
int PUMP_ON_LEVEL = 20;  // Default, can be changed via Dashboard
int PUMP_OFF_LEVEL = 90; // Default, can be changed via Dashboard
bool schedulesEnabled = true;
int PRE_SCHEDULE_LIMIT = 65; // Limit before schedule
int RECOVERY_TRIGGER = 70;   // Trigger for missed schedule recovery

// Active Schedule Hours (0-23)
int sched1 = 8;
int sched2 = 14;
int sched3 = 20;

// Flow Fault (Dry Run) Protection
bool flowFaultEnabled = true;
int flowCheckMin = 5; // Check after 5 mins
int flowMinInc = 4;   // Must increase by 4%
int flowRetryMin = 15; // Retry after 15 mins

// ================= PINS =================
#define PIN_RELAY_PUMP 32
#define PIN_DHT 4
#define PIN_BUZZER 13
#define PIN_US_SIG 5
#define PIN_SW_RESET 33
#define I2C_SDA 21
#define I2C_SCL 22
#define DF_RX 16
#define DF_TX 17

// ================= OBJECTS =================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
RTC_DS3231 rtc;
DHT dht(PIN_DHT, DHT11);
NewPing sonar(PIN_US_SIG, PIN_US_SIG, 400);
HardwareSerial dfSerial(1);

WebSocketsClient webSocket;
Preferences preferences;
WebServer server(80);

// ================= VARIABLES =================
int waterLevelPercent = 0;
bool pumpStatus = false;
bool manualMode =
    false; // true = controlled by web, false = automatic sensor logic
// --- Advanced Logic Variables ---
bool recoveryPending = false;
int lastScheduleDay = -1;
int lastScheduleHour = -1;

// --- Pump Safety Variables ---
unsigned long pumpStartTime = 0;
int startWaterLevel = 0;
bool pumpErrorState = false;      // Temporary 15 min error
bool maxTimeErrorState = false;   // Permanent 60 min lock
unsigned long errorStartTime = 0;
const unsigned long ERROR_COOLDOWN_MS = 15 * 60 * 1000; // 15 mins
const unsigned long SAFETY_CHECK_MS = 5 * 60 * 1000;    // 5 mins
const unsigned long MAX_RUN_TIME_MS = 60UL * 60UL * 1000UL;   // 60 mins max run time
const int SAFETY_INCREASE_PERCENT = 2;                  // 2% increase required

// Flow Fault State
bool flowFaultState = false;
unsigned long flowFaultTriggerTime = 0;
bool flowFaultBeeped = false; // Only long beep once when fault triggers

unsigned long lastReadTime = 0;
unsigned long lastSendTime = 0;
unsigned long resetBtnTimer = 0;
bool resetBtnActive = false;
float temperature = 0.0;

// ================= FUNCTION PROTOTYPES =================
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length);
void sendDataToServer();
void handleResetButton();
void readSensors();
void controlPump();
void checkPumpSafety();
void updateDisplay();

void beep(int times, int duration);
void syncRTC();
void loadSettings();
void saveSettings();
void handleRoot();
void handleSaveConfig();
// OTA Functions
void checkForFirmwareUpdate();
String fetchLatestVersion();
void downloadAndApplyFirmware();
bool startOTAUpdate(WiFiClient *client, int contentLength);

void setup() {
  Serial.begin(115200);

  // 1. Initialize Pins
  pinMode(PIN_RELAY_PUMP, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_SW_RESET, INPUT_PULLUP);
  digitalWrite(PIN_RELAY_PUMP, LOW);

  // Load settings from NVS
  loadSettings();

  // 2. Initialize Display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 Failed"));
    for (;;)
      ;
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.println(F("Booting System..."));
  display.display();
  delay(1000);

  // 3. Initialize Sensors
  dht.begin();
  if (!rtc.begin()) {
    Serial.println("No RTC");
  }
  dfSerial.begin(9600, SERIAL_8N1, DF_RX, DF_TX);

  // 4. Connect to WiFi using WiFiManager
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Connecting WiFi...");
  display.println("If stuck, connect to:");
  display.println("AP: Home_TANK");
  display.display();

  WiFiManager wm;

  // Custom Parameter for Server IP
  WiFiManagerParameter custom_server_ip("server", "Server IP",
                                        websockets_server_host, 40);
  wm.addParameter(&custom_server_ip);
  wm.setConnectTimeout(60); // Timeout if cannot connect

  // AutoConnect
  bool res = wm.autoConnect("Home_TANK"); // AP Name

  if (!res) {
    Serial.println("Failed to connect");
    display.println("WiFi Failed");
    // ESP.restart(); // Optional: restart if failed
  } else {
    Serial.println("WiFi Connected");
    display.println("WiFi OK");
    display.println(WiFi.localIP());

    // WiFiManager provides custom_server_ip. 
    // We only use this if nothing was saved in NVS yet.
    if (String(custom_server_ip.getValue()) != "" && String(custom_server_ip.getValue()) != "homepump.espserver.site") {
      strcpy(websockets_server_host, custom_server_ip.getValue());
      preferences.begin("farmwire", false);
      preferences.putString("ws_host", websockets_server_host);
      preferences.end();
    }
    
    Serial.print("Connecting to WebSocket Server: ");
    Serial.println(websockets_server_host);

    // Check for updates immediately on boot
    checkForFirmwareUpdate();

    // Sync RTC with NTP
    syncRTC();
  }
  display.display();
  delay(1000);

  // 5. Connect WebSocket (WSS)
  webSocket.beginSSL(websockets_server_host, websockets_server_port, "/");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);

  // 6. Start Web Server for Local Config
  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSaveConfig);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  webSocket.loop();      // Handle network traffic
  server.handleClient(); // Handle local web server
  handleResetButton();

  // OTA Update Check (Every 5 mins)
  if (millis() - lastUpdateCheck >= updateCheckInterval) {
    lastUpdateCheck = millis();
    if (WiFi.status() == WL_CONNECTED) {
      checkForFirmwareUpdate();
    }
  }

  // Read sensors every 2 seconds
  if (millis() - lastReadTime > 2000) {
    readSensors();
    controlPump(); // Logic handles Auto vs Manual inside
    checkPumpSafety(); // 5 min safety check
    updateDisplay();
    sendDataToServer();
    lastReadTime = millis();
  }
}

// --- WebSocket Event Handler ---
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
  case WStype_DISCONNECTED:
    Serial.println("[WSc] Disconnected!");
    break;
  case WStype_CONNECTED:
    Serial.println("[WSc] Connected to server");
    break;
  case WStype_TEXT:
    Serial.printf("[WSc] Got text: %s\n", payload);

    // Parse JSON Command
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (!error) {
      if (!doc.containsKey("command"))
        return;
      const char *command = doc["command"];
      if (strcmp(command, "PUMP_ON") == 0) {
        if (!pumpErrorState && !maxTimeErrorState && !flowFaultState) { // Block manual turn on if in error
          manualMode = true;
          pumpStatus = true;
          pumpStartTime = millis();
          startWaterLevel = waterLevelPercent;
          digitalWrite(PIN_RELAY_PUMP, HIGH);
          beep(1, 100);
        } else {
          beep(3, 50); // Beep quickly to indicate blocked
        }
      } else if (strcmp(command, "PUMP_OFF") == 0) {
        manualMode = true;
        pumpStatus = false;
        digitalWrite(PIN_RELAY_PUMP, LOW);
        beep(1, 100);
      } else if (strcmp(command, "AUTO") == 0) {
        manualMode = false;
        beep(2, 50);
      } else if (strcmp(command, "SETTINGS") == 0) {
        // Parse settings
        if (doc.containsKey("min"))
          PUMP_ON_LEVEL = doc["min"];
        if (doc.containsKey("max"))
          PUMP_OFF_LEVEL = doc["max"];
        if (doc.containsKey("sched"))
          schedulesEnabled = doc["sched"];
        if (doc.containsKey("pre"))
          PRE_SCHEDULE_LIMIT = doc["pre"];
        if (doc.containsKey("rec"))
          RECOVERY_TRIGGER = doc["rec"];
        
        if (doc.containsKey("s1")) sched1 = doc["s1"];
        if (doc.containsKey("s2")) sched2 = doc["s2"];
        if (doc.containsKey("s3")) sched3 = doc["s3"];

        if (doc.containsKey("ff_en")) flowFaultEnabled = doc["ff_en"];
        if (doc.containsKey("ff_chk")) flowCheckMin = doc["ff_chk"];
        if (doc.containsKey("ff_inc")) flowMinInc = doc["ff_inc"];
        if (doc.containsKey("ff_ret")) flowRetryMin = doc["ff_ret"];

        saveSettings(); // Save to NVS
        Serial.println("Settings Applied & Saved to Flash!");
        beep(3, 100);   // Confirm settings update
      }
    }
    break;
  }
}

void sendDataToServer() {
  if (WiFi.status() == WL_CONNECTED) {
    StaticJsonDocument<512> doc;
    doc["command"] = "STATUS";
    doc["level"] = waterLevelPercent;
    doc["pump"] = pumpStatus;
    doc["temp"] = temperature;
    doc["wifi"] = WiFi.SSID();
    doc["ip"] = WiFi.localIP().toString();
    doc["rssi"] = WiFi.RSSI();
    
    String mode;
    if (maxTimeErrorState) {
      mode = "MAX_TIME_ERROR";
    } else if (flowFaultState) {
      mode = "FLOW_FAULT_WAIT";
      unsigned long elapsed = millis() - flowFaultTriggerTime;
      unsigned long requiredMs = flowRetryMin * 60000UL;
      int remainingSec = (requiredMs > elapsed) ? ((requiredMs - elapsed) / 1000) : 0;
      doc["ff_sec_left"] = remainingSec;
    } else if (pumpErrorState) {
      mode = "ERROR_COOLDOWN";
    } else {
      mode = manualMode ? "MANUAL" : "AUTO";
    }
    doc["mode"] = mode;

    // Broadcast current settings so Dashboard stays in sync
    JsonObject settings = doc.createNestedObject("settings");
    settings["min"] = PUMP_ON_LEVEL;
    settings["max"] = PUMP_OFF_LEVEL;
    settings["sched"] = schedulesEnabled;
    settings["pre"] = PRE_SCHEDULE_LIMIT;
    settings["rec"] = RECOVERY_TRIGGER;
    settings["s1"] = sched1;
    settings["s2"] = sched2;
    settings["s3"] = sched3;
    settings["h_cm"] = TANK_HEIGHT_CM;
    settings["ff_en"] = flowFaultEnabled;
    settings["ff_chk"] = flowCheckMin;
    settings["ff_inc"] = flowMinInc;
    settings["ff_ret"] = flowRetryMin;

    String jsonString;
    serializeJson(doc, jsonString);
    webSocket.sendTXT(jsonString);
  }
}

// --- Standard Logic ---

void handleResetButton() {
  if (digitalRead(PIN_SW_RESET) == LOW) {
    if (!resetBtnActive) {
      resetBtnActive = true;
      resetBtnTimer = millis();
    }
    if (millis() - resetBtnTimer > 10000) {
      display.clearDisplay();
      display.setCursor(0, 20);
      display.println("Resetting WiFi...");
      display.display();
      beep(3, 100);

      WiFiManager wm;
      wm.resetSettings();

      ESP.restart();
    }
  } else {
    resetBtnActive = false;
  }
}

void readSensors() {
  unsigned int uS = sonar.ping_median(5);
  int distanceCM = sonar.convert_cm(uS);

  if (distanceCM > 0) {
    int rawLevel =
        map(distanceCM, TANK_HEIGHT_CM + SENSOR_GAP_CM, SENSOR_GAP_CM, 0, 100);
    waterLevelPercent = constrain(rawLevel, 0, 100);
  }

  float t = dht.readTemperature();
  if (!isnan(t))
    temperature = t;
}

void controlPump() {
  if (manualMode || maxTimeErrorState || pumpErrorState || flowFaultState)
    return;

  DateTime now = rtc.now();
  int currentHour = now.hour();
  int currentDay = now.day();

  // Reset tracker on new day
  if (currentDay != lastScheduleDay) {
    lastScheduleDay = currentDay;
    lastScheduleHour = -1;
  }

  // --- 1. Schedule & Recovery Detection ---
  if (schedulesEnabled) {
    int slots[] = {sched1, sched2, sched3}; // Dynamic Schedule times

    for (int slot : slots) {
      // A. Check if currently IN schedule window (first 10 mins)
      if (currentHour == slot && now.minute() < 10) {
        // Avoid re-triggering if already handled
        if (lastScheduleHour != slot) {
          if (waterLevelPercent <= 85) {
            // Trigger Schedule Start
            if (!pumpStatus && !pumpErrorState && !maxTimeErrorState && !flowFaultState) {
              pumpStatus = true;
              pumpStartTime = millis();
              startWaterLevel = waterLevelPercent;
              digitalWrite(PIN_RELAY_PUMP, HIGH);
              beep(1, 500);
            }
            lastScheduleHour = slot; // Mark done
          } else {
            // Tank full enough, skip schedule
            lastScheduleHour = slot; // Mark skipped/done
          }
        }
      }
      // B. Check if we MISSED this slot (e.g. power outage during 8am)
      else if (currentHour > slot && lastScheduleHour < slot) {
        // We differ from the last run slot, so we missed it.
        // (Simple check: if we haven't recorded running this slot, and it's
        // past time) BUT avoid marking if we simply skipped it earlier. Logic
        // deficiency: lastScheduleHour tracks *successful/skipped* run. If it
        // is -1, and hour is 9, we missed 8. Correct.
        recoveryPending = true;
      }
    }
  }

  // --- 2. Start Logic (Non-Scheduled) ---
  if (!pumpStatus && !pumpErrorState && !maxTimeErrorState && !flowFaultState) {
    // Normal Low Level Start
    if (waterLevelPercent <= PUMP_ON_LEVEL) {
      pumpStatus = true;
      pumpStartTime = millis();
      startWaterLevel = waterLevelPercent;
      digitalWrite(PIN_RELAY_PUMP, HIGH);
      beep(1, 500);
    }
    // Recovery Start (Missed Schedule)
    else if (recoveryPending && waterLevelPercent <= RECOVERY_TRIGGER) {
      pumpStatus = true;
      pumpStartTime = millis();
      startWaterLevel = waterLevelPercent;
      digitalWrite(PIN_RELAY_PUMP, HIGH);
      beep(3, 200);
    }
  }

  // --- 3. Stop Logic ---
  if (pumpStatus) {
    int limit = PUMP_OFF_LEVEL; // Default 90%

    // Pre-Schedule Throttling (1 hour before schedules)
    if (currentHour == (sched1 - 1) || currentHour == (sched2 - 1) || currentHour == (sched3 - 1)) {
      limit = PRE_SCHEDULE_LIMIT;
    }

    if (waterLevelPercent >= limit) {
      pumpStatus = false;
      digitalWrite(PIN_RELAY_PUMP, LOW);
      beep(2, 200);

      // If we topped up fully (>=90), clear recovery
      if (waterLevelPercent >= 90) {
        recoveryPending = false;
      }
    }
  }
}

void checkPumpSafety() {
  unsigned long nowMs = millis();
  
  // 0. Permanent Lock Check
  if (maxTimeErrorState) {
      return; // Device is permanently locked until reset
  }

  // 1. Check if we need to clear the ERROR cooldown
  if (pumpErrorState) {
    if (nowMs - errorStartTime >= ERROR_COOLDOWN_MS) {
      pumpErrorState = false; // Cooldown finished
      beep(2, 500); // 2 long beeps to signal readiness
      Serial.println("Pump Safety Cooldown Finished. Returning to normal operation.");
    }
    return; // Don't do other safety checks while in cooldown
  }

  // --- Flow Fault / Dry Run Auto-Retry Logic ---
  if (flowFaultState) {
    unsigned long elapsed = millis() - flowFaultTriggerTime;
    unsigned long requiredWaitMs = flowRetryMin * 60000UL;
    
    if (elapsed >= requiredWaitMs) {
      // Retry period complete, clear fault, let controlPump() try again natively
      flowFaultState = false;
      flowFaultBeeped = false;
      Serial.println("Flow Fault cooldown complete. Resuming AUTO operations.");
    } else {
      // Force pump to stay off during cooldown
      if (pumpStatus || digitalRead(PIN_RELAY_PUMP) == HIGH) {
        pumpStatus = false;
        digitalWrite(PIN_RELAY_PUMP, LOW);
      }
      return; 
    }
  }

  // 2. Main Safety Checks:
  if (pumpStatus) {
    // A. 60-Minute Max Run-Time Lock
    if (nowMs - pumpStartTime >= MAX_RUN_TIME_MS) {
        Serial.println("CRITICAL: PUMP RAN FOR > 60 MINS. FORCING OFF AND LOCKING SYSTEM.");
        pumpStatus = false;
        digitalWrite(PIN_RELAY_PUMP, LOW);
        maxTimeErrorState = true;
        manualMode = false;
        beep(10, 50); // Continuous quick beeping
        return;
    }

    // B. 5-Minute Increase Check (Original safety check)
    if (nowMs - pumpStartTime >= SAFETY_CHECK_MS) {
      // It's been 5 minutes since pump turned on.
      if (waterLevelPercent < (startWaterLevel + SAFETY_INCREASE_PERCENT)) {
        // Water did not increase by 2%
        Serial.println("SAFETY TRIGGER: Water level did not increase within 5 mins.");
        
        // Turn OFF pump immediately
        pumpStatus = false;
        digitalWrite(PIN_RELAY_PUMP, LOW);
        
        // Trigger Error State
        pumpErrorState = true;
        errorStartTime = nowMs;
        
        // Disable Manual Mode so it returns to Auto after cooldown if it was forced
        manualMode = false;
        
        // Alert Sound: 5 quick beeps
        beep(5, 100);
      }
    }

    // C. Flow Fault / Dry Run Detection (New logic)
    if (flowFaultEnabled) {
      unsigned long checkMs = flowCheckMin * 60000UL;
      if (nowMs - pumpStartTime >= checkMs) {
        int expectedMinLevel = startWaterLevel + flowMinInc;
        if (waterLevelPercent < expectedMinLevel) {
          // Motor ran for X mins but water didn't rise Y%. Shut off!
          pumpStatus = false;
          digitalWrite(PIN_RELAY_PUMP, LOW);
          
          flowFaultState = true;
          flowFaultTriggerTime = nowMs;
          
          Serial.println("WARNING: Flow Fault Detected! Pump shut down. Waiting for retry.");
          
          if(!flowFaultBeeped) {
             beep(3, 500); // Alert user locally
             flowFaultBeeped = true;
          }
          return;
        }
      }
    }
  }
}

void updateDisplay() {
  DateTime now = rtc.now();
  display.clearDisplay();

  // Time & Date
  display.setTextSize(1);
  display.setCursor(0, 0);

  // Date DD/MM
  if (now.day() < 10)
    display.print('0');
  display.print(now.day());
  display.print('/');
  if (now.month() < 10)
    display.print('0');
  display.print(now.month());
  display.print(" ");

  // Time HH:MM
  if (now.hour() < 10)
    display.print('0');
  display.print(now.hour());
  display.print(':');
  if (now.minute() < 10)
    display.print('0');
  display.print(now.minute());

  display.setCursor(85, 0);
  display.print((int)temperature);
  display.print("C");
  if (WiFi.status() == WL_CONNECTED)
    display.print("W");

  // Level
  display.setTextSize(2);
  display.setCursor(0, 15);
  display.print("Lvl: ");
  display.print(waterLevelPercent);
  display.print("%");

  // Bar Graph
  display.drawRect(0, 35, 128, 10, WHITE);
  int barWidth = map(waterLevelPercent, 0, 100, 0, 126);
  display.fillRect(2, 37, barWidth, 6, WHITE);

  // Pump Status
  display.setTextSize(1);
  display.setCursor(0, 50);
  
  if (maxTimeErrorState) {
    display.print("SYS LOCKED");
  } else if (flowFaultState) {
    display.print("FLOW FAULT");
  } else if (pumpErrorState) {
    display.print("ERR WAIT:");
  } else if (manualMode) {
    display.print("MANUAL: ");
  } else {
    display.print("AUTO: ");
  }

  if (pumpStatus)
    display.print("ON");
  else
    display.print("OFF");

  display.display();
}

// ================= PERSISTENCE & LOCAL SERVER =================
void loadSettings() {
  preferences.begin("farmwire", true); // Read-only
  PUMP_ON_LEVEL = preferences.getInt("on", 20);
  PUMP_OFF_LEVEL = preferences.getInt("off", 90);
  PRE_SCHEDULE_LIMIT = preferences.getInt("pre", 65);
  RECOVERY_TRIGGER = preferences.getInt("rec", 70);
  schedulesEnabled = preferences.getBool("sched", true);
  sched1 = preferences.getInt("s1", 8);
  sched2 = preferences.getInt("s2", 14);
  sched3 = preferences.getInt("s3", 20);
  
  flowFaultEnabled = preferences.getBool("ff_en", true);
  flowCheckMin = preferences.getInt("ff_chk", 5);
  flowMinInc = preferences.getInt("ff_inc", 4);
  flowRetryMin = preferences.getInt("ff_ret", 15);

  TANK_HEIGHT_CM = preferences.getInt("h_cm", 100);
  String savedHost = preferences.getString("ws_host", "");
  if (savedHost != "") {
    savedHost.toCharArray(websockets_server_host, 64);
  }
  
  preferences.end();
  Serial.println("Settings loaded from NVS");
}

void saveSettings() {
  preferences.begin("farmwire", false); // Read-write
  preferences.putInt("on", PUMP_ON_LEVEL);
  preferences.putInt("off", PUMP_OFF_LEVEL);
  preferences.putInt("pre", PRE_SCHEDULE_LIMIT);
  preferences.putInt("rec", RECOVERY_TRIGGER);
  preferences.putBool("sched", schedulesEnabled);
  // TANK_HEIGHT_CM is saved separately when updated via web
  preferences.end();
  Serial.println("Settings saved to NVS");
}

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta name=\"viewport\" "
                "content=\"width=device-width, initial-scale=1\">";
  html += "<style>body{font-family:sans-serif;margin:20px;text-align:center; background:#f4f7f6; color:#333;}"
          ".container{background:#fff; padding:25px; border-radius:12px; box-shadow:0 4px 10px rgba(0,0,0,0.1); max-width:400px; margin:0 auto;}"
          "input{padding:10px; font-size:1rem; width:100%; box-sizing:border-box; border:1px solid #ccc; border-radius:6px; margin-bottom:15px;}"
          "label{display:block; text-align:left; font-weight:bold; margin-bottom:5px; font-size:0.9rem; color:#555;}"
          "button{padding:12px 20px; font-size:1.1rem; background:#007bff; color:white; border:none; border-radius:6px; cursor:pointer; width:100%; font-weight:bold;}"
          "button:hover{background:#0056b3;}</style>";
  html += "</head><body>";
  html += "<div class=\"container\">";
  html += "<h2 style=\"color:#007bff;\">🚰 ESP32 Settings Panel</h2>";
  html += "<form action=\"/save\" method=\"POST\">";
  
  html += "<label>WebSocket Server URL:</label>";
  html += "<input type=\"text\" name=\"wshost\" value=\"" + String(websockets_server_host) + "\" placeholder=\"wss://domain.com or IP\"><br>";

  html += "<label>Total Tank Height (cm):</label>";
  html += "<input type=\"number\" name=\"height\" value=\"" + String(TANK_HEIGHT_CM) + "\" min=\"50\" max=\"500\"><br>";
  
  html += "<button type=\"submit\">Save & Restart</button>";
  html += "</form></div></body></html>";
  server.send(200, "text/html", html);
}

void handleSaveConfig() {
  if (server.hasArg("height") && server.hasArg("wshost")) {
    int h = server.arg("height").toInt();
    String newHost = server.arg("wshost");
    newHost.trim();

    if (h > 50 && h < 500 && newHost.length() > 0) {
      TANK_HEIGHT_CM = h;
      newHost.toCharArray(websockets_server_host, 64);

      preferences.begin("farmwire", false);
      preferences.putInt("h_cm", TANK_HEIGHT_CM);
      preferences.putString("ws_host", websockets_server_host);
      preferences.end();

      String successHtml = "<!DOCTYPE html><html><body style=\"font-family:sans-serif; text-align:center; margin-top:50px;\">";
      successHtml += "<h1 style=\"color:green;\">✅ Saved Successfully!</h1>";
      successHtml += "<p><strong>New Tank Height:</strong> " + String(h) + " cm</p>";
      successHtml += "<p><strong>New Server URL:</strong> " + newHost + "</p>";
      successHtml += "<p style=\"color:red; font-weight:bold;\">Restarting ESP32 in 3 seconds...</p>";
      successHtml += "</body></html>";

      server.send(200, "text/html", successHtml);
      beep(2, 200);
      delay(3000);
      ESP.restart(); // Restart to connect to the new WS URL
    } else {
      server.send(400, "text/plain", "Error: Invalid Height or Empty Server URL.");
    }
  } else {
    server.send(400, "text/plain", "Missing Arguments.");
  }
}

void beep(int times, int duration) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PIN_BUZZER, HIGH);
    delay(duration);
    digitalWrite(PIN_BUZZER, LOW);
    if (i < times - 1)
      delay(100);
  }
}

void syncRTC() {
  Serial.println("Syncing RTC with NTP...");
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Syncing Time...");
  display.display();

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }

  Serial.println("Got time from NTP");
  // Adjust RTC
  rtc.adjust(DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1,
                      timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min,
                      timeinfo.tm_sec));
  Serial.println("RTC Adjusted");
}

// ================= OTA FUNCTIONS =================
void checkForFirmwareUpdate() {
  Serial.println("Checking for firmware update...");

  String latestVersion = fetchLatestVersion();

  if (latestVersion == "") {
    Serial.println("Could not verify latest version.");
    return;
  }

  Serial.println("Current Version: " + String(currentFirmwareVersion));
  Serial.println("Latest Version: " + latestVersion);

  if (latestVersion != currentFirmwareVersion) {
    Serial.println("Update available. Starting OTA...");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Updating Firmware...");
    display.println("Do not turn off!");
    display.display();
    downloadAndApplyFirmware();
  } else {
    Serial.println("Device is up to date.");
  }
}

String fetchLatestVersion() {
  WiFiClientSecure client;
  client.setInsecure(); // Skip certificate validation for GitHub

  HTTPClient http;
  if (!http.begin(client, versionUrl)) {
    Serial.println("Unable to connect to Version URL");
    return "";
  }

  int httpCode = http.GET();
  String latestVersion = "";

  if (httpCode == HTTP_CODE_OK) {
    latestVersion = http.getString();
    latestVersion.trim();
  } else {
    Serial.printf("Failed to fetch version. HTTP code: %d\n", httpCode);
  }

  http.end();
  return latestVersion;
}

void downloadAndApplyFirmware() {
  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  if (!http.begin(client, firmwareUrl)) {
    Serial.println("Unable to connect to Firmware URL");
    return;
  }

  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    int contentLength = http.getSize();
    if (contentLength > 0) {
      WiFiClient *stream = http.getStreamPtr();
      if (startOTAUpdate(stream, contentLength)) {
        Serial.println("OTA Success! Restarting...");
        display.clearDisplay();
        display.println("Update Success!");
        display.println("Restarting...");
        display.display();
        delay(1000);
        ESP.restart();
      } else {
        Serial.println("OTA Failed.");
      }
    }
  }
  http.end();
}

bool startOTAUpdate(WiFiClient *client, int contentLength) {
  if (!Update.begin(contentLength))
    return false;

  size_t written = 0;
  uint8_t buffer[1024];
  unsigned long lastDataTime = millis();

  while (written < contentLength) {
    if (client->available()) {
      int bytesRead = client->read(buffer, sizeof(buffer));
      if (bytesRead > 0) {
        Update.write(buffer, bytesRead);
        written += bytesRead;
        lastDataTime = millis();
      }
    }
    if (millis() - lastDataTime > 20000) {
      Update.abort();
      return false;
    }
    yield();
  }
  return Update.end();
}