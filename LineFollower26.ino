/*
  ESP32 line follower for 5 IR sensors + 2 DC motors.
  - Robot mode is controlled from web UI (STOP/DRIVE/MAX TEST).
  - PID/PD control is kept and optimized for speed and stability.
  - Uses ESP32 LEDC PWM for motor speed control.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <Preferences.h>

// ESP32 motor pins (L298N/L293 style H-bridge)
// Change only if your wiring differs.
#define RMF 26  // Right motor forward
#define RMB 27  // Right motor backward
#define LMF 14  // Left motor forward
#define LMB 25  // Left motor backward
#define RMS 19  // Right motor PWM (LEDC)
#define LMS 18  // Left motor PWM (LEDC)

// Optional status LED
#define LED_STATUS 2


// 5 sensors from left to right (top view of robot)
const byte SENSOR_COUNT = 5;
const byte sensorPins[SENSOR_COUNT] = { 36, 39, 34, 35, 32 };

// Adjust these thresholds from your real readings.
int threshold[SENSOR_COUNT] = { 2000, 2000, 2000, 2000, 2000 };

// If black gives HIGH analog value, keep true. If inverse, set false.
const bool BLACK_IS_HIGH = true;

// Speed and PID tuning
const int MAX_SPEED = 255;
int baseSpeed = 255;
int maxDriveSpeed = 255;
int minDriveSpeed = -125;

const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;
float Kp = 0.15f;
float Ki = 0.002f;
float Kd = 1.60f;

// Left motor speed multiplier relative to right motor speed.
// 1.00 = no compensation, >1.00 boosts left motor, <1.00 reduces left motor.
float leftRightRatio = 0.80f;

// Wi-Fi AP + web settings
const char *AP_SSID = "Proton ETF";
const char *AP_PASSWORD = "PROTONETF3";
const char *OTA_HOSTNAME = "LineFollower26";
const char *OTA_PASSWORD = "PROTONETF3";

WebServer server(80);
Preferences prefs;
bool wifiApSecured = false;
String telemetryLine = "Robot spreman.";
unsigned long telemetrySeq = 0;

// Line position scale: 0 ... 4000 (center = 2000)
const int CENTER_POSITION = 2000;

int sensorRaw[SENSOR_COUNT];
byte sensorDigital[SENSOR_COUNT];

bool isDriveMode = false;
bool isMaxSpeedTestMode = false;

float error = 0.0f;
float previousError = 0.0f;
float integral = 0.0f;
int lastKnownLinePosition = CENTER_POSITION;

void startAccessPoint();
void setupOTA();
void setupWebServer();
void handleRoot();
void handleSave();
void handleSetMode();
void handleTelemetry();
void loadSettings();
void saveSettings();
void publishTelemetry(const String &turnLabel, const String &activeSensors, int linePosition, bool lineFound, int leftSpeed, int rightSpeed);
String buildActiveSensorList();
String classifyTurnLabel(int linePosition, bool lineFound);
String buildHtmlPage(const String &message);

void setup() {
  pinMode(LMF, OUTPUT);
  pinMode(LMB, OUTPUT);
  pinMode(RMF, OUTPUT);
  pinMode(RMB, OUTPUT);
  pinMode(LMS, OUTPUT);
  pinMode(RMS, OUTPUT);

  pinMode(LED_STATUS, OUTPUT);

  analogReadResolution(12);

  ledcAttach(LMS, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(RMS, PWM_FREQ, PWM_RESOLUTION);

  Serial.begin(115200);

  // Always boot in idle mode until an explicit start command is received.
  isDriveMode = false;
  isMaxSpeedTestMode = false;

  loadSettings();
  startAccessPoint();
  setupOTA();
  setupWebServer();
  stopMotors();
  publishTelemetry("idle", "nijedan", CENTER_POSITION, false, 0, 0);
}

void loop() {
  ArduinoOTA.handle();
  server.handleClient();

  if (!isDriveMode) {
    if (!isMaxSpeedTestMode) {
      stopMotors();
      digitalWrite(LED_STATUS, LOW);
      return;
    }

    int testSpeed = constrain(maxDriveSpeed, 0, MAX_SPEED);
    int leftTestSpeed = (int)((float)testSpeed * leftRightRatio);
    leftTestSpeed = constrain(leftTestSpeed, 0, testSpeed);
    setMotorSpeeds(leftTestSpeed, testSpeed);
    digitalWrite(LED_STATUS, HIGH);
    publishTelemetry("max speed test", "nijedan", CENTER_POSITION, true, leftTestSpeed, testSpeed);
    return;
  }

  if (isMaxSpeedTestMode) {
    stopMotors();
    digitalWrite(LED_STATUS, LOW);
    return;
  }

  digitalWrite(LED_STATUS, HIGH);
  followLineStep();
}

bool readLineSensors(int &linePositionOut) {
  long weightedSum = 0;
  int activeCount = 0;

  for (byte i = 0; i < SENSOR_COUNT; i++) {
    sensorRaw[i] = analogRead(sensorPins[i]);

    if (BLACK_IS_HIGH) {
      sensorDigital[i] = (sensorRaw[i] > threshold[i]) ? 1 : 0;
    } else {
      sensorDigital[i] = (sensorRaw[i] < threshold[i]) ? 1 : 0;
    }

    if (sensorDigital[i]) {
      weightedSum += (long)i * 1000L;
      activeCount++;
    }
  }

  if (activeCount > 0) {
    linePositionOut = (int)(weightedSum / activeCount);
    lastKnownLinePosition = linePositionOut;
    return true;
  }

  // If line is lost, continue turning toward the previous side.
  if (previousError >= 0) {
    linePositionOut = 4500;
  } else {
    linePositionOut = -500;
  }
  return false;
}

void startAccessPoint() {
  WiFi.mode(WIFI_AP);

  if (strlen(AP_PASSWORD) >= 8) {
    wifiApSecured = WiFi.softAP(AP_SSID, AP_PASSWORD);
  } else {
    wifiApSecured = false;
    WiFi.softAP(AP_SSID);
    Serial.println("[WiFi] AP password is shorter than 8 chars; started OPEN network.");
  }

  IPAddress ip = WiFi.softAPIP();
  Serial.print("[WiFi] AP SSID: ");
  Serial.println(AP_SSID);
  Serial.print("[WiFi] AP secure: ");
  Serial.println(wifiApSecured ? "YES" : "NO");
  Serial.print("[WiFi] AP IP: ");
  Serial.println(ip);
}

void setupOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    stopMotors();
    Serial.println("[OTA] Start");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("[OTA] End");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("[OTA] Progress: %u%%\r", (progress * 100U) / total);
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA] Error[%u]\n", error);
  });

  ArduinoOTA.begin();
  Serial.println("[OTA] Ready");
  Serial.print("[OTA] Hostname: ");
  Serial.println(OTA_HOSTNAME);
}

void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/mode", HTTP_POST, handleSetMode);
  server.on("/telemetry", HTTP_GET, handleTelemetry);
  server.onNotFound([]() {
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Redirecting...");
  });
  server.begin();
  Serial.println("[Web] Server started on port 80");
}

void handleRoot() {
  server.send(200, "text/html", buildHtmlPage(""));
}

void handleSetMode() {
  if (server.hasArg("drive")) {
    String modeArg = server.arg("drive");
    modeArg.toLowerCase();
    if (modeArg == "2" || modeArg == "max" || modeArg == "test") {
      isMaxSpeedTestMode = true;
      isDriveMode = false;
    } else {
      isDriveMode = (modeArg == "1" || modeArg == "on" || modeArg == "true");
      isMaxSpeedTestMode = false;
    }
  }

  if (isMaxSpeedTestMode) {
    int testSpeed = constrain(maxDriveSpeed, 0, MAX_SPEED);
    int leftTestSpeed = (int)((float)testSpeed * leftRightRatio);
    leftTestSpeed = constrain(leftTestSpeed, 0, testSpeed);
    publishTelemetry("max speed test", "nijedan", CENTER_POSITION, true, leftTestSpeed, testSpeed);
  } else if (!isDriveMode) {
    stopMotors();
    publishTelemetry("stop", "nijedan", CENTER_POSITION, false, 0, 0);
  } else {
    publishTelemetry("drive mode", "nijedan", CENTER_POSITION, true, 0, 0);
  }

  integral = 0.0f;
  previousError = 0.0f;

  String modeMessage = "Drive mode disabled.";
  if (isMaxSpeedTestMode) {
    modeMessage = "MAX SPEED test mode enabled.";
  } else if (isDriveMode) {
    modeMessage = "Drive mode enabled.";
  }

  server.send(200, "text/html", buildHtmlPage(modeMessage));
}

void handleSave() {
  if (server.hasArg("kp")) Kp = server.arg("kp").toFloat();
  if (server.hasArg("ki")) Ki = server.arg("ki").toFloat();
  if (server.hasArg("kd")) Kd = server.arg("kd").toFloat();

  if (server.hasArg("leftRightRatio")) {
    leftRightRatio = server.arg("leftRightRatio").toFloat();
    leftRightRatio = constrain(leftRightRatio, 0.20f, 2.00f);
  }

  if (server.hasArg("baseSpeed")) baseSpeed = server.arg("baseSpeed").toInt();
  if (server.hasArg("maxDriveSpeed")) maxDriveSpeed = server.arg("maxDriveSpeed").toInt();
  if (server.hasArg("minDriveSpeed")) minDriveSpeed = server.arg("minDriveSpeed").toInt();

  integral = 0.0f;
  previousError = 0.0f;

  saveSettings();
  server.send(200, "text/html", buildHtmlPage("Saved successfully."));
}

void handleTelemetry() {
  server.send(200, "text/plain", String(telemetrySeq) + "|" + telemetryLine);
}

void loadSettings() {
  prefs.begin("lf-settings", true);
  Kp = prefs.getFloat("kp", Kp);
  Ki = prefs.getFloat("ki", Ki);
  Kd = prefs.getFloat("kd", Kd);
  leftRightRatio = prefs.getFloat("lrRatio", leftRightRatio);
  baseSpeed = prefs.getInt("baseSpeed", baseSpeed);
  maxDriveSpeed = prefs.getInt("maxSpeed", maxDriveSpeed);
  minDriveSpeed = prefs.getInt("minSpeed", minDriveSpeed);
  prefs.end();

  // Keep loaded values in a valid and flexible range.
  Kp = constrain(Kp, 0.05f, 2.00f);
  Ki = constrain(Ki, 0.0f, 0.05f);
  Kd = constrain(Kd, 0.20f, 6.00f);
  leftRightRatio = constrain(leftRightRatio, 0.20f, 2.00f);
  baseSpeed = constrain(baseSpeed, 0, MAX_SPEED);
  maxDriveSpeed = constrain(maxDriveSpeed, 0, MAX_SPEED);
  minDriveSpeed = constrain(minDriveSpeed, -MAX_SPEED, MAX_SPEED);
  if (minDriveSpeed > maxDriveSpeed) {
    minDriveSpeed = maxDriveSpeed;
  }
}

void saveSettings() {
  prefs.begin("lf-settings", false);
  prefs.putFloat("kp", Kp);
  prefs.putFloat("ki", Ki);
  prefs.putFloat("kd", Kd);
  prefs.putFloat("lrRatio", leftRightRatio);
  prefs.putInt("baseSpeed", baseSpeed);
  prefs.putInt("maxSpeed", maxDriveSpeed);
  prefs.putInt("minSpeed", minDriveSpeed);
  prefs.end();
}

String buildHtmlPage(const String &message) {
  String html;
  html.reserve(4200);
  html += "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>LineFollower PID Setup</title>";
  html += "<style>body{font-family:Arial,sans-serif;margin:24px;background:#f4f6f8;color:#1b1f24;}";
  html += ".card{max-width:520px;margin:auto;background:#fff;padding:18px;border-radius:12px;box-shadow:0 6px 20px rgba(0,0,0,.08);}";
  html += "h2{margin-top:0;}label{display:block;margin:10px 0 4px;}input{width:100%;padding:10px;font-size:16px;box-sizing:border-box;}";
  html += "button{margin-top:14px;padding:10px 14px;font-size:16px;cursor:pointer;}";
  html += ".ok{color:#0b7a24;font-weight:bold;margin-bottom:8px;} .meta{font-size:13px;color:#4a5560;margin-bottom:10px;}";
  html += ".controls{display:flex;gap:10px;flex-wrap:wrap;margin:12px 0 16px;}";
  html += ".controls form{margin:0;}";
  html += ".btn-drive{background:#0b7a24;color:#fff;border:0;border-radius:8px;}";
  html += ".btn-stop{background:#b42318;color:#fff;border:0;border-radius:8px;}";
  html += ".btn-max{background:#1f6feb;color:#fff;border:0;border-radius:8px;}";
  html += ".console{background:#0d1117;color:#8bffb0;border-radius:10px;padding:12px;height:190px;overflow:auto;";
  html += "font-family:Consolas,monospace;font-size:13px;line-height:1.45;white-space:pre-wrap;border:1px solid #1f2937;margin:10px 0 16px;}";
  html += ".console .line{margin:0 0 4px;}";
  html += "</style></head><body><div class='card'>";
  html += "<h2>ESP32 PID and Speed Settings</h2>";

  if (message.length() > 0) {
    html += "<div class='ok'>" + message + "</div>";
  }

  html += "<div class='meta'>SSID: ";
  html += AP_SSID;
  html += " | AP IP: ";
  html += WiFi.softAPIP().toString();
  html += " | Mode: ";
  html += isMaxSpeedTestMode ? "MAX TEST" : (isDriveMode ? "DRIVE" : "STOP");
  html += "</div>";

  html += "<div class='meta'>OTA hostname: ";
  html += OTA_HOSTNAME;
  html += " | OTA port: 3232</div>";

  html += "<div class='controls'>";
  html += "<form method='POST' action='/mode'><input type='hidden' name='drive' value='1'><button class='btn-drive' type='submit'>Start Robot</button></form>";
  html += "<form method='POST' action='/mode'><input type='hidden' name='drive' value='max'><button class='btn-max' type='submit'>MAX SPEED TEST</button></form>";
  html += "<form method='POST' action='/mode'><input type='hidden' name='drive' value='0'><button class='btn-stop' type='submit'>Stop Robot</button></form>";
  html += "</div>";

  html += "<h3>Web konzola</h3><div id='console' class='console'></div>";
  html += "<script>let lastSeq=-1;const box=document.getElementById('console');";
  html += "function addLine(text){const row=document.createElement('div');row.className='line';row.textContent=text;box.appendChild(row);box.scrollTop=box.scrollHeight;}";
  html += "function pollTelemetry(){fetch('/telemetry',{cache:'no-store'}).then(r=>r.text()).then(t=>{const split=t.indexOf('|');if(split<0)return;const seq=parseInt(t.slice(0,split),10);const msg=t.slice(split+1);if(!Number.isNaN(seq)&&seq!==lastSeq){lastSeq=seq;addLine(msg);}}).catch(()=>{});}setInterval(pollTelemetry,250);pollTelemetry();</script>";

  html += "<form method='POST' action='/save'>";
  html += "<label>Kp</label><input type='text' name='kp' value='" + String(Kp, 6) + "'>";
  html += "<label>Ki</label><input type='text' name='ki' value='" + String(Ki, 6) + "'>";
  html += "<label>Kd</label><input type='text' name='kd' value='" + String(Kd, 6) + "'>";
  html += "<label>leftRightRatio (left/right)</label><input type='text' name='leftRightRatio' value='" + String(leftRightRatio, 4) + "'>";

  html += "<label>baseSpeed</label><input type='text' name='baseSpeed' value='" + String(baseSpeed) + "'>";
  html += "<label>maxDriveSpeed</label><input type='text' name='maxDriveSpeed' value='" + String(maxDriveSpeed) + "'>";
  html += "<label>minDriveSpeed</label><input type='text' name='minDriveSpeed' value='" + String(minDriveSpeed) + "'>";

  html += "<button type='submit'>Save</button></form>";
  html += "</div></body></html>";
  return html;
}

void followLineStep() {
  int linePosition;
  bool lineFound = readLineSensors(linePosition);

  error = (float)(linePosition - CENTER_POSITION);
  float derivative = error - previousError;

  if (lineFound) {
    integral += error;
    if (integral > 5000) integral = 5000;
    if (integral < -5000) integral = -5000;
  } else {
    integral *= 0.8f;
  }

  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  previousError = error;

  int dynamicBase = baseSpeed;
  if (abs((int)error) > 1200) {
    dynamicBase = baseSpeed - 70;
  } else if (abs((int)error) > 700) {
    dynamicBase = baseSpeed - 40;
  }

  int leftSpeed = dynamicBase + (int)correction;
  int rightSpeed = dynamicBase - (int)correction;

  // Software compensation for mechanical mismatch between motors.
  leftSpeed = (int)((float)leftSpeed * leftRightRatio);

  leftSpeed = constrain(leftSpeed, minDriveSpeed, maxDriveSpeed);
  rightSpeed = constrain(rightSpeed, minDriveSpeed, maxDriveSpeed);

  setMotorSpeeds(leftSpeed, rightSpeed);

  String turnLabel = classifyTurnLabel(linePosition, lineFound);
  String activeSensors = buildActiveSensorList();
  publishTelemetry(turnLabel, activeSensors, linePosition, lineFound, leftSpeed, rightSpeed);
}

void setMotorSpeeds(int left, int right) {
  if (left >= 0) {
    digitalWrite(LMF, HIGH);
    digitalWrite(LMB, LOW);
  } else {
    digitalWrite(LMF, LOW);
    digitalWrite(LMB, HIGH);
    left = -left;
  }

  if (right >= 0) {
    digitalWrite(RMF, HIGH);
    digitalWrite(RMB, LOW);
  } else {
    digitalWrite(RMF, LOW);
    digitalWrite(RMB, HIGH);
    right = -right;
  }

  left = constrain(left, 0, MAX_SPEED);
  right = constrain(right, 0, MAX_SPEED);
  ledcWrite(LMS, left);
  ledcWrite(RMS, right);
}

void stopMotors() {
  ledcWrite(LMS, 0);
  ledcWrite(RMS, 0);
  digitalWrite(LMF, LOW);
  digitalWrite(LMB, LOW);
  digitalWrite(RMF, LOW);
  digitalWrite(RMB, LOW);
}

void publishTelemetry(const String &turnLabel, const String &activeSensors, int linePosition, bool lineFound, int leftSpeed, int rightSpeed) {
  telemetrySeq++;
  telemetryLine = String("#") + String(telemetrySeq) + " " + turnLabel + " | senzori: " + activeSensors +
                  " | pozicija: " + String(linePosition) +
                  " | levo: " + String(leftSpeed) +
                  " | desno: " + String(rightSpeed) +
                  (lineFound ? " | linija: DETECTED" : " | linija: LOST");
}

String buildActiveSensorList() {
  static const char *sensorNames[SENSOR_COUNT] = {
    "ostro-levo",
    "levo",
    "pravo",
    "desno",
    "ostro-desno"
  };

  String result;
  for (byte i = 0; i < SENSOR_COUNT; i++) {
    if (sensorDigital[i]) {
      if (result.length() > 0) {
        result += ", ";
      }
      result += sensorNames[i];
    }
  }

  if (result.length() == 0) {
    result = "nijedan";
  }

  return result;
}

String classifyTurnLabel(int linePosition, bool lineFound) {
  if (!lineFound) {
    return "linija izgubljena";
  }

  int offset = linePosition - CENTER_POSITION;
  if (offset <= -1200) return "ostro-levo";
  if (offset <= -400) return "levo";
  if (offset < 400) return "pravo";
  if (offset < 1200) return "desno";
  return "ostro-desno";
}