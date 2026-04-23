/*
  ESP32 line follower for 5 IR sensors + 2 DC motors + one START/STOP button.
  - Button toggles robot mode between STOP and DRIVE.
  - PID/PD control is kept and optimized for speed and stability.
  - Uses ESP32 LEDC PWM for motor speed control.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// ESP32 motor pins (L298N/L293 style H-bridge)
// Change only if your wiring differs.
#define RMF 26  // Right motor forward
#define RMB 27  // Right motor backward
#define LMF 14  // Left motor forward
#define LMB 25  // Left motor backward
#define RMS 19  // Right motor PWM (LEDC)
#define LMS 18  // Left motor PWM (LEDC)

// Single toggle button (INPUT_PULLUP)
#define BTN_MODE 23

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
int baseSpeed = 220;
int maxDriveSpeed = 255;
int minDriveSpeed = -170;

const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;
float Kp = 0.22f;
float Ki = 0.000f;
float Kd = 2.60f;

// Left motor speed multiplier relative to right motor speed.
// 1.00 = no compensation, >1.00 boosts left motor, <1.00 reduces left motor.
float leftRightRatio = 1.00f;

// Wi-Fi AP + web settings
const char *AP_SSID = "Proton ETF";
const char *AP_PASSWORD = "PROTONETF3";

WebServer server(80);
Preferences prefs;
bool wifiApSecured = false;

// Line position scale: 0 ... 4000 (center = 2000)
const int CENTER_POSITION = 2000;

int sensorRaw[SENSOR_COUNT];
byte sensorDigital[SENSOR_COUNT];

bool isDriveMode = false;
bool lastButtonRead = HIGH;
bool buttonStableState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceMs = 35;

float error = 0.0f;
float previousError = 0.0f;
float integral = 0.0f;
int lastKnownLinePosition = CENTER_POSITION;

void startAccessPoint();
void setupWebServer();
void handleRoot();
void handleSave();
void handleSetMode();
void loadSettings();
void saveSettings();
String buildHtmlPage(const String &message);

void setup() {
  pinMode(LMF, OUTPUT);
  pinMode(LMB, OUTPUT);
  pinMode(RMF, OUTPUT);
  pinMode(RMB, OUTPUT);
  pinMode(LMS, OUTPUT);
  pinMode(RMS, OUTPUT);

  pinMode(BTN_MODE, INPUT_PULLUP);
  pinMode(LED_STATUS, OUTPUT);

  analogReadResolution(12);

  ledcAttach(LMS, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(RMS, PWM_FREQ, PWM_RESOLUTION);

  Serial.begin(115200);
  loadSettings();
  startAccessPoint();
  setupWebServer();
  stopMotors();
}

void loop() {
  server.handleClient();
  updateModeButton();

  if (!isDriveMode) {
    stopMotors();
    digitalWrite(LED_STATUS, LOW);
    return;
  }

  digitalWrite(LED_STATUS, HIGH);
  followLineStep();
}

void updateModeButton() {
  bool readNow = digitalRead(BTN_MODE);

  if (readNow != lastButtonRead) {
    lastDebounceTime = millis();
    lastButtonRead = readNow;
  }

  if ((millis() - lastDebounceTime) > debounceMs && readNow != buttonStableState) {
    buttonStableState = readNow;

    // Toggle mode on press (LOW because INPUT_PULLUP)
    if (buttonStableState == LOW) {
      isDriveMode = !isDriveMode;
      if (!isDriveMode) {
        stopMotors();
      }
    }
  }
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

void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/mode", HTTP_POST, handleSetMode);
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
    isDriveMode = (modeArg == "1" || modeArg == "on" || modeArg == "true");
  }

  if (!isDriveMode) {
    stopMotors();
  }

  integral = 0.0f;
  previousError = 0.0f;

  server.send(200, "text/html", buildHtmlPage(isDriveMode ? "Drive mode enabled." : "Drive mode disabled."));
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
  html.reserve(2400);
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
  html += isDriveMode ? "DRIVE" : "STOP";
  html += "</div>";

  html += "<div class='controls'>";
  html += "<form method='POST' action='/mode'><input type='hidden' name='drive' value='1'><button class='btn-drive' type='submit'>Start Robot</button></form>";
  html += "<form method='POST' action='/mode'><input type='hidden' name='drive' value='0'><button class='btn-stop' type='submit'>Stop Robot</button></form>";
  html += "</div>";

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