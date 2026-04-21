/*
  ESP32 line follower for 5 IR sensors + 2 DC motors + one START/STOP button.
  - Button toggles robot mode between STOP and DRIVE.
  - PID/PD control is kept and optimized for speed and stability.
  - Uses ESP32 LEDC PWM for motor speed control.
*/

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
  stopMotors();
}

void loop() {
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