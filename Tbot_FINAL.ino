// ==========================================================
//  TB Final Project - All Tasks 1–7
//  Hardware + low-level borrowed from Lab A/B
//  Logic reorganized into simple modes
//  Kyle Arquilla
// ==========================================================

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL53L0X.h>
#include "MPU6050.h"               // you used this in the labs
#include "SimpleDeadReckoning.h"   // same as Lab B
#include <math.h>

// ==========================================================
//  PIN DEFINES (from Lab A/B)
// ==========================================================

// IR sensors
#define IRD1 D2
#define IRD2 A1

// I2C
#define SDA_PIN A4
#define SCL_PIN A5

// Motor Driver
#define MTA1 A6
#define MTA2 A7
#define MTB1 D8
#define MTB2 D9

// Encoder Pins
#define MEA1 D10
#define MEA2 D11
#define MEB1 D12
#define MEB2 D13

// Onboard LEDs
#define LED1 D6
#define LED2 D7

// Servo pins (not used here, but defined for completeness)
#define SERVO1 A2
#define SERVO2 A3
#define SERVO3 D3
#define SERVO4 D4

// ==========================================================
//  OLED Display (same style as Lab B, but we actually use it)
// ==========================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Simple OLED print helper
void oledPrint(const String &l1, const String &l2 = "", const String &l3 = "") {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.println(l1);
  if (l2.length()) display.println(l2);
  if (l3.length()) display.println(l3);
  display.display();
}

// ==========================================================
//  Motor PWM (LEDC) – from Lab B
// ==========================================================
const int pwmFreq       = 500;
const int pwmResolution = 10;
const int pwmA1_Ch      = 0;
const int pwmA2_Ch      = 1;
const int pwmB1_Ch      = 2;
const int pwmB2_Ch      = 3;

// ==========================================================
//  Encoder & Dead Reckoning – from Lab B
// ==========================================================
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;
volatile uint8_t prevStateA = 0, prevStateB = 0;

const int   ticksPerRev = 12 * 4;  // 12 ticks per channel, 4x decoding
const float gearRatio   = 30.0;    // from Lab B

// SimpleDeadReckoning constructor from Lab B:
// (ticksPerRev*gearRatio, wheelDiameter_mm?, axle_length_cm?)
// We just reuse it exactly as in the lab.
SimpleDeadReckoning odom(ticksPerRev * gearRatio, 3.39f, 15.5f);

// Quad encoder read macros from Lab B
#define READ_ENC_A() ((digitalRead(MEA1) << 1) | digitalRead(MEA2))
#define READ_ENC_B() ((digitalRead(MEB1) << 1) | digitalRead(MEB2))

// ==========================================================
//  Wi-Fi / UDP (same credentials + port as Lab B)
// ==========================================================
const char* ssid     = "Pretty fly or A WIFI";
const char* password = "Spooky091993";

WiFiUDP udp;
const int UDP_PORT = 9000;
char incomingBuf[64];

IPAddress pcIP;
uint16_t pcPort = 0;
String lastCmd  = "";

// Pose + lidar send helpers for PC
void sendPoseUDP(float x, float y, float thetaDeg) {
  if (pcPort == 0) return;
  String msg = "P," + String(x, 3) + "," + String(y, 3) + "," + String(thetaDeg, 3);
  udp.beginPacket(pcIP, pcPort);
  udp.print(msg);
  udp.endPacket();
}

void sendLidarUDP(float xw, float yw) {
  if (pcPort == 0) return;
  String msg = "L," + String(xw, 3) + "," + String(yw, 3);
  udp.beginPacket(pcIP, pcPort);
  udp.print(msg);
  udp.endPacket();
}

// ==========================================================
//  IMU + Lidar
// ==========================================================
MPU6050 imu;              // not heavily used; odom handles heading
Adafruit_VL53L0X lox;

// ==========================================================
//  Motion LEDs (from Lab B)
// ==========================================================
void updateMotionLEDs(bool moving) {
  static unsigned long lastToggle = 0;
  static bool ledState = false;
  unsigned long now = millis();

  if (moving) {
    if (now - lastToggle >= 200UL) {
      lastToggle = now;
      ledState = !ledState;
      digitalWrite(LED1, ledState);
      digitalWrite(LED2, ledState);
    }
  } else {
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
  }
}

// ==========================================================
//  Motor helpers (from Lab B, slightly trimmed)
// ==========================================================
void setupMotorPWM() {
  ledcSetup(pwmA1_Ch, pwmFreq, pwmResolution);
  ledcSetup(pwmA2_Ch, pwmFreq, pwmResolution);
  ledcSetup(pwmB1_Ch, pwmFreq, pwmResolution);
  ledcSetup(pwmB2_Ch, pwmFreq, pwmResolution);

  ledcAttachPin(MTA1, pwmA1_Ch);
  ledcAttachPin(MTA2, pwmA2_Ch);
  ledcAttachPin(MTB1, pwmB1_Ch);
  ledcAttachPin(MTB2, pwmB2_Ch);
}

void setMotorA(int pwm) {
  pwm = constrain(pwm, -1023, 1023);
  if (pwm >= 0) {
    ledcWrite(pwmA1_Ch, pwm);
    ledcWrite(pwmA2_Ch, 0);
  } else {
    ledcWrite(pwmA1_Ch, 0);
    ledcWrite(pwmA2_Ch, -pwm);
  }
}

void setMotorB(int pwm) {
  pwm = constrain(pwm, -1023, 1023);
  if (pwm >= 0) {
    ledcWrite(pwmB1_Ch, 0);
    ledcWrite(pwmB2_Ch, pwm);
  } else {
    ledcWrite(pwmB1_Ch, -pwm);
    ledcWrite(pwmB2_Ch, 0);
  }
}

void stopMotors() {
  setMotorA(0);
  setMotorB(0);
  updateMotionLEDs(false);
}

// ==========================================================
//  Encoders – EXACT ISR logic from Lab B
// ==========================================================
void IRAM_ATTR handleEncoderA() {
  uint8_t state = READ_ENC_A();
  uint8_t combo = (prevStateA << 2) | state;
  switch (combo) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      encoderCountA++;
      break;
    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:
      encoderCountA--;
      break;
  }
  prevStateA = state;
}

void IRAM_ATTR handleEncoderB() {
  uint8_t state = READ_ENC_B();
  uint8_t combo = (prevStateB << 2) | state;
  switch (combo) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      encoderCountB++;
      break;
    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:
      encoderCountB--;
      break;
  }
  prevStateB = state;
}

void setupEncoderInterrupt() {
  prevStateA = READ_ENC_A();
  prevStateB = READ_ENC_B();
  attachInterrupt(digitalPinToInterrupt(MEA1), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MEA2), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MEB1), handleEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MEB2), handleEncoderB, CHANGE);
}

// ==========================================================
//  Odometry + Velocity (built on top of your Lab B style)
// ==========================================================
float robotV = 0.0f;   // linear m/s
float robotW = 0.0f;   // angular deg/s

float angleDiffDeg(float a, float b) {
  float d = a - b;
  while (d > 180.0f)  d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

void updateOdometryAndVelocity() {
  static unsigned long lastT = 0;
  static float lastX = 0, lastY = 0, lastTh = 0;

  // Convert encoder counts to ticks (same as Q2 in Lab B)
  float leftTicks  = -(float)encoderCountA;  // A negative forward
  float rightTicks =  (float)encoderCountB;  // B positive forward

  // In Lab B Q2 you used headingDeg = 0.0 because you weren’t using IMU yet.
  float headingDeg = 0.0f;
  odom.updateLocation(leftTicks, rightTicks, headingDeg);

  unsigned long now = millis();
  if (lastT == 0) {
    lastT = now;
    lastX = odom.getXLocation();
    lastY = odom.getYLocation();
    lastTh = odom.getTheta();
    robotV = 0;
    robotW = 0;
    return;
  }

  float dt = (now - lastT) / 1000.0f;
  if (dt <= 0) return;

  float x = odom.getXLocation();
  float y = odom.getYLocation();
  float th = odom.getTheta();  // library's heading; we’ll treat as degrees

  float dx   = x - lastX;
  float dy   = y - lastY;
  float dth  = angleDiffDeg(th, lastTh);

  robotV = sqrtf(dx * dx + dy * dy) / dt;    // m/s
  robotW = dth / dt;                         // deg/s

  lastX = x; lastY = y; lastTh = th; lastT = now;
}

float distanceFrom(float x0, float y0) {
  float x = odom.getXLocation();
  float y = odom.getYLocation();
  float dx = x - x0;
  float dy = y - y0;
  return sqrtf(dx * dx + dy * dy);
}

// ==========================================================
//  Simple Line-Follow step (P controller from your LineFollow_A)
// ==========================================================
void lineFollowStep() {
  const int   PWM_MAX   = 1000;
  const int   PWM_MIN   = 150;
  const int   BASE      = 300;
  const float KP        = 1400.0f;
  const float SCALE     = 4000.0f;
  const float DEADBAND  = 0.03f;
  const int   ERR_SIGN  = -1;

  int Lraw = analogRead(IRD1);
  int Rraw = analogRead(IRD2);

  float e = (float)(Rraw - Lraw) / SCALE;  // + => more on right => turn right
  e *= (float)ERR_SIGN;
  if (fabs(e) < DEADBAND) e = 0.0f;

  float u = KP * e;

  float headroom = min((float)(PWM_MAX - BASE), (float)(BASE - PWM_MIN));
  if (u >  headroom) u =  headroom;
  if (u < -headroom) u = -headroom;

  int pwmLeft  = (int)(BASE + u);
  int pwmRight = (int)(BASE - u);

  if (pwmLeft  > PWM_MAX) pwmLeft  = PWM_MAX;
  if (pwmRight > PWM_MAX) pwmRight = PWM_MAX;
  if (pwmLeft  > 0 && pwmLeft  < PWM_MIN) pwmLeft  = PWM_MIN;
  if (pwmRight > 0 && pwmRight < PWM_MIN) pwmRight = PWM_MIN;
  if (pwmLeft  < 0) pwmLeft  = 0;
  if (pwmRight < 0) pwmRight = 0;

  setMotorA(pwmLeft);
  setMotorB(pwmRight);
  updateMotionLEDs(true);
}

// ==========================================================
//  Modes / Tasks
// ==========================================================
enum Mode {
  IDLE = 0,
  TASK1,
  TASK2,
  TASK3,
  TASK4,
  TASK5,
  TASK6,
  TASK7
};

Mode mode = IDLE;

// ==========================================================
//  UDP command handler
//  Commands:
//   "1".."7" = select task
//   "F","B","L","R","S" = manual drive in Task 6/7
// ==========================================================
void handleNetwork() {
  int packetSize = udp.parsePacket();
  if (packetSize <= 0) return;

  int len = udp.read(incomingBuf, sizeof(incomingBuf) - 1);
  if (len <= 0) return;
  incomingBuf[len] = 0;

  String msg = String(incomingBuf);
  msg.trim();

  pcIP   = udp.remoteIP();
  pcPort = udp.remotePort();
  lastCmd = msg;

  if      (msg == "1") mode = TASK1;
  else if (msg == "2") mode = TASK2;
  else if (msg == "3") mode = TASK3;
  else if (msg == "4") mode = TASK4;
  else if (msg == "5") mode = TASK5;
  else if (msg == "6") mode = TASK6;
  else if (msg == "7") mode = TASK7;

  // Manual control only in Task 6/7
  if (mode == TASK6 || mode == TASK7) {
    if      (msg == "F") { setMotorA(400);  setMotorB(400);  updateMotionLEDs(true); }
    else if (msg == "B") { setMotorA(-400); setMotorB(-400); updateMotionLEDs(true); }
    else if (msg == "L") { setMotorA(-300); setMotorB(300);  updateMotionLEDs(true); }
    else if (msg == "R") { setMotorA(300);  setMotorB(-300); updateMotionLEDs(true); }
    else if (msg == "S") { stopMotors(); }
  }
}

// ==========================================================
//  Task 1 – Straight 80cm
//  OLED: x, y, theta, v, w
// ==========================================================
void task1_run() {
  static bool init = false;
  static float x0, y0;

  if (!init) {
    encoderCountA = 0;
    encoderCountB = 0;
    odom.setXLocation(0.0f);
    odom.setYLocation(0.0f);
    init = true;
  }

  float dist = distanceFrom(0.0f, 0.0f);
  float x    = odom.getXLocation();
  float y    = odom.getYLocation();
  float th   = odom.getTheta();

  // drive straight
  setMotorA(400);
  setMotorB(400);
  updateMotionLEDs(true);

  String l1 = "T1 x=" + String(x, 2) + " y=" + String(y, 2);
  String l2 = "th=" + String(th, 1) + " deg";
  String l3 = "v=" + String(robotV, 2) + " w=" + String(robotW, 1);
  oledPrint(l1, l2, l3);
  sendPoseUDP(x, y, th);

  if (dist >= 0.80f) {
    stopMotors();
    init = false;
    mode = IDLE;
  }
}

// ==========================================================
//  Helper: timed 90°-ish turn (you can tune the constant)
// ==========================================================
void timedTurn(float dir) {
  // dir = +1 left, -1 right
  const unsigned long TURN_MS = 900;   // tune to ~90 deg on your robot
  unsigned long start = millis();
  while (millis() - start < TURN_MS) {
    setMotorA(-300 * dir);
    setMotorB(300 * dir);
    updateMotionLEDs(true);
    updateOdometryAndVelocity();
    handleNetwork();   // still responsive
    delay(10);
  }
  stopMotors();
}

// ==========================================================
//  Task 2 – Uni-direction 40cm square
//  OLED: heading (theta)
// ==========================================================
void task2_run() {
  static bool init = false;
  static int side  = 0;
  static float sx, sy;

  if (!init) {
    encoderCountA = 0;
    encoderCountB = 0;
    odom.setXLocation(0.0f);
    odom.setYLocation(0.0f);
    side = 0;
    sx = 0.0f;
    sy = 0.0f;
    init = true;
  }

  if (side >= 4) {
    stopMotors();
    init = false;
    mode = IDLE;
    return;
  }

  float d = distanceFrom(sx, sy);
  if (d < 0.40f) {
    setMotorA(350);
    setMotorB(350);
    updateMotionLEDs(true);

    float th = odom.getTheta();
    String l1 = "T2 side " + String(side + 1);
    String l2 = "theta=" + String(th, 1) + " deg";
    oledPrint(l1, l2);
  } else {
    stopMotors();
    delay(200);
    timedTurn(+1);
    sx = odom.getXLocation();
    sy = odom.getYLocation();
    side++;
  }
}

// ==========================================================
//  Task 3 – Bi-direction square, show v & w in real-time
// ==========================================================
void task3_run() {
  static bool init = false;
  static int phase = 0;  // 0..3 forward sides, 4..7 backward sides
  static float sx, sy;

  if (!init) {
    encoderCountA = 0;
    encoderCountB = 0;
    odom.setXLocation(0.0f);
    odom.setYLocation(0.0f);
    phase = 0;
    sx = 0.0f;
    sy = 0.0f;
    init = true;
  }

  if (phase >= 8) {
    stopMotors();
    init = false;
    mode = IDLE;
    return;
  }

  bool forward = (phase < 4);
  float d = distanceFrom(sx, sy);

  if (d < 0.40f) {
    int pwm = forward ? 350 : -350;
    setMotorA(pwm);
    setMotorB(pwm);
    updateMotionLEDs(true);

    float th = odom.getTheta();
    String l1 = String("T3 ") + (forward ? "F" : "B") + " side " + String((phase % 4) + 1);
    String l2 = "v=" + String(robotV, 2) + " m/s";
    String l3 = "w=" + String(robotW, 1) + " deg/s";
    oledPrint(l1, l2, l3);
  } else {
    stopMotors();
    delay(200);
    timedTurn(forward ? +1.0f : -1.0f);
    sx = odom.getXLocation();
    sy = odom.getYLocation();
    phase++;
  }
}

// ==========================================================
//  Task 4 – Line-follow square, OLED: X,Y
// ==========================================================
void task4_run() {
  static bool init = false;
  static float sx, sy;

  if (!init) {
    encoderCountA = 0;
    encoderCountB = 0;
    odom.setXLocation(0.0f);
    odom.setYLocation(0.0f);
    sx = 0.0f;
    sy = 0.0f;
    init = true;
  }

  lineFollowStep();
  float x = odom.getXLocation();
  float y = odom.getYLocation();
  oledPrint("T4 LineFollow", "X=" + String(x, 2), "Y=" + String(y, 2));

  // "back near start" + basically slow = stop
  if (distanceFrom(sx, sy) < 0.10f && robotV < 0.02f && millis() > 5000) {
    stopMotors();
    init = false;
    mode = IDLE;
  }
}

// ==========================================================
//  Task 5 – Line-follow + PC path drawing
//  Same as Task 4 but sends pose to PC
// ==========================================================
void task5_run() {
  static bool init = false;
  static float sx, sy;

  if (!init) {
    encoderCountA = 0;
    encoderCountB = 0;
    odom.setXLocation(0.0f);
    odom.setYLocation(0.0f);
    sx = 0.0f;
    sy = 0.0f;
    init = true;
  }

  lineFollowStep();
  float x = odom.getXLocation();
  float y = odom.getYLocation();
  float th = odom.getTheta();
  oledPrint("T5 Line+PC", "X=" + String(x,2), "Y=" + String(y,2));
  sendPoseUDP(x, y, th);

  if (distanceFrom(sx, sy) < 0.10f && robotV < 0.02f && millis() > 5000) {
    stopMotors();
    init = false;
    mode = IDLE;
  }
}

// ==========================================================
//  Task 6 – Manual letters
//  PC sends F/L/R/B/S, robot streams pose
// ==========================================================
void task6_run() {
  float x = odom.getXLocation();
  float y = odom.getYLocation();
  float th = odom.getTheta();
  sendPoseUDP(x, y, th);

  oledPrint("T6 Manual", "CMD:" + lastCmd, "PC:" + pcIP.toString());
}

// ==========================================================
//  Task 7 – Manual + lidar OGM
// ==========================================================
void task7_run() {
  float x = odom.getXLocation();
  float y = odom.getYLocation();
  float th = odom.getTheta();
  sendPoseUDP(x, y, th);

  // single forward lidar sample, same as your style
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus == 0) {
    float r = measure.RangeMilliMeter / 1000.0f;
    float thRad = th * PI / 180.0f;
    float xw = x + r * cosf(thRad);
    float yw = y + r * sinf(thRad);
    sendLidarUDP(xw, yw);
  }

  oledPrint("T7 OGM", "CMD:" + lastCmd, "PC:" + pcIP.toString());
}

// ==========================================================
//  SETUP
// ==========================================================
void setup() {
  Serial.begin(115200);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);

  pinMode(IRD1, INPUT);
  pinMode(IRD2, INPUT);

  pinMode(MEA1, INPUT);
  pinMode(MEA2, INPUT);
  pinMode(MEB1, INPUT);
  pinMode(MEB2, INPUT);

  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);

  Wire.begin(SDA_PIN, SCL_PIN);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  setupMotorPWM();
  setMotorA(0);
  setMotorB(0);
  updateMotionLEDs(false);

  setupEncoderInterrupt();

  // WiFi
  WiFi.begin(ssid, password);
  oledPrint("WiFi...", "", "");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
  }
  udp.begin(UDP_PORT);
  oledPrint("WiFi OK", WiFi.localIP().toString(), "UDP:9000");

  // IMU (we don't rely on it for odom, but it can be used if needed)
  imu.initialize();

  // Lidar
  if (!lox.begin()) {
    oledPrint("Lidar FAIL", "", "");
  } else {
    oledPrint("Lidar OK", "", "");
  }

  // Reset odom
  encoderCountA = 0;
  encoderCountB = 0;
  odom.setXLocation(0.0f);
  odom.setYLocation(0.0f);
}

// ==========================================================
//  LOOP
// ==========================================================
void loop() {
  handleNetwork();
  updateOdometryAndVelocity();

  switch (mode) {
    case TASK1: task1_run(); break;
    case TASK2: task2_run(); break;
    case TASK3: task3_run(); break;
    case TASK4: task4_run(); break;
    case TASK5: task5_run(); break;
    case TASK6: task6_run(); break;
    case TASK7: task7_run(); break;
    case IDLE:
    default:
      stopMotors();
      oledPrint("IDLE","Send 1-7 via UDP","");
      break;
  }

  delay(10);
}
