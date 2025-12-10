// ==========================================================
//  TB Final Project - All Tasks 1–7
//  Hardware + low-level from Lab A/B
//  Manual mode (Task 6/7) updated so L/R make CURVED motion
//  Task 2: encoder-based 40cm sides + IMU-based 90deg turns
//  Kyle Arquilla
// ==========================================================

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL53L0X.h>
#include "MPU6050.h"
#include "SimpleDeadReckoning.h"
#include <math.h>
#include <driver/adc.h>

// ==========================================================
//  PIN DEFINES (from Lab A/B)
// ==========================================================

// IR sensors
#define IRD1 D2
#define IRD2 A1

// I2C2
#define SDA_PIN A4
#define SCL_PIN A5

// Motor Driver
#define MTA1 A6
#define MTA2 A7
#define MTB1 D8
#define MTB2 D9

// Encoders
#define MEA1 D10
#define MEA2 D11
#define MEB1 D12
#define MEB2 D13

// LEDs
#define LED1 D6
#define LED2 D7

// Servo pins (not used, but defined)
#define SERVO1 A2
#define SERVO2 A3
#define SERVO3 D3
#define SERVO4 D4

// ==========================================================
//  OLED
// ==========================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void oledPrint(const String lines[], int count) {
  display.clearDisplay();
  display.setRotation(2);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  for (int i = 0; i < count; i++) {
    display.println(lines[i]);   // supports \n inside
  }

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

const int   ticksPerRev = 12 * 4;
const float gearRatio   = 30.0;

// NOTE: using 1.0 (cm) radius and 5.5 (cm) wheel distance => odom in cm
SimpleDeadReckoning odom(ticksPerRev * gearRatio, 0.6f, 8.0f);

// Quad encoder macros
#define READ_ENC_A() ((digitalRead(MEA1) << 1) | digitalRead(MEA2))
#define READ_ENC_B() ((digitalRead(MEB1) << 1) | digitalRead(MEB2))

// ==========================================================
//  Wi-Fi / UDP
// ==========================================================
const char* ssid     = "Pretty fly or A WIFI";
const char* password = "Spooky091993";

WiFiUDP udp;
const int UDP_PORT = 9000;
char incomingBuf[64];

IPAddress pcIP;
uint16_t pcPort = 0;
String lastCmd  = "";

// Pose + lidar send
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
MPU6050 imu;
Adafruit_VL53L0X lox;

// ==========================================================
//  IMU yaw integration (gyro Z) for 90° turns
// ==========================================================
float imuYawDeg   = 0.0f;
float gyroZBias   = 0.0f;
bool  imuYawReady = false;
unsigned long imuLastMs = 0;

// Simple bias calibration at rest (call once in setup)
void calibrateGyroZ(int samples = 500) {
  int16_t ax, ay, az, gx, gy, gz;
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(2);
  }
  gyroZBias = (float)sum / (float)samples;
}

// Reset yaw integration to 0°
void resetIMUYaw() {
  imuYawDeg   = 0.0f;
  imuYawReady = false;
}

// Update yaw integration; call frequently while turning
void updateIMUYaw() {
  int16_t ax, ay, az, gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  unsigned long now = millis();
  if (!imuYawReady) {
    imuLastMs   = now;
    imuYawReady = true;
    return;
  }

  float dt = (now - imuLastMs) / 1000.0f;
  imuLastMs = now;

  // Assuming default ±250 dps: 131 LSB/(deg/s)
  float gz_dps = ((float)gz - gyroZBias) / 131.0f;

  imuYawDeg += gz_dps * dt;

  // Optional wrap; not strictly needed for single 90° turn
  if (imuYawDeg > 180.0f)  imuYawDeg -= 360.0f;
  if (imuYawDeg < -180.0f) imuYawDeg += 360.0f;
}

// ==========================================================
//  Motion LEDs
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
//  Motor helpers
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
//  Encoders – ISRs
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
//  Odometry + Velocity
// ==========================================================
float robotV = 0.0f;   // "cm/s" with current odom config
float robotW = 0.0f;   // deg/s

float angleDiffDeg(float a, float b) {
  float d = a - b;
  while (d > 180.0f)  d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

void updateOdometryAndVelocity() {
  static unsigned long lastT = 0;
  static float lastX = 0, lastY = 0, lastTh = 0;

  float leftTicks  = -(float)encoderCountA;  // A negative forward
  float rightTicks =  (float)encoderCountB;  // B positive forward

  float headingDeg = 0.0f;                   // same as Lab B Q2
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

  float x  = odom.getXLocation();  // cm
  float y  = odom.getYLocation();  // cm
  float th = odom.getTheta();      // deg

  float dx  = x - lastX;
  float dy  = y - lastY;
  float dth = angleDiffDeg(th, lastTh);

  robotV = sqrtf(dx * dx + dy * dy) / dt;  // cm/s with current config
  robotW = dth / dt;

  lastX = x; lastY = y; lastTh = th; lastT = now;
}

float distanceFrom(float x0, float y0) {
  float x = odom.getXLocation();
  float y = odom.getYLocation();
  float dx = x - x0;
  float dy = y - y0;
  return sqrtf(dx * dx + dy * dy);   // cm with current config
}

void lineFollowStep() {
  const int   PWM_MAX       = 1000;
  const int   PWM_MOVE_MIN  = 150;       // below this, motor basically doesn't move
  const int   BASE          = 300;
  const float KP            = 4000.0f;   // strong correction (your value)
  const float SCALE         = 4000.0f;
  const float DEADBAND      = 0.03f;
  const int   ERR_SIGN      = -1;

  // Threshold for "sensor sees line" (tune this based on your readings)
  const int   IR_ON_THRESH  = 1800;

  // Opposite snap tuning
  const int   SNAP_MAG      = 400;       // size of opposite snap in PWM units
                                         // (bump to 200 if you want more kick)

  static bool  lastBothOn = false;
  static int   lastDir    = 0;           // +1 = steering one way, -1 = the other

  // ===== Read IR sensors =====
  int Rraw = adc1_get_raw(ADC1_CHANNEL_1);  // right IR
  int Lraw = adc1_get_raw(ADC1_CHANNEL_4);  // left IR

  bool leftOn  = (Lraw > IR_ON_THRESH);
  bool rightOn = (Rraw > IR_ON_THRESH);
  bool bothOn  = leftOn && rightOn;

  float u = 0.0f;  // steering term

  if (!bothOn) {
    // ================================================
    // Normal edge-follow P control (one-sensor region)
    // ================================================
    float e = (float)(Rraw - Lraw) / SCALE;  // right - left
    e *= (float)ERR_SIGN;
    if (fabs(e) < DEADBAND) e = 0.0f;

    u = KP * e;

    // Clamp u so BASE ± u stays within [0, PWM_MAX]
    float headroom = (float)(PWM_MAX - BASE);
    if (u >  headroom) u =  headroom;
    if (u < -headroom) u = -headroom;

    // Track steering direction based on u
    if (u > 0.0f)      lastDir = +1;
    else if (u < 0.0f) lastDir = -1;
    // if u == 0, keep lastDir as-is

  } else {
    // ================================================
    // Both sensors see the line
    // ================================================
    if (!lastBothOn && bothOn) {
      // Rising edge: we *just* got back on the line
      // → apply a single opposite snap of the previous steering

      if (lastDir > 0) {
        // we had been steering in + direction → snap in - direction
        u = -SNAP_MAG;
      } else if (lastDir < 0) {
        // we had been steering in - direction → snap in + direction
        u = +SNAP_MAG;
      } else {
        // no good history, just go straight
        u = 0.0f;
      }
    } else {
      // Already on the line after the snap → go straight
      u = 0.0f;
    }
  }

  lastBothOn = bothOn;

  // ===== Convert steering term to wheel PWMs =====
  int pwmLeft  = (int)(BASE + u);
  int pwmRight = (int)(BASE - u);

  // clamp to [0, PWM_MAX]
  if (pwmLeft  > PWM_MAX) pwmLeft  = PWM_MAX;
  if (pwmRight > PWM_MAX) pwmRight = PWM_MAX;

  // never reverse in this task
  if (pwmLeft  < 0) pwmLeft  = 0;
  if (pwmRight < 0) pwmRight = 0;

  // tiny positive values that don't really move the wheel → 0
  if (pwmLeft  > 0 && pwmLeft  < PWM_MOVE_MIN) pwmLeft  = 0;
  if (pwmRight > 0 && pwmRight < PWM_MOVE_MIN) pwmRight = 0;

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
//  UDP command handler (includes UPDATED manual behavior)
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

  // ===== Manual control (Task 6 & 7) with CURVED motion =====
  if (mode == TASK6 || mode == TASK7) {
    const int FWD_FAST = 500;
    const int FWD_SLOW = 200;
    const int REV_FAST = -500;
    const int REV_SLOW = -200;

    if (msg == "F") {
      // straight forward
      setMotorA(FWD_FAST);
      setMotorB(FWD_FAST);
      updateMotionLEDs(true);
    }
    else if (msg == "B") {
      // straight backward
      setMotorA(REV_FAST);
      setMotorB(REV_FAST);
      updateMotionLEDs(true);
    }
    else if (msg == "L") {
      // forward + curve left (left wheel slower)
      setMotorA(FWD_SLOW);
      setMotorB(FWD_FAST);
      updateMotionLEDs(true);
    }
    else if (msg == "R") {
      // forward + curve right (right wheel slower)
      setMotorA(FWD_FAST);
      setMotorB(FWD_SLOW);
      updateMotionLEDs(true);
    }
    else if (msg == "S") {
      stopMotors();
    }
  }
}


// ==========================================================
//  Helper: IMU-based ~90 deg turn for tasks 2/3 (autonomous)
//  dir = +1 => right, dir = -1 => left
// ==========================================================
void timedTurn(float dir) {
  const float TARGET_DEG = 95.0f;                 // desired turn angle
  const float TARGET_RAD = TARGET_DEG * 3.141592f / 180.0f;
  const int   PWM_TURN   = 250;

  // reset IMU yaw integration (still useful if you want to log it)
  resetIMUYaw();

  // starting orientation from dead reckoning (radians)
  float theta0 = odom.getTheta();

  while (true) {
    // update sensors / odometry
    updateIMUYaw();
    updateOdometryAndVelocity();

    // current orientation in radians from odom
    float theta = odom.getTheta();

    // smallest angular difference [-pi, pi]
    float dtheta = theta - theta0;
    while (dtheta >  3.141592f) dtheta -= 2.0f * 3.141592f;
    while (dtheta < -3.141592f) dtheta += 2.0f * 3.141592f;
    float dtheta_abs = fabs(dtheta);

    // --- Stop condition ---
    // primary: odom angle >= 90 deg
    // secondary: IMU says we've turned enough
    if (dtheta_abs >= TARGET_RAD /*|| imuYawDeg >= TARGET_DEG */) {
      break;
    }

    // spin in place
    int pwmA =  PWM_TURN * dir;   // left wheel
    int pwmB = -PWM_TURN * dir;   // right wheel

    setMotorA(pwmA);
    setMotorB(pwmB);
    updateMotionLEDs(true);

    handleNetwork();
    delay(5);
  }

  stopMotors();
}

// ==========================================================
//  Task 1 – Straight 80cm, show x,y,theta,v,w
//  NOTE: with odom in cm, dist is also in cm; threshold 0.80f
//  is now 0.8cm (you can change this to 80.0f if you want 80cm).
// ==========================================================
void task1_run() {
  static bool init   = false;
  static long startA = 0;
  static long startB = 0;

  // wheel / encoder parameters (same as Task 2)
  const float TICKS_PER_REV = (float)(ticksPerRev * gearRatio); // 1440
  const float WHEEL_RADIUS  = 0.6f;                             // cm
  const float TARGET_DIST_CM = 80.0f;                           // 80 cm

  if (!init) {
    encoderCountA = 0;
    encoderCountB = 0;
    odom.setXLocation(0.0f);
    odom.setYLocation(0.0f);

    startA = encoderCountA;
    startB = encoderCountB;
    init   = true;
  }

  // --- distance along this straight segment from encoder ticks
  long dA = encoderCountA - startA;
  long dB = encoderCountB - startB;

  // match sign convention from updateOdometryAndVelocity / Task 2:
  // leftTicks  = -encoderCountA; rightTicks = encoderCountB;
  float leftTicks  = -(float)dA;
  float rightTicks =  (float)dB;
  float avgTicks   = 0.5f * (leftTicks + rightTicks);

  float dist_cm = avgTicks * (2.0f * PI * WHEEL_RADIUS / TICKS_PER_REV);

  if (dist_cm < TARGET_DIST_CM) {
    // drive straight forward, like Task 2
    setMotorA(400);
    setMotorB(400);
    updateMotionLEDs(true);

    float x  = odom.getXLocation();
    float y  = odom.getYLocation();
    float th = odom.getTheta();

    String l1 = "T1 x=" + String(x, 2) + " y=" + String(y, 2);
    String l2 = "th=" + String(th, 1) + " deg";
    String l3 = "v=" + String(robotV, 2) + " w=" + String(robotW, 1);
    String msg[] = {l1, l2, l3};
    oledPrint(msg, 3);
    sendPoseUDP(x, y, th);
  } else {
    // reached 80 cm → stop and go idle
    stopMotors();
    init = false;
    mode = IDLE;
  }
}

// ==========================================================
//  Task 2 – Uni-direction 40cm square, show heading
//  Uses encoder-based deltaS (cm) instead of distanceFrom()
// ==========================================================
void task2_run() {
  static bool init   = false;
  static int  side   = 0;
  static long startA = 0;
  static long startB = 0;

  if (!init) {
    encoderCountA = 0;
    encoderCountB = 0;
    odom.setXLocation(0.0f);
    odom.setYLocation(0.0f);

    side   = 0;
    startA = encoderCountA;
    startB = encoderCountB;
    init   = true;
  }

  if (side >= 4) {
    stopMotors();
    init = false;
    mode = IDLE;
    return;
  }

  // Distance along THIS side from encoder ticks
  long dA = encoderCountA - startA;
  long dB = encoderCountB - startB;

  // match sign convention from updateOdometryAndVelocity:
  // leftTicks  = -encoderCountA; rightTicks = encoderCountB;
  float leftTicks  = -(float)dA;
  float rightTicks =  (float)dB;
  float avgTicks   = 0.5f * (leftTicks + rightTicks);

  const float TICKS_PER_REV = (float)(ticksPerRev * gearRatio); // 1440
  const float WHEEL_RADIUS  = 0.6f;                             // cm

  // Same math as SimpleDeadReckoning: deltaS = avgTicks * (2πR / ticks_per_rev)
  float dist_cm = avgTicks * (2.0f * 3.141592f * WHEEL_RADIUS / TICKS_PER_REV);

  const float SIDE_LEN_CM = 40.0f;   // 40 cm side

  if (dist_cm < SIDE_LEN_CM) {
    setMotorA(350);
    setMotorB(350);
    updateMotionLEDs(true);

    float th = odom.getTheta();  // deg
    String l1 = "T2 side " + String(side + 1);
    String l2 = "theta=" + String(th, 1) + " deg";
    String l3 = "d=" + String(dist_cm, 1) + " cm";
    String msg[] = {l1, l2, l3};
    oledPrint(msg, 3);
  } else {
    stopMotors();
    delay(200);
    timedTurn(-1.0f);   // IMU-based ~90° turn

    startA = encoderCountA;
    startB = encoderCountB;
    side++;
  }
}

// ==========================================================
//  Task 3 – 40cm square forward, then 40cm square backward
//  Uses same encoder math as Task 2, but drives both
//  directions. Turns are skipped at the end of each square.
// ==========================================================
void task3_run() {
  static bool init = false;
  static int  side = 0;      // 0..3 forward, 4..7 backward
  static long startA = 0;
  static long startB = 0;

  const float SIDE_LEN_CM    = 40.0f;      // 40 cm
  const float WHEEL_RADIUS   = 0.6f;       // cm
  const float TICKS_PER_REV  = (float)(ticksPerRev * gearRatio); // 1440

  if (!init) {
    encoderCountA = 0;
    encoderCountB = 0;
    odom.setXLocation(0.0f);
    odom.setYLocation(0.0f);

    side   = 0;
    startA = encoderCountA;
    startB = encoderCountB;
    init   = true;
  }

  // DONE AFTER 8 SIDES
  if (side >= 8) {
    stopMotors();
    init = false;
    mode = IDLE;
    return;
  }

  // First 4 sides = forward square, last 4 = backward square
  bool forward = (side < 4);

  // Encoder deltas since this side started
  long dA = encoderCountA - startA;
  long dB = encoderCountB - startB;

  // Match same sign convention as Task 2
  float leftTicks  = -(float)dA;
  float rightTicks =  (float)dB;
  float avgTicks   = 0.5f * (leftTicks + rightTicks);

  // Same deltaS formula as Task 2
  float dist_cm = avgTicks * (2.0f * 3.141592f * WHEEL_RADIUS / TICKS_PER_REV);

  // ==========================================================
  // DRIVE FORWARD OR BACKWARD
  // ==========================================================
  if (fabs(dist_cm) < SIDE_LEN_CM) {

    int pwm = forward ? 350 : -350;   // same mag, sign flips direction

    setMotorA(pwm);
    setMotorB(pwm);
    updateMotionLEDs(true);

    String l1 = "T3 side " + String((side % 4) + 1) + (forward ? " F" : " B");
    String l2 = "d=" + String(dist_cm, 1) + " cm";
    String msg[] = { l1, l2 };
    oledPrint(msg, 2);

  } else {
    // Finished this side
    stopMotors();
    delay(200);

    // Decide whether we should turn here:
    //  - Forward square: turn after sides 0,1,2 (NOT 3)
    //  - Backward square: turn after sides 4,5,6 (NOT 7)
    bool needTurn = false;
    if (side < 3) {
      // sides 0,1,2  (forward)
      needTurn = true;
    } else if (side >= 4 && side < 7) {
      // sides 4,5,6  (backward)
      needTurn = true;
    }

    if (needTurn) {
      // Task 2 used timedTurn(-1.0f) for the correct 90° direction.
      // Keep that for forward, reverse it for backward.
      float dir = forward ? -1.0f : +1.0f;
      timedTurn(dir);
    }

    // Reset encoder baselines for next side
    startA = encoderCountA;
    startB = encoderCountB;
    side++;
  }
}




// ==========================================================
//  Task 4 – Line-follow square, OLED: x,y
// ==========================================================
void task4_run() {
  static bool init = false;
  static float sx, sy;
  static unsigned long startMs = 0;

  if (!init) {
    encoderCountA = 0;
    encoderCountB = 0;
    odom.setXLocation(0.0f);
    odom.setYLocation(0.0f);
    sx = 0.0f;
    sy = 0.0f;
    startMs = millis();
    init = true;
  }

  lineFollowStep();
  float x = odom.getXLocation();
  float y = odom.getYLocation();
  String msg[] = {
    "T4 LineFollow",
    "X=" + String(x, 2),
    "Y=" + String(y, 2)
  };
  oledPrint(msg, 3);

  if (distanceFrom(sx, sy) < 0.10f && robotV < 0.02f && millis() - startMs > 5000) {
    stopMotors();
    init = false;
    mode = IDLE;
  }
}

// ==========================================================
//  Task 5 – Line-follow + PC path
// ==========================================================
void task5_run() {
  static bool init = false;
  static float sx, sy;
  static unsigned long startMs = 0;

  if (!init) {
    encoderCountA = 0;
    encoderCountB = 0;
    odom.setXLocation(0.0f);
    odom.setYLocation(0.0f);
    sx = 0.0f;
    sy = 0.0f;
    startMs = millis();
    init = true;
  }

  lineFollowStep();
  float x = odom.getXLocation();
  float y = odom.getYLocation();
  float th = odom.getTheta();

  String msg[] = {
    "T5 Line+PC",
    "X=" + String(x,2),
    "Y=" + String(y,2)
  };
  oledPrint(msg, 3);
  sendPoseUDP(x, y, th);

  if (distanceFrom(sx, sy) < 0.10f && robotV < 0.02f && millis() - startMs > 5000) {
    stopMotors();
    init = false;
    mode = IDLE;
  }
}

// ==========================================================
//  Task 6 – Manual letters (Z,U,O,P,S,D,G,N,M)
// ==========================================================
void task6_run() {
  float x = odom.getXLocation();
  float y = odom.getYLocation();
  float th = odom.getTheta();
  sendPoseUDP(x, y, th);

  String msg7[] = {"T6 Manual", "CMD:" + lastCmd, "PC:" + pcIP.toString()};
  oledPrint(msg7, 3);
}

// ==========================================================
//  Task 7 – Manual + lidar OGM
// ==========================================================
void task7_run() {
  float x = odom.getXLocation();
  float y = odom.getYLocation();
  float th = odom.getTheta();
  sendPoseUDP(x, y, th);

  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus == 0) {
    float r = measure.RangeMilliMeter / 1000.0f;
    float thRad = th * PI / 180.0f;
    float xw = x + r * cosf(thRad);
    float yw = y + r * sinf(thRad);
    sendLidarUDP(xw, yw);
  }

  String msg8[] = {"T7 OGM", "CMD:" + lastCmd, "PC:" + pcIP.toString()};
  oledPrint(msg8, 3);
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

  // Fast ADC configuration for IR sensors
  adc1_config_width(ADC_WIDTH_12Bit);

  // TODO: make sure these channels match your wiring.
  // Example from prof:
  adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_12);  // right IR
  adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_12);  // left IR

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();
  display.setRotation(2);

  setupMotorPWM();
  setMotorA(0);
  setMotorB(0);
  updateMotionLEDs(false);

  setupEncoderInterrupt();

  WiFi.begin(ssid, password);
  String msg1[] = {"WiFi...", "", ""};
  oledPrint(msg1, 3);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
  }
  udp.begin(UDP_PORT);
  String msg2[] = {"WiFi OK", WiFi.localIP().toString(), "UDP:9000"};
  oledPrint(msg2, 3);

  imu.initialize();
  calibrateGyroZ();   // IMU bias at rest

  if (!lox.begin()) {
    String msg3[] = {"Lidar FAIL", "", ""};
    oledPrint(msg3, 3);
  } else {
    String msg4[] = {"Lidar OK", "", ""};
    oledPrint(msg4, 3);
  }

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
      String msg5[] = {"IDLE", "Send 1-7 via UDP", "\n", WiFi.localIP().toString(), "UDP:9000"};
      oledPrint(msg5, 5);
      break;
  }

  delay(10);
}
