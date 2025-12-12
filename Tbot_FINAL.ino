// ==========================================================
//  TB Final Project - All Tasks 1–7 (NO SimpleDeadReckoning)
//  - Encoders give distance (2πR * ticks / ticks_per_rev)
//  - MPU6050_light gives heading (yaw)
//  - We integrate x,y ourselves from distance + heading
//  - Q2: 40cm square forward
//  - Q3: 40cm square forward + 40cm square backward (IMU PID)
//  Kyle Arquilla
// ==========================================================

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL53L0X.h>
#include <MPU6050_light.h>
#include <driver/adc.h>
#include <math.h>

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
    display.println(lines[i]);
  }
  display.display();
}

// ==========================================================
//  Motor PWM (LEDC)
// ==========================================================
const int pwmFreq       = 500;
const int pwmResolution = 10;  // 0..1023
const int pwmA1_Ch      = 0;
const int pwmA2_Ch      = 1;
const int pwmB1_Ch      = 2;
const int pwmB2_Ch      = 3;

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
//  Encoder & Odom
// ==========================================================
volatile long encoderCountA = 0;   // left
volatile long encoderCountB = 0;   // right
volatile uint8_t prevStateA = 0, prevStateB = 0;

const int   ticksPerRev    = 12 * 4;   // 48 ticks per motor shaft
const float gearRatio      = 30.0f;    // 30:1
const float TICKS_PER_REV  = (float)(ticksPerRev * gearRatio); // 1440
const float WHEEL_RADIUS_CM = 0.6f;    // you had this; distance math matches

// Quad encoder macros
#define READ_ENC_A() ((digitalRead(MEA1) << 1) | digitalRead(MEA2))
#define READ_ENC_B() ((digitalRead(MEB1) << 1) | digitalRead(MEB2))

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
//  Global Pose (x, y, theta) + velocity
//  Units: cm and degrees
// ==========================================================
float poseX_cm   = 0.0f;
float poseY_cm   = 0.0f;
float poseTh_deg = 0.0f;   // yaw from IMU

long prevEncA = 0;
long prevEncB = 0;

float robotV = 0.0f;       // cm/s
float robotW = 0.0f;       // deg/s

unsigned long lastOdomMs = 0;
float lastOdomX = 0.0f;
float lastOdomY = 0.0f;
float lastOdomTh = 0.0f;

float angleDiffDeg(float a, float b) {
  float d = a - b;
  while (d > 180.0f)  d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

// forward decl for heading
float getHeadingDeg();

// integrate encoders + IMU heading
void updateOdometryAndVelocity() {
  unsigned long now = millis();

  // encoder deltas
  long eA = encoderCountA;
  long eB = encoderCountB;
  long dA = eA - prevEncA;
  long dB = eB - prevEncB;
  prevEncA = eA;
  prevEncB = eB;

  // same sign convention as your old code:
  // left forward = negative, right forward = positive
  float leftTicksDelta  = -(float)dA;
  float rightTicksDelta =  (float)dB;

  // linear distance this step (cm)
  float avgTicks = 0.5f * (leftTicksDelta + rightTicksDelta);
  float deltaS   = avgTicks * (2.0f * PI * WHEEL_RADIUS_CM / TICKS_PER_REV);

  // heading from IMU (deg)
  poseTh_deg = getHeadingDeg();
  float thRad = poseTh_deg * PI / 180.0f;

  // update x,y
  poseX_cm += deltaS * cosf(thRad);
  poseY_cm += deltaS * sinf(thRad);

  // velocity
  if (lastOdomMs == 0) {
    lastOdomMs = now;
    lastOdomX  = poseX_cm;
    lastOdomY  = poseY_cm;
    lastOdomTh = poseTh_deg;
    robotV = 0.0f;
    robotW = 0.0f;
    return;
  }

  float dt = (now - lastOdomMs) / 1000.0f;
  if (dt <= 0) return;

  float dx  = poseX_cm   - lastOdomX;
  float dy  = poseY_cm   - lastOdomY;
  float dth = angleDiffDeg(poseTh_deg, lastOdomTh);

  robotV = sqrtf(dx * dx + dy * dy) / dt;
  robotW = dth / dt;

  lastOdomMs = now;
  lastOdomX  = poseX_cm;
  lastOdomY  = poseY_cm;
  lastOdomTh = poseTh_deg;
}

float distanceFrom(float x0, float y0) {
  float dx = poseX_cm - x0;
  float dy = poseY_cm - y0;
  return sqrtf(dx * dx + dy * dy);
}

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

void sendPoseUDP(float x_cm, float y_cm, float theta_deg) {
  if (pcPort == 0) return;
  String msg = "P," + String(x_cm, 3) + "," + String(y_cm, 3) + "," + String(theta_deg, 3);
  udp.beginPacket(pcIP, pcPort);
  udp.print(msg);
  udp.endPacket();
}

void sendLidarUDP(float xw_cm, float yw_cm) {
  if (pcPort == 0) return;
  String msg = "L," + String(xw_cm, 3) + "," + String(yw_cm, 3);
  udp.beginPacket(pcIP, pcPort);
  udp.print(msg);
  udp.endPacket();
}

// ==========================================================
//  IMU + Lidar
// ==========================================================
MPU6050 mpu(Wire);
Adafruit_VL53L0X lox;

void initIMU() {
  // Wire.begin(...) is called in setup with custom pins
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {
    // stuck if IMU not found
  }

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // if needed
  mpu.calcOffsets(); // gyro + accel
  Serial.println(F("IMU offsets done"));
}

float getHeadingDeg() {
  // yaw from MPU6050_light
  return mpu.getAngleZ();
}

// ==========================================================
//  Heading PID
// ==========================================================
float headingTargetDeg = 0.0f;
float hErrInt          = 0.0f;
float hErrPrev         = 0.0f;

// tune these
float Kp_heading = 6.0f;
float Ki_heading = 0.02f; 
float Kd_heading = 0.5f;

void resetHeadingPID(float targetDeg) {
  headingTargetDeg = targetDeg;
  hErrInt          = 0.0f;
  hErrPrev         = 0.0f;
}

// basePWM > 0 => forward, basePWM < 0 => backward
void headingHoldStep(int basePWM) {
  float heading = getHeadingDeg();

  float err = angleDiffDeg(headingTargetDeg, heading);  // wrap [-180,180]

  hErrInt += err;
  float d  = err - hErrPrev;
  hErrPrev = err;

  float u = Kp_heading * err + Ki_heading * hErrInt + Kd_heading * d;

  u = -u;

  int leftPWM  = basePWM - (int)u;
  int rightPWM = basePWM + (int)u;

  leftPWM  = constrain(leftPWM,  -1000, 1000);
  rightPWM = constrain(rightPWM, -1000, 1000);

  setMotorA(leftPWM);
  setMotorB(rightPWM);
}

// basePWM < 0 => backward with heading hold (for Q3 reverse legs)
void headingHoldStepReverse(int basePWM) {
  float heading = getHeadingDeg();

  float err = angleDiffDeg(headingTargetDeg, heading);  // wrap [-180,180]

  hErrInt += err;
  float d  = err - hErrPrev;
  hErrPrev = err;

  // standard PID term
  float u = Kp_heading * err + Ki_heading * hErrInt + Kd_heading * d;

  // For reverse, flip the steering compared to the forward case
  int leftPWM  = basePWM + (int)u;
  int rightPWM = basePWM - (int)u;

  leftPWM  = constrain(leftPWM,  -1000, 1000);
  rightPWM = constrain(rightPWM, -1000, 1000);

  setMotorA(leftPWM);
  setMotorB(rightPWM);
}

void lineFollowStep() {
  const int FWD_PWM      = 200;   // normal forward speed
  const int TURN_PWM     = 250;   // normal turn speed on edge
  const int SEARCH_FWD   = 100;   // inner wheel in search arc
  const int SEARCH_TURN  = 300;   // outer wheel in search arc
  const int PWM_MOVE_MIN = 100;
  const int ADC_MAX      = 4095;
  const int IR_ON_THRESH = 2700;  // tune between floor and black (after inversion)

  const unsigned long SEARCH_PIVOT_MS = 300;  // hold inner wheel for 0.5s

  // lost == true → we are in "search" mode
  static bool  lost         = false;
  // lastDir: -1 = last correction was "turn left", +1 = "turn right", 0 = straight / unknown
  static int   lastDir      = 0;
  // searchPhase: 0 = none, 1 = pivot (inner=0), 2 = creeping arc
  static int   searchPhase  = 0;
  static unsigned long phaseStartMs = 0;

  // ===== Read IR sensors =====
  int Rraw = adc1_get_raw(ADC1_CHANNEL_1);  // right IR
  int Lraw = adc1_get_raw(ADC1_CHANNEL_4);  // left  IR

  // BLACK line → invert so dark = big
  Rraw = ADC_MAX - Rraw;
  Lraw = ADC_MAX - Lraw;

  bool leftOn   = (Lraw > IR_ON_THRESH);
  bool rightOn  = (Rraw > IR_ON_THRESH);
  bool bothOn   = leftOn && rightOn;
  bool anyOn    = leftOn || rightOn;
  bool bothOff  = !anyOn;

  int pwmLeft  = 0;
  int pwmRight = 0;

  // ==========================
  // NORMAL FOLLOWING (not lost)
  // ==========================
  if (!lost) {
    searchPhase = 0;  // not in search when we're on the line

    if (bothOff) {
      // FIRST time we lose the line → mark lost and start search phase 1
      lost         = true;
      searchPhase  = 1;
      phaseStartMs = millis();
      if (lastDir == 0) {
        // default search direction if we have no history
        lastDir = +1;
      }
    } else {
      // We still see the line somewhere → standard 2-sensor behavior
      if (leftOn && rightOn) {
        // centered on black → go straight
        pwmLeft  = FWD_PWM;
        pwmRight = FWD_PWM;
        lastDir  = 0;
      } else if (leftOn && !rightOn) {
        // line under LEFT sensor → "turn RIGHT"
        pwmLeft  = TURN_PWM;
        pwmRight = 0;
        lastDir  = +1;
      } else if (!leftOn && rightOn) {
        // line under RIGHT sensor → "turn LEFT"
        pwmLeft  = 0;
        pwmRight = TURN_PWM;
        lastDir  = -1;
      }
    }
  }

  // ==========================
  // SEARCH MODE (lost == true)
  // ==========================
  if (lost) {
    unsigned long now = millis();

    if (anyOn) {
      // We SEE the line again (doesn't matter which sensor) →
      // exit search and drive straight a bit.
      lost        = false;
      searchPhase = 0;
      pwmLeft     = FWD_PWM;
      pwmRight    = FWD_PWM;
      lastDir     = 0;
    } else {
      // Still lost → 2-phase behavior:
      if (searchPhase == 1) {
        // ---- PHASE 1: pivot in place, inner wheel held at 0 ----
        if (lastDir <= 0) {
          // search LEFT: left = inner (0), right = outer
          pwmLeft  = 0;
          pwmRight = SEARCH_TURN;
        } else {
          // search RIGHT: right = inner (0), left = outer
          pwmLeft  = SEARCH_TURN;
          pwmRight = 0;
        }

        if (now - phaseStartMs >= SEARCH_PIVOT_MS) {
          // after 0.5s, move to creeping arc
          searchPhase  = 2;
          phaseStartMs = now;
        }
      } else {
        // ---- PHASE 2: creeping forward arc ----
        if (lastDir <= 0) {
          // arc LEFT: left = inner (slow), right = outer (fast)
          pwmLeft  = SEARCH_FWD;
          pwmRight = SEARCH_TURN;
        } else {
          // arc RIGHT: right = inner (slow), left = outer (fast)
          pwmLeft  = SEARCH_TURN;
          pwmRight = SEARCH_FWD;
        }
      }
    }
  }

  // ==========================
  // Deadband & apply
  // ==========================
  if (pwmLeft  > 0 && pwmLeft  < PWM_MOVE_MIN)  pwmLeft  = 0;
  if (pwmRight > 0 && pwmRight < PWM_MOVE_MIN) pwmRight = 0;

  setMotorA(pwmLeft);
  setMotorB(pwmRight);
  updateMotionLEDs(pwmLeft != 0 || pwmRight != 0);
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
//  UDP command handler (+ manual control for Task 6/7)
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

  // ===== Task selection (same as before) =====
  if      (msg == "1") mode = TASK1;
  else if (msg == "2") mode = TASK2;
  else if (msg == "3") mode = TASK3;
  else if (msg == "4") mode = TASK4;
  else if (msg == "5") mode = TASK5;
  else if (msg == "6") mode = TASK6;
  else if (msg == "7") mode = TASK7;

  // ===== Manual / PC control (Task 6 & 7) =====
  if (mode == TASK6 || mode == TASK7) {
    // First, handle M,left,right commands from the Python client
    if (msg.startsWith("M,")) {
      int c1 = msg.indexOf(',');          // first comma after 'M'
      int c2 = msg.indexOf(',', c1 + 1);  // second comma

      if (c1 > 0 && c2 > c1) {
        int leftPWM  = msg.substring(c1 + 1, c2).toInt();
        int rightPWM = msg.substring(c2 + 1).toInt();

        // Apply PWM directly to motors
        setMotorA(leftPWM);
        setMotorB(rightPWM);

        bool moving = (leftPWM != 0 || rightPWM != 0);
        updateMotionLEDs(moving);
      }

      // We handled this packet completely; don't fall through
      return;
    }

    // Legacy single-letter controls (still work if you ever send them)
    const int FWD_FAST = 500;
    const int FWD_SLOW = 200;
    const int REV_FAST = -500;
    const int REV_SLOW = -200;

    if (msg == "F") {
      setMotorA(FWD_FAST);
      setMotorB(FWD_FAST);
      updateMotionLEDs(true);
    }
    else if (msg == "B") {
      setMotorA(REV_FAST);
      setMotorB(REV_FAST);
      updateMotionLEDs(true);
    }
    else if (msg == "L") {
      setMotorA(FWD_SLOW);
      setMotorB(FWD_FAST);
      updateMotionLEDs(true);
    }
    else if (msg == "R") {
      setMotorA(FWD_FAST);
      setMotorB(FWD_SLOW);
      updateMotionLEDs(true);
    }
    else if (msg == "S") {
      stopMotors();
      updateMotionLEDs(false);
    }
  }
}

// ==========================================================
//  IMU-based ~90 deg turn (dir=+1 right, dir=-1 left)
// ==========================================================
void timedTurn(float dir) {
  const float TARGET_DEG = 90.0f;   // tweak to tighten corners
  const int   PWM_TURN   = 200;

  float yaw0 = getHeadingDeg();

  while (true) {
    mpu.update();
    updateOdometryAndVelocity();

    float yaw  = getHeadingDeg();
    float dyaw = angleDiffDeg(yaw, yaw0);

    if (fabs(dyaw) >= TARGET_DEG) {
      break;
    }

    int pwmA =  PWM_TURN * dir;
    int pwmB = -PWM_TURN * dir;
    setMotorA(pwmA);
    setMotorB(pwmB);
    updateMotionLEDs(true);

    handleNetwork();
    delay(5);
  }

  stopMotors();
  updateMotionLEDs(false);
}

// ==========================================================
//  Task 1 – Straight 80cm, show x,y,theta,v,w, send pose
// ==========================================================
void task1_run() {
  static bool init   = false;
  static long startA = 0;
  static long startB = 0;

  const float TARGET_DIST_CM = 80.0f;

  if (!init) {
    encoderCountA = 0;
    encoderCountB = 0;
    prevEncA = encoderCountA;
    prevEncB = encoderCountB;

    poseX_cm = 0.0f;
    poseY_cm = 0.0f;
    poseTh_deg = getHeadingDeg();
    lastOdomMs = 0;

    startA = encoderCountA;
    startB = encoderCountB;
    init   = true;
  }

  long dA = encoderCountA - startA;
  long dB = encoderCountB - startB;

  float leftTicks  = -(float)dA;
  float rightTicks =  (float)dB;
  float avgTicks   = 0.5f * (leftTicks + rightTicks);

  float dist_cm = avgTicks * (2.0f * PI * WHEEL_RADIUS_CM / TICKS_PER_REV);

  if (fabs(dist_cm) < TARGET_DIST_CM) {
    setMotorA(400);
    setMotorB(400);
    updateMotionLEDs(true);

    String l1 = "T1 x=" + String(poseX_cm, 2) + " y=" + String(poseY_cm, 2);
    String l2 = "th=" + String(poseTh_deg, 1) + " deg";
    String l3 = "v=" + String(robotV, 2) + " w=" + String(robotW, 1);
    String msg[] = {l1, l2, l3};
    oledPrint(msg, 3);
    sendPoseUDP(poseX_cm, poseY_cm, poseTh_deg);
  } else {
    stopMotors();
    updateMotionLEDs(false);
    init = false;
    mode = IDLE;
  }
}

// ==========================================================
//  Task 2 – 40cm square forward (encoders + IMU turn)
// ==========================================================
void task2_run() {
  static bool init   = false;
  static int  side   = 0;
  static long startA = 0;
  static long startB = 0;

  const float SIDE_LEN_CM = 40.0f;

  if (!init) {
    encoderCountA = 0;
    encoderCountB = 0;
    prevEncA = encoderCountA;
    prevEncB = encoderCountB;

    poseX_cm = 0.0f;
    poseY_cm = 0.0f;
    poseTh_deg = getHeadingDeg();
    lastOdomMs = 0;

    side   = 0;
    startA = encoderCountA;
    startB = encoderCountB;
    init   = true;
  }

  if (side >= 4) {
    stopMotors();
    updateMotionLEDs(false);
    init = false;
    mode = IDLE;
    return;
  }

  long dA = encoderCountA - startA;
  long dB = encoderCountB - startB;

  float leftTicks  = -(float)dA;
  float rightTicks =  (float)dB;
  float avgTicks   = 0.5f * (leftTicks + rightTicks);

  float dist_cm = avgTicks * (2.0f * PI * WHEEL_RADIUS_CM / TICKS_PER_REV);

  if (fabs(dist_cm) < SIDE_LEN_CM) {
    setMotorA(350);
    setMotorB(350);
    updateMotionLEDs(true);

    String l1 = "T2 side " + String(side + 1);
    String l2 = "th=" + String(poseTh_deg, 1) + " deg";
    String l3 = "d=" + String(dist_cm, 1) + " cm";
    String msg[] = {l1, l2, l3};
    oledPrint(msg, 3);
    sendPoseUDP(poseX_cm, poseY_cm, poseTh_deg);
  } else {
    stopMotors();
    updateMotionLEDs(false);
    delay(200);
    timedTurn(-1.0f);   // turn left each corner

    startA = encoderCountA;
    startB = encoderCountB;
    side++;

    resetHeadingPID(getHeadingDeg());
  }
}

// ==========================================================
//  Task 3 – 40cm square forward, then 40cm square backward
//  Uses encoder distance + IMU heading PID + timedTurn
// ==========================================================
void task3_run() {
  static bool init   = false;
  static int  side   = 0;      // 0..3 fwd, 4..7 back
  static long startA = 0;
  static long startB = 0;

  const float SIDE_LEN_CM   = 40.0f;
  const int   BASE_PWM      = 350;

  if (!init) {
    encoderCountA = 0;
    encoderCountB = 0;
    prevEncA = encoderCountA;
    prevEncB = encoderCountB;

    poseX_cm = 0.0f;
    poseY_cm = 0.0f;
    poseTh_deg = getHeadingDeg();
    lastOdomMs = 0;

    side   = 0;
    startA = encoderCountA;
    startB = encoderCountB;

    resetHeadingPID(getHeadingDeg());
    init = true;
  }

  if (side >= 8) {
    stopMotors();
    updateMotionLEDs(false);
    init = false;
    mode = IDLE;
    return;
  }

  bool forward = (side < 4);
  int  dirSign = forward ? +1 : -1;

  long dA = encoderCountA - startA;
  long dB = encoderCountB - startB;

  float leftTicks  = -(float)dA;
  float rightTicks =  (float)dB;
  float avgTicks   = 0.5f * (leftTicks + rightTicks);

  float dist_cm = fabs(
    avgTicks * (2.0f * PI * WHEEL_RADIUS_CM / TICKS_PER_REV)
  );

  if (dist_cm < SIDE_LEN_CM) {
    int base = BASE_PWM * dirSign;

    if (forward) {
      // forward legs (0..3): use the original heading PID
      headingHoldStep(base);
    } else {
      // reverse legs (4..7): use reverse-aware PID
      headingHoldStepReverse(base);
    }

    updateMotionLEDs(true);

    String l1 = forward ? "T3 Fwd square" : "T3 Back square";
    String l2 = "Side " + String((side % 4) + 1) + " d=" + String(dist_cm, 1) + "cm";
    String l3 = "head=" + String(poseTh_deg, 1);
    String msg[] = { l1, l2, l3 };
    oledPrint(msg, 3);

    sendPoseUDP(poseX_cm, poseY_cm, poseTh_deg);
    return;
  }

  // end of this leg
  stopMotors();
  updateMotionLEDs(false);
  delay(200);

  bool needTurn = false;
  if (side < 3)                  needTurn = true; // forward 3 corners
  else if (side > 3 && side < 7) needTurn = true; // backward 3 corners

  if (needTurn) {
    float turnDir = forward ? -1.0f : +1.0f;  // same pattern as before
    timedTurn(turnDir);
  }

  startA = encoderCountA;
  startB = encoderCountB;
  side++;

  resetHeadingPID(getHeadingDeg());
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
    prevEncA = encoderCountA;
    prevEncB = encoderCountB;

    poseX_cm = 0.0f;
    poseY_cm = 0.0f;
    poseTh_deg = getHeadingDeg();
    lastOdomMs = 0;

    sx = poseX_cm;
    sy = poseY_cm;
    startMs = millis();
    init = true;
  }

  lineFollowStep();

  String msg[] = {
    "T4 LineFollow",
    "X=" + String(poseX_cm, 2),
    "Y=" + String(poseY_cm, 2)
  };
  oledPrint(msg, 3);

  if (distanceFrom(sx, sy) < 0.10f && robotV < 0.02f && millis() - startMs > 5000) {
    stopMotors();
    updateMotionLEDs(false);
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
    prevEncA = encoderCountA;
    prevEncB = encoderCountB;

    poseX_cm = 0.0f;
    poseY_cm = 0.0f;
    poseTh_deg = getHeadingDeg();
    lastOdomMs = 0;

    sx = poseX_cm;
    sy = poseY_cm;
    startMs = millis();
    init = true;
  }

  // *** EXACT SAME BEHAVIOR AS T4 ***
  lineFollowStep();

  String msg[] = {
    "T5 Line+PC",                     // only the title is different
    "X=" + String(poseX_cm, 2),
    "Y=" + String(poseY_cm, 2)
  };
  oledPrint(msg, 3);

  // *** EXTRA: send pose like Q3 ***
  sendPoseUDP(poseX_cm, poseY_cm, poseTh_deg);

  if (distanceFrom(sx, sy) < 0.10f && robotV < 0.02f && millis() - startMs > 5000) {
    stopMotors();
    updateMotionLEDs(false);
    init = false;
    mode = IDLE;
  }
}

// ==========================================================
//  Task 6 – Manual letters (Z,U,O,P,S,D,G,N,M) + pose
// ==========================================================
void task6_run() {
  sendPoseUDP(poseX_cm, poseY_cm, poseTh_deg);
  String msg[] = {"T6 Manual", "CMD:" + lastCmd, "PC:" + pcIP.toString()};
  oledPrint(msg, 3);
}

// ==========================================================
//  Task 7 – Manual + lidar OGM
// ==========================================================
void task7_run() {
  sendPoseUDP(poseX_cm, poseY_cm, poseTh_deg);

  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus == 0) {
    float r_cm  = measure.RangeMilliMeter / 10.0f;   // mm -> cm
    float thRad = poseTh_deg * PI / 180.0f;
    float xw = poseX_cm + r_cm * cosf(thRad);
    float yw = poseY_cm + r_cm * sinf(thRad);
    sendLidarUDP(xw, yw);
  }

  String msg[] = {"T7 OGM", "CMD:" + lastCmd, "PC:" + pcIP.toString()};
  oledPrint(msg, 3);
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

  initIMU();
  mpu.update();  // prime angles

  if (!lox.begin()) {
    String msg3[] = {"Lidar FAIL", "", ""};
    oledPrint(msg3, 3);
  } else {
    String msg4[] = {"Lidar OK", "", ""};
    oledPrint(msg4, 3);
  }

  encoderCountA = 0;
  encoderCountB = 0;
  prevEncA = encoderCountA;
  prevEncB = encoderCountB;

  poseX_cm   = 0.0f;
  poseY_cm   = 0.0f;
  poseTh_deg = getHeadingDeg();
  lastOdomMs = 0;
}

// ==========================================================
//  LOOP
// ==========================================================
void loop() {
  handleNetwork();
  mpu.update();
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
    default: {
      stopMotors();
      updateMotionLEDs(false);
      String msg[] = {"IDLE", "Send 1-7 via UDP", WiFi.localIP().toString(), "UDP:9000"};
      oledPrint(msg, 4);
      break;
    }
  }

  delay(10);
}
