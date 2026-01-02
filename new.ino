/*
  Robot Mega Firmware (CRSF + USB JSON)
  - Modes: CRSF (manual) / USB (autopilot) / ESTOP
  - USB Watchdog: 3s (requested)
  - Non-blocking motor duration (no delay)
  - Supports:
      1) Legacy:
         { "servos": {...}, "motors": {"left":..,"right":..,"duration":..} }
      2) Typed:
         {"type":"mode","mode":"crsf|usb"}
         {"type":"estop","enabled":true|false}
         {"type":"cmd_vel","v":-1..1,"w":-1..1}
*/

#include <AlfredoCRSF.h>
#include <ServoEasing.hpp>
#include <ArduinoJson.h>

// --- Configuration ---
// CRSF Setup (RX2 on Mega is Pin 17, TX2 is Pin 16)
#define CRSF_BAUDRATE 115200

// Pin Definitions
const int PIN_WAIST      = 2;
const int PIN_SHOULDER_1 = 3;
const int PIN_SHOULDER_2 = 4; // Secondary Shoulder Servo (Opposite Direction)
const int PIN_ELBOW      = 5;
const int PIN_WRIST_P    = 6;
const int PIN_WRIST_R    = 7;
const int PIN_GRIPPER    = 8;

// Base Motor Pins
const int PIN_LEFT_PWM   = 9;
const int PIN_LEFT_DIR   = 10;
const int PIN_RIGHT_PWM  = 11;
const int PIN_RIGHT_DIR  = 12;

// --- Tuning ---
const int   ARM_SPEED = 90;
const int   GRIPPER_SPEED = 200;

const float WAIST_SENSITIVITY     = 0.0005;
const float SHOULDER_SENSITIVITY  = 0.0002;
const float ELBOW_SENSITIVITY     = 0.0001;
const float WRIST_SENSITIVITY     = 0.001;
const float GRIPPER_SENSITIVITY   = 0.001;
const int   DEADZONE = 30;

// Startup & Home Configuration (PWM Values in Microseconds)
const int STARTUP_SPEED = 20;
const int WAIST_HOME_PWM    = 1500;
const int SHOULDER_HOME_PWM = 2400;
const int ELBOW_HOME_PWM    = 2400;
const int WRIST_P_HOME_PWM  = 1600;
const int WRIST_R_HOME_PWM  = 1500;
const int GRIPPER_HOME_PWM  = 1000;

// --- Control / Safety ---
enum ControlMode { MODE_CRSF, MODE_USB, MODE_ESTOP };
ControlMode controlMode = MODE_CRSF;

bool estopEnabled = false;

// USB Watchdog requested: 3 seconds
const unsigned long USB_WATCHDOG_MS = 3000;
unsigned long lastUsbCmdMs = 0;

// cmd_vel scaling (tune later)
float cmdVelV = 0.0f;             // -1..+1
float cmdVelW = 0.0f;             // -1..+1
const float CMDVEL_TO_PWM = 180.0f; // maps to motor PWM magnitude
const float CMDVEL_TURN_GAIN = 1.0f;

// Non-blocking motor duration handling
bool motorTimedActive = false;
unsigned long motorEndMs = 0;
int motorTimedLeft = 0;
int motorTimedRight = 0;

// --- State Variables for Relative Control (angles) ---
float tWaist, tShoulder, tElbow, tWristP, tWristR, tGripper;

// --- Objects ---
AlfredoCRSF crsf;
ServoEasing sWaist;
ServoEasing sShoulder1;
ServoEasing sShoulder2;
ServoEasing sElbow;
ServoEasing sWristP;
ServoEasing sWristR;
ServoEasing sGripper;

// --- Variables ---
int chDrive, chTurn;
int chWaist, chShoulder, chElbow, chWristP, chWristR, chGripper;
int currentLeftSpeed = 0;
int currentRightSpeed = 0;

// --- USB RX line buffer (non-blocking) ---
static char rxLine[768];
static size_t rxLen = 0;

// Conversion Helper
long map_long(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int pwmToAngle(int pwm) {
  int val = constrain(pwm, 544, 2400);
  return (int)map_long(val, 544, 2400, 0, 180);
}

static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void setSpeedForAllServos(int speed) {
  sWaist.setSpeed(speed);
  sShoulder1.setSpeed(speed);
  sShoulder2.setSpeed(speed);
  sElbow.setSpeed(speed);
  sWristP.setSpeed(speed);
  sWristR.setSpeed(speed);
  sGripper.setSpeed(speed);
}

void setMotor(int pinPWM, int pinDir, int speed) {
  // Your tested logic:
  // Forward:  DIR=LOW, PWM=0..255 (255 fast)
  // Backward: DIR=HIGH, PWM inverted (0 fast, 255 stop)
  if (speed > 0) {
    digitalWrite(pinDir, LOW);
    analogWrite(pinPWM, speed);
  } else {
    digitalWrite(pinDir, HIGH);
    analogWrite(pinPWM, 255 - abs(speed));
  }
}

void stopBase() {
  digitalWrite(PIN_LEFT_DIR, HIGH);
  digitalWrite(PIN_RIGHT_DIR, HIGH);
  analogWrite(PIN_LEFT_PWM, 255);
  analogWrite(PIN_RIGHT_PWM, 255);
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
}

void controlMobileBase(int throttle, int steering) {
  int speedFwd  = map(throttle, 1000, 2000, -255, 255);
  int speedTurn = map(steering, 1000, 2000, -255, 255);

  if (abs(speedFwd) < 20) speedFwd = 0;
  if (abs(speedTurn) < 20) speedTurn = 0;

  int leftSpeed  = speedFwd - speedTurn;
  int rightSpeed = speedFwd + speedTurn;

  leftSpeed  = constrain(leftSpeed,  -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  currentLeftSpeed = leftSpeed;
  currentRightSpeed = rightSpeed;

  setMotor(PIN_LEFT_PWM, PIN_LEFT_DIR, leftSpeed);
  setMotor(PIN_RIGHT_PWM, PIN_RIGHT_DIR, rightSpeed);
}

void applyCmdVel() {
  float v = clampf(cmdVelV, -1.0f, 1.0f);
  float w = clampf(cmdVelW, -1.0f, 1.0f);

  float left  = (v - CMDVEL_TURN_GAIN * w) * CMDVEL_TO_PWM;
  float right = (v + CMDVEL_TURN_GAIN * w) * CMDVEL_TO_PWM;

  int leftPWM  = (int)constrain(left,  -255.0f, 255.0f);
  int rightPWM = (int)constrain(right, -255.0f, 255.0f);

  currentLeftSpeed = leftPWM;
  currentRightSpeed = rightPWM;

  setMotor(PIN_LEFT_PWM, PIN_LEFT_DIR, leftPWM);
  setMotor(PIN_RIGHT_PWM, PIN_RIGHT_DIR, rightPWM);
}

void readCrsfAndControl() {
  chWaist    = crsf.getChannel(4);
  chShoulder = crsf.getChannel(2);
  chDrive    = crsf.getChannel(3);
  chTurn     = crsf.getChannel(1);
  chElbow    = crsf.getChannel(6);
  chWristP   = crsf.getChannel(7);
  chWristR   = crsf.getChannel(8);
  chGripper  = crsf.getChannel(9);

  // Waist
  if (abs(chWaist - 1500) > DEADZONE) tWaist += (chWaist - 1500) * WAIST_SENSITIVITY;
  tWaist = constrain(tWaist, 0, 180);
  sWaist.write((int)tWaist);

  // Shoulder (dual inverse)
  if (abs(chShoulder - 1500) > DEADZONE) tShoulder += (chShoulder - 1500) * SHOULDER_SENSITIVITY;
  tShoulder = constrain(tShoulder, 0, 180);
  sShoulder1.write((int)tShoulder);
  sShoulder2.write(180 - (int)tShoulder);

  // Elbow
  if (abs(chElbow - 1500) > DEADZONE) tElbow += (chElbow - 1500) * ELBOW_SENSITIVITY;
  tElbow = constrain(tElbow, 0, 180);
  sElbow.write((int)tElbow);

  // Wrist Pitch
  if (abs(chWristP - 1500) > DEADZONE) tWristP += (chWristP - 1500) * WRIST_SENSITIVITY;
  tWristP = constrain(tWristP, 0, 180);
  sWristP.write((int)tWristP);

  // Wrist Roll
  if (abs(chWristR - 1500) > DEADZONE) tWristR += (chWristR - 1500) * WRIST_SENSITIVITY;
  tWristR = constrain(tWristR, 0, 180);
  sWristR.write((int)tWristR);

  // Gripper
  if (abs(chGripper - 1500) > DEADZONE) tGripper += (chGripper - 1500) * GRIPPER_SENSITIVITY;
  tGripper = constrain(tGripper, 0, 90);
  sGripper.write((int)tGripper);

  // Base
  controlMobileBase(chDrive, chTurn);
}

void printStatusJSON() {
  StaticJsonDocument<512> doc;

  doc["mode"] = (controlMode == MODE_CRSF) ? "crsf" : (controlMode == MODE_USB) ? "usb" : "estop";
  doc["estop"] = estopEnabled;
  doc["usbWatchdogMs"] = USB_WATCHDOG_MS;
  doc["usbLastCmdAgoMs"] = (long)(millis() - lastUsbCmdMs);

  JsonObject servos = doc.createNestedObject("servos");
  servos["waist"]   = (int)tWaist;
  servos["shoulder"]= (int)tShoulder;
  servos["elbow"]   = (int)tElbow;
  servos["wristP"]  = (int)tWristP;
  servos["wristR"]  = (int)tWristR;
  servos["gripper"] = (int)tGripper;

  JsonObject motors = doc.createNestedObject("motors");
  motors["left"] = currentLeftSpeed;
  motors["right"] = currentRightSpeed;
  motors["timedActive"] = motorTimedActive;

  serializeJson(doc, Serial);
  Serial.println();
}

void handleJsonDocument(JsonDocument& doc) {
  lastUsbCmdMs = millis();

  const char* type = doc["type"] | nullptr;

  // --- Typed messages ---
  if (type) {
    if (strcmp(type, "mode") == 0) {
      const char* m = doc["mode"] | "crsf";
      if (strcmp(m, "usb") == 0) controlMode = MODE_USB;
      else if (strcmp(m, "crsf") == 0) controlMode = MODE_CRSF;
      return;
    }

    if (strcmp(type, "estop") == 0) {
      estopEnabled = doc["enabled"] | false;
      if (estopEnabled) {
        controlMode = MODE_ESTOP;
        stopBase();
      } else {
        // When releasing estop, default back to CRSF unless you prefer USB
        controlMode = MODE_CRSF;
      }
      return;
    }

    if (strcmp(type, "cmd_vel") == 0) {
      cmdVelV = doc["v"] | 0.0f;
      cmdVelW = doc["w"] | 0.0f;
      controlMode = MODE_USB;
      motorTimedActive = false; // cancel timed motor command
      return;
    }
  }

  // --- Legacy format (backward compatible) ---
  if (doc.containsKey("servos")) {
    JsonObject servos = doc["servos"];

    if (servos.containsKey("speed")) setSpeedForAllServos(servos["speed"].as<int>());

    if (servos.containsKey("waist"))    { tWaist = servos["waist"].as<int>(); sWaist.startEaseTo((int)tWaist); }
    if (servos.containsKey("shoulder")) {
      tShoulder = servos["shoulder"].as<int>();
      sShoulder1.startEaseTo((int)tShoulder);
      sShoulder2.startEaseTo(180 - (int)tShoulder);
    }
    if (servos.containsKey("elbow"))    { tElbow  = servos["elbow"].as<int>();  sElbow.startEaseTo((int)tElbow); }
    if (servos.containsKey("wristP"))   { tWristP = servos["wristP"].as<int>(); sWristP.startEaseTo((int)tWristP); }
    if (servos.containsKey("wristR"))   { tWristR = servos["wristR"].as<int>(); sWristR.startEaseTo((int)tWristR); }
    if (servos.containsKey("gripper"))  { tGripper= servos["gripper"].as<int>();sGripper.startEaseTo((int)tGripper); }

    controlMode = MODE_USB; // treat as USB ownership
  }

  if (doc.containsKey("motors")) {
    JsonObject motors = doc["motors"];
    int left = motors["left"] | 0;
    int right = motors["right"] | 0;
    int duration = motors["duration"] | 0;

    controlMode = MODE_USB;
    cmdVelV = 0; cmdVelW = 0; // if raw motors, cancel cmd_vel

    if (duration > 0) {
      motorTimedActive = true;
      motorTimedLeft = constrain(left, -255, 255);
      motorTimedRight = constrain(right, -255, 255);
      motorEndMs = millis() + (unsigned long)duration;
    } else {
      motorTimedActive = false;
      currentLeftSpeed = constrain(left, -255, 255);
      currentRightSpeed = constrain(right, -255, 255);
      setMotor(PIN_LEFT_PWM, PIN_LEFT_DIR, currentLeftSpeed);
      setMotor(PIN_RIGHT_PWM, PIN_RIGHT_DIR, currentRightSpeed);
    }
  }
}

void pollUsbJsonNonBlocking() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    // handle CRLF
    if (c == '\r') continue;

    if (c == '\n') {
      rxLine[rxLen] = '\0';
      if (rxLen > 0) {
        StaticJsonDocument<512> doc;
        DeserializationError err = deserializeJson(doc, rxLine);
        if (!err) {
          handleJsonDocument(doc);
        }
      }
      rxLen = 0;
      continue;
    }

    // accumulate
    if (rxLen < sizeof(rxLine) - 1) {
      rxLine[rxLen++] = c;
    } else {
      // overflow -> reset line (avoid blocking/garbage)
      rxLen = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(CRSF_BAUDRATE);
  crsf.begin(Serial2);

  Serial.println("Attach Servos...");

  sWaist.attach(PIN_WAIST, 90, 544, 2400);

  sShoulder1.attach(PIN_SHOULDER_1, 180, 544, 2400);
  sShoulder2.attach(PIN_SHOULDER_2, 0, 544, 2400);

  sElbow.attach(PIN_ELBOW, 180, 544, 2400);

  sWristP.attach(PIN_WRIST_P, 90, 544, 2400);
  sWristR.attach(PIN_WRIST_R, 90, 544, 2400);
  sGripper.attach(PIN_GRIPPER, 90, 544, 2400);

  Serial.println("Starting Homing Sequence (PWM -> Angle)...");

  setSpeedForAllServos(STARTUP_SPEED);

  tWaist    = pwmToAngle(WAIST_HOME_PWM);
  tShoulder = pwmToAngle(SHOULDER_HOME_PWM);
  tElbow    = pwmToAngle(ELBOW_HOME_PWM);
  tWristP   = pwmToAngle(WRIST_P_HOME_PWM);
  tWristR   = pwmToAngle(WRIST_R_HOME_PWM);
  tGripper  = pwmToAngle(GRIPPER_HOME_PWM);

  sWaist.startEaseTo((int)tWaist);

  sShoulder1.startEaseTo((int)tShoulder);
  sShoulder2.startEaseTo(180 - (int)tShoulder);

  sElbow.startEaseTo((int)tElbow);
  sWristP.startEaseTo((int)tWristP);
  sWristR.startEaseTo((int)tWristR);
  sGripper.startEaseTo((int)tGripper);

  while (sWaist.isMoving() || sShoulder1.isMoving() || sShoulder2.isMoving() ||
         sElbow.isMoving() || sWristP.isMoving() || sWristR.isMoving() || sGripper.isMoving()) {
    delay(20);
  }

  Serial.println("Homing Complete.");
  setSpeedForAllServos(ARM_SPEED);

  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_DIR, OUTPUT);

  stopBase();
  lastUsbCmdMs = millis();

  Serial.println("Robot Ready.");
}

void loop() {
  // Always update CRSF (non-blocking)
  crsf.update();

  // Read USB JSON (non-blocking line parser)
  pollUsbJsonNonBlocking();

  // ESTOP has priority
  if (estopEnabled || controlMode == MODE_ESTOP) {
    stopBase();
  } else {
    // USB watchdog only in USB mode
    if (controlMode == MODE_USB) {
      if (millis() - lastUsbCmdMs > USB_WATCHDOG_MS) {
        // timeout -> stop base, keep mode USB but zero cmd_vel
        stopBase();
        cmdVelV = 0; cmdVelW = 0;
        motorTimedActive = false;
      }
    }

    if (controlMode == MODE_CRSF) {
      if (crsf.isLinkUp()) {
        readCrsfAndControl();
      } else {
        stopBase();
      }
    } else if (controlMode == MODE_USB) {
      // timed motor command overrides cmd_vel
      if (motorTimedActive) {
        if (millis() < motorEndMs) {
          currentLeftSpeed = motorTimedLeft;
          currentRightSpeed = motorTimedRight;
          setMotor(PIN_LEFT_PWM, PIN_LEFT_DIR, currentLeftSpeed);
          setMotor(PIN_RIGHT_PWM, PIN_RIGHT_DIR, currentRightSpeed);
        } else {
          motorTimedActive = false;
          stopBase();
        }
      } else {
        applyCmdVel();
      }
    }
  }

  // Periodic status output
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    printStatusJSON();
  }
}
