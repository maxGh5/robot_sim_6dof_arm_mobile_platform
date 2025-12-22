/*
 * Design A - Smooth Robot Control (ServoEasing + CRSF)
 * Hardware: Arduino Mega 2560 + RadioMaster BR1
 * * DEPENDENCIES:
 * 1. AlfredoCRSF (Manage Libraries -> "AlfredoCRSF")
 * 2. ServoEasing (Manage Libraries -> "ServoEasing")
 */

#include <AlfredoCRSF.h>
#include <ServoEasing.hpp> // Note the .hpp extension for this library

// --- Configuration ---

// CRSF Setup (RX1 on Mega is Pin 19)
#define CRSF_BAUDRATE 420000

// Pin Definitions
const int PIN_WAIST     = 2;
const int PIN_SHOULDER  = 3;
const int PIN_ELBOW     = 4;
const int PIN_WRIST_P   = 5;
const int PIN_WRIST_R   = 6;
const int PIN_GRIPPER   = 7;

// Base Motor Pins
const int PIN_LEFT_PWM  = 8;
const int PIN_LEFT_DIR  = 9;
const int PIN_RIGHT_PWM = 10;
const int PIN_RIGHT_DIR = 11;

// --- Tuning ---
// Speed is in Degrees per Second. 
// Lower = Smoother/Heavier feel. Higher = Snappier.
const int ARM_SPEED = 90;      
const int GRIPPER_SPEED = 200; 

// --- Objects ---
AlfredoCRSF crsf;
ServoEasing sWaist;
ServoEasing sShoulder;
ServoEasing sElbow;
ServoEasing sWristP;
ServoEasing sWristR;
ServoEasing sGripper;

// --- Variables ---
int chDrive, chTurn;
int chWaist, chShoulder, chElbow, chWristP, chWristR, chGripper;

void setup() {
  Serial.begin(115200);
  Serial1.begin(CRSF_BAUDRATE);
  crsf.begin(Serial1);

  // 1. Attach Servos with Initial Angles
  // attach(pin, start_angle)
  sWaist.attach(PIN_WAIST, 90);
  sShoulder.attach(PIN_SHOULDER, 90);
  sElbow.attach(PIN_ELBOW, 90);
  sWristP.attach(PIN_WRIST_P, 90);
  sWristR.attach(PIN_WRIST_R, 90);
  sGripper.attach(PIN_GRIPPER, 0);

  // 2. Configure Smoothing
  // EASE_CUBIC_IN_OUT gives a nice "Slow Start -> Fast -> Slow Stop" motion
  sWaist.setEasingType(EASE_CUBIC_IN_OUT);
  sShoulder.setEasingType(EASE_CUBIC_IN_OUT);
  sElbow.setEasingType(EASE_CUBIC_IN_OUT);
  sWristP.setEasingType(EASE_CUBIC_IN_OUT);
  sWristR.setEasingType(EASE_CUBIC_IN_OUT);
  sGripper.setEasingType(EASE_LINEAR); // Grippers usually want linear response

  // 3. Set Speed (Degrees per Second)
  setSpeedForAllServos(ARM_SPEED);
  sGripper.setSpeed(GRIPPER_SPEED); // Gripper can be faster

  // Base Motor Setup
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_DIR, OUTPUT);
  
  Serial.println("Robot Initialized with ServoEasing.");
}

void loop() {
  // 1. Update Receiver
  crsf.update();

  // 2. Control Loop
  if (crsf.isLinkUp()) {
    
    // --- Read Channels (Adjust mapping to your Radio) ---
    chWaist    = crsf.getChannel(1);
    chShoulder = crsf.getChannel(2);
    chDrive    = crsf.getChannel(3);
    chTurn     = crsf.getChannel(4);
    chElbow    = crsf.getChannel(5);
    chWristP   = crsf.getChannel(6);
    chWristR   = crsf.getChannel(7);
    chGripper  = crsf.getChannel(8);

    // --- Arm Control (Non-Blocking) ---
    // startEaseTo() tells the library to calculate the curve to the new target.
    // It runs in the background using interrupts/timers.
    
    // Waist
    int angleWaist = map(chWaist, 1000, 2000, 0, 180);
    if (sWaist.getCurrentAngle() != angleWaist) sWaist.startEaseTo(angleWaist);

    // Shoulder
    int angleShoulder = map(chShoulder, 1000, 2000, 45, 135); // Limited range
    if (sShoulder.getCurrentAngle() != angleShoulder) sShoulder.startEaseTo(angleShoulder);

    // Elbow
    int angleElbow = map(chElbow, 1000, 2000, 0, 180);
    if (sElbow.getCurrentAngle() != angleElbow) sElbow.startEaseTo(angleElbow);

    // Wrist Pitch
    int angleWristP = map(chWristP, 1000, 2000, 0, 180);
    if (sWristP.getCurrentAngle() != angleWristP) sWristP.startEaseTo(angleWristP);

    // Wrist Roll
    int angleWristR = map(chWristR, 1000, 2000, 0, 180);
    if (sWristR.getCurrentAngle() != angleWristR) sWristR.startEaseTo(angleWristR);

    // Gripper
    int angleGrip = (chGripper > 1500) ? 90 : 0;
    if (sGripper.getCurrentAngle() != angleGrip) sGripper.startEaseTo(angleGrip);

    // --- Base Control (Standard PWM) ---
    controlMobileBase(chDrive, chTurn);

  } else {
    stopBase();
  }
  
  // Note: ServoEasing on AVR (Mega) uses interrupts, so we don't strictly 
  // need to call a update() function in the loop for the servos to move.
}

// --- Helper Functions ---

void controlMobileBase(int throttle, int steering) {
  int speedFwd = map(throttle, 1000, 2000, -255, 255);
  int speedTurn = map(steering, 1000, 2000, -255, 255);

  if (abs(speedFwd) < 20) speedFwd = 0;
  if (abs(speedTurn) < 20) speedTurn = 0;

  int leftSpeed = speedFwd + speedTurn;
  int rightSpeed = speedFwd - speedTurn;

  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  setMotor(PIN_LEFT_PWM, PIN_LEFT_DIR, leftSpeed);
  setMotor(PIN_RIGHT_PWM, PIN_RIGHT_DIR, rightSpeed);
}

void setMotor(int pinPWM, int pinDir, int speed) {
  if (speed > 0) {
    digitalWrite(pinDir, HIGH);
    analogWrite(pinPWM, speed);
  } else {
    digitalWrite(pinDir, LOW);
    analogWrite(pinPWM, abs(speed));
  }
}

void stopBase() {
  analogWrite(PIN_LEFT_PWM, 0);
  analogWrite(PIN_RIGHT_PWM, 0);
}

// Helper to set speed for all arm joints
void setSpeedForAllServos(int speed) {
  sWaist.setSpeed(speed);
  sShoulder.setSpeed(speed);
  sElbow.setSpeed(speed);
  sWristP.setSpeed(speed);
  sWristR.setSpeed(speed);
}
