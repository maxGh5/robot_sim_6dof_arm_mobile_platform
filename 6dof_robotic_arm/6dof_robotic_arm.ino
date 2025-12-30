#include <AlfredoCRSF.h>
#include <ServoEasing.hpp> // Note the .hpp extension for this library
#include <Servo.h>         // Explicitly include Servo for direct control
#include <ArduinoJson.h>

// --- Configuration ---

// CRSF Setup (RX2 on Mega is Pin 17, TX2 is Pin 16)
#define CRSF_BAUDRATE 115200 // Must match Receiver Output Baud Rate

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
// Speed is in Degrees per Second. 
// Lower = Smoother/Heavier feel. Higher = Snappier.
const int ARM_SPEED = 90;      
const int GRIPPER_SPEED = 200; 
const float SENSITIVITY = 0.005; // 0.005 * 500 = 2.5 deg/loop (Fast)
const float ELBOW_SENSITIVITY = 0.0001; // Slower for direct Servo control
const float SHOULDER_SENSITIVITY = 0.0002; // Slower for heavy shoulder
const int DEADZONE = 30;

// State Variables for Relative Control
float tWaist, tShoulder, tElbow, tWristP, tWristR, tGripper; 

// --- Objects ---
AlfredoCRSF crsf;
ServoEasing sWaist;
Servo sShoulder1; // CHANGED: Direct Servo
Servo sShoulder2; // CHANGED: Direct Servo
Servo sElbow; 
ServoEasing sWristP;
ServoEasing sWristR;
ServoEasing sGripper;

// --- Variables ---
int chDrive, chTurn;
int chWaist, chShoulder, chElbow, chWristP, chWristR, chGripper;

void setup() {
  Serial.begin(115200);
  Serial2.begin(CRSF_BAUDRATE);
  crsf.begin(Serial2);

  // 1. Attach Servos with Initial Angles
  // attach(pin, start_angle)
  sWaist.attach(PIN_WAIST, 90);    tWaist = 90;
  
  // Shoulder Direct Config
  sShoulder1.attach(PIN_SHOULDER_1);
  sShoulder2.attach(PIN_SHOULDER_2);
  tShoulder = 180;
  sShoulder1.write((int)tShoulder);
  sShoulder2.write(180 - (int)tShoulder);

  
  // Elbow Direct Config
  sElbow.attach(PIN_ELBOW); 
  sElbow.write(180);
  tElbow = 180;

  sWristP.attach(PIN_WRIST_P, 100); tWristP = 100;
  sWristR.attach(PIN_WRIST_R, 180); tWristR = 180;
  sGripper.attach(PIN_GRIPPER, 0);  tGripper = 0;

  // 2. Configure Smoothing
  // EASE_CUBIC_IN_OUT gives a nice "Slow Start -> Fast -> Slow Stop" motion
  sWaist.setEasingType(EASE_CUBIC_IN_OUT);
  // sShoulder1.setEasingType(EASE_CUBIC_IN_OUT); // Direct
  // sShoulder2.setEasingType(EASE_CUBIC_IN_OUT); // Direct
  // sElbow.setEasingType(EASE_CUBIC_IN_OUT); // Removed for direct control
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
  
  Serial.println("Robot Initialized (Elbow & Shoulder using Standard Servo).");
}

void loop() {
  // 1. Update Receiver
  crsf.update();

  // 2. Control Loop
  if (crsf.isLinkUp()) {
    
    // --- Read Channels (Adjust mapping to your Radio) ---
    chWaist    = crsf.getChannel(4); // Mapped to Ch4 (Rudder/Left Stick X) to avoid conflict with Steering
    chShoulder = crsf.getChannel(2);
    chDrive    = crsf.getChannel(3);
    chTurn     = crsf.getChannel(1); // Mapped to Ch1 (Aileron/Right Stick X)
    chElbow    = crsf.getChannel(6); // Swapped to Ch6 (Pot?)
    chWristP   = crsf.getChannel(5); // Swapped to Ch5 (Switch?)
    chWristR   = crsf.getChannel(7);
    chGripper  = crsf.getChannel(8);

    // DEBUG: Elbow & Shoulder
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200) { // Print 5 times/sec
      lastPrint = millis();
      Serial.print("Elbow: "); Serial.print(tElbow);
      Serial.print(" | Shou: "); Serial.println(tShoulder);
    }

    // --- Arm Control (Relative/Rate) ---
    // Update target angles based on joystick deflection
    
    // Waist
    if (abs(chWaist - 1500) > DEADZONE) tWaist += (chWaist - 1500) * SENSITIVITY;
    tWaist = constrain(tWaist, 0, 180);
    if (sWaist.getCurrentAngle() != (int)tWaist) sWaist.startEaseTo((int)tWaist);

    // Shoulder - Direct Write
    if (abs(chShoulder - 1500) > DEADZONE) tShoulder += (chShoulder - 1500) * SHOULDER_SENSITIVITY;
    tShoulder = constrain(tShoulder, 0, 180);                                                 
    // Write to both servos, inversely
    sShoulder1.write((int)tShoulder);
    sShoulder2.write(180 - (int)tShoulder);

    // Elbow - Direct Write
    if (abs(chElbow - 1500) > DEADZONE) tElbow += (chElbow - 1500) * ELBOW_SENSITIVITY;
    tElbow = constrain(tElbow, 0, 180);
    // Since we are not easing, we write directly. 
    // To smooth it slightly, the relative update rate IS the smoothing.
    sElbow.write((int)tElbow);

    // Wrist Pitch
    if (abs(chWristP - 1500) > DEADZONE) tWristP += (chWristP - 1500) * SENSITIVITY;
    tWristP = constrain(tWristP, 0, 180);
    if (sWristP.getCurrentAngle() != (int)tWristP) sWristP.startEaseTo((int)tWristP);

    // Wrist Roll
    if (abs(chWristR - 1500) > DEADZONE) tWristR += (chWristR - 1500) * SENSITIVITY;
    tWristR = constrain(tWristR, 0, 180);
    if (sWristR.getCurrentAngle() != (int)tWristR) sWristR.startEaseTo((int)tWristR);

    // Gripper (Keep absolute toggle-like or Rate? Let's use Rate for precision)
    if (abs(chGripper - 1500) > DEADZONE) tGripper += (chGripper - 1500) * SENSITIVITY;
    tGripper = constrain(tGripper, 0, 90); // Gripper usually 0-90
    if (sGripper.getCurrentAngle() != (int)tGripper) sGripper.startEaseTo((int)tGripper);

    // --- Base Control (Standard PWM) ---
    controlMobileBase(chDrive, chTurn);

  } else {
    stopBase();
  }
  

  // Note: ServoEasing on AVR (Mega) uses interrupts, so we don't strictly 
  // need to call a update() function in the loop for the servos to move.
  
  // 3. JSON Control (USB)
  if (Serial.available()) {
    processJSON();
  }
}

// --- Helper Functions ---

void controlMobileBase(int throttle, int steering) {
  int speedFwd = map(throttle, 1000, 2000, -255, 255);
  int speedTurn = map(steering, 1000, 2000, -255, 255);

  if (abs(speedFwd) < 20) speedFwd = 0;
  if (abs(speedTurn) < 20) speedTurn = 0;

  int leftSpeed = speedFwd - speedTurn;
  int rightSpeed = speedFwd + speedTurn;

  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  setMotor(PIN_LEFT_PWM, PIN_LEFT_DIR, leftSpeed);
  setMotor(PIN_RIGHT_PWM, PIN_RIGHT_DIR, rightSpeed);
}

void setMotor(int pinPWM, int pinDir, int speed) {
  if (speed > 0) {
    // Forward (Stick Up) - Normal Logic
    // Based on testing: DIR=LOW is Forward, PWM 255 is Fast.
    digitalWrite(pinDir, LOW);
    analogWrite(pinPWM, speed); 
  } else {
    // Backward (Stick Down) - Inverted Logic
    // Based on testing: DIR=HIGH is Backward, PWM 0 is Fast (255 is Stop).
    digitalWrite(pinDir, HIGH);
    analogWrite(pinPWM, 255 - abs(speed));
  }
}

void stopBase() {
  // Safe Stop: DIR=HIGH (Inverted) + PWM 255 (Stop)
  digitalWrite(PIN_LEFT_DIR, HIGH);
  digitalWrite(PIN_RIGHT_DIR, HIGH);
  analogWrite(PIN_LEFT_PWM, 255);
  analogWrite(PIN_RIGHT_PWM, 255);
}

// Helper to set speed for all arm joints
void setSpeedForAllServos(int speed) {
  sWaist.setSpeed(speed);
  // sShoulder1.setSpeed(speed);
  // sShoulder2.setSpeed(speed); 
  // sElbow.setSpeed(speed);
  sWristP.setSpeed(speed);
  sWristR.setSpeed(speed);
  sWristP.setSpeed(speed);
  sWristR.setSpeed(speed);
}

void processJSON() {
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, Serial);

  if (error) {
    // If it's just a newline or garbage, ignore or print error
    // Serial.print(F("deserializeJson() failed: ")); 
    // Serial.println(error.f_str());
    return;
  }

  // --- Process Servos ---
  if (doc.containsKey("servos")) {
    JsonObject servos = doc["servos"];
    
    // Check for speed first
    if (servos.containsKey("speed")) {
       setSpeedForAllServos(servos["speed"].as<int>());
    }

    if (servos.containsKey("waist"))    { tWaist = servos["waist"].as<int>(); sWaist.startEaseTo((int)tWaist); }
    if (servos.containsKey("shoulder")) {
      tShoulder = servos["shoulder"].as<int>();
      sShoulder1.write((int)tShoulder); 
      sShoulder2.write(180 - (int)tShoulder);
    }
    if (servos.containsKey("elbow"))    { tElbow = servos["elbow"].as<int>(); sElbow.write((int)tElbow); } // Direct
    if (servos.containsKey("wristP"))   { tWristP = servos["wristP"].as<int>(); sWristP.startEaseTo((int)tWristP); }
    if (servos.containsKey("wristR"))   { tWristR = servos["wristR"].as<int>(); sWristR.startEaseTo((int)tWristR); }
    if (servos.containsKey("gripper"))  { tGripper = servos["gripper"].as<int>(); sGripper.startEaseTo((int)tGripper); }
  }

  // --- Process Motors ---
  if (doc.containsKey("motors")) {
    JsonObject motors = doc["motors"];
    int left = motors["left"] | 0;
    int right = motors["right"] | 0;
    int duration = motors["duration"] | 0;

    setMotor(PIN_LEFT_PWM, PIN_LEFT_DIR, left);
    setMotor(PIN_RIGHT_PWM, PIN_RIGHT_DIR, right);

    if (duration > 0) {
      delay(duration); // Blocking delay for motor duration if specified
      setMotor(PIN_LEFT_PWM, PIN_LEFT_DIR, 0);
      setMotor(PIN_RIGHT_PWM, PIN_RIGHT_DIR, 0);
    }
  }
}
