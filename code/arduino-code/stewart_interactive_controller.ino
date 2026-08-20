#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pca9685(0x40);

constexpr uint8_t SERVO_COUNT = 6;
constexpr uint8_t LASER_PIN = D7;
constexpr float PCA_FREQUENCY = 50.0f;
constexpr unsigned long MOVE_MS = 180;    // Smooth response time per key press

// Runtime-adjustable control settings. Use keys 1-5 for step size and 6-9
// for range. The sketch always starts in a conservative mode after reset.
float stepUs = 10.0f;
float poseLimitUs = 100.0f;
float combinedLimitUs = 220.0f;

const uint8_t servoChannel[SERVO_COUNT] = {0, 1, 2, 3, 4, 5};

// Change an entry if that installed servo moves opposite to the desired motion.
int8_t servoDirection[SERVO_COUNT] = {+1, -1, +1, -1, +1, -1};

uint16_t servoCenterUs[SERVO_COUNT] = {1500, 1500, 1500, 1500, 1500, 1500};
// Wide hard limits for a nominal 500-2500 us / 180-degree DS3225. The 50 us
// endpoint margin avoids deliberately driving against the mechanical stops.
uint16_t servoMinUs[SERVO_COUNT] = {550, 550, 550, 550, 550, 550};
uint16_t servoMaxUs[SERVO_COUNT] = {2450, 2450, 2450, 2450, 2450, 2450};

float currentPulseUs[SERVO_COUNT] = {1500, 1500, 1500, 1500, 1500, 1500};

// Servo positions viewed from above:
//
//                 BACK
//              3       4
//          2               5
//              1       0
//                 FRONT
const float servoX[SERVO_COUNT] = {0.5, -0.5, -1.0, -0.5, 0.5, 1.0};
const float servoY[SERVO_COUNT] = {-0.866, -0.866, 0.0, 0.866, 0.866, 0.0};

// Preliminary alternating-leg yaw pattern. This is only a diagnostic until
// the real base/platform joint coordinates and horn geometry are calibrated.
const float servoYaw[SERVO_COUNT] = {1.0, -1.0, 1.0, -1.0, 1.0, -1.0};

// Persistent "game" controls. Repeated key presses accumulate here.
float poseX = 0;
float poseY = 0;
float poseZ = 0;
float poseRoll = 0;
float posePitch = 0;
float poseYaw = 0;
bool laserIsOn = false;

void setLaser(bool turnOn) {
  laserIsOn = turnOn;
  digitalWrite(LASER_PIN, laserIsOn ? HIGH : LOW);
  Serial.print("Laser: ");
  Serial.println(laserIsOn ? "ON" : "OFF");
}

void toggleLaser() {
  setLaser(!laserIsOn);
}

float clampValue(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

uint16_t constrainPulse(uint8_t servo, float pulseUs) {
  return (uint16_t)round(clampValue(pulseUs, servoMinUs[servo], servoMaxUs[servo]));
}

void writeServoPulse(uint8_t servo, float pulseUs) {
  uint16_t safePulse = constrainPulse(servo, pulseUs);
  pca9685.writeMicroseconds(servoChannel[servo], safePulse);
  currentPulseUs[servo] = safePulse;
}

void moveToLogicalOffsets(const float requested[SERVO_COUNT], unsigned long durationMs) {
  float startPulse[SERVO_COUNT];
  float endPulse[SERVO_COUNT];
  float largest = 0;

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    largest = max(largest, fabs(requested[i]));
  }

  // Preserve the requested shape while scaling it into the safe envelope.
  float scale = largest > combinedLimitUs ? combinedLimitUs / largest : 1.0f;

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    startPulse[i] = currentPulseUs[i];
    float logicalOffset = requested[i] * scale;
    endPulse[i] = servoCenterUs[i] + servoDirection[i] * logicalOffset;
  }

  constexpr unsigned long UPDATE_MS = 20;
  unsigned long steps = max(1UL, durationMs / UPDATE_MS);

  for (unsigned long step = 1; step <= steps; step++) {
    float fraction = (float)step / (float)steps;
    float smooth = fraction * fraction * (3.0f - 2.0f * fraction);

    for (uint8_t servo = 0; servo < SERVO_COUNT; servo++) {
      float pulse = startPulse[servo] + (endPulse[servo] - startPulse[servo]) * smooth;
      writeServoPulse(servo, pulse);
    }
    delay(UPDATE_MS);
  }
}

void applyPose() {
  float offsets[SERVO_COUNT];

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    // X/Y are diagnostic sway controls, not true Cartesian translations.
    // Roll/pitch use the same simple geometric weighting from the test sketch.
    offsets[i] = poseZ
               + poseX * servoX[i]
               + poseY * servoY[i]
               + poseRoll * servoX[i]
               + posePitch * servoY[i]
               + poseYaw * servoYaw[i];
  }

  moveToLogicalOffsets(offsets, MOVE_MS);
}

void printPose() {
  Serial.print("Pose  X="); Serial.print(poseX, 0);
  Serial.print("  Y="); Serial.print(poseY, 0);
  Serial.print("  Z="); Serial.print(poseZ, 0);
  Serial.print("  Roll="); Serial.print(poseRoll, 0);
  Serial.print("  Pitch="); Serial.print(posePitch, 0);
  Serial.print("  Yaw="); Serial.print(poseYaw, 0);
  Serial.print("  | step="); Serial.print(stepUs, 0);
  Serial.print("  range=+/-"); Serial.println(combinedLimitUs, 0);
}

void changeControl(float &control, float amount) {
  control = clampValue(control + amount, -poseLimitUs, poseLimitUs);
  applyPose();
  printPose();
}

void setStep(float newStepUs) {
  stepUs = newStepUs;
  Serial.print("Step size set to ");
  Serial.print(stepUs, 0);
  Serial.println(" us per key press");
}

void setRange(float newPoseLimitUs, float newCombinedLimitUs) {
  poseLimitUs = newPoseLimitUs;
  combinedLimitUs = newCombinedLimitUs;

  // Bring an already accumulated pose inside the newly selected envelope.
  poseX = clampValue(poseX, -poseLimitUs, poseLimitUs);
  poseY = clampValue(poseY, -poseLimitUs, poseLimitUs);
  poseZ = clampValue(poseZ, -poseLimitUs, poseLimitUs);
  poseRoll = clampValue(poseRoll, -poseLimitUs, poseLimitUs);
  posePitch = clampValue(posePitch, -poseLimitUs, poseLimitUs);
  poseYaw = clampValue(poseYaw, -poseLimitUs, poseLimitUs);
  applyPose();

  Serial.print("Range set: each control +/-");
  Serial.print(poseLimitUs, 0);
  Serial.print(" us; combined servo limit +/-");
  Serial.print(combinedLimitUs, 0);
  Serial.println(" us");
}

void homePlatform() {
  poseX = poseY = poseZ = poseRoll = posePitch = poseYaw = 0;
  applyPose();
  Serial.println("HOME");
  printPose();
}

void demoPause() {
  delay(650);
}

void runMasterDemo() {
  // Keep this deliberately smaller than the selected interactive range.
  float amount = min(poseLimitUs, 120.0f);

  Serial.println("MASTER DEMO START");
  Serial.println("X/Y/yaw are diagnostic patterns until geometric calibration.");
  homePlatform();
  demoPause();

  Serial.println("Z + / -");
  poseZ = amount; applyPose(); demoPause();
  poseZ = -amount; applyPose(); demoPause();
  homePlatform(); demoPause();

  Serial.println("X sway + / - (translation and tilt are currently coupled)");
  poseX = amount; applyPose(); demoPause();
  poseX = -amount; applyPose(); demoPause();
  homePlatform(); demoPause();

  Serial.println("Y sway + / - (translation and tilt are currently coupled)");
  poseY = amount; applyPose(); demoPause();
  poseY = -amount; applyPose(); demoPause();
  homePlatform(); demoPause();

  Serial.println("Roll + / -");
  poseRoll = amount; applyPose(); demoPause();
  poseRoll = -amount; applyPose(); demoPause();
  homePlatform(); demoPause();

  Serial.println("Pitch + / -");
  posePitch = amount; applyPose(); demoPause();
  posePitch = -amount; applyPose(); demoPause();
  homePlatform(); demoPause();

  Serial.println("Yaw diagnostic + / -");
  poseYaw = amount * 0.6f; applyPose(); demoPause();
  poseYaw = -amount * 0.6f; applyPose(); demoPause();
  homePlatform();

  Serial.println("MASTER DEMO COMPLETE; HOME");
}

void printMenu() {
  Serial.println();
  Serial.println("Interactive Stewart Platform");
  Serial.println("Motion controls:");
  Serial.println("  x / X : X sway + / -");
  Serial.println("  y / Y : Y sway + / -");
  Serial.println("  u / d : Z up / down");
  Serial.println("  r / R : roll + / -");
  Serial.println("  p / P : pitch + / -");
  Serial.println("  a / A : yaw diagnostic + / -");
  Serial.println("  h     : smooth return HOME");
  Serial.println("  t     : master motion demonstration");
  Serial.println("  l     : laser ON");
  Serial.println("  L     : laser OFF");
  Serial.println("  k     : toggle laser ON/OFF");
  Serial.println();
  Serial.println("Step-size gears:");
  Serial.println("  1=5 us, 2=10 us, 3=25 us, 4=50 us, 5=100 us");
  Serial.println("Range gears (approximate; verify your own servo calibration):");
  Serial.println("  6=+/-220 us  conservative");
  Serial.println("  7=+/-450 us  medium");
  Serial.println("  8=+/-700 us  large");
  Serial.println("  9=+/-900 us  near full nominal travel");
  Serial.println("  ?     : show this menu");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  // Establish the safe state before initializing the servo controller.
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);
  laserIsOn = false;

  Wire.begin(A4, A5);
  pca9685.begin();
  pca9685.setOscillatorFrequency(25000000);
  pca9685.setPWMFreq(PCA_FREQUENCY);
  delay(500);

  for (uint8_t servo = 0; servo < SERVO_COUNT; servo++) {
    currentPulseUs[servo] = servoCenterUs[servo];
    writeServoPulse(servo, servoCenterUs[servo]);
    delay(100);
  }

  Serial.println("PCA9685 initialized; platform at HOME.");
  Serial.println("Laser initialized OFF.");
  printMenu();
}

void loop() {
  if (!Serial.available()) return;

  char command = Serial.read();

  switch (command) {
    case 'x': changeControl(poseX, +stepUs); break;
    case 'X': changeControl(poseX, -stepUs); break;
    case 'y': changeControl(poseY, +stepUs); break;
    case 'Y': changeControl(poseY, -stepUs); break;
    case 'u': changeControl(poseZ, +stepUs); break;
    case 'd': changeControl(poseZ, -stepUs); break;
    case 'r': changeControl(poseRoll, +stepUs); break;
    case 'R': changeControl(poseRoll, -stepUs); break;
    case 'p': changeControl(posePitch, +stepUs); break;
    case 'P': changeControl(posePitch, -stepUs); break;
    case 'a': changeControl(poseYaw, +stepUs); break;
    case 'A': changeControl(poseYaw, -stepUs); break;
    case 'h': homePlatform(); break;
    case 't': runMasterDemo(); break;
    case 'l': setLaser(true); break;
    case 'L': setLaser(false); break;
    case 'k': toggleLaser(); break;
    case '1': setStep(1); break;
    case '2': setStep(10); break;
    case '3': setStep(25); break;
    case '4': setStep(50); break;
    case '5': setStep(100); break;
    case '6': setRange(220, 220); break;
    case '7': setRange(450, 450); break;
    case '8': setRange(700, 700); break;
    case '9': setRange(900, 900); break;
    case '?': printMenu(); break;
    case '\n':
    case '\r': break;
    default:
      Serial.print("Unknown key: ");
      Serial.println(command);
      break;
  }
}
