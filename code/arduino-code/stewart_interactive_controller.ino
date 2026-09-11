// This sketch is written with the help of ChatGPT

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <WiFi.h>

Adafruit_PWMServoDriver pca9685(0x40);

constexpr uint8_t SERVO_COUNT = 6;
constexpr uint8_t LASER_PIN = D7;
constexpr uint8_t JOYSTICK_X_PIN = A0;
constexpr uint8_t JOYSTICK_Y_PIN = A1;
constexpr uint8_t JOYSTICK_BUTTON_PIN = D2;
constexpr float PCA_FREQUENCY = 50.0f;
constexpr unsigned long MOVE_MS = 180;    // Smooth response time per key press

// Joystick tuning for the Nano ESP32's 12-bit ADC (nominally 0-4095).
// Holding the stick farther from center produces faster accumulated motion.
constexpr int JOYSTICK_ADC_MAX = 4095;
constexpr int JOYSTICK_DEAD_ZONE = 180;
constexpr float JOYSTICK_MAX_RATE_US_PER_SECOND = 300.0f;
constexpr unsigned long JOYSTICK_UPDATE_MS = 40;
constexpr unsigned long BUTTON_DEBOUNCE_MS = 35;
constexpr unsigned long BUTTON_HOME_HOLD_MS = 1000;
constexpr unsigned long JOYSTICK_HOME_MOVE_MS = 600;
constexpr uint16_t WIFI_CONSOLE_PORT = 2323;
constexpr unsigned long WIFI_RETRY_MS = 15000;

// Replace only the password value before uploading. Keeping the Wi-Fi
// connection in this main sketch makes it reconnect after every restart.
const char* WIFI_SSID = "ULink";
const char* WIFI_PASSWORD = "xxxx";

WiFiServer consoleServer(WIFI_CONSOLE_PORT);
WiFiClient wifiConsole;
bool wifiServerStarted = false;
unsigned long lastWiFiAttemptMs = 0;

// Send status text to USB Serial and to the connected wireless console.
class TeeConsole : public Print {
 public:
  size_t write(uint8_t value) override {
    size_t written = Serial.write(value);
    if (wifiConsole && wifiConsole.connected()) {
      wifiConsole.write(value);
    }
    return written;
  }

  size_t write(const uint8_t* buffer, size_t size) override {
    size_t written = Serial.write(buffer, size);
    if (wifiConsole && wifiConsole.connected()) {
      wifiConsole.write(buffer, size);
    }
    return written;
  }
};

TeeConsole Console;

// Change either flag if an axis moves opposite to the direction you prefer.
constexpr bool REVERSE_JOYSTICK_ROLL = false;
constexpr bool REVERSE_JOYSTICK_PITCH = true;

// Runtime-adjustable control settings. Start in step gear 5 and range gear 8.
// Keys 1-5 and 6-9 can still change these settings after startup.
float stepUs = 100.0f;
float poseLimitUs = 700.0f;
float combinedLimitUs = 950.0f;

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

int joystickCenterX = JOYSTICK_ADC_MAX / 2;
int joystickCenterY = JOYSTICK_ADC_MAX / 2;
unsigned long lastJoystickUpdateMs = 0;
bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;
unsigned long lastButtonChangeMs = 0;
unsigned long buttonPressedMs = 0;
bool longPressHandled = false;
bool homeRequested = false;

void updateJoystickButton();
void homePlatform(unsigned long durationMs = MOVE_MS);
void handleCommand(char command);
void printPose();
void printMenu();

void beginWiFiAttempt() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWiFiAttemptMs = millis();
  Console.println("Connecting to Wi-Fi in background...");
}

void startWiFiConsole() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  Console.print("Nano ESP32 Wi-Fi MAC: ");
  Console.println(WiFi.macAddress());
  beginWiFiAttempt();
}

void serviceWiFiConsole() {
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiConsole) wifiConsole.stop();
    wifiServerStarted = false;

    if (millis() - lastWiFiAttemptMs >= WIFI_RETRY_MS) {
      WiFi.disconnect();
      beginWiFiAttempt();
    }
    return;
  }

  if (!wifiServerStarted) {
    consoleServer.begin();
    wifiServerStarted = true;
    Console.print("Wi-Fi console ready at ");
    Console.print(WiFi.localIP());
    Console.print(":");
    Console.println(WIFI_CONSOLE_PORT);
  }

  if (!wifiConsole || !wifiConsole.connected()) {
    WiFiClient newClient = consoleServer.available();
    if (newClient) {
      wifiConsole.stop();
      wifiConsole = newClient;
      wifiConsole.setNoDelay(true);
      Console.println("Wireless console connected.");
      printPose();
      printMenu();
    }
  }

  while (wifiConsole && wifiConsole.connected() && wifiConsole.available()) {
    handleCommand((char)wifiConsole.read());
  }
}

void setLaser(bool turnOn) {
  laserIsOn = turnOn;
  digitalWrite(LASER_PIN, laserIsOn ? HIGH : LOW);
  Console.print("Laser: ");
  Console.println(laserIsOn ? "ON" : "OFF");
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
    // The older motion code is blocking, so explicitly service the click here.
    updateJoystickButton();
    delay(UPDATE_MS);
  }
}

void applyPose(unsigned long durationMs = MOVE_MS) {
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

  moveToLogicalOffsets(offsets, durationMs);
}

void calibrateJoystickCenter() {
  constexpr uint8_t SAMPLE_COUNT = 32;
  long sumX = 0;
  long sumY = 0;

  Console.println("Release joystick and keep it centered: calibrating...");
  delay(300);
  for (uint8_t sample = 0; sample < SAMPLE_COUNT; sample++) {
    sumX += analogRead(JOYSTICK_X_PIN);
    sumY += analogRead(JOYSTICK_Y_PIN);
    delay(5);
  }

  joystickCenterX = sumX / SAMPLE_COUNT;
  joystickCenterY = sumY / SAMPLE_COUNT;
  lastJoystickUpdateMs = millis();

  Console.print("Joystick center: X=");
  Console.print(joystickCenterX);
  Console.print(" Y=");
  Console.println(joystickCenterY);
}

// Convert an ADC reading into a signed -1..+1 velocity command. Squaring the
// magnitude provides fine control near center while retaining full speed.
float joystickVelocity(int reading, int center, bool reverseAxis) {
  int offset = reading - center;
  int magnitude = abs(offset);
  if (magnitude <= JOYSTICK_DEAD_ZONE) return 0.0f;

  int availableRange = offset >= 0
                     ? JOYSTICK_ADC_MAX - center
                     : center;
  availableRange = max(availableRange, JOYSTICK_DEAD_ZONE + 1);

  float normalized = (float)(magnitude - JOYSTICK_DEAD_ZONE)
                   / (float)(availableRange - JOYSTICK_DEAD_ZONE);
  normalized = clampValue(normalized, 0.0f, 1.0f);
  normalized *= normalized;

  float velocity = offset >= 0 ? normalized : -normalized;
  return reverseAxis ? -velocity : velocity;
}

void updateJoystickButton() {
  bool reading = digitalRead(JOYSTICK_BUTTON_PIN);
  unsigned long now = millis();

  if (reading != lastButtonReading) {
    lastButtonChangeMs = now;
    lastButtonReading = reading;
  }

  if ((now - lastButtonChangeMs) >= BUTTON_DEBOUNCE_MS
      && reading != stableButtonState) {
    stableButtonState = reading;
    if (stableButtonState == LOW) {
      buttonPressedMs = now;
      longPressHandled = false;
    } else if (!longPressHandled) {
      // A short click keeps its original laser-toggle behavior.
      toggleLaser();
    }
  }

  if (stableButtonState == LOW
      && !longPressHandled
      && (now - buttonPressedMs) >= BUTTON_HOME_HOLD_MS) {
    // Defer the actual move until updateJoystick() is at top level. This
    // avoids starting HOME from inside an already-running smooth servo move.
    longPressHandled = true;
    homeRequested = true;
  }
}

void printJoystickButtonState() {
  int state = digitalRead(JOYSTICK_BUTTON_PIN);
  Console.print("Joystick SW on D2 is ");
  Console.print(state == LOW ? "PRESSED" : "released");
  Console.print(" (raw=");
  Console.print(state);
  Console.println("; expected pressed=0, released=1)");
}

void updateJoystick() {
  updateJoystickButton();

  if (homeRequested) {
    homeRequested = false;
    homePlatform(JOYSTICK_HOME_MOVE_MS);
    lastJoystickUpdateMs = millis();
    return;
  }

  // Do not let a displaced stick move the platform away while its button is
  // being held for HOME.
  if (stableButtonState == LOW) {
    lastJoystickUpdateMs = millis();
    return;
  }

  unsigned long now = millis();
  unsigned long elapsedMs = now - lastJoystickUpdateMs;
  if (elapsedMs < JOYSTICK_UPDATE_MS) return;
  lastJoystickUpdateMs = now;

  float rollVelocity = joystickVelocity(
    analogRead(JOYSTICK_X_PIN), joystickCenterX, REVERSE_JOYSTICK_ROLL
  );
  float pitchVelocity = joystickVelocity(
    analogRead(JOYSTICK_Y_PIN), joystickCenterY, REVERSE_JOYSTICK_PITCH
  );

  if (rollVelocity == 0.0f && pitchVelocity == 0.0f) return;

  // Limit a long scheduling pause so it cannot produce one unexpectedly large move.
  float elapsedSeconds = min(elapsedMs, 100UL) / 1000.0f;
  poseRoll = clampValue(
    poseRoll + rollVelocity * JOYSTICK_MAX_RATE_US_PER_SECOND * elapsedSeconds,
    -poseLimitUs,
    poseLimitUs
  );
  posePitch = clampValue(
    posePitch + pitchVelocity * JOYSTICK_MAX_RATE_US_PER_SECOND * elapsedSeconds,
    -poseLimitUs,
    poseLimitUs
  );

  applyPose(JOYSTICK_UPDATE_MS);
}

void printPose() {
  Console.print("Pose  X="); Console.print(poseX, 0);
  Console.print("  Y="); Console.print(poseY, 0);
  Console.print("  Z="); Console.print(poseZ, 0);
  Console.print("  Roll="); Console.print(poseRoll, 0);
  Console.print("  Pitch="); Console.print(posePitch, 0);
  Console.print("  Yaw="); Console.print(poseYaw, 0);
  Console.print("  | step="); Console.print(stepUs, 0);
  Console.print("  axis range=+/-"); Console.print(poseLimitUs, 0);
  Console.print("  leg limit=+/-"); Console.println(combinedLimitUs, 0);
}

void changeControl(float &control, float amount) {
  control = clampValue(control + amount, -poseLimitUs, poseLimitUs);
  applyPose();
  printPose();
}

void setStep(float newStepUs) {
  stepUs = newStepUs;
  Console.print("Step size set to ");
  Console.print(stepUs, 0);
  Console.println(" us per key press");
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

  Console.print("Range set: each control +/-");
  Console.print(poseLimitUs, 0);
  Console.print(" us; combined servo limit +/-");
  Console.print(combinedLimitUs, 0);
  Console.println(" us");
}

void homePlatform(unsigned long durationMs) {
  poseX = poseY = poseZ = poseRoll = posePitch = poseYaw = 0;
  applyPose(durationMs);
  Console.println("HOME");
  printPose();
}

void demoPause() {
  unsigned long startMs = millis();
  while (millis() - startMs < 650) {
    updateJoystickButton();
    delay(5);
  }
}

void runMasterDemo() {
  // Keep this deliberately smaller than the selected interactive range.
  float amount = min(poseLimitUs, 120.0f);

  Console.println("MASTER DEMO START");
  Console.println("X/Y/yaw are diagnostic patterns until geometric calibration.");
  homePlatform();
  demoPause();

  Console.println("Z + / -");
  poseZ = amount; applyPose(); demoPause();
  poseZ = -amount; applyPose(); demoPause();
  homePlatform(); demoPause();

  Console.println("X sway + / - (translation and tilt are currently coupled)");
  poseX = amount; applyPose(); demoPause();
  poseX = -amount; applyPose(); demoPause();
  homePlatform(); demoPause();

  Console.println("Y sway + / - (translation and tilt are currently coupled)");
  poseY = amount; applyPose(); demoPause();
  poseY = -amount; applyPose(); demoPause();
  homePlatform(); demoPause();

  Console.println("Roll + / -");
  poseRoll = amount; applyPose(); demoPause();
  poseRoll = -amount; applyPose(); demoPause();
  homePlatform(); demoPause();

  Console.println("Pitch + / -");
  posePitch = amount; applyPose(); demoPause();
  posePitch = -amount; applyPose(); demoPause();
  homePlatform(); demoPause();

  Console.println("Yaw diagnostic + / -");
  poseYaw = amount * 0.6f; applyPose(); demoPause();
  poseYaw = -amount * 0.6f; applyPose(); demoPause();
  homePlatform();

  Console.println("MASTER DEMO COMPLETE; HOME");
}

void printMenu() {
  Console.println();
  Console.println("Interactive Stewart Platform");
  Console.println("Motion controls:");
  Console.println("  x / X : X sway + / -");
  Console.println("  y / Y : Y sway + / -");
  Console.println("  u / d : Z up / down");
  Console.println("  r / R : roll + / -");
  Console.println("  p / P : pitch + / -");
  Console.println("  a / A : yaw diagnostic + / -");
  Console.println("  h     : smooth return HOME");
  Console.println("  s     : show current pose, step size, and range");
  Console.println("  t     : master motion demonstration");
  Console.println("  l     : laser ON");
  Console.println("  L     : laser OFF");
  Console.println("  k     : toggle laser ON/OFF");
  Console.println("  j     : recalibrate joystick center (release stick first)");
  Console.println("  b     : report joystick button state for wiring diagnosis");
  Console.println("Joystick:");
  Console.println("  X axis: roll; Y axis: pitch; farther = faster");
  Console.println("  Short click: toggle laser ON/OFF");
  Console.println("  Hold 1 second: smooth return HOME");
  Console.println("  Release holds the current roll/pitch position");
  Console.println();
  Console.println("Step-size gears:");
  Console.println("  1=1 us, 2=10 us, 3=25 us, 4=50 us, 5=100 us");
  Console.println("Range gears (approximate; verify your own servo calibration):");
  Console.println("  6=+/-220 us  conservative");
  Console.println("  7=+/-450 us  medium");
  Console.println("  8=axis +/-700 us, combined leg +/-950 us  large");
  Console.println("  9=+/-900 us  near full nominal travel");
  Console.println("  ?     : show this menu");
  Console.println();
}

void handleCommand(char command) {
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
    case 's': printPose(); break;
    case 't': runMasterDemo(); break;
    case 'l': setLaser(true); break;
    case 'L': setLaser(false); break;
    case 'k': toggleLaser(); break;
    case 'j': calibrateJoystickCenter(); break;
    case 'b': printJoystickButtonState(); break;
    case '1': setStep(1); break;
    case '2': setStep(10); break;
    case '3': setStep(25); break;
    case '4': setStep(50); break;
    case '5': setStep(100); break;
    case '6': setRange(220, 220); break;
    case '7': setRange(450, 450); break;
    case '8': setRange(700, 950); break;
    case '9': setRange(900, 900); break;
    case '?': printMenu(); break;
    case '\n':
    case '\r': break;
    default:
      Console.print("Unknown key: ");
      Console.println(command);
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  // Establish the safe state before initializing the servo controller.
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);
  laserIsOn = false;

  pinMode(JOYSTICK_BUTTON_PIN, INPUT_PULLUP);
  analogReadResolution(12);
  lastButtonReading = digitalRead(JOYSTICK_BUTTON_PIN);
  stableButtonState = lastButtonReading;

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

  Console.println("PCA9685 initialized; platform at HOME.");
  Console.println("Laser initialized OFF.");
  calibrateJoystickCenter();
  Console.println("Startup setting: step gear 5, range gear 8.");
  printPose();
  printMenu();

  // Wi-Fi starts only after the laser and servos have reached safe states.
  // Connection and retries happen in the background, so joystick and USB
  // controls remain available even when the network is down.
  startWiFiConsole();
}

void loop() {
  serviceWiFiConsole();
  updateJoystick();

  while (Serial.available()) {
    handleCommand((char)Serial.read());
  }
}
