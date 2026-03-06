/*******************************************************************
 * Teensy 4.1: Robot Brain (TB6612FNG, Encoders, QTRX-HD-15A)
 * FEATURES: Acceleration Ramp, Aggressive Sharp Turn Overrides,
 *           Broken/Dashed Line Recovery, Smooth Curve Handling
 *******************************************************************/

#include <QTRSensors.h>
#include <Encoder.h>

// Forward declaration (needed for calibration sweep)
void setMotors(int leftSpeed, int rightSpeed);

// ==========================================
// 1. PIN DEFINITIONS
// ==========================================
// --- Motor Pins (TB6612FNG) ---
const int PWMA_PIN = 7;
const int AIN1_PIN = 9;
const int AIN2_PIN = 8;
const int PWMB_PIN = 4;
const int BIN1_PIN = 5;
const int BIN2_PIN = 6;  

// --- QTRX Control Pins ---
const uint8_t ODD_CTRL = 10;
const uint8_t EVEN_CTRL = 11;
const uint8_t NUM_SENSORS = 15;
const uint8_t SENSOR_PINS[NUM_SENSORS] = { A14, A13, A12, A11, A10, A9, A8, A7, A6, A5, A4, A3, A2, A1, A0 };
QTRSensors qtr;
uint16_t sensorValues[NUM_SENSORS];

// --- Encoders ---
Encoder leftEnc(0, 1);
Encoder rightEnc(2, 3);

// ==========================================
// 2. GLOBALS: PID
// ==========================================
// PID gains — tuned for smooth curve following
float Kp = 0.045, Ki = 0.0001, Kd = 0.55;
int lastError = 0;
long integralError = 0;
const long INTEGRAL_CAP = 40000; // Anti-windup limit
float filteredDerivative = 0.0;  // Smoothed derivative for curves
const float DERIV_FILTER = 0.6;  // Low-pass filter coefficient (0-1, higher = more smoothing)

// Curve-rate tracking: detect when error is accelerating (curve tightening)
int prevError = 0;        // error from 2 cycles ago
float errorRate = 0.0;    // how fast the error is changing

int baseSpeed = 200;
int currentBaseSpeed = 200;
int maxSpeedLimit = 240;

const uint16_t BLACK_THRESHOLD = 700;  // Above this = sensor clearly sees black
int lastDirection = 0;  // Remember which way we were turning (+/- error)

// --- Lost Line / Broken Line Recovery ---
bool lineLost = false;
unsigned long lineLostTime = 0;
const unsigned long MAX_GAP_TIME = 800;   // Max ms to bridge a dashed gap (increased for wider gaps)
const int LOST_LINE_SPEED = 140;          // Forward speed while bridging a straight gap

// Save the ACTUAL motor outputs when line was last seen,
// so we can replay the same curve through a gap
int savedLeftSpeed  = 0;
int savedRightSpeed = 0;
int savedPosition   = 7000;  // Last known line position

// --- Acceleration Timers ---
unsigned long startTime = 0;
unsigned long elapsedTime = 0;
const unsigned long accelerationTime = 500; // 500ms soft start

// ==========================================
// 3. SETUP & CALIBRATION
// ==========================================
void setup() {
  pinMode(13, OUTPUT);

  // --- Motor Setup ---
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  analogWriteFrequency(PWMA_PIN, 20000);
  analogWriteFrequency(PWMB_PIN, 20000);

  // --- QTR Wake Up & Calibration ---
  pinMode(ODD_CTRL, OUTPUT);
  pinMode(EVEN_CTRL, OUTPUT);
  digitalWrite(ODD_CTRL, HIGH);
  digitalWrite(EVEN_CTRL, HIGH);
  qtr.setTypeAnalog();
  qtr.setSensorPins(SENSOR_PINS, NUM_SENSORS);
  qtr.setEmitterPin(QTRNoEmitterPin);

  // --- AUTO-CALIBRATE (with motor sweep for proper min/max learning) ---
  digitalWrite(13, HIGH);
  unsigned long calibStart = millis();
  int sweepPhase = 0;
  while (millis() - calibStart < 5000) {
    qtr.calibrate();
    // Sweep left-right so sensors see both black and white
    unsigned long elapsed = millis() - calibStart;
    if (elapsed < 1000) {          // Turn left
      setMotors(-80, 80);
    } else if (elapsed < 2500) {   // Turn right
      setMotors(80, -80);
    } else if (elapsed < 4000) {   // Turn left again
      setMotors(-80, 80);
    } else {                       // Return to center
      setMotors(60, -60);
    }
  }
  setMotors(0, 0);  // Stop after calibration
  digitalWrite(13, LOW);

  startTime = millis();
}

// ==========================================
// 4. MAIN LOOP
// ==========================================
void loop() {

  // --- A. ACCELERATION RAMP ---
  elapsedTime = millis() - startTime;
  if (elapsedTime <= accelerationTime) {
    currentBaseSpeed = map(elapsedTime, 0, accelerationTime, 0, baseSpeed);
  } else {
    currentBaseSpeed = baseSpeed;
  }

  // --- B. READ SENSORS ---
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 7000;
  int leftSpeed = 0;
  int rightSpeed = 0;

  // --- COUNT BLACK SENSORS & DETECT EDGES ---
  uint8_t blackCount = 0;
  int8_t firstBlack = -1, lastBlack = -1;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i] > BLACK_THRESHOLD) {
      blackCount++;
      if (firstBlack == -1) firstBlack = i;
      lastBlack = i;
    }
  }

  // ==========================================================
  // --- C. BROKEN / DASHED LINE RECOVERY ---
  // When NO sensors see the line, REPLAY the last motor speeds
  // so we continue the same curve trajectory through the gap.
  // ==========================================================
  if (blackCount == 0) {
    if (!lineLost) {
      // Just lost the line — snapshot current state
      lineLost = true;
      lineLostTime = millis();
      // savedLeftSpeed/savedRightSpeed are updated every cycle
      // when line IS visible (see end of loop), so they hold
      // the exact motor outputs from right before the gap.
    }

    unsigned long gapElapsed = millis() - lineLostTime;

    if (gapElapsed < MAX_GAP_TIME) {
      // === REPLAY the curve through the gap ===
      // Use the saved motor speeds but scale down gradually
      // so we don't overshoot if the gap is long.
      float scaleFactor;
      if (gapElapsed < 200) {
        scaleFactor = 1.0;      // First 200ms: full replay
      } else if (gapElapsed < 500) {
        scaleFactor = 0.85;     // 200-500ms: slight reduction
      } else {
        scaleFactor = 0.70;     // 500-800ms: slower
      }

      // If we were on a curve, replay the differential.
      // If we were going straight, just drive forward.
      int gapLeft, gapRight;
      int speedDiff = savedLeftSpeed - savedRightSpeed;

      if (abs(speedDiff) > 20) {
        // We were curving — replay the same left/right ratio
        gapLeft  = (int)(savedLeftSpeed  * scaleFactor);
        gapRight = (int)(savedRightSpeed * scaleFactor);
        // Ensure minimum forward motion on both wheels
        // (inner wheel can be 0 but not heavily negative)
        gapLeft  = constrain(gapLeft,  -20, 220);
        gapRight = constrain(gapRight, -20, 220);
      } else {
        // We were going mostly straight — drive forward
        int gapSpeed = (int)(LOST_LINE_SPEED * scaleFactor);
        gapLeft  = gapSpeed;
        gapRight = gapSpeed;
      }

      setMotors(gapLeft, gapRight);
      return;  // Skip normal PID this cycle
    } else {
      // Gap too long — search spin in the last known direction
      if (lastDirection >= 0) {
        setMotors(90, -90);   // Spin right
      } else {
        setMotors(-90, 90);   // Spin left
      }
      return;
    }
  } else {
    // Line found — reset lost-line state
    if (lineLost) {
      lineLost = false;
      integralError = 0;  // Reset integral to avoid wind-up from gap
    }
  }

  // --- WIDE-LINE / OFF-CENTER CORRECTION ---
  if (blackCount >= 13) {
    // Nearly ALL sensors black — this is NOT a valid line reading.
    // Keep turning in the last known direction to recover.
    error = (lastDirection >= 0) ? 6000 : -6000;
  } else if (blackCount >= 8) {
    // Many sensors black but edges visible — use edge midpoint
    float edgeMid = (firstBlack + lastBlack) / 2.0;
    float arrayCenterIdx = (NUM_SENSORS - 1) / 2.0; // = 7.0
    error = (int)((edgeMid - arrayCenterIdx) * 1000);
  }

  // Track which direction we're heading for recovery
  if (abs(error) > 500) {
    lastDirection = error;
  }

  // --- PID CALCULATION (with Integral + Filtered Derivative) ---
    integralError += error;
    integralError = constrain(integralError, -INTEGRAL_CAP, INTEGRAL_CAP);

    // Smooth the derivative term to reduce jitter on curves
    float rawDerivative = (float)(error - lastError);
    filteredDerivative = (DERIV_FILTER * filteredDerivative) + ((1.0 - DERIV_FILTER) * rawDerivative);

    // Track curve rate: how fast is the error accelerating?
    // Positive = curve is tightening, so we need to pre-steer harder
    errorRate = 0.7 * errorRate + 0.3 * (float)(error - 2 * lastError + prevError);
    prevError = lastError;

    int motorSpeed = (Kp * error) + (Ki * integralError) + (Kd * filteredDerivative);
    lastError = error;

    // --- PROGRESSIVE SPEED ZONES ---
    // Slow down in curves, but NOT so much that the robot stalls.
    // The key: keep enough forward speed to FOLLOW the curve, not pivot.
    int activeBase = currentBaseSpeed;
    int absError = abs(error);
    float absRate = fabs(errorRate);

    // If the curve is tightening rapidly, slow down more proactively
    float rateFactor = 1.0;
    if (absRate > 200) rateFactor = 0.85;
    if (absRate > 500) rateFactor = 0.75;

    if (absError > 5000) {        // Extreme — nearly off track
      activeBase = constrain(activeBase * 0.35 * rateFactor, 50, activeBase);
    } else if (absError > 3500) { // Very sharp curve
      activeBase = constrain(activeBase * 0.45 * rateFactor, 55, activeBase);
    } else if (absError > 2000) { // Sharp curve
      activeBase = constrain(activeBase * 0.55 * rateFactor, 60, activeBase);
    } else if (absError > 1200) { // Moderate curve
      activeBase = constrain(activeBase * 0.70 * rateFactor, 65, activeBase);
    } else if (absError > 600) {  // Gentle curve
      activeBase = constrain(activeBase * 0.85, 70, activeBase);
    }

    leftSpeed = activeBase + motorSpeed;
    rightSpeed = activeBase - motorSpeed;

    // ===========================================================
    // --- DIFFERENTIAL STEERING (NOT pivoting!) ---
    // On sharp curves: SLOW or STOP the inner wheel, keep outer
    // wheel running. NEVER reverse the inner wheel unless the
    // line is at the absolute extreme edge (last-resort recovery).
    // This keeps the robot FOLLOWING the curve instead of spinning.
    // ===========================================================

    if (position <= 1000) {
      // LAST RESORT: line at far extreme left edge — slight reverse inner
      leftSpeed  = -40;
      rightSpeed = constrain(activeBase * 1.2, 100, 200);
      integralError = 0;
    } else if (position >= 13000) {
      // LAST RESORT: line at far extreme right edge — slight reverse inner
      leftSpeed  = constrain(activeBase * 1.2, 100, 200);
      rightSpeed = -40;
      integralError = 0;
    } else if (position <= 2500) {
      // Sharp left: stop inner wheel, push outer
      leftSpeed  = 0;
      rightSpeed = constrain(activeBase * 1.3, 90, 220);
      integralError = integralError / 2;
    } else if (position >= 11500) {
      // Sharp right: stop inner wheel, push outer
      leftSpeed  = constrain(activeBase * 1.3, 90, 220);
      rightSpeed = 0;
      integralError = integralError / 2;
    } else if (position <= 3500) {
      // Moderate-sharp left: slow inner wheel significantly
      leftSpeed  = constrain(activeBase * 0.15, 10, 40);
      rightSpeed = constrain(activeBase * 1.1, 80, 210);
    } else if (position >= 10500) {
      // Moderate-sharp right: slow inner wheel significantly
      leftSpeed  = constrain(activeBase * 1.1, 80, 210);
      rightSpeed = constrain(activeBase * 0.15, 10, 40);
    } else if (position <= 4500) {
      // Moderate left: reduce inner, boost outer
      leftSpeed  = constrain(leftSpeed, activeBase * 0.2, activeBase * 0.5);
      rightSpeed = constrain(rightSpeed, activeBase * 0.8, activeBase * 1.1);
    } else if (position >= 9500) {
      // Moderate right: reduce inner, boost outer
      leftSpeed  = constrain(leftSpeed, activeBase * 0.8, activeBase * 1.1);
      rightSpeed = constrain(rightSpeed, activeBase * 0.2, activeBase * 0.5);
    }
    // else: position ~5000-9000 = normal PID handles it fine

    // --- Final clamp: never exceed limits ---
    leftSpeed  = constrain(leftSpeed,  -80, maxSpeedLimit);
    rightSpeed = constrain(rightSpeed, -80, maxSpeedLimit);

    // Save current motor outputs so gap-bridging can replay them
    savedLeftSpeed  = leftSpeed;
    savedRightSpeed = rightSpeed;
    savedPosition   = position;

    setMotors(leftSpeed, rightSpeed);
}

// ==========================================
// 5. TB6612FNG MOTOR DRIVE
// ==========================================
void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if (leftSpeed >= 0) {
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
  } else {
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
  }
  analogWrite(PWMA_PIN, abs(leftSpeed));

  if (rightSpeed >= 0) {
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
  } else {
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
  }
  analogWrite(PWMB_PIN, abs(rightSpeed));
}
