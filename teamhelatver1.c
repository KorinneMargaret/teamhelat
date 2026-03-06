/*******************************************************************
 * Teensy 4.1: Robot Brain (TB6612FNG, Encoders, QTRX-HD-15A)
 * FEATURES: Acceleration Ramp, Arc-Coasting, Auto-Search, Encoder 90° Turns
 *******************************************************************/

#include <QTRSensors.h>
#include <Encoder.h>

// Forward declaration 
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
// 2. GLOBALS: PID & SPEED
// ==========================================
// PID gains
float Kp = 0.04, Ki = 0.000, Kd = 0.6;
int lastError = 0;
long integralError = 0;
const long INTEGRAL_CAP = 50000;

int baseSpeed = 240;
int currentBaseSpeed = 240;
int maxSpeedLimit = 240;

const uint16_t BLACK_THRESHOLD = 700;  
int lastDirection = 0;  

// --- Advanced Line Handlers ---
bool lineDetected = true;
unsigned long lineLostTime = 0;
uint16_t lastValidPosition = 7000; 
int lastGoodLeftSpeed = 0;
int lastGoodRightSpeed = 0;

// --- ENCODER TARGET (TUNE THIS NUMBER) ---
// This is the number of ticks required for one wheel to execute a 90-degree turn.
// Start at 800 and adjust up or down based on physical testing.
const long TICKS_FOR_90_DEG = 800; 

// --- Acceleration Timers ---
unsigned long startTime = 0;
unsigned long elapsedTime = 0;
const unsigned long accelerationTime = 500; 

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

  digitalWrite(13, HIGH);
  unsigned long calibStart = millis();
  while (millis() - calibStart < 5000) {
    qtr.calibrate();
    unsigned long elapsed = millis() - calibStart;
    if (elapsed < 1000) setMotors(-80, 80);
    else if (elapsed < 2500) setMotors(80, -80);
    else if (elapsed < 4000) setMotors(-80, 80);
    else setMotors(60, -60);
  }
  setMotors(0, 0); 
  digitalWrite(13, LOW);

  startTime = millis();
}

// ==========================================
// 4. SMART ENCODER TURN FUNCTION
// ==========================================
// Executes a precise, closed-loop spin. Aborts early if the line is found.
void executeEncoderTurn(int direction) {
  // Reset encoders to 0
  leftEnc.write(0);
  rightEnc.write(0);
  
  // Use a slightly lower, controlled speed for accurate encoder counting
  int turnSpeed = baseSpeed * 0.7; 
  
  while (abs(leftEnc.read()) < TICKS_FOR_90_DEG && abs(rightEnc.read()) < TICKS_FOR_90_DEG) {
    if (direction < 0) { // Turn Left
      setMotors(-turnSpeed, turnSpeed);
    } else {             // Turn Right
      setMotors(turnSpeed, -turnSpeed);
    }
    
    // SAFETY CATCH: Check sensors during the turn!
    qtr.readLineBlack(sensorValues);
    uint8_t currentBlack = 0;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      if (sensorValues[i] > BLACK_THRESHOLD) currentBlack++;
    }
    
    // If we see the line midway through the turn, abort the spin and return to PID tracking
    if (currentBlack > 0) {
      break; 
    }
  }
  // Briefly stop motors to kill momentum after the pivot
  setMotors(0, 0); 
  delay(20); 
}

// ==========================================
// 5. MAIN LOOP
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

  // --- SAVE VALID POSITION ---
  if (blackCount > 0) {
    lastValidPosition = position;
  }

  // --- C. ADVANCED LOST LINE & GAP HANDLING ---
  if (blackCount == 0) {
    if (lineDetected) {
      lineLostTime = millis(); 
      lineDetected = false;
    }
    
    unsigned long lostDuration = millis() - lineLostTime;

    // SCENARIO 1: SHARP CORNER OVERSHOOT (Line fell off extreme edges)
    if (lastValidPosition <= 2000 || lastValidPosition >= 12000) {
      if (lastValidPosition <= 2000) {
        executeEncoderTurn(-1); // -1 triggers a perfect 90-degree left spin
      } else {
        executeEncoderTurn(1);  // +1 triggers a perfect 90-degree right spin
      }
      
      // Reset the lost timer so it doesn't instantly trigger a sweeping search after the turn
      lineLostTime = millis(); 
      return; 
    }

    // SCENARIO 2: BROKEN LINE (Line vanished from the center/middle)
    if (lostDuration < 300) { 
      // ARC-COASTING
      setMotors(lastGoodLeftSpeed, lastGoodRightSpeed);
      return; 
    } 
    
    // SCENARIO 3: TRULY LOST (Gap lasted too long)
    int searchPhase = (lostDuration - 300) / 400; 
    int spinSpeed = currentBaseSpeed * 0.7;

    if (lastDirection < 0) { 
      if (searchPhase % 2 == 0) setMotors(-spinSpeed, spinSpeed); 
      else setMotors(spinSpeed, -spinSpeed);                      
    } else {                 
      if (searchPhase % 2 == 0) setMotors(spinSpeed, -spinSpeed); 
      else setMotors(-spinSpeed, spinSpeed);                      
    }
    return; 
    
  } else {
    lineDetected = true; 
  }

  // --- WIDE-LINE / OFF-CENTER CORRECTION ---
  if (blackCount >= 13) {
    error = (lastDirection >= 0) ? 6000 : -6000;
  } else if (blackCount >= 8) {
    float edgeMid = (firstBlack + lastBlack) / 2.0;
    float arrayCenterIdx = (NUM_SENSORS - 1) / 2.0; 
    error = (int)((edgeMid - arrayCenterIdx) * 1000);
  }

  if (abs(error) > 500) {
    lastDirection = error;
  }

  // --- PID CALCULATION ---
  integralError += error;
  integralError = constrain(integralError, -INTEGRAL_CAP, INTEGRAL_CAP);
  int motorSpeed = (Kp * error) + (Ki * integralError) + (Kd * (error - lastError));
  lastError = error;

  // --- PROGRESSIVE SPEED ZONES ---
  int activeBase = currentBaseSpeed;
  int absError = abs(error);
  if (absError > 3000) {        
    activeBase = constrain(activeBase * 0.25, 40, activeBase);
  } else if (absError > 2000) { 
    activeBase = constrain(activeBase * 0.35, 45, activeBase);
  } else if (absError > 1200) { 
    activeBase = constrain(activeBase * 0.50, 50, activeBase);
  } else if (absError > 600) {  
    activeBase = constrain(activeBase * 0.65, 55, activeBase);
  }

  leftSpeed = activeBase + motorSpeed;
  rightSpeed = activeBase - motorSpeed;

  // --- AGGRESSIVE SHARP TURN OVERRIDES ---
  if (position <= 3500) {         
    leftSpeed = constrain(-activeBase * 1.8, -250, -120);
    rightSpeed = constrain(activeBase * 1.8, 120, 250);
    integralError = 0;
  } else if (position >= 10500) { 
    leftSpeed = constrain(activeBase * 1.8, 120, 250);
    rightSpeed = constrain(-activeBase * 1.8, -250, -120);
    integralError = 0;
  }

  // --- SAVE TRAJECTORY FOR ARC-COASTING ---
  lastGoodLeftSpeed = leftSpeed;
  lastGoodRightSpeed = rightSpeed;

  setMotors(leftSpeed, rightSpeed);
}

// ==========================================
// 6. TB6612FNG MOTOR DRIVE
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