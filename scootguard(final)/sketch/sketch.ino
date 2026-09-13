/*
 * ============================================================
 * Smart E-Scooter Drunk Driving Prevention and Real-Time Unsafe Riding Control System
 * ============================================================
* Hardware Pin Map:
 * - D2 : button (INPUT_PULLUP)
 * - D3 : buzzer (PWM)
 * - D5 : servo motor signal
 * - D6 : rear red LED
 * - D9 : front RGB RED (Common Anode)
 * - D10: front RGB GREEN
 * - D11: front RGB BLUE
 * - A0 : MQ-3 alcohol sensor
 * - A4/A5: MPU6050 (I2C)
 * ============================================================
 */
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Servo.h>
#include <Arduino_RouterBridge.h>
//Pin definitions
const int BUTTON_PIN    = 2;
const int BUZZER_PIN    = 3;
const int SERVO_PIN     = 5;
const int REAR_LED_PIN  = 6;
const int RGB_RED_PIN   = 9;
const int RGB_GREEN_PIN = 10;
const int RGB_BLUE_PIN  = 11;
const int ALCOHOL_PIN   = A0;
// Alcohol thresholds
const int ALCOHOL_NO_BREATH = 220;
const int ALCOHOL_DRUNK     = 550;
// Gyroscope settings
const unsigned long GYRO_SAMPLE_INTERVAL = 100;
const int   JUDGE_SAMPLE_COUNT = 40;

// AI decision check interval
// Python handles the continuous AI inference, but the MCU updates the judgment
// every 4 seconds. This ensures "2 consecutive detections" equals ~8 seconds of
// sustained anomalies, preventing false alerts from 100ms transient misclassifications.
const unsigned long AI_CHECK_INTERVAL = 4000;
unsigned long lastAiCheckTime = 0;

// Triggers anomalous driving alert if exceeded 2 consecutive times (8s based on AI_CHECK_INTERVAL)
const int WARN_COUNT_TRIGGER = 2;  // ★ 3 → 2
// ALERT timer 
const unsigned long ALERT_BUZZER_DELAY = 5000;
const unsigned long ALERT_AUTO_DRUNK   = 15000;
const unsigned long BLINK_INTERVAL     = 300;
// motor settings
const int MOTOR_NEUTRAL = 90;
const int MOTOR_MAX     = 150;
const unsigned long ACCEL_INTERVAL = 200;
const int            ACCEL_STEP    = 1;
const unsigned long DECEL_INTERVAL = 250;
const int            DECEL_STEP    = 1;
// system state
enum SystemState {
  STATE_WAITING,
  STATE_ACCEL,
  STATE_DRIVING,
  STATE_ALERT,
  STATE_RECHECK,
  STATE_DECEL
};
SystemState currentState = STATE_WAITING;

enum AlcoholStatus {
  NO_BREATH,
  NORMAL_BREATH,
  ALCOHOL_BREATH
};
AlcoholStatus alcoholStatus = NO_BREATH;
// objects
MPU6050 mpu6050(Wire2);
Servo myServo;
// gyroscope offset
float offsetGyroX = 0, offsetGyroY = 0, offsetGyroZ = 0;
// gyroscope cache (I2C conflict prevention, updated only in loop())
volatile float cachedGyroX = 0;
volatile float cachedGyroZ = 0;
// 4-second decision buffer (kept for Serial logging)
float unstableSampleSum = 0;
float turnSampleSum = 0;
int   sampleCount = 0;
unsigned long lastSampleTime = 0;
// consecutive detection counter
int warnCount = 0;
//AI decision result
// 0 = normal, 1 = turn, 2 = unstable
volatile int aiResultCode = 0;

// session cumulative counters for cloud integration
// Accumulates during a single ride session (from WAITING until returning to WAITING).
// Reset by Python main.py via reset_session_counters() at the start of a new session.
unsigned long alertEventCount     = 0;  // Total entries into STATE_ALERT
unsigned long recheckPassCount    = 0;  // Total successful recoveries to "Normal"
unsigned long decelEventCount     = 0;  // Total entries into STATE_DECEL (forced deceleration)
unsigned long reactionTimeSumMs   = 0;  // Cumulative alert-to-button response duration (ms)
unsigned long reactionTimeCount   = 0;  // Number of valid response events included in the sum

// Keep get_alcohol_value() returning real-time raw data for the dashboard.
// Since the cloud report (session summary) requires the average value at the actual 
// measurement completion time, it is cached separately only inside runAlcoholMeasurement().
int lastMeasuredAlcoholValue = 0;
// motor
int  currentMotorSpeed = MOTOR_NEUTRAL;
unsigned long lastMotorStepTime = 0;
// timing / state
unsigned long lastBlinkTime  = 0;
unsigned long alertStartTime = 0;
bool blinkState      = false;
bool buzzerStarted   = false;
bool stoppedMsgPrinted = false;
// button debounce
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;
// RGB LED (Common Anode)
void setColor(int r, int g, int b) {
  analogWrite(RGB_RED_PIN,   255 - r);
  analogWrite(RGB_GREEN_PIN, 255 - g);
  analogWrite(RGB_BLUE_PIN,  255 - b);
}
// all OFF
void allOff() {
  setColor(0, 0, 0);
  digitalWrite(REAR_LED_PIN, LOW);
  noTone(BUZZER_PIN);
  currentMotorSpeed = MOTOR_NEUTRAL;
  myServo.write(currentMotorSpeed);
}
// button release detection
bool checkButtonRelease() {
  bool cur = digitalRead(BUTTON_PIN);
  bool released = false;
  if (cur != lastButtonState) lastDebounceTime = millis();
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    static bool wasPressed = false;
    if (cur == LOW) wasPressed = true;
    else if (wasPressed && cur == HIGH) { wasPressed = false; released = true; }
  }
  lastButtonState = cur;
  return released;
}
// gyroscope calibration
void calibrateGyro() {
  Serial.println(F("[Calibration] calibration in progress... Please do not move!"));
  float sumX = 0, sumY = 0, sumZ = 0;
  const int samples = 100;
  for (int i = 0; i < samples; i++) {
    mpu6050.update();
    sumX += mpu6050.getGyroX();
    sumY += mpu6050.getGyroY();
    sumZ += mpu6050.getGyroZ();
    delay(10);
  }
  offsetGyroX = sumX / samples;
  offsetGyroY = sumY / samples;
  offsetGyroZ = sumZ / samples;
  Serial.println(F("[Calibration] Complete!"));
}
// Roll (X-axis) - Uses cached values from loop() for logging
float getUnstableDeviationOnly() {
  return abs(cachedGyroX);
}
// Yaw (Z-axis) - Uses cached values from loop() for logging
float getTurnDeviationOnly() {
  return abs(cachedGyroZ);
}
// Clear judgment buffers
void resetSampleBuffer() {
  unstableSampleSum = 0;
  turnSampleSum = 0;
  sampleCount = 0;
  lastSampleTime = millis();
}
void resetWarnCount() {
  warnCount = 0;
}
// Motor failsafe write
void safeServoWrite(int speed) {
  if (speed < MOTOR_NEUTRAL) speed = MOTOR_NEUTRAL;
  if (speed > MOTOR_MAX)     speed = MOTOR_MAX;
  myServo.write(speed);
}
// Motor acceleration -> true: MAX speed
bool accelMotor() {
  if (currentMotorSpeed >= MOTOR_MAX) return true;
  unsigned long now = millis();
  if (now - lastMotorStepTime >= ACCEL_INTERVAL) {
    lastMotorStepTime = now;
    currentMotorSpeed += ACCEL_STEP;
    safeServoWrite(currentMotorSpeed);
  }
  return (currentMotorSpeed >= MOTOR_MAX);
}
// Motor deceleration -> true: Full stop
bool decelMotor() {
  if (currentMotorSpeed <= MOTOR_NEUTRAL) {
    safeServoWrite(MOTOR_NEUTRAL);
    return true;
  }
  unsigned long now = millis();
  if (now - lastMotorStepTime >= DECEL_INTERVAL) {
    lastMotorStepTime = now;
    currentMotorSpeed -= DECEL_STEP;
    safeServoWrite(currentMotorSpeed);
  }
  return (currentMotorSpeed <= MOTOR_NEUTRAL);
}
// Alcohol test -> true: Normal / false: Positive or undetected
bool runAlcoholMeasurement() {
  Serial.println(F("\n========== [Start alcohol measurement] =========="));
  setColor(0, 255, 0);
  tone(BUZZER_PIN, 1200); delay(200); noTone(BUZZER_PIN);
  delay(1000);
  long total = 0;
  for (int i = 0; i < 10; i++) {
    int v = analogRead(ALCOHOL_PIN);
    total += v;
    Serial.print(F("  sample ")); Serial.print(i + 1);
    Serial.print(F("/10: ")); Serial.println(v);
    delay(300);
  }
  int avg = total / 10;
  Serial.print(F("[result] average value: ")); Serial.println(avg);

// Cloud report: Caches the average value at the actual measurement completion time.
// (Dashboard's get_alcohol_value() continues to return raw data independently.)
  lastMeasuredAlcoholValue = avg;

  if (avg < ALCOHOL_NO_BREATH) {
    Serial.println(F("[result] No breath detected → Yellow LED for 3s"));
    setColor(255, 255, 0); delay(3000); allOff();
    alcoholStatus = NO_BREATH;
    return false;
  } else if (avg >= ALCOHOL_DRUNK) {
    Serial.println(F("[Result] !!! ALCOHOL DETECTED !!!"));
    setColor(255, 0, 0);
    for (int i = 0; i < 3; i++) {
      tone(BUZZER_PIN, 1000); delay(300);
      tone(BUZZER_PIN, 700);  delay(300);
    }
    noTone(BUZZER_PIN); delay(2000); allOff();
    alcoholStatus = ALCOHOL_BREATH;
    return false;
  } else {
    alcoholStatus = NORMAL_BREATH;
    Serial.println(F("[Result]Pass  → Blue LED for 3s"));
    setColor(0, 0, 255);
    tone(BUZZER_PIN, 1500); delay(200); noTone(BUZZER_PIN);
    delay(2800);
    setColor(0, 0, 0);
    return true;
  }
}

int get_alcohol_value() {
  return analogRead(ALCOHOL_PIN);
}
int get_alcohol_status() {
  return (int)alcoholStatus;
}
int get_permission() {
  if (currentState == STATE_ACCEL ||
      currentState == STATE_DRIVING ||
      currentState == STATE_ALERT ||
      currentState == STATE_RECHECK) {
    return 1;  // ALLOWED
  }
  return 0;    // BLOCKED
}

int get_state() {
  return (int)currentState;
}
int get_warn_count() {
  return warnCount;
}
// Data collecting (Edge Impulse)
// Bridge returns cached data only, skipping direct I2C communication
float get_gyro_x() {
  return cachedGyroX;
}
float get_gyro_z() {
  return cachedGyroZ;
}
// For receiving AI judgement (Python -> MCU)
// code: 0=normal, 1=turn, 2=unstable
void set_ai_result(int code) {
  aiResultCode = code;
}

// Bridge function for cloud integration
unsigned long get_alert_count() {
  return alertEventCount;
}

unsigned long get_recheck_pass_count() {
  return recheckPassCount;
}

unsigned long get_decel_count() {
  return decelEventCount;
}

unsigned long get_reaction_time_sum() {
  return reactionTimeSumMs;
}

unsigned long get_reaction_time_count() {
  return reactionTimeCount;
}

// Average alcohol value at the end of the session
int get_last_measured_alcohol() {
  return lastMeasuredAlcoholValue;
}

// Called by Python at the start of a new ride session -> Resets all accumulated counters
// (Returns int 1 because having a return value makes the bridge function safer)
int reset_session_counters() {
  alertEventCount     = 0;
  recheckPassCount    = 0;
  decelEventCount     = 0;
  reactionTimeSumMs   = 0;
  reactionTimeCount   = 0;
  Serial.println(F("[Cloud] Session counters reset"));
  return 1;
}
// SETUP
void setup() {
  Serial.begin(9600);
  Monitor.begin();
  Bridge.begin();
  Bridge.provide("get_alcohol_value", get_alcohol_value);
  Bridge.provide("get_alcohol_status", get_alcohol_status);
  Bridge.provide("get_permission", get_permission);
  Bridge.provide("get_state", get_state);
  Bridge.provide("get_warn_count", get_warn_count);
  Bridge.provide("get_gyro_x", get_gyro_x);
  Bridge.provide("get_gyro_z", get_gyro_z);
  Bridge.provide("set_ai_result", set_ai_result);

  // Register bridge functions for cloud integration
  Bridge.provide("get_alert_count", get_alert_count);
  Bridge.provide("get_recheck_pass_count", get_recheck_pass_count);
  Bridge.provide("get_decel_count", get_decel_count);
  Bridge.provide("get_reaction_time_sum", get_reaction_time_sum);
  Bridge.provide("get_reaction_time_count", get_reaction_time_count);
  Bridge.provide("get_last_measured_alcohol", get_last_measured_alcohol);
  Bridge.provide("reset_session_counters", reset_session_counters);

  pinMode(BUTTON_PIN,    INPUT_PULLUP);
  pinMode(BUZZER_PIN,    OUTPUT);
  pinMode(REAR_LED_PIN,  OUTPUT);
  pinMode(RGB_RED_PIN,   OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN,  OUTPUT);
  myServo.attach(SERVO_PIN);
  allOff();
  Wire2.begin();
  //Wire.setWireTimeout(3000, true);
  mpu6050.begin();
  calibrateGyro();
  currentState = STATE_WAITING;
  Serial.println(F("[Ready] Press the button to start the alcohol test."));
}
// LOOP
void loop() {
  unsigned long now = millis();
  //Read I2C once (to prevent conflict with bridge functions)

  mpu6050.update();
  cachedGyroX = mpu6050.getGyroX() - offsetGyroX;
  cachedGyroZ = mpu6050.getGyroZ() - offsetGyroZ;
  switch (currentState) {
    // Wait
    case STATE_WAITING:
      if (checkButtonRelease()) {
        if (runAlcoholMeasurement()) {
          currentMotorSpeed = MOTOR_NEUTRAL;
          safeServoWrite(currentMotorSpeed);
          lastMotorStepTime = now;
          resetSampleBuffer();
          resetWarnCount();
          currentState = STATE_ACCEL;
          Serial.println(F("[ACCEL] Motor gradually accelerates"));
        }
      }
      break;
    // ACCEL
    case STATE_ACCEL: {
      bool reached = accelMotor();
      if (now - lastSampleTime >= GYRO_SAMPLE_INTERVAL) {
        lastSampleTime = now;
        unstableSampleSum += getUnstableDeviationOnly();
        turnSampleSum += getTurnDeviationOnly();
        sampleCount++;
        if (sampleCount >= JUDGE_SAMPLE_COUNT) resetSampleBuffer();
      }
      if (reached) {
        resetSampleBuffer();
        resetWarnCount();
        currentState = STATE_DRIVING;
        Serial.println(F("[DRIVING] Max speed reached. Starts gyro monitoring."));
      }
      break;
    }
    // Constant driving + AI judgment monitoring
    // (Python continuously updates aiResultCode via set_ai_result every 100ms,
    //  but the MCU reflects that value into the "judgment" only once every 4 seconds)
    case STATE_DRIVING:
      safeServoWrite(MOTOR_MAX);

      // For reference logs only (not used in actual judgment, for monitoring purposes)
      if (now - lastSampleTime >= GYRO_SAMPLE_INTERVAL) {
        lastSampleTime = now;
        unstableSampleSum += getUnstableDeviationOnly();
        turnSampleSum += getTurnDeviationOnly();
        sampleCount++;
        if (sampleCount >= JUDGE_SAMPLE_COUNT) {
          float avgUnstable = unstableSampleSum / sampleCount;
          float avgTurn = turnSampleSum / sampleCount;
          Serial.print(F("[X-axis average] "));
          Serial.print(avgUnstable);
          Serial.print(F("[Z-axis average] "));
          Serial.println(avgTurn);
          resetSampleBuffer();
        }
      }

      // Actual Judgment: Check AI results only once every 4 seconds
      if (now - lastAiCheckTime >= AI_CHECK_INTERVAL) {
        lastAiCheckTime = now;

        // 0 = normal, 1 = turn, 2 = unstable
        Serial.print(F("[AI Decision Code] "));
        Serial.print(aiResultCode);
        Serial.print(F("   Consecutive Exceed: "));
        Serial.print(warnCount);
        Serial.println(F("times"));

        if (aiResultCode == 2) {  // unstable
          warnCount++;
          Serial.print(F("  ↑ Unstable detected! Consecutive "));
          Serial.print(warnCount);
          Serial.println(F("times"));

          if (warnCount >= WARN_COUNT_TRIGGER) {
            Serial.println(F("[Detection] Abnormal driving confirmed!"));
            
            alertStartTime = now;
            blinkState    = false;
            buzzerStarted = false;
            lastBlinkTime = now;
            currentState  = STATE_ALERT;

            // Cloud integration - Abnormal driving event count
            alertEventCount++;
          }
        } else {
          // normal(0) or turn(1) → Consecutive counter reset
          if (warnCount > 0) {
            Serial.println(F("  ↓ Normal/turn → Consecutive counter reset"));
            warnCount = 0;
          }
        }
      }
      break;

    // Abnormal detection
    case STATE_ALERT: {
      unsigned long elapsed = now - alertStartTime;
      if (elapsed >= ALERT_AUTO_DRUNK) {
        Serial.println(F("[Alarm] No response → Automatically judged as drunk driving. Deceleration start."));
        noTone(BUZZER_PIN);
        setColor(0, 0, 0);
        lastMotorStepTime = now;
        stoppedMsgPrinted = false;
        currentState = STATE_DECEL;

        // Cloud integration - Forced deceleration event count
        decelEventCount++;
        break;
      }
      if (elapsed >= ALERT_BUZZER_DELAY && !buzzerStarted) {
        buzzerStarted = true;
        Serial.println(F("Alarm] 5 seconds elapsed → Add buzzer"));
      }
      if (now - lastBlinkTime >= BLINK_INTERVAL) {
        lastBlinkTime = now;
        blinkState = !blinkState;
        if (blinkState) {
          setColor(255, 0, 0);
          if (buzzerStarted) tone(BUZZER_PIN, 900);
        } else {
          setColor(0, 0, 0);
          if (buzzerStarted) noTone(BUZZER_PIN);
        }
      }
      if (checkButtonRelease()) {
        noTone(BUZZER_PIN);
        setColor(0, 0, 0);
        Serial.println(F("[Remeasure] Button input → Re-measure alcohol"));

        // Cloud integration - Cumulative re-measurement of reaction time (ALERT start ~ button response)
        reactionTimeSumMs += (now - alertStartTime);
        reactionTimeCount++;

        currentState = STATE_RECHECK;
      }
      break;
    }

    // Remeasure:  Directly to DRIVING without speed reset
    case STATE_RECHECK:
      if (runAlcoholMeasurement()) {
        Serial.println(F("[Remeasure] Normal → Resume driving while maintaining current speed"));
        // Keep currentMotorSpeed as is → Maintatin DRIVING without stopping
        safeServoWrite(currentMotorSpeed);
        lastMotorStepTime = now;
        resetSampleBuffer();
        resetWarnCount();
        aiResultCode = 0;
        currentState = STATE_DRIVING;  // ★ Skip ACCEL state and jump straight to DRIVING

        // Cloud integration: Count re-check passes to track estimated false alerts
        recheckPassCount++;
      } else {
        Serial.println(F("[Re-check] Result: Alcohol detected -> Start deceleration"));
        lastMotorStepTime = now;
        stoppedMsgPrinted = false;
        currentState = STATE_DECEL;

        // Cloud integration: Keeps track of how many times a forced deceleration happened

        decelEventCount++;
      }
      break;
      
    // decel → stop
    case STATE_DECEL: {
      bool stopped = decelMotor();
      if (!stopped) {
        if (now - lastBlinkTime >= BLINK_INTERVAL) {
          lastBlinkTime = now;
          blinkState = !blinkState;
          if (blinkState) {
            setColor(255, 0, 0);
            digitalWrite(REAR_LED_PIN, HIGH);
            tone(BUZZER_PIN, 1000);
          } else {
            setColor(0, 0, 0);
            digitalWrite(REAR_LED_PIN, LOW);
            noTone(BUZZER_PIN);
          }
        }
      } else {
        allOff();
        if (!stoppedMsgPrinted) {
          stoppedMsgPrinted = true;
          Serial.println(F("[Stop] Motor full stop. Press the button to restart."));
        }
        if (checkButtonRelease()) {
          Serial.println(F("[Reset] Return to initial waiting state"));
          stoppedMsgPrinted = false;
          currentState = STATE_WAITING;
        }
      }
      break;
    }
  }
}