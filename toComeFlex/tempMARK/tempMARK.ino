#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

// =====================================================
// HARDWARE CONFIGURATION
// =====================================================

const int FLEX_SENSOR_PIN = A2;
const int YAW_RESET_PIN = 8;

Adafruit_MPU6050 mpu;

// IMU processing at 200 Hz
const unsigned long IMU_PERIOD_US = 5000;

// Serial output at 20 Hz
const unsigned long OUTPUT_PERIOD_MS = 50;

// =====================================================
// FLEX-SENSOR CALIBRATION
// =====================================================

const int FLEX_FLAT = 5;
const int FLEX_HALF_BENT = 73;
const int FLEX_FULLY_BENT = 127;

// EMA smoothing factor
const float FLEX_ALPHA = 0.15f;

// Prevent state flickering near thresholds
const float FLEX_HYSTERESIS = 3.0f;

float filteredFlexValue = 0.0f;
float bendPercentage = 0.0f;
bool flexFilterInitialized = false;

int rawFlexValue = 0;

// =====================================================
// FLEX STATES
// =====================================================

enum FingerState {
  FINGER_FLAT,
  FINGER_SLIGHT_BEND,
  FINGER_HALF_BENT,
  FINGER_BENT,
  FINGER_FULLY_BENT
};

FingerState currentFingerState = FINGER_FLAT;

const float FLEX_STATE_THRESHOLDS[4] = {
  20.0f,
  40.0f,
  65.0f,
  85.0f
};

// =====================================================
// ACCELEROMETER CALIBRATION
// =====================================================

const float GRAVITY = 9.80665f;

const float xMax = 1.05f;
const float xMin = -1.01f;

const float yMax = 0.38f;
const float yMin = -1.68f;

const float zMax = 1.00f;
const float zMin = -1.07f;

const float xOffset = (xMax + xMin) / 2.0f;
const float yOffset = (yMax + yMin) / 2.0f;
const float zOffset = (zMax + zMin) / 2.0f;

const float xScale = 2.0f / (xMax - xMin);
const float yScale = 2.0f / (yMax - yMin);
const float zScale = 2.0f / (zMax - zMin);

// =====================================================
// IMU VARIABLES
// =====================================================

float Ax = 0.0f;
float Ay = 0.0f;
float Az = 0.0f;

float gyroXOffset = 0.0f;
float gyroYOffset = 0.0f;
float gyroZOffset = 0.0f;

float gyroRollRate = 0.0f;
float gyroPitchRate = 0.0f;
float gyroYawRate = 0.0f;

float rollAcc = 0.0f;
float pitchAcc = 0.0f;

float rollComp = 0.0f;
float pitchComp = 0.0f;
float yawG = 0.0f;

float accelerationMagnitude = 1.0f;
float accelerometerTrust = 1.0f;

const float COMPLEMENTARY_TAU = 0.40f;

// =====================================================
// STATIONARY DETECTION AND BIAS CORRECTION
// =====================================================

bool imuStationary = false;
float stationaryDuration = 0.0f;

const float STATIONARY_ACCEL_ERROR = 0.08f;
const float STATIONARY_GYRO_LIMIT = 0.75f;
const float STATIONARY_REQUIRED_TIME = 1.0f;

const float GYRO_BIAS_TAU = 20.0f;

// =====================================================
// PALM ORIENTATION STATES
// =====================================================

enum PalmState {
  PALM_LEVEL,
  PALM_LEFT,
  PALM_RIGHT,
  PALM_FORWARD,
  PALM_BACK
};

PalmState currentPalmState = PALM_LEVEL;

// A palm tilt must exceed 25 degrees to enter a tilt state.
const float PALM_ENTRY_ANGLE = 25.0f;

// It returns to level below approximately 18 degrees.
const float PALM_EXIT_ANGLE = 18.0f;

// =====================================================
// MOTION STATES
// =====================================================

enum MotionState {
  MOTION_STILL,
  MOTION_MOVING,
  MOTION_YAW_POSITIVE,
  MOTION_YAW_NEGATIVE
};

MotionState currentMotionState = MOTION_STILL;

const float GENERAL_MOTION_THRESHOLD = 45.0f;
const float YAW_MOTION_THRESHOLD = 90.0f;

unsigned long motionHoldUntilMs = 0;

// =====================================================
// TIMING
// =====================================================

unsigned long previousImuTimeUs = 0;
unsigned long previousOutputTimeMs = 0;

// =====================================================
// YAW RESET BUTTON
// =====================================================

bool lastRawButtonState = HIGH;
bool stableButtonState = HIGH;

unsigned long buttonChangeTimeMs = 0;
const unsigned long DEBOUNCE_TIME_MS = 30;

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

  pinMode(FLEX_SENSOR_PIN, INPUT);
  pinMode(YAW_RESET_PIN, INPUT_PULLUP);

  Serial.println("Starting fused flex and IMU system...");

  if (!mpu.begin()) {
    Serial.println("MPU6050 not detected.");

    while (true) {
      delay(500);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("Keep the glove completely stationary.");
  delay(1500);

  calibrateGyroscope();
  initializeOrientation();
  initializeFlexSensor();

  previousImuTimeUs = micros();
  previousOutputTimeMs = millis();

  Serial.println("Fused gesture system ready.");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  handleYawResetButton();

  unsigned long currentTimeUs = micros();

  if (
    (unsigned long)(currentTimeUs - previousImuTimeUs)
    >= IMU_PERIOD_US
  ) {
    float dt =
      (currentTimeUs - previousImuTimeUs) * 0.000001f;

    previousImuTimeUs = currentTimeUs;

    // Reject abnormal timing intervals
    if (dt <= 0.0f || dt > 0.020f) {
      dt = IMU_PERIOD_US * 0.000001f;
    }

    updateFlexSensor();
    updateImu(dt);

    updateFingerState(bendPercentage);
    updatePalmState();
    updateMotionState();
  }

  unsigned long currentTimeMs = millis();

  if (
    (unsigned long)(currentTimeMs - previousOutputTimeMs)
    >= OUTPUT_PERIOD_MS
  ) {
    previousOutputTimeMs = currentTimeMs;

    sendFusionData();
  }
}

// =====================================================
// FLEX SENSOR
// =====================================================

void initializeFlexSensor() {
  long total = 0;

  for (int i = 0; i < 100; i++) {
    total += analogRead(FLEX_SENSOR_PIN);
    delay(2);
  }

  filteredFlexValue = total / 100.0f;
  flexFilterInitialized = true;
}

void updateFlexSensor() {
  rawFlexValue = analogRead(FLEX_SENSOR_PIN);

  int medianValue = readFlexMedianOfFive();

  if (!flexFilterInitialized) {
    filteredFlexValue = medianValue;
    flexFilterInitialized = true;
  }

  // Exponential Moving Average filter
  filteredFlexValue =
    (FLEX_ALPHA * medianValue)
    + ((1.0f - FLEX_ALPHA) * filteredFlexValue);

  bendPercentage =
    flexValueToPercentage(filteredFlexValue);
}

int readFlexMedianOfFive() {
  int samples[5];

  for (int i = 0; i < 5; i++) {
    samples[i] = analogRead(FLEX_SENSOR_PIN);
    delayMicroseconds(300);
  }

  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 5; j++) {
      if (samples[j] < samples[i]) {
        int temporary = samples[i];
        samples[i] = samples[j];
        samples[j] = temporary;
      }
    }
  }

  return samples[2];
}

float flexValueToPercentage(float sensorValue) {
  if (sensorValue <= FLEX_FLAT) {
    return 0.0f;
  }

  if (sensorValue >= FLEX_FULLY_BENT) {
    return 100.0f;
  }

  if (sensorValue <= FLEX_HALF_BENT) {
    return mapFloat(
      sensorValue,
      FLEX_FLAT,
      FLEX_HALF_BENT,
      0.0f,
      50.0f
    );
  }

  return mapFloat(
    sensorValue,
    FLEX_HALF_BENT,
    FLEX_FULLY_BENT,
    50.0f,
    100.0f
  );
}

void updateFingerState(float percentage) {
  int stateIndex = static_cast<int>(currentFingerState);

  // Increasing bend
  while (
    stateIndex < static_cast<int>(FINGER_FULLY_BENT)
    && percentage
       >= FLEX_STATE_THRESHOLDS[stateIndex]
          + FLEX_HYSTERESIS
  ) {
    stateIndex++;
  }

  // Decreasing bend
  while (
    stateIndex > static_cast<int>(FINGER_FLAT)
    && percentage
       < FLEX_STATE_THRESHOLDS[stateIndex - 1]
         - FLEX_HYSTERESIS
  ) {
    stateIndex--;
  }

  currentFingerState =
    static_cast<FingerState>(stateIndex);
}

// =====================================================
// GYROSCOPE CALIBRATION
// =====================================================

void calibrateGyroscope() {
  const int CALIBRATION_SAMPLES = 1000;

  double sumX = 0.0;
  double sumY = 0.0;
  double sumZ = 0.0;

  // Discard initial readings
  for (int i = 0; i < 100; i++) {
    sensors_event_t acceleration;
    sensors_event_t gyro;
    sensors_event_t temperature;

    mpu.getEvent(
      &acceleration,
      &gyro,
      &temperature
    );

    delay(2);
  }

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    sensors_event_t acceleration;
    sensors_event_t gyro;
    sensors_event_t temperature;

    mpu.getEvent(
      &acceleration,
      &gyro,
      &temperature
    );

    sumX += gyro.gyro.x;
    sumY += gyro.gyro.y;
    sumZ += gyro.gyro.z;

    delay(2);
  }

  gyroXOffset = sumX / CALIBRATION_SAMPLES;
  gyroYOffset = sumY / CALIBRATION_SAMPLES;
  gyroZOffset = sumZ / CALIBRATION_SAMPLES;
}

// =====================================================
// INITIAL IMU ORIENTATION
// =====================================================

void initializeOrientation() {
  const int SAMPLE_COUNT = 200;

  float sumAx = 0.0f;
  float sumAy = 0.0f;
  float sumAz = 0.0f;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    sensors_event_t acceleration;
    sensors_event_t gyro;
    sensors_event_t temperature;

    mpu.getEvent(
      &acceleration,
      &gyro,
      &temperature
    );

    float rawAx =
      acceleration.acceleration.x / GRAVITY;

    float rawAy =
      acceleration.acceleration.y / GRAVITY;

    float rawAz =
      acceleration.acceleration.z / GRAVITY;

    sumAx += xScale * (rawAx - xOffset);
    sumAy += yScale * (rawAy - yOffset);
    sumAz += zScale * (rawAz - zOffset);

    delay(5);
  }

  Ax = sumAx / SAMPLE_COUNT;
  Ay = sumAy / SAMPLE_COUNT;
  Az = sumAz / SAMPLE_COUNT;

  pitchAcc =
    atan2(
      Ay,
      sqrt((Az * Az) + (Ax * Ax))
    ) * RAD_TO_DEG;

  rollAcc =
    atan2(
      Ax,
      sqrt((Az * Az) + (Ay * Ay))
    ) * RAD_TO_DEG;

  rollComp = rollAcc;
  pitchComp = pitchAcc;
  yawG = 0.0f;
}

// =====================================================
// IMU UPDATE
// =====================================================

void updateImu(float dt) {
  sensors_event_t acceleration;
  sensors_event_t gyro;
  sensors_event_t temperature;

  mpu.getEvent(
    &acceleration,
    &gyro,
    &temperature
  );

  float rawAx =
    acceleration.acceleration.x / GRAVITY;

  float rawAy =
    acceleration.acceleration.y / GRAVITY;

  float rawAz =
    acceleration.acceleration.z / GRAVITY;

  Ax = xScale * (rawAx - xOffset);
  Ay = yScale * (rawAy - yOffset);
  Az = zScale * (rawAz - zOffset);

  accelerationMagnitude =
    sqrt((Ax * Ax) + (Ay * Ay) + (Az * Az));

  float correctedGx =
    gyro.gyro.x - gyroXOffset;

  float correctedGy =
    gyro.gyro.y - gyroYOffset;

  float correctedGz =
    gyro.gyro.z - gyroZOffset;

  // Preserve your original axis convention
  gyroRollRate =
    -correctedGy * RAD_TO_DEG;

  gyroPitchRate =
    correctedGx * RAD_TO_DEG;

  gyroYawRate =
    correctedGz * RAD_TO_DEG;

  updateStationaryBias(
    gyro.gyro.x,
    gyro.gyro.y,
    gyro.gyro.z,
    dt
  );

  // Recalculate after possible bias update
  correctedGx = gyro.gyro.x - gyroXOffset;
  correctedGy = gyro.gyro.y - gyroYOffset;
  correctedGz = gyro.gyro.z - gyroZOffset;

  gyroRollRate =
    -correctedGy * RAD_TO_DEG;

  gyroPitchRate =
    correctedGx * RAD_TO_DEG;

  gyroYawRate =
    correctedGz * RAD_TO_DEG;

  pitchAcc =
    atan2(
      Ay,
      sqrt((Az * Az) + (Ax * Ax))
    ) * RAD_TO_DEG;

  rollAcc =
    atan2(
      Ax,
      sqrt((Az * Az) + (Ay * Ay))
    ) * RAD_TO_DEG;

  float alpha =
    COMPLEMENTARY_TAU /
    (COMPLEMENTARY_TAU + dt);

  accelerometerTrust =
    calculateAccelerometerTrust(
      accelerationMagnitude
    );

  float accelerometerCorrection =
    (1.0f - alpha) * accelerometerTrust;

  float predictedRoll =
    rollComp + (gyroRollRate * dt);

  float predictedPitch =
    pitchComp + (gyroPitchRate * dt);

  float rollError =
    wrapAngle180(rollAcc - predictedRoll);

  float pitchError =
    wrapAngle180(pitchAcc - predictedPitch);

  rollComp =
    predictedRoll
    + accelerometerCorrection * rollError;

  pitchComp =
    predictedPitch
    + accelerometerCorrection * pitchError;

  yawG = wrapAngle180(
    yawG + gyroYawRate * dt
  );
}

// =====================================================
// ONLINE GYRO-BIAS CORRECTION
// =====================================================

void updateStationaryBias(
  float rawGx,
  float rawGy,
  float rawGz,
  float dt
) {
  float correctedX =
    (rawGx - gyroXOffset) * RAD_TO_DEG;

  float correctedY =
    (rawGy - gyroYOffset) * RAD_TO_DEG;

  float correctedZ =
    (rawGz - gyroZOffset) * RAD_TO_DEG;

  float totalAngularRate =
    sqrt(
      correctedX * correctedX
      + correctedY * correctedY
      + correctedZ * correctedZ
    );

  bool accelerationStable =
    fabs(accelerationMagnitude - 1.0f)
    < STATIONARY_ACCEL_ERROR;

  bool gyroStable =
    totalAngularRate < STATIONARY_GYRO_LIMIT;

  if (accelerationStable && gyroStable) {
    stationaryDuration += dt;
  } else {
    stationaryDuration = 0.0f;
  }

  imuStationary =
    stationaryDuration >= STATIONARY_REQUIRED_TIME;

  if (imuStationary) {
    float beta =
      dt / (GYRO_BIAS_TAU + dt);

    gyroXOffset +=
      beta * (rawGx - gyroXOffset);

    gyroYOffset +=
      beta * (rawGy - gyroYOffset);

    gyroZOffset +=
      beta * (rawGz - gyroZOffset);
  }
}

// =====================================================
// PALM ORIENTATION RECOGNITION
// =====================================================

void updatePalmState() {
  float absoluteRoll = fabs(rollComp);
  float absolutePitch = fabs(pitchComp);

  if (currentPalmState == PALM_LEVEL) {
    if (
      absoluteRoll < PALM_ENTRY_ANGLE
      && absolutePitch < PALM_ENTRY_ANGLE
    ) {
      return;
    }
  } else {
    if (
      absoluteRoll < PALM_EXIT_ANGLE
      && absolutePitch < PALM_EXIT_ANGLE
    ) {
      currentPalmState = PALM_LEVEL;
      return;
    }
  }

  // Use the largest tilt direction
  if (absoluteRoll >= absolutePitch) {
    if (rollComp >= 0.0f) {
      currentPalmState = PALM_LEFT;
    } else {
      currentPalmState = PALM_RIGHT;
    }
  } else {
    if (pitchComp >= 0.0f) {
      currentPalmState = PALM_FORWARD;
    } else {
      currentPalmState = PALM_BACK;
    }
  }
}

// =====================================================
// PALM MOTION RECOGNITION
// =====================================================

void updateMotionState() {
  float totalAngularRate =
    sqrt(
      gyroRollRate * gyroRollRate
      + gyroPitchRate * gyroPitchRate
      + gyroYawRate * gyroYawRate
    );

  unsigned long currentTimeMs = millis();

  if (gyroYawRate >= YAW_MOTION_THRESHOLD) {
    currentMotionState = MOTION_YAW_POSITIVE;
    motionHoldUntilMs = currentTimeMs + 150;
  }
  else if (gyroYawRate <= -YAW_MOTION_THRESHOLD) {
    currentMotionState = MOTION_YAW_NEGATIVE;
    motionHoldUntilMs = currentTimeMs + 150;
  }
  else if (totalAngularRate >= GENERAL_MOTION_THRESHOLD) {
    currentMotionState = MOTION_MOVING;
    motionHoldUntilMs = currentTimeMs + 100;
  }
  else if (
    (long)(currentTimeMs - motionHoldUntilMs) >= 0
  ) {
    currentMotionState = MOTION_STILL;
  }
}

// =====================================================
// ACCELEROMETER TRUST
// =====================================================

float calculateAccelerometerTrust(float magnitude) {
  float error = fabs(magnitude - 1.0f);

  const float FULL_TRUST_ERROR = 0.08f;
  const float ZERO_TRUST_ERROR = 0.30f;

  if (error <= FULL_TRUST_ERROR) {
    return 1.0f;
  }

  if (error >= ZERO_TRUST_ERROR) {
    return 0.0f;
  }

  return 1.0f -
    (
      (error - FULL_TRUST_ERROR)
      /
      (ZERO_TRUST_ERROR - FULL_TRUST_ERROR)
    );
}

// =====================================================
// YAW RESET BUTTON
// =====================================================

void handleYawResetButton() {
  bool rawButtonState =
    digitalRead(YAW_RESET_PIN);

  if (rawButtonState != lastRawButtonState) {
    buttonChangeTimeMs = millis();
    lastRawButtonState = rawButtonState;
  }

  if (
    millis() - buttonChangeTimeMs
    >= DEBOUNCE_TIME_MS
  ) {
    if (rawButtonState != stableButtonState) {
      stableButtonState = rawButtonState;

      if (stableButtonState == LOW) {
        yawG = 0.0f;
      }
    }
  }
}

// =====================================================
// SERIAL DATA
// =====================================================

void sendFusionData() {
  /*
    Output format:

    FUSION,
    rawFlex,
    filteredFlex,
    bendPercentage,
    fingerState,
    roll,
    pitch,
    yaw,
    rollRate,
    pitchRate,
    yawRate,
    palmState,
    motionState,
    accelerometerTrust,
    stationary,
    fusedGesture
  */

  Serial.print("FUSION,");
  Serial.print(rawFlexValue);
  Serial.print(',');

  Serial.print(filteredFlexValue, 2);
  Serial.print(',');

  Serial.print(bendPercentage, 1);
  Serial.print(',');

  Serial.print(getFingerStateName(currentFingerState));
  Serial.print(',');

  Serial.print(rollComp, 2);
  Serial.print(',');

  Serial.print(pitchComp, 2);
  Serial.print(',');

  Serial.print(yawG, 2);
  Serial.print(',');

  Serial.print(gyroRollRate, 2);
  Serial.print(',');

  Serial.print(gyroPitchRate, 2);
  Serial.print(',');

  Serial.print(gyroYawRate, 2);
  Serial.print(',');

  Serial.print(getPalmStateName(currentPalmState));
  Serial.print(',');

  Serial.print(getMotionStateName(currentMotionState));
  Serial.print(',');

  Serial.print(accelerometerTrust, 2);
  Serial.print(',');

  Serial.print(imuStationary ? 1 : 0);
  Serial.print(',');

  // Fused gesture label
  Serial.print(getFingerStateName(currentFingerState));
  Serial.print('_');
  Serial.print(getPalmStateName(currentPalmState));
  Serial.print('_');
  Serial.println(getMotionStateName(currentMotionState));
}

// =====================================================
// STATE NAMES
// =====================================================

const char* getFingerStateName(FingerState state) {
  switch (state) {
    case FINGER_FLAT:
      return "FLAT";

    case FINGER_SLIGHT_BEND:
      return "SLIGHT_BEND";

    case FINGER_HALF_BENT:
      return "HALF_BENT";

    case FINGER_BENT:
      return "BENT";

    case FINGER_FULLY_BENT:
      return "FULLY_BENT";

    default:
      return "UNKNOWN";
  }
}

const char* getPalmStateName(PalmState state) {
  switch (state) {
    case PALM_LEVEL:
      return "LEVEL";

    case PALM_LEFT:
      return "LEFT";

    case PALM_RIGHT:
      return "RIGHT";

    case PALM_FORWARD:
      return "FORWARD";

    case PALM_BACK:
      return "BACK";

    default:
      return "UNKNOWN";
  }
}

const char* getMotionStateName(MotionState state) {
  switch (state) {
    case MOTION_STILL:
      return "STILL";

    case MOTION_MOVING:
      return "MOVING";

    case MOTION_YAW_POSITIVE:
      return "YAW_POSITIVE";

    case MOTION_YAW_NEGATIVE:
      return "YAW_NEGATIVE";

    default:
      return "UNKNOWN";
  }
}

// =====================================================
// UTILITY FUNCTIONS
// =====================================================

float mapFloat(
  float input,
  float inputMin,
  float inputMax,
  float outputMin,
  float outputMax
) {
  return outputMin +
    (
      (input - inputMin)
      * (outputMax - outputMin)
      / (inputMax - inputMin)
    );
}

float wrapAngle180(float angle) {
  while (angle > 180.0f) {
    angle -= 360.0f;
  }

  while (angle < -180.0f) {
    angle += 360.0f;
  }

  return angle;
}