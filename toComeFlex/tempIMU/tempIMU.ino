#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

// =====================================================
// USER SETTINGS
// =====================================================

const int YAW_RESET_PIN = 8;

// IMU calculation frequency
const unsigned long IMU_PERIOD_US = 5000;   // 200 Hz

// Serial transmission frequency
const unsigned long OUTPUT_PERIOD_MS = 25;  // 40 Hz

// true  = Arduino Serial Plotter format
// false = CSV format for future Python/sensor fusion
const bool SERIAL_PLOTTER_MODE = true;

// Complementary-filter time constant
// Larger = smoother, but slower accelerometer correction
const float COMPLEMENTARY_TAU = 0.40f;

// Enable slow gyro-bias correction while stationary
const bool ENABLE_ONLINE_GYRO_BIAS = true;

// Bias adjustment time constant
const float GYRO_BIAS_TAU = 20.0f;

// Standard gravitational acceleration
const float GRAVITY = 9.80665f;


// =====================================================
// ACCELEROMETER CALIBRATION VALUES
// =====================================================

// Keep your previously measured calibration values.
// The Y-axis calibration should preferably be verified again.

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
// MPU6050 OBJECT
// =====================================================

Adafruit_MPU6050 mpu;


// =====================================================
// SENSOR VARIABLES
// =====================================================

// Calibrated accelerometer values in g
float Ax = 0.0f;
float Ay = 0.0f;
float Az = 0.0f;

// Gyroscope rates in degrees per second
float gyroRollRate = 0.0f;
float gyroPitchRate = 0.0f;
float gyroYawRate = 0.0f;

// Accelerometer-based orientation
float rollAcc = 0.0f;
float pitchAcc = 0.0f;

// Complementary-filter orientation
float rollComp = 0.0f;
float pitchComp = 0.0f;

// Relative yaw angle
float yawG = 0.0f;

// Acceleration information
float accelerationMagnitude = 1.0f;
float accelerometerTrust = 1.0f;


// =====================================================
// GYROSCOPE CALIBRATION
// =====================================================

// Offsets are stored in radians per second because
// Adafruit MPU6050 returns gyroscope measurements in rad/s.

float gyroXOffset = 0.0f;
float gyroYOffset = 0.0f;
float gyroZOffset = 0.0f;


// =====================================================
// TIMING
// =====================================================

unsigned long previousImuTimeUs = 0;
unsigned long previousOutputTimeMs = 0;


// =====================================================
// STATIONARY DETECTION
// =====================================================

bool imuStationary = false;
float stationaryDuration = 0.0f;

// Sensor is considered stationary when acceleration
// is close to 1 g and angular velocity is very small.

const float STATIONARY_ACCEL_ERROR = 0.08f;
const float STATIONARY_GYRO_LIMIT = 0.75f;
const float STATIONARY_REQUIRED_TIME = 1.0f;


// =====================================================
// BUTTON DEBOUNCE
// =====================================================

bool lastRawButtonState = HIGH;
bool stableButtonState = HIGH;

unsigned long buttonChangeTimeMs = 0;
const unsigned long DEBOUNCE_TIME_MS = 30;


// =====================================================
// FUNCTION DECLARATIONS
// =====================================================

void calibrateGyroscope();
void initializeOrientation();
void updateImu(float dt);
void updateStationaryBias(
  float rawGx,
  float rawGy,
  float rawGz,
  float dt
);

void handleYawResetButton();
void printImuData();

float calculateAccelerometerTrust(float magnitude);
float wrapAngle180(float angle);


// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);

  Wire.begin();

  // Faster I2C communication for more consistent timing
  Wire.setClock(400000);

  pinMode(YAW_RESET_PIN, INPUT_PULLUP);

  Serial.println("Starting MPU6050...");

  if (!mpu.begin()) {
    Serial.println("MPU6050 not detected.");

    while (true) {
      delay(500);
    }
  }

  Serial.println("MPU6050 detected.");

  // Highest accelerometer precision
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);

  // Better gyro resolution than the original ±2000 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // Internal digital low-pass filter
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Allow the sensor to stabilize thermally
  Serial.println("Keep the IMU completely stationary...");
  delay(1500);

  calibrateGyroscope();
  initializeOrientation();

  previousImuTimeUs = micros();
  previousOutputTimeMs = millis();

  Serial.println("IMU ready.");
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

    /*
      Prevent an unusually long interruption from causing
      a large false integrated angle.
    */
    if (dt <= 0.0f || dt > 0.020f) {
      dt = IMU_PERIOD_US * 0.000001f;
    }

    updateImu(dt);
  }

  unsigned long currentTimeMs = millis();

  if (
    (unsigned long)(currentTimeMs - previousOutputTimeMs)
    >= OUTPUT_PERIOD_MS
  ) {
    previousOutputTimeMs = currentTimeMs;
    printImuData();
  }
}


// =====================================================
// INITIAL GYROSCOPE CALIBRATION
// =====================================================

void calibrateGyroscope() {
  const int CALIBRATION_SAMPLES = 1000;

  double sumX = 0.0;
  double sumY = 0.0;
  double sumZ = 0.0;

  double squareSumX = 0.0;
  double squareSumY = 0.0;
  double squareSumZ = 0.0;

  Serial.println("Calibrating gyroscope...");
  Serial.println("Do not move the sensor.");

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

    squareSumX += gyro.gyro.x * gyro.gyro.x;
    squareSumY += gyro.gyro.y * gyro.gyro.y;
    squareSumZ += gyro.gyro.z * gyro.gyro.z;

    delay(2);
  }

  gyroXOffset = sumX / CALIBRATION_SAMPLES;
  gyroYOffset = sumY / CALIBRATION_SAMPLES;
  gyroZOffset = sumZ / CALIBRATION_SAMPLES;

  float varianceX =
    (squareSumX / CALIBRATION_SAMPLES)
    - (gyroXOffset * gyroXOffset);

  float varianceY =
    (squareSumY / CALIBRATION_SAMPLES)
    - (gyroYOffset * gyroYOffset);

  float varianceZ =
    (squareSumZ / CALIBRATION_SAMPLES)
    - (gyroZOffset * gyroZOffset);

  if (varianceX < 0.0f) varianceX = 0.0f;
  if (varianceY < 0.0f) varianceY = 0.0f;
  if (varianceZ < 0.0f) varianceZ = 0.0f;

  float standardDeviationX =
    sqrt(varianceX) * RAD_TO_DEG;

  float standardDeviationY =
    sqrt(varianceY) * RAD_TO_DEG;

  float standardDeviationZ =
    sqrt(varianceZ) * RAD_TO_DEG;

  Serial.println("Gyroscope calibration completed.");

  Serial.print("X offset (rad/s): ");
  Serial.println(gyroXOffset, 6);

  Serial.print("Y offset (rad/s): ");
  Serial.println(gyroYOffset, 6);

  Serial.print("Z offset (rad/s): ");
  Serial.println(gyroZOffset, 6);

  Serial.print("X noise standard deviation (deg/s): ");
  Serial.println(standardDeviationX, 4);

  Serial.print("Y noise standard deviation (deg/s): ");
  Serial.println(standardDeviationY, 4);

  Serial.print("Z noise standard deviation (deg/s): ");
  Serial.println(standardDeviationZ, 4);

  if (
    standardDeviationX > 0.8f ||
    standardDeviationY > 0.8f ||
    standardDeviationZ > 0.8f
  ) {
    Serial.println(
      "Warning: Movement may have occurred during calibration."
    );
  }
}


// =====================================================
// INITIAL ORIENTATION
// =====================================================

void initializeOrientation() {
  const int INITIAL_SAMPLES = 200;

  float sumAx = 0.0f;
  float sumAy = 0.0f;
  float sumAz = 0.0f;

  Serial.println("Initializing roll and pitch...");

  for (int i = 0; i < INITIAL_SAMPLES; i++) {
    sensors_event_t acceleration;
    sensors_event_t gyro;
    sensors_event_t temperature;

    mpu.getEvent(
      &acceleration,
      &gyro,
      &temperature
    );

    float rawAx = acceleration.acceleration.x / GRAVITY;
    float rawAy = acceleration.acceleration.y / GRAVITY;
    float rawAz = acceleration.acceleration.z / GRAVITY;

    float calibratedAx = xScale * (rawAx - xOffset);
    float calibratedAy = yScale * (rawAy - yOffset);
    float calibratedAz = zScale * (rawAz - zOffset);

    sumAx += calibratedAx;
    sumAy += calibratedAy;
    sumAz += calibratedAz;

    delay(5);
  }

  Ax = sumAx / INITIAL_SAMPLES;
  Ay = sumAy / INITIAL_SAMPLES;
  Az = sumAz / INITIAL_SAMPLES;

  /*
    These equations preserve the axis convention
    used in your original program.
  */
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

  Serial.print("Initial roll: ");
  Serial.println(rollComp, 2);

  Serial.print("Initial pitch: ");
  Serial.println(pitchComp, 2);
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

  // Convert acceleration from m/s^2 to g
  float rawAx = acceleration.acceleration.x / GRAVITY;
  float rawAy = acceleration.acceleration.y / GRAVITY;
  float rawAz = acceleration.acceleration.z / GRAVITY;

  // Apply accelerometer offset and scale correction
  Ax = xScale * (rawAx - xOffset);
  Ay = yScale * (rawAy - yOffset);
  Az = zScale * (rawAz - zOffset);

  accelerationMagnitude =
    sqrt((Ax * Ax) + (Ay * Ay) + (Az * Az));

  // Residual gyro values after bias removal
  float correctedGx =
    gyro.gyro.x - gyroXOffset;

  float correctedGy =
    gyro.gyro.y - gyroYOffset;

  float correctedGz =
    gyro.gyro.z - gyroZOffset;

  /*
    Preserve your original gyro orientation:

      Roll  uses negative Y gyro.
      Pitch uses positive X gyro.
      Yaw   uses positive Z gyro.
  */
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

  /*
    Recalculate gyro rates because the online bias
    may have been updated.
  */
  correctedGx =
    gyro.gyro.x - gyroXOffset;

  correctedGy =
    gyro.gyro.y - gyroYOffset;

  correctedGz =
    gyro.gyro.z - gyroZOffset;

  gyroRollRate =
    -correctedGy * RAD_TO_DEG;

  gyroPitchRate =
    correctedGx * RAD_TO_DEG;

  gyroYawRate =
    correctedGz * RAD_TO_DEG;

  // Accelerometer orientation
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

  /*
    Time-dependent complementary-filter coefficient.

    At dt = 0.005 s and tau = 0.40 s:
    alpha is approximately 0.988.
  */
  float alpha =
    COMPLEMENTARY_TAU /
    (COMPLEMENTARY_TAU + dt);

  accelerometerTrust =
    calculateAccelerometerTrust(
      accelerationMagnitude
    );

  /*
    Reduce accelerometer correction when the hand is
    undergoing significant linear acceleration.
  */
  float accelerometerCorrection =
    (1.0f - alpha) * accelerometerTrust;

  // Gyroscope prediction
  float predictedRoll =
    rollComp + (gyroRollRate * dt);

  float predictedPitch =
    pitchComp + (gyroPitchRate * dt);

  // Adaptive complementary correction
  float rollError =
    wrapAngle180(rollAcc - predictedRoll);

  float pitchError =
    wrapAngle180(pitchAcc - predictedPitch);

  rollComp =
    predictedRoll
    + (accelerometerCorrection * rollError);

  pitchComp =
    predictedPitch
    + (accelerometerCorrection * pitchError);

  // Relative yaw from gyro only
  yawG += gyroYawRate * dt;
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
  float correctedGx =
    (rawGx - gyroXOffset) * RAD_TO_DEG;

  float correctedGy =
    (rawGy - gyroYOffset) * RAD_TO_DEG;

  float correctedGz =
    (rawGz - gyroZOffset) * RAD_TO_DEG;

  float totalAngularRate =
    sqrt(
      (correctedGx * correctedGx)
      + (correctedGy * correctedGy)
      + (correctedGz * correctedGz)
    );

  bool accelerationIsStable =
    fabs(accelerationMagnitude - 1.0f)
    < STATIONARY_ACCEL_ERROR;

  bool gyroIsStable =
    totalAngularRate
    < STATIONARY_GYRO_LIMIT;

  if (accelerationIsStable && gyroIsStable) {
    stationaryDuration += dt;
  } else {
    stationaryDuration = 0.0f;
  }

  imuStationary =
    stationaryDuration >= STATIONARY_REQUIRED_TIME;

  if (
    ENABLE_ONLINE_GYRO_BIAS &&
    imuStationary
  ) {
    /*
      Slowly move the bias toward the current stationary
      gyro reading. This reduces temperature-related drift.
    */
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
// ACCELEROMETER TRUST
// =====================================================

float calculateAccelerometerTrust(float magnitude) {
  float error = fabs(magnitude - 1.0f);

  /*
    Full accelerometer trust:
      acceleration magnitude within ±0.08 g

    Zero accelerometer trust:
      acceleration differs by at least 0.30 g
  */
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
      (error - FULL_TRUST_ERROR) /
      (ZERO_TRUST_ERROR - FULL_TRUST_ERROR)
    );
}


// =====================================================
// NON-BLOCKING YAW RESET BUTTON
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

      // Button has just been pressed
      if (stableButtonState == LOW) {
        yawG = 0.0f;
      }
    }
  }
}


// =====================================================
// SERIAL OUTPUT
// =====================================================

void printImuData() {
  if (SERIAL_PLOTTER_MODE) {
    Serial.print("rollComp:");
    Serial.print(rollComp, 2);
    Serial.print(',');

    Serial.print("pitchComp:");
    Serial.print(pitchComp, 2);
    Serial.print(',');

    Serial.print("yawG:");
    Serial.print(yawG, 2);
    Serial.print(',');

    Serial.print("accMagnitude:");
    Serial.print(accelerationMagnitude, 3);
    Serial.print(',');

    Serial.print("accTrust:");
    Serial.print(accelerometerTrust * 100.0f, 1);
    Serial.print(',');

    Serial.print("stationary:");
    Serial.print(imuStationary ? 20 : 0);
    Serial.print(',');

    Serial.print("UL:");
    Serial.print(90);
    Serial.print(',');

    Serial.print("LL:");
    Serial.println(-90);
  } else {
    /*
      Future Python/sensor-fusion CSV format:

      IMU,
      roll,
      pitch,
      yaw,
      rollRate,
      pitchRate,
      yawRate,
      Ax,
      Ay,
      Az,
      accelerationMagnitude,
      accelerometerTrust,
      stationary
    */

    Serial.print("IMU,");
    Serial.print(rollComp, 3);
    Serial.print(',');

    Serial.print(pitchComp, 3);
    Serial.print(',');

    Serial.print(yawG, 3);
    Serial.print(',');

    Serial.print(gyroRollRate, 3);
    Serial.print(',');

    Serial.print(gyroPitchRate, 3);
    Serial.print(',');

    Serial.print(gyroYawRate, 3);
    Serial.print(',');

    Serial.print(Ax, 4);
    Serial.print(',');

    Serial.print(Ay, 4);
    Serial.print(',');

    Serial.print(Az, 4);
    Serial.print(',');

    Serial.print(accelerationMagnitude, 4);
    Serial.print(',');

    Serial.print(accelerometerTrust, 3);
    Serial.print(',');

    Serial.println(imuStationary ? 1 : 0);
  }
}


// =====================================================
// ANGLE WRAPPING
// =====================================================

float wrapAngle180(float angle) {
  while (angle > 180.0f) {
    angle -= 360.0f;
  }

  while (angle < -180.0f) {
    angle += 360.0f;
  }

  return angle;
}