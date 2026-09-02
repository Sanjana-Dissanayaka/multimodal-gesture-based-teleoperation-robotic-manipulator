#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

// ---------------- IMU VARIABLES ----------------
float Ax, Ay, Az;
float Gx, Gy, Gz;

float rollG = 0, pitchG = 0, yawG = 0;
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;

unsigned long tStart = 0;

float xMax = 1.09, xMin = -1.01;
float yMax = 0.38, yMin = -1.68;
float zMax = 1.07, zMin = -1.07;

float xOffset = (xMax + xMin) / 2.0;
float yOffset = (yMax + yMin) / 2.0;
float zOffset = (zMax + zMin) / 2.0;

float xScale = 2.0 / (xMax - xMin);
float yScale = 2.0 / (yMax - yMin);
float zScale = 2.0 / (zMax - zMin);

float rollRAW, pitchRAW;
float rollComp = 0, pitchComp = 0;
float deltaRoll = 0, deltaPitch = 0;

Adafruit_MPU6050 mpu;

// ---------------- FLEX PINS ----------------
const int THUMB_PIN  = A1;
const int INDEX_PIN  = A2;
const int MIDDLE_PIN = A3;
const int RING_PIN   = A6;
const int LITTLE_PIN = A7;

// ---------------- CALIBRATION VALUES ----------------
float thumbFlat  = 5.00;
float thumbBent  = 150.00;

float indexFlat  = 160.00;
float indexBent  = 80.00;

float middleFlat = 160.00;
float middleBent = 80.00;

float ringFlat   = 160.00;
float ringBent   = 80.0;

float littleFlat = 160.00;
float littleBent = 80.0;

// ---------------- FILTER VARIABLES ----------------
float thumbFiltered = 0;
float indexFiltered = 0;
float middleFiltered = 0;
float ringFiltered = 0;
float littleFiltered = 0;

float thumbPrev = 0;
float indexPrev = 0;
float middlePrev = 0;
float ringPrev = 0;
float littlePrev = 0;

const float flexAlpha = 0.92;
const float angleDeadband = 0.4;

// ---------------- YAW RESET ----------------
const int yawResetPin = 8;
bool lastButtonState = HIGH;

// ---------------- FUNCTIONS ----------------
float averageRead(int pin) {
  long sum = 0;
  for (int i = 0; i < 15; i++) {
    sum += analogRead(pin);
  }
  return (float)sum / 15.0;
}

float mapFloat(float x, float inMin, float inMax, float outMin, float outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

float getFlexAngle(float filteredValue, float flatValue, float bentValue) {
  float angle;

  if (flatValue < bentValue) {
    angle = mapFloat(filteredValue, flatValue, bentValue, 0, 90);
  } else {
    angle = mapFloat(filteredValue, bentValue, flatValue, 90, 0);
  }

  return constrain(angle, 0, 90);
}

float applyDeadband(float currentAngle, float previousAngle) {
  if (abs(currentAngle - previousAngle) < angleDeadband) {
    return previousAngle;
  }
  return currentAngle;
}

void calibrateGyro() {
  Serial.println("Calibrating Gyro: Keep still");
  delay(1000);

  float sumX = 0, sumY = 0, sumZ = 0;
  const int numPoints = 100;

  for (int i = 0; i < numPoints; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sumX += g.gyro.x;
    sumY += g.gyro.y;
    sumZ += g.gyro.z;

    delay(10);
  }

  gyroXOffset = sumX / numPoints;
  gyroYOffset = sumY / numPoints;
  gyroZOffset = sumZ / numPoints;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(yawResetPin, INPUT_PULLUP);

  if (!mpu.begin()) {
    Serial.println("MPU6050 NOT FOUND");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  thumbFiltered  = averageRead(THUMB_PIN);
  indexFiltered  = averageRead(INDEX_PIN);
  middleFiltered = averageRead(MIDDLE_PIN);
  ringFiltered   = averageRead(RING_PIN);
  littleFiltered = averageRead(LITTLE_PIN);

  calibrateGyro();

  tStart = millis();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Ax = a.acceleration.x / 9.81;
  Ay = a.acceleration.y / 9.81;
  Az = a.acceleration.z / 9.81;

  Gx = g.gyro.x - gyroXOffset;
  Gy = -(g.gyro.y - gyroYOffset);
  Gz = g.gyro.z - gyroZOffset;

  unsigned long now = millis();
  float dt = (now - tStart) / 1000.0;
  tStart = now;

  yawG += dt * Gz * 180.0 / PI;

  deltaRoll  = dt * Gy * 180.0 / PI;
  deltaPitch = dt * Gx * 180.0 / PI;

  bool currentButtonState = digitalRead(yawResetPin);
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    yawG = 0;
    Serial.println("YawReset:1");
    delay(150);
  }
  lastButtonState = currentButtonState;

  Ax = xScale * (Ax - xOffset);
  Ay = yScale * (Ay - yOffset);
  Az = zScale * (Az - zOffset);

  pitchRAW = atan2(Ay, sqrt(Az * Az + Ax * Ax)) * 180.0 / PI;
  rollRAW  = atan2(Ax, sqrt(Az * Az + Ay * Ay)) * 180.0 / PI;

  rollComp  = 0.1 * rollRAW  + 0.9 * (rollComp + deltaRoll);
  pitchComp = 0.1 * pitchRAW + 0.9 * (pitchComp + deltaPitch);

  thumbFiltered  = flexAlpha * thumbFiltered  + (1.0 - flexAlpha) * averageRead(THUMB_PIN);
  indexFiltered  = flexAlpha * indexFiltered  + (1.0 - flexAlpha) * averageRead(INDEX_PIN);
  middleFiltered = flexAlpha * middleFiltered + (1.0 - flexAlpha) * averageRead(MIDDLE_PIN);
  ringFiltered   = flexAlpha * ringFiltered   + (1.0 - flexAlpha) * averageRead(RING_PIN);
  littleFiltered = flexAlpha * littleFiltered + (1.0 - flexAlpha) * averageRead(LITTLE_PIN);

  float thumbAngle  = applyDeadband(getFlexAngle(thumbFiltered, thumbFlat, thumbBent), thumbPrev);
  float indexAngle  = applyDeadband(getFlexAngle(indexFiltered, indexFlat, indexBent), indexPrev);
  float middleAngle = applyDeadband(getFlexAngle(middleFiltered, middleFlat, middleBent), middlePrev);
  float ringAngle   = applyDeadband(getFlexAngle(ringFiltered, ringFlat, ringBent), ringPrev);
  float littleAngle = applyDeadband(getFlexAngle(littleFiltered, littleFlat, littleBent), littlePrev);

  thumbPrev = thumbAngle;
  indexPrev = indexAngle;
  middlePrev = middleAngle;
  ringPrev = ringAngle;
  littlePrev = littleAngle;

  Serial.print("rollComp:");
  Serial.print(rollComp, 2);
  Serial.print(",");

  Serial.print("pitchComp:");
  Serial.print(pitchComp, 2);
  Serial.print(",");

  Serial.print("yawG:");
  Serial.print(yawG, 2);
  Serial.print(",");

  Serial.print("Thumb:");
  Serial.print(thumbAngle, 2);
  Serial.print(",");

  Serial.print("Index:");
  Serial.print(indexAngle, 2);
  Serial.print(",");

  Serial.print("Middle:");
  Serial.print(middleAngle, 2);
  Serial.print(",");

  Serial.print("Ring:");
  Serial.print(ringAngle, 2);
  Serial.print(",");

  Serial.print("Little:");
  Serial.println(littleAngle, 2);

  delay(10);
}