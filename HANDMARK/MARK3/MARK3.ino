#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

// ---------------- IMU VARIABLES ----------------
float Ax, Ay, Az;
float Gx, Gy, Gz;

float yawG = 0;
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
float thumbFlat  = 18.00;
float thumbBent  = 118.00;

float indexFlat  = 15.00;
float indexBent  = 120.00;

float middleFlat = 694.00;
float middleBent = 704.00;

float ringFlat   = 690.00;
float ringBent   = 730.00;

float littleFlat = 680.00;
float littleBent = 692.00;

// ---------------- FILTER VARIABLES ----------------
float thumbFiltered  = 0;
float indexFiltered  = 0;
float middleFiltered = 0;
float ringFiltered   = 0;
float littleFiltered = 0;

float thumbPrevAngle  = 0;
float indexPrevAngle  = 0;
float middlePrevAngle = 0;
float ringPrevAngle   = 0;
float littlePrevAngle = 0;

// General flex filter
const float alphaRise = 0.93;
const float alphaFall = 0.80;
const float adcDeadband = 2.0;
const float angleDeadband = 0.4;

// Little finger compensation
const float littleGain = 2.2;
const float littleAngleDeadband = 0.15;
const float littleAdcDeadband = 0.5;
const float littleAlphaRise = 0.86;
const float littleAlphaFall = 0.70;

// ---------------- BUTTONS ----------------
const int flexResetPin = 7;
const int yawResetPin  = 8;

bool lastFlexButtonState = HIGH;
bool lastYawButtonState  = HIGH;

// ---------------- FUNCTIONS ----------------
float averageRead(int pin)
{
  long sum = 0;

  for (int i = 0; i < 25; i++)
  {
    sum += analogRead(pin);
  }

  return (float)sum / 25.0;
}

float adaptiveFilter(float filtered, float raw)
{
  if (abs(raw - filtered) < adcDeadband)
  {
    return filtered;
  }

  float alpha = (raw > filtered) ? alphaRise : alphaFall;

  return alpha * filtered + (1.0 - alpha) * raw;
}

float adaptiveFilterLittle(float filtered, float raw)
{
  if (abs(raw - filtered) < littleAdcDeadband)
  {
    return filtered;
  }

  float alpha = (raw > filtered) ? littleAlphaRise : littleAlphaFall;

  return alpha * filtered + (1.0 - alpha) * raw;
}

float mapFloat(float x, float inMin, float inMax, float outMin, float outMax)
{
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

float getFlexAngle(float filteredValue, float flatValue, float bentValue)
{
  float angle;

  if (flatValue < bentValue)
  {
    angle = mapFloat(filteredValue, flatValue, bentValue, 0, 90);
  }
  else
  {
    angle = mapFloat(filteredValue, bentValue, flatValue, 90, 0);
  }

  return constrain(angle, 0, 90);
}

float applyAngleDeadband(float currentAngle, float previousAngle)
{
  if (abs(currentAngle - previousAngle) < angleDeadband)
  {
    return previousAngle;
  }

  return currentAngle;
}

float applyLittleDeadband(float currentAngle, float previousAngle)
{
  if (abs(currentAngle - previousAngle) < littleAngleDeadband)
  {
    return previousAngle;
  }

  return currentAngle;
}

void resetFlexToCurrentNeutral()
{
  thumbFiltered  = averageRead(THUMB_PIN);
  indexFiltered  = averageRead(INDEX_PIN);
  middleFiltered = averageRead(MIDDLE_PIN);
  ringFiltered   = averageRead(RING_PIN);
  littleFiltered = averageRead(LITTLE_PIN);

  thumbPrevAngle  = 0;
  indexPrevAngle  = 0;
  middlePrevAngle = 0;
  ringPrevAngle   = 0;
  littlePrevAngle = 0;

  Serial.println("FlexReset:1");
}

void calibrateGyro()
{
  Serial.println("Calibrating Gyro: Keep still");
  delay(1000);

  float sumX = 0, sumY = 0, sumZ = 0;
  const int numPoints = 100;

  for (int i = 0; i < numPoints; i++)
  {
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

// ---------------- SETUP ----------------
void setup()
{
  Serial.begin(115200);
  Wire.begin();

  pinMode(flexResetPin, INPUT_PULLUP);
  pinMode(yawResetPin, INPUT_PULLUP);

  if (!mpu.begin())
  {
    Serial.println("MPU6050 NOT FOUND");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  resetFlexToCurrentNeutral();
  calibrateGyro();

  tStart = millis();
}

// ---------------- LOOP ----------------
void loop()
{
  unsigned long now = millis();
  float dt = (now - tStart) / 1000.0;
  tStart = now;

  // ---------------- BUTTONS ----------------
  bool flexButtonState = digitalRead(flexResetPin);
  bool yawButtonState  = digitalRead(yawResetPin);

  if (flexButtonState == LOW && lastFlexButtonState == HIGH)
  {
    resetFlexToCurrentNeutral();
    delay(150);
  }

  if (yawButtonState == LOW && lastYawButtonState == HIGH)
  {
    yawG = 0;
    Serial.println("YawReset:1");
    delay(150);
  }

  lastFlexButtonState = flexButtonState;
  lastYawButtonState  = yawButtonState;

  // ---------------- IMU READ ----------------
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Ax = a.acceleration.x / 9.81;
  Ay = a.acceleration.y / 9.81;
  Az = a.acceleration.z / 9.81;

  Gx = g.gyro.x - gyroXOffset;
  Gy = -(g.gyro.y - gyroYOffset);
  Gz = g.gyro.z - gyroZOffset;

  yawG += dt * Gz * 180.0 / PI;

  deltaRoll  = dt * Gy * 180.0 / PI;
  deltaPitch = dt * Gx * 180.0 / PI;

  Ax = xScale * (Ax - xOffset);
  Ay = yScale * (Ay - yOffset);
  Az = zScale * (Az - zOffset);

  pitchRAW = atan2(Ay, sqrt(Az * Az + Ax * Ax)) * 180.0 / PI;
  rollRAW  = atan2(Ax, sqrt(Az * Az + Ay * Ay)) * 180.0 / PI;

  rollComp  = 0.1 * rollRAW  + 0.9 * (rollComp + deltaRoll);
  pitchComp = 0.1 * pitchRAW + 0.9 * (pitchComp + deltaPitch);

  // ---------------- FLEX FILTERING ----------------
  thumbFiltered  = adaptiveFilter(thumbFiltered,  averageRead(THUMB_PIN));
  indexFiltered  = adaptiveFilter(indexFiltered,  averageRead(INDEX_PIN));
  middleFiltered = adaptiveFilter(middleFiltered, averageRead(MIDDLE_PIN));
  ringFiltered   = adaptiveFilter(ringFiltered,   averageRead(RING_PIN));
  littleFiltered = adaptiveFilterLittle(littleFiltered, averageRead(LITTLE_PIN));

  // ---------------- ANGLE MAPPING ----------------
  float thumbAngle  = getFlexAngle(thumbFiltered,  thumbFlat,  thumbBent);
  float indexAngle  = getFlexAngle(indexFiltered,  indexFlat,  indexBent);
  float middleAngle = getFlexAngle(middleFiltered, middleFlat, middleBent);
  float ringAngle   = getFlexAngle(ringFiltered,   ringFlat,   ringBent);
  float littleAngle = getFlexAngle(littleFiltered, littleFlat, littleBent);

  // Boost weak little-finger response
  littleAngle *= littleGain;
  littleAngle = constrain(littleAngle, 0, 90);

  // ---------------- ANGLE DEADBAND ----------------
  thumbAngle  = applyAngleDeadband(thumbAngle,  thumbPrevAngle);
  indexAngle  = applyAngleDeadband(indexAngle,  indexPrevAngle);
  middleAngle = applyAngleDeadband(middleAngle, middlePrevAngle);
  ringAngle   = applyAngleDeadband(ringAngle,   ringPrevAngle);
  littleAngle = applyLittleDeadband(littleAngle, littlePrevAngle);

  thumbPrevAngle  = thumbAngle;
  indexPrevAngle  = indexAngle;
  middlePrevAngle = middleAngle;
  ringPrevAngle   = ringAngle;
  littlePrevAngle = littleAngle;

  // ---------------- SERIAL OUTPUT ----------------
  //Serial.print("T:");
  //Serial.print(now);
  //Serial.print(",");

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