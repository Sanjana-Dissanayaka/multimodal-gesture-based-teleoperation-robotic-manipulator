#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

// ---------------- IMU VARIABLES ----------------

float Ax, Ay, Az;
float Gx, Gy, Gz;

float rollG = 0;
float pitchG = 0;
float yawG = 0;

float gyroXOffset = 0;
float gyroYOffset = 0;
float gyroZOffset = 0;

long int tStart = millis();

float xMax = 1.09;
float xMin = -1.01;
float yMax = .38;
float yMin = -1.68;
float zMax = 1.07;
float zMin = -1.07;

float xOffset = (xMax + xMin) / 2;
float yOffset = (yMax + yMin) / 2;
float zOffset = (zMax + zMin) / 2;

float xScale = 2 / (xMax - xMin);
float yScale = 2 / (yMax - yMin);
float zScale = 2 / (zMax - zMin);

float rollLP = 0;
float pitchLP = 0;

float rollRAW;
float pitchRAW;

float rollComp = 0;
float pitchComp = 0;

float deltaRoll = 0;
float deltaPitch = 0;

Adafruit_MPU6050 mpu;

// ---------------- FLEX VARIABLES ----------------

const int FLEX1_PIN = A2;
const int FLEX2_PIN = A3;

float flex1Flat = 73.04;
float flex1Bent = 238.42;

float flex2Flat = 55.19;
float flex2Bent = 237.59;

float flex1Filtered = 0;
float flex2Filtered = 0;

const float flexAlpha = 0.90;

// ---------------- YAW RESET ----------------

const int yawResetPin = 8;
bool lastButtonState = HIGH;

// ---------------- FUNCTIONS ----------------

float averageRead(int pin) {
  long sum = 0;

  for (int i = 0; i < 10; i++) {
    sum += analogRead(pin);
    delay(2);
  }

  return (float)sum / 10.0;
}

float mapFloat(float x, float inMin, float inMax, float outMin, float outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void calibrateGyro() {
  Serial.println("Calibrating the Gyro: Keep Completely Stationary");
  delay(1000);

  float sumX = 0;
  float sumY = 0;
  float sumZ = 0;

  int numPoints = 100;

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

// ---------------- SETUP ----------------

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(yawResetPin, INPUT_PULLUP);

  mpu.begin();

  Serial.println("MPU6050 Started!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  flex1Filtered = averageRead(FLEX1_PIN);
  flex2Filtered = averageRead(FLEX2_PIN);

  calibrateGyro();

  tStart = millis();
}

// ---------------- LOOP ----------------

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Ax = a.acceleration.x / 9.81;
  Ay = a.acceleration.y / 9.81;
  Az = a.acceleration.z / 9.81;

  Gx = g.gyro.x - gyroXOffset;
  Gy = -(g.gyro.y - gyroYOffset);
  Gz = g.gyro.z - gyroZOffset;

  rollG = rollG + (millis() - tStart) / 1000. * Gy * 360. / 2. / 3.14;
  pitchG = pitchG + (millis() - tStart) / 1000. * Gx * 360. / 2. / 3.14;
  yawG = yawG + (millis() - tStart) / 1000. * Gz * 360. / 2. / 3.14;

  deltaRoll = (millis() - tStart) / 1000. * Gy * 360. / 2. / 3.14;
  deltaPitch = (millis() - tStart) / 1000. * Gx * 360. / 2. / 3.14;

  tStart = millis();

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

  pitchRAW = atan2(Ay, sqrt(Az * Az + Ax * Ax)) * 360 / (2 * 3.14);
  rollRAW = atan2(Ax, sqrt(Az * Az + Ay * Ay)) * 360 / (2 * 3.14);

  pitchLP = .75 * pitchLP + .25 * pitchRAW;
  rollLP = .75 * rollLP + .25 * rollRAW;

  rollComp = .1 * rollRAW + .9 * (rollComp + deltaRoll);
  pitchComp = .1 * pitchRAW + .9 * (pitchComp + deltaPitch);

  float flex1Raw = averageRead(FLEX1_PIN);
  float flex2Raw = averageRead(FLEX2_PIN);

  flex1Filtered = flexAlpha * flex1Filtered + (1.0 - flexAlpha) * flex1Raw;
  flex2Filtered = flexAlpha * flex2Filtered + (1.0 - flexAlpha) * flex2Raw;

  float flex1Angle = mapFloat(flex1Filtered, flex1Flat, flex1Bent, 0, 90);
  float flex2Angle = mapFloat(flex2Filtered, flex2Flat, flex2Bent, 0, 90);

  flex1Angle = constrain(flex1Angle, 0, 90);
  flex2Angle = constrain(flex2Angle, 0, 90);

  Serial.print("rollComp:");
  Serial.print(rollComp, 2);
  Serial.print(",");

  Serial.print("pitchComp:");
  Serial.print(pitchComp, 2);
  Serial.print(",");

  Serial.print("yawG:");
  Serial.print(yawG, 2);
  Serial.print(",");

  Serial.print("Flex1:");
  Serial.print(flex1Angle, 2);
  Serial.print(",");

  Serial.print("Flex2:");
  Serial.println(flex2Angle, 2);

  delay(20);
}