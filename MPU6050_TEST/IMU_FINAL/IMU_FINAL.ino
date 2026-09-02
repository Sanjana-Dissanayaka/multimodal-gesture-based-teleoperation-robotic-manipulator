#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

float Ax;
float Ay;
float Az;

float Gx;
float Gy;
float Gz;

float rollG = 0;
float pitchG = 0;
float yawG = 0;

float gyroXOffset = 0;
float gyroYOffset = 0;
float gyroZOffset = 0;

long int tStart = millis();

// Calibrated accelerometer values
float xMax= 1.05;
float xMin= -1.01;
float yMax= .38;
float yMin= -1.68;
float zMax= 1.00 ;
float zMin= -1.07;

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

// Yaw reset button
const int yawResetPin = 8;
bool lastButtonState = HIGH;

Adafruit_MPU6050 mpu;

void calibrateGyro() {
  Serial.println("Calibrating the Gyro: Keep Completely Stationary");
  delay(1000);

  int i;
  float sumX = 0;
  float sumY = 0;
  float sumZ = 0;
  int numPoints = 100;

  for (i = 0; i < numPoints; i = i + 1) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sumX = sumX + g.gyro.x;
    sumY = sumY + g.gyro.y;
    sumZ = sumZ + g.gyro.z;

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

  mpu.begin();

  Serial.println("MPU6050 Started!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

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

  rollG = rollG + (millis() - tStart) / 1000. * Gy * 360. / 2. / 3.14;
  pitchG = pitchG + (millis() - tStart) / 1000. * Gx * 360. / 2. / 3.14;
  yawG = yawG + (millis() - tStart) / 1000. * Gz * 360. / 2. / 3.14;

  deltaRoll = (millis() - tStart) / 1000. * Gy * 360. / 2. / 3.14;
  deltaPitch = (millis() - tStart) / 1000. * Gx * 360. / 2. / 3.14;

  tStart = millis();

  // Manual yaw reset button on D8
  bool currentButtonState = digitalRead(yawResetPin);

  if (currentButtonState == LOW && lastButtonState == HIGH) {
    yawG = 0;
    Serial.println("Yaw Reset");
    delay(150);   // debounce
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

  Serial.print("rollComp:");
  Serial.print(rollComp);
  Serial.print(',');

  Serial.print("pitchComp:");
  Serial.print(pitchComp);
  Serial.print(',');

  Serial.print("yawG:");
  Serial.print(yawG);
  Serial.print(',');

  Serial.print("UL:");
  Serial.print(90);
  Serial.print(',');

  Serial.print("LL:");
  Serial.println(-90);

  delay(1);
}