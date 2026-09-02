#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float xMax = -1000;
float xMin = 1000;
float yMax = -1000;
float yMin = 1000;
float zMax = -1000;
float zMin = 1000;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("MPU6050 NOT FOUND");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

  Serial.println("Accelerometer Calibration Started");
  Serial.println("Rotate sensor slowly in all directions.");
  delay(2000);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float Ax = a.acceleration.x / 9.81;
  float Ay = a.acceleration.y / 9.81;
  float Az = a.acceleration.z / 9.81;

  if (Ax > xMax) xMax = Ax;
  if (Ax < xMin) xMin = Ax;

  if (Ay > yMax) yMax = Ay;
  if (Ay < yMin) yMin = Ay;

  if (Az > zMax) zMax = Az;
  if (Az < zMin) zMin = Az;

  Serial.println("----- COPY THESE VALUES -----");

  Serial.print("float xMax = ");
  Serial.print(xMax, 5);
  Serial.println(";");

  Serial.print("float xMin = ");
  Serial.print(xMin, 5);
  Serial.println(";");

  Serial.print("float yMax = ");
  Serial.print(yMax, 5);
  Serial.println(";");

  Serial.print("float yMin = ");
  Serial.print(yMin, 5);
  Serial.println(";");

  Serial.print("float zMax = ");
  Serial.print(zMax, 5);
  Serial.println(";");

  Serial.print("float zMin = ");
  Serial.print(zMin, 5);
  Serial.println(";");

  Serial.println("-----------------------------");

  delay(500);
}