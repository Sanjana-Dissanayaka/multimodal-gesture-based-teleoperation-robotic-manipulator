#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
float Ax;
float Ay;
float Az;

float Gx;
float Gy;
float Gz;

float rollG=0;
float pitchG=0;
float yawG=0;

int tStart = millis();

// If You want Calibrated results, you must rotate
// your board to all possible orientations, and
// record your velues of xMax, xMin, yMax, yMin
// zMax, zMin. I show you how to do that in LESSON
// 79 on my youtube channel. Put in your values and then
// uncomment the code below. Don't use my values, you have
// to measure your own for it to work.

float xMax= 1.01;
float xMin= -.99;
float yMax= .38;
float yMin= -1.63;
float zMax=.97 ;
float zMin= -1.05;

float xOffset = (xMax+xMin)/2;
float yOffset = (yMax+yMin)/2;
float zOffset = (zMax+zMin)/2;

float xScale = 2/(xMax -xMin);
float yScale = 2/(yMax - yMin);
float zScale = 2/(zMax -zMin);

float roll = 0;
float pitch = 0;
float rollRAW;
float pitchRAW;

Adafruit_MPU6050 mpu;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  mpu.begin();
  Serial.println("MPU6050 Started!");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

tStart = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Ax = a.acceleration.x / 9.81;
  Ay = a.acceleration.y / 9.81;
  Az = a.acceleration.z / 9.81;

  Gx = g.gyro.x;
  Gy = g.gyro.y;
  Gz = g.gyro.z;

rollG = rollG + (millis() - tStart)/1000. * Gy *360./2./3.14;
pitchG = pitchG + (millis() - tStart)/1000. * Gx *360./2./3.14;
yawG = yawG + (millis() - tStart)/1000. * Gz *360./2./3.14;
tStart = millis();

  //Uncomment these lines of code to do the callibration
  //This will only work if you have measured your max
  //and min values, and put them in at the top of the code
  Ax = xScale*(Ax-xOffset);
  Ay = yScale*(Ay-yOffset);
  Az = zScale*(Az-zOffset);

  pitchRAW = atan2(Ay, sqrt(Az * Az + Ax * Ax)) * 360 / (2 * 3.14);
  rollRAW = atan2(Ax, sqrt(Az * Az + Ay * Ay)) * 360 / (2 * 3.14);

  pitch = .75 * pitch + .25 * pitchRAW;
  roll = .75 * roll + .25 * rollRAW;

  Serial.print("Gx:");
  Serial.print(Gx);
  Serial.print(',');
  Serial.print("Gy:");
  Serial.print(Gy);
  Serial.print(',');
  Serial.print("Gz:");
  Serial.print(Gz);
  Serial.print(',');
  Serial.print("rollG:");
  Serial.print(rollG);
  Serial.print(',');
  Serial.print("pitchG:");
  Serial.print(pitchG);
  Serial.print(',');
  Serial.print("yawG:");
  Serial.print(yawG);
  Serial.print(',');
  Serial.print("UL:");
  Serial.print(45);
  Serial.print(',');
  Serial.print("LL:");
  Serial.println(-45);

  //   Serial.print("Ax:");
  //   Serial.print(Ax);
  //   Serial.print(',');
  //   Serial.print("Ay:");
  //   Serial.print(Ay);
  //   Serial.print(',');
  //    Serial.print("Az:");
  //   Serial.print(Az);
  //   Serial.print(',');
  //  Serial.print("UL:");
  //  Serial.print(1);
  //  Serial.print(',');
  //  Serial.print("LL:");
  //  Serial.println(-1);
  delay(10);
}