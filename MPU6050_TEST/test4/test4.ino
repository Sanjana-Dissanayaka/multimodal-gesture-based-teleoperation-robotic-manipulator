#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
float Ax;
float Ay;
float Az;

// If You want Calibrated results, you must rotate
// your board to all possible orientations, and 
// record your velues of xMax, xMin, yMax, yMin
// zMax, zMin. I show you how to do that in LESSON
// 79 on my youtube channel. Put in your values and then
// uncomment the code below. Don't use my values, you have
// to measure your own for it to work. (For GY-9250)

float xMax= .99;
float xMin= -1.02;
float yMax= 1.01;
float yMin= -1.00;
float zMax= 1.04 ;
float zMin= -.97;

// float xOffset = (xMax+xMin)/2;
// float yOffset = (yMax+yMin)/2;
// float zOffset = (zMax+zMin)/2;

// float xScale = 2/(xMax -xMin);
// float yScale = 2/(yMax - yMin);
// float zScale = 2/(zMax -zMin);

float roll=0;
float pitch=0;
float rollRAW;
float pitchRAW;

Adafruit_MPU6050 mpu;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  mpu.begin();
  Serial.println("MPU6050 Started!");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);


}

void loop() {
  // put your main code here, to run repeatedly:
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Ax=a.acceleration.x/9.81;
  Ay=a.acceleration.y/9.81;
  Az=a.acceleration.z/9.81;

//Uncomment these lines of code to do the callibration
//This will only work if you have measured your max
//and min values, and put them in at the top of the code
  // Ax = xScale*(Ax-xOffset);
  // Ay = yScale*(Ay-yOffset);
  // Az = zScale*(Az-zOffset);

  pitchRAW = atan2(Ay,sqrt(Az*Az+Ax*Ax))*360/(2*3.14);
  rollRAW = atan2(Ax,sqrt(Az*Az+Ay*Ay))*360/(2*3.14);

  pitch = .75*pitch + .25*pitchRAW;
  roll = .75*roll + .25*rollRAW;

  Serial.print("RollRAW:");
  Serial.print(rollRAW);
  Serial.print(',');
  Serial.print("PitchRAW:");
  Serial.print(pitchRAW);
    Serial.print(',');
  Serial.print("Roll:");
  Serial.print(roll);
  Serial.print(',');
  Serial.print("Pitch:");
  Serial.print(pitch);
    Serial.print(',');
   Serial.print("UL:");
 Serial.print(90);
 Serial.print(',');
 Serial.print("LL:");
 Serial.println(-90);



//   Serial.print("Ax:");
//   Serial.print(Ax);
//   Serial.print(',');
//   Serial.print("Ay:");
//   Serial.print(Ay);
//   Serial.print(',');
//     Serial.print("Az:");
//   Serial.print(Az);
//   Serial.print(',');
//  Serial.print("UL:");
//  Serial.print(1);
//  Serial.print(',');
//  Serial.print("LL:");
//  Serial.println(-1);

  delay(50);

}