#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
float Ax;
float Ay;
 
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
 
  Serial.print(Ax);
  Serial.print(',');
  Serial.println(Ay);
 
  delay(50);
 
}