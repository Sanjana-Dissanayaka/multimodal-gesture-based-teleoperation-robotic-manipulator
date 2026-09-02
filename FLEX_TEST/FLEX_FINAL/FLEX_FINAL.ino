const int FLEX1_PIN = A0;
const int FLEX2_PIN = A2;

float flex1Flat = 73.04;
float flex1Bent = 238.42;

float flex2Flat = 55.19;
float flex2Bent = 237.59;

float flex1Filtered = 0;
float flex2Filtered = 0;

const float alpha = 0.90;

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

void setup() {
  Serial.begin(115200);

  flex1Filtered = averageRead(FLEX1_PIN);
  flex2Filtered = averageRead(FLEX2_PIN);
}

void loop() {
  float flex1Raw = averageRead(FLEX1_PIN);
  float flex2Raw = averageRead(FLEX2_PIN);

  flex1Filtered = alpha * flex1Filtered + (1.0 - alpha) * flex1Raw;
  flex2Filtered = alpha * flex2Filtered + (1.0 - alpha) * flex2Raw;

  float flex1Angle = mapFloat(flex1Filtered, flex1Flat, flex1Bent, 0, 90);
  float flex2Angle = mapFloat(flex2Filtered, flex2Flat, flex2Bent, 0, 90);

  flex1Angle = constrain(flex1Angle, 0, 90);
  flex2Angle = constrain(flex2Angle, 0, 90);

  Serial.print("Flex1:");
  Serial.print(flex1Angle, 2);
  Serial.print(",");

  Serial.print("Flex2:");
  Serial.println(flex2Angle, 2);

  delay(20);
}