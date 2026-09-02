const int FLEX1_PIN = A1;  // Thumb
const int FLEX2_PIN = A2;  // Index
const int FLEX3_PIN = A3;  // Middle
const int FLEX4_PIN = A6;  // Ring
const int FLEX5_PIN = A7;  // Little

float flex1Flat = 18.00;
float flex1Bent = 118.00;

float flex2Flat = 160;
float flex2Bent = 80;

float flex3Flat = 160;
float flex3Bent = 80;

float flex4Flat = 160;
float flex4Bent = 80;

float flex5Flat = 160;
float flex5Bent = 80;

float flex1Filtered = 0;
float flex2Filtered = 0;
float flex3Filtered = 0;
float flex4Filtered = 0;
float flex5Filtered = 0;

float flex1PrevAngle = 0;
float flex2PrevAngle = 0;
float flex3PrevAngle = 0;
float flex4PrevAngle = 0;
float flex5PrevAngle = 0;

const float alpha = 0.92;        // higher = smoother
const float angleDeadband = 0.4; // removes small jitter

float averageRead(int pin)
{
  long sum = 0;

  for (int i = 0; i < 15; i++)
  {
    sum += analogRead(pin);
  }

  return (float)sum / 15.0;
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

float applyDeadband(float currentAngle, float previousAngle)
{
  if (abs(currentAngle - previousAngle) < angleDeadband)
  {
    return previousAngle;
  }

  return currentAngle;
}

void setup()
{
  Serial.begin(115200);

  flex1Filtered = averageRead(FLEX1_PIN);
  flex2Filtered = averageRead(FLEX2_PIN);
  flex3Filtered = averageRead(FLEX3_PIN);
  flex4Filtered = averageRead(FLEX4_PIN);
  flex5Filtered = averageRead(FLEX5_PIN);
}

void loop()
{
  float flex1Raw = averageRead(FLEX1_PIN);
  float flex2Raw = averageRead(FLEX2_PIN);
  float flex3Raw = averageRead(FLEX3_PIN);
  float flex4Raw = averageRead(FLEX4_PIN);
  float flex5Raw = averageRead(FLEX5_PIN);

  flex1Filtered = alpha * flex1Filtered + (1.0 - alpha) * flex1Raw;
  flex2Filtered = alpha * flex2Filtered + (1.0 - alpha) * flex2Raw;
  flex3Filtered = alpha * flex3Filtered + (1.0 - alpha) * flex3Raw;
  flex4Filtered = alpha * flex4Filtered + (1.0 - alpha) * flex4Raw;
  flex5Filtered = alpha * flex5Filtered + (1.0 - alpha) * flex5Raw;

  float flex1Angle = getFlexAngle(flex1Filtered, flex1Flat, flex1Bent);
  float flex2Angle = getFlexAngle(flex2Filtered, flex2Flat, flex2Bent);
  float flex3Angle = getFlexAngle(flex3Filtered, flex3Flat, flex3Bent);
  float flex4Angle = getFlexAngle(flex4Filtered, flex4Flat, flex4Bent);
  float flex5Angle = getFlexAngle(flex5Filtered, flex5Flat, flex5Bent);

  flex1Angle = applyDeadband(flex1Angle, flex1PrevAngle);
  flex2Angle = applyDeadband(flex2Angle, flex2PrevAngle);
  flex3Angle = applyDeadband(flex3Angle, flex3PrevAngle);
  flex4Angle = applyDeadband(flex4Angle, flex4PrevAngle);
  flex5Angle = applyDeadband(flex5Angle, flex5PrevAngle);

  flex1PrevAngle = flex1Angle;
  flex2PrevAngle = flex2Angle;
  flex3PrevAngle = flex3Angle;
  flex4PrevAngle = flex4Angle;
  flex5PrevAngle = flex5Angle;

  Serial.print("Thumb:");
  Serial.print(flex1Angle, 2);
  Serial.print(",");

  Serial.print("Index:");
  Serial.print(flex2Angle, 2);
  Serial.print(",");

  Serial.print("Middle:");
  Serial.print(flex3Angle, 2);
  Serial.print(",");

  Serial.print("Ring:");
  Serial.print(flex4Angle, 2);
  Serial.print(",");

  Serial.print("Little:");
  Serial.println(flex5Angle, 2);

  delay(10);
}