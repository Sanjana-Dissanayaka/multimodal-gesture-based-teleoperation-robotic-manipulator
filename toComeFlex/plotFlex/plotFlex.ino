const int FLEX_SENSOR_PIN = A7;

void setup() {
  Serial.begin(115200);
}

void loop() {
  int rawFlexValue = analogRead(FLEX_SENSOR_PIN);

  // Only print the number so Arduino Serial Plotter
  // can display it clearly.
  Serial.println(rawFlexValue);

  delay(20);
}