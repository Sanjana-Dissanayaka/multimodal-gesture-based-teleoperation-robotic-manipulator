// SpectraSymbol FS-L-0055-253-ST
// Basic raw analog input reader

const int FLEX_SENSOR_PIN = A7;

void setup() {
  Serial.begin(115200);

  // Analog input pins do not normally require pinMode(),
  // but declaring it is acceptable.
  pinMode(FLEX_SENSOR_PIN, INPUT);

  Serial.println("Flex sensor raw input reader");
  Serial.println("Raw ADC value:");
}

void loop() {
  int rawFlexValue = analogRead(FLEX_SENSOR_PIN);

  Serial.println(rawFlexValue);

  delay(50);
}