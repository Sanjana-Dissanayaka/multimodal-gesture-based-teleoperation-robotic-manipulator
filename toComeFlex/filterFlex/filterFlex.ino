/*
  Flex Sensor Filtered Reader

  Calibration:
  Flat       = 5
  Half-bent  = 73
  Fully bent = 127

  Filter:
  Exponential Moving Average (EMA)
*/

const int FLEX_SENSOR_PIN = A2;

const int FLEX_FLAT       = 5;
const int FLEX_HALF_BENT  = 73;
const int FLEX_FULLY_BENT = 127;

// EMA smoothing factor
// Smaller value = smoother but slower
// Larger value  = faster but noisier
const float ALPHA = 0.15;

float filteredFlexValue = 0.0;
bool filterInitialized = false;

unsigned long previousPrintTime = 0;
const unsigned long PRINT_INTERVAL_MS = 20;

void setup() {
  Serial.begin(115200);
  pinMode(FLEX_SENSOR_PIN, INPUT);

  Serial.println("Flex sensor filtered reader");
  Serial.println("Raw\tFiltered\tBendPercent");
}

void loop() {
  // Read the raw sensor value
  int rawFlexValue = analogRead(FLEX_SENSOR_PIN);

  // Initialize the filter using the first ADC reading
  if (!filterInitialized) {
    filteredFlexValue = rawFlexValue;
    filterInitialized = true;
  }

  // Exponential Moving Average filter
  filteredFlexValue =
      (ALPHA * rawFlexValue) +
      ((1.0 - ALPHA) * filteredFlexValue);

  // Convert filtered value into 0-100% bend
  float bendPercentage =
      calculateBendPercentage(filteredFlexValue);

  // Print at approximately 50 Hz
  if (millis() - previousPrintTime >= PRINT_INTERVAL_MS) {
    previousPrintTime = millis();

    Serial.print(rawFlexValue);
    Serial.print('\t');

    Serial.print(filteredFlexValue, 2);
    Serial.print('\t');

    Serial.println(bendPercentage, 1);
  }
}

/*
  Piecewise calibration:

  Flat to half-bent:
      5 to 73 becomes 0% to 50%

  Half-bent to fully bent:
      73 to 127 becomes 50% to 100%

  This preserves your measured half-bent calibration point.
*/
float calculateBendPercentage(float sensorValue) {
  // Limit values to the calibrated operating range
  if (sensorValue < FLEX_FLAT) {
    sensorValue = FLEX_FLAT;
  }

  if (sensorValue > FLEX_FULLY_BENT) {
    sensorValue = FLEX_FULLY_BENT;
  }

  float percentage;

  if (sensorValue <= FLEX_HALF_BENT) {
    percentage =
        50.0 *
        (sensorValue - FLEX_FLAT) /
        (FLEX_HALF_BENT - FLEX_FLAT);
  } else {
    percentage =
        50.0 +
        50.0 *
        (sensorValue - FLEX_HALF_BENT) /
        (FLEX_FULLY_BENT - FLEX_HALF_BENT);
  }

  return percentage;
}