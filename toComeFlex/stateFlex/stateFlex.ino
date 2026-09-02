/*
  SpectraSymbol Flex Sensor Reader

  Calibration:
    Raw 5   = 0% bend   (flat)
    Raw 73  = 50% bend  (half-bent)
    Raw 127 = 100% bend (maximum comfortable bend)

  Filtering:
    1. Median-of-5 filter removes sudden spikes.
    2. EMA low-pass filter smooths the signal.

  Sensor behavior:
    ADC value increases as bending increases.
*/

const int FLEX_SENSOR_PIN = A2;

// Actual measured calibration values
const int FLEX_FLAT       = 5;
const int FLEX_HALF_BENT  = 73;
const int FLEX_FULLY_BENT = 127;

// EMA smoothing coefficient
// 0.10 = smoother and slower
// 0.15 = balanced
// 0.25 = faster and less smooth
const float ALPHA = 0.15;

float filteredValue = 0.0;
bool filterInitialized = false;

unsigned long previousPrintTime = 0;
const unsigned long PRINT_INTERVAL_MS = 20;

void setup() {
  Serial.begin(115200);
  pinMode(FLEX_SENSOR_PIN, INPUT);

  Serial.println("Flex Sensor Bend Reader");
  Serial.println("Raw\tMedian\tFiltered\tBend%\tState");
}

void loop() {
  // One direct reading for observation
  int rawValue = analogRead(FLEX_SENSOR_PIN);

  // Remove isolated ADC spikes
  int medianValue = readMedianOfFive();

  // Initialize EMA with the first valid reading
  if (!filterInitialized) {
    filteredValue = medianValue;
    filterInitialized = true;
  }

  // Exponential Moving Average low-pass filter
  filteredValue =
      (ALPHA * medianValue) +
      ((1.0 - ALPHA) * filteredValue);

  // Convert the filtered reading into 0-100% bend
  float bendPercentage = flexToBendPercentage(filteredValue);

  if (millis() - previousPrintTime >= PRINT_INTERVAL_MS) {
    previousPrintTime = millis();

    Serial.print(rawValue);
    Serial.print('\t');

    Serial.print(medianValue);
    Serial.print('\t');

    Serial.print(filteredValue, 2);
    Serial.print('\t');

    Serial.print(bendPercentage, 1);
    Serial.print('\t');

    Serial.println(getBendState(bendPercentage));
  }
}


/*
  Piecewise mapping:

  Raw 5 to 73:
    Maps from 0% to 50%.

  Raw 73 to 127:
    Maps from 50% to 100%.

  This ensures that the measured half-bent value
  corresponds exactly to 50%.
*/
float flexToBendPercentage(float sensorValue) {
  // Clamp readings outside the calibrated range
  if (sensorValue <= FLEX_FLAT) {
    return 0.0;
  }

  if (sensorValue >= FLEX_FULLY_BENT) {
    return 100.0;
  }

  // Flat to half-bent
  if (sensorValue <= FLEX_HALF_BENT) {
    return mapFloat(
      sensorValue,
      FLEX_FLAT,
      FLEX_HALF_BENT,
      0.0,
      50.0
    );
  }

  // Half-bent to fully bent
  return mapFloat(
    sensorValue,
    FLEX_HALF_BENT,
    FLEX_FULLY_BENT,
    50.0,
    100.0
  );
}


/*
  Floating-point linear mapping function.
*/
float mapFloat(
  float input,
  float inputMin,
  float inputMax,
  float outputMin,
  float outputMax
) {
  return outputMin +
         ((input - inputMin) *
          (outputMax - outputMin) /
          (inputMax - inputMin));
}


/*
  Takes five readings and returns the middle value.
  This removes isolated high or low spikes.
*/
int readMedianOfFive() {
  int samples[5];

  for (int i = 0; i < 5; i++) {
    samples[i] = analogRead(FLEX_SENSOR_PIN);
    delayMicroseconds(500);
  }

  // Sort the five readings
  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 5; j++) {
      if (samples[j] < samples[i]) {
        int temporary = samples[i];
        samples[i] = samples[j];
        samples[j] = temporary;
      }
    }
  }

  // Return the middle value
  return samples[2];
}


/*
  Human-readable bend classification.
*/
const char* getBendState(float bendPercentage) {
  if (bendPercentage < 20.0) {
    return "FLAT";
  }

  if (bendPercentage < 40.0) {
    return "SLIGHT_BEND";
  }

  if (bendPercentage < 65.0) {
    return "HALF_BENT";
  }

  if (bendPercentage < 85.0) {
    return "BENT";
  }

  return "FULLY_BENT";
}