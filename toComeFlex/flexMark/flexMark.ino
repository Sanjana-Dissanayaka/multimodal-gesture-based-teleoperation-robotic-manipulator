/*
  Single Flex-Sensor Gesture Recognition

  Calibration:
    Raw 5   = 0% bend
    Raw 73  = 50% bend
    Raw 127 = 100% bend

  Serial output:
    DATA,raw,filtered,bendPercentage,state
*/

const int FLEX_SENSOR_PIN = A7;

// Calibration values
const int FLEX_FLAT       = 5;
const int FLEX_HALF_BENT  = 73;
const int FLEX_FULLY_BENT = 127;

// EMA coefficient
// Lower value = smoother but slower response
const float ALPHA = 0.15;

// State hysteresis prevents state flickering
const float HYSTERESIS = 3.0;

// State transition thresholds
const float STATE_THRESHOLDS[4] = {
  20.0,  // FLAT -> SLIGHT_BEND
  40.0,  // SLIGHT_BEND -> HALF_BENT
  65.0,  // HALF_BENT -> BENT
  85.0   // BENT -> FULLY_BENT
};

enum BendState {
  FLAT,
  SLIGHT_BEND,
  HALF_BENT,
  BENT,
  FULLY_BENT
};

BendState currentState = FLAT;

float filteredValue = 0.0;
bool filterInitialized = false;

unsigned long previousSendTime = 0;
const unsigned long SEND_INTERVAL_MS = 25;


void setup() {
  Serial.begin(115200);
  pinMode(FLEX_SENSOR_PIN, INPUT);

  delay(1000);

  Serial.println("Flex sensor gesture recognition started");
}


void loop() {
  // Direct reading for monitoring
  int rawValue = analogRead(FLEX_SENSOR_PIN);

  // Remove isolated ADC spikes
  int medianValue = readMedianOfFive();

  // Initialize EMA from the first valid value
  if (!filterInitialized) {
    filteredValue = medianValue;
    filterInitialized = true;
  }

  // Exponential Moving Average filter
  filteredValue =
      (ALPHA * medianValue) +
      ((1.0 - ALPHA) * filteredValue);

  // Convert sensor reading to bend percentage
  float bendPercentage =
      flexToBendPercentage(filteredValue);

  // Recognize finger state
  updateGestureState(bendPercentage);

  // Send approximately 40 samples per second
  if (millis() - previousSendTime >= SEND_INTERVAL_MS) {
    previousSendTime = millis();

    Serial.print("DATA,");
    Serial.print(rawValue);
    Serial.print(",");

    Serial.print(filteredValue, 2);
    Serial.print(",");

    Serial.print(bendPercentage, 1);
    Serial.print(",");

    Serial.println(getStateName(currentState));
  }
}


/*
  Piecewise mapping preserves all three measured
  calibration points exactly:

    5   -> 0%
    73  -> 50%
    127 -> 100%
*/
float flexToBendPercentage(float sensorValue) {
  // Clamp values to calibration range
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
  Hysteresis-based state recognition.

  A state must cross slightly beyond a threshold
  before changing. This prevents rapid switching
  when the value is close to a boundary.
*/
void updateGestureState(float bendPercentage) {
  int stateIndex = static_cast<int>(currentState);

  // Check upward transitions
  while (
    stateIndex < static_cast<int>(FULLY_BENT) &&
    bendPercentage >=
      STATE_THRESHOLDS[stateIndex] + HYSTERESIS
  ) {
    stateIndex++;
  }

  // Check downward transitions
  while (
    stateIndex > static_cast<int>(FLAT) &&
    bendPercentage <
      STATE_THRESHOLDS[stateIndex - 1] - HYSTERESIS
  ) {
    stateIndex--;
  }

  currentState = static_cast<BendState>(stateIndex);
}


/*
  Median-of-five filter removes isolated high
  and low ADC spikes.
*/
int readMedianOfFive() {
  int samples[5];

  for (int i = 0; i < 5; i++) {
    samples[i] = analogRead(FLEX_SENSOR_PIN);
    delayMicroseconds(400);
  }

  // Sort samples from lowest to highest
  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 5; j++) {
      if (samples[j] < samples[i]) {
        int temporary = samples[i];
        samples[i] = samples[j];
        samples[j] = temporary;
      }
    }
  }

  return samples[2];
}


const char* getStateName(BendState state) {
  switch (state) {
    case FLAT:
      return "FLAT";

    case SLIGHT_BEND:
      return "SLIGHT_BEND";

    case HALF_BENT:
      return "HALF_BENT";

    case BENT:
      return "BENT";

    case FULLY_BENT:
      return "FULLY_BENT";

    default:
      return "UNKNOWN";
  }
}