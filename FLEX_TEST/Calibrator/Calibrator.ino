const int FLEX_PIN = A3;

const int SAMPLE_COUNT = 500;
const int SAMPLE_DELAY_MS = 2;
const int STABILIZE_SECONDS = 5;

void clearSerialBuffer() {
  while (Serial.available()) {
    Serial.read();
  }
}

void waitForKeyPress() {
  clearSerialBuffer();

  while (!Serial.available()) {
    // wait for user input
  }

  clearSerialBuffer();
}

void collectFlexData(float &averageValue, int &minValue, int &maxValue) {
  long sum = 0;

  minValue = 1023;
  maxValue = 0;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    int value = analogRead(FLEX_PIN);

    sum += value;

    if (value < minValue) {
      minValue = value;
    }

    if (value > maxValue) {
      maxValue = value;
    }

    delay(SAMPLE_DELAY_MS);
  }

  averageValue = (float)sum / SAMPLE_COUNT;
}

void measurePosition(const char* positionName,
                     float &averageValue,
                     int &minValue,
                     int &maxValue) {
  Serial.println();
  Serial.println("--------------------------------");
  Serial.print("Place flex sensor in ");
  Serial.print(positionName);
  Serial.println(" position.");
  Serial.println("Press any key then ENTER to start.");
  Serial.println("--------------------------------");

  waitForKeyPress();

  Serial.println();
  Serial.println("Stabilizing...");

  for (int i = STABILIZE_SECONDS; i > 0; i--) {
    Serial.print(i);
    Serial.println("...");
    delay(1000);
  }

  Serial.println("Collecting samples...");

  collectFlexData(averageValue, minValue, maxValue);

  Serial.println("Measurement complete.");
}

void setup() {
  Serial.begin(115200);

  Serial.println();
  Serial.println("================================");
  Serial.println(" ONE FLEX SENSOR CALIBRATION");
  Serial.println("================================");
  Serial.println("Connection:");
  Serial.println("Flex sensor voltage divider output -> A1");
  Serial.println();
}

void loop() {
  float flatAverage;
  float bentAverage;

  int flatMin;
  int flatMax;

  int bentMin;
  int bentMax;

  measurePosition("FULLY FLAT", flatAverage, flatMin, flatMax);

  measurePosition("FULLY BENT", bentAverage, bentMin, bentMax);

  Serial.println();
  Serial.println("============ RESULTS ============");
  Serial.println();

  Serial.println("FLAT POSITION:");
  Serial.print("Average = ");
  Serial.println(flatAverage, 2);
  Serial.print("Minimum = ");
  Serial.println(flatMin);
  Serial.print("Maximum = ");
  Serial.println(flatMax);
  Serial.print("Noise Range = ");
  Serial.println(flatMax - flatMin);

  Serial.println();

  Serial.println("BENT POSITION:");
  Serial.print("Average = ");
  Serial.println(bentAverage, 2);
  Serial.print("Minimum = ");
  Serial.println(bentMin);
  Serial.print("Maximum = ");
  Serial.println(bentMax);
  Serial.print("Noise Range = ");
  Serial.println(bentMax - bentMin);

  Serial.println();
  Serial.println("COPY THESE INTO YOUR MAIN CODE:");
  Serial.print("float flexFlat = ");
  Serial.print(flatAverage, 2);
  Serial.println(";");

  Serial.print("float flexBent = ");
  Serial.print(bentAverage, 2);
  Serial.println(";");

  Serial.println();
  Serial.println("Suggested mapping:");
  Serial.println("angle = map(filteredValue, flexFlat, flexBent, 0, 90);");

  Serial.println();
  Serial.println("================================");
  Serial.println("Reset board to calibrate again.");
  Serial.println("================================");

  while (1) {
    // stop here
  }
}