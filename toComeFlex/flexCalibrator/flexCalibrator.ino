/*
  SpectraSymbol Flex Sensor Calibration
  Sensor pin: A0

  Serial commands:
  F = Calibrate flat position
  H = Calibrate half-bent position
  B = Calibrate fully bent position
  P = Print calibration results
  R = Read current raw value
*/

const int FLEX_SENSOR_PIN = A0;

const int SAMPLE_COUNT = 100;
const int SAMPLE_DELAY_MS = 5;

int flatValue = -1;
int halfBentValue = -1;
int fullyBentValue = -1;

void setup() {
  Serial.begin(115200);
  pinMode(FLEX_SENSOR_PIN, INPUT);

  delay(1000);

  Serial.println("==================================");
  Serial.println(" FLEX SENSOR CALIBRATION");
  Serial.println("==================================");
  Serial.println("Place the sensor in the required position.");
  Serial.println();
  Serial.println("Send:");
  Serial.println("F = Record flat position");
  Serial.println("H = Record half-bent position");
  Serial.println("B = Record fully bent position");
  Serial.println("P = Print calibration results");
  Serial.println("R = Read current raw value");
  Serial.println("==================================");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    // Convert lowercase letters to uppercase
    if (command >= 'a' && command <= 'z') {
      command = command - 32;
    }

    switch (command) {
      case 'F':
        Serial.println("\nKeep the sensor FLAT...");
        delay(2000);

        flatValue = readAverageValue();

        Serial.print("Flat value recorded: ");
        Serial.println(flatValue);
        break;

      case 'H':
        Serial.println("\nKeep the sensor HALF-BENT...");
        delay(2000);

        halfBentValue = readAverageValue();

        Serial.print("Half-bent value recorded: ");
        Serial.println(halfBentValue);
        break;

      case 'B':
        Serial.println("\nKeep the sensor FULLY BENT...");
        delay(2000);

        fullyBentValue = readAverageValue();

        Serial.print("Fully bent value recorded: ");
        Serial.println(fullyBentValue);
        break;

      case 'P':
        printCalibrationResults();
        break;

      case 'R':
        Serial.print("Current raw value: ");
        Serial.println(analogRead(FLEX_SENSOR_PIN));
        break;
    }
  }
}

/*
  Reads multiple samples and returns their average.
*/
int readAverageValue() {
  long total = 0;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    total += analogRead(FLEX_SENSOR_PIN);
    delay(SAMPLE_DELAY_MS);
  }

  return total / SAMPLE_COUNT;
}

/*
  Prints all recorded calibration values.
*/
void printCalibrationResults() {
  Serial.println("\n==================================");
  Serial.println(" CALIBRATION RESULTS");
  Serial.println("==================================");

  Serial.print("Flat value      = ");
  printValue(flatValue);

  Serial.print("Half-bent value = ");
  printValue(halfBentValue);

  Serial.print("Fully bent value= ");
  printValue(fullyBentValue);

  Serial.println("==================================");

  if (flatValue >= 0 &&
      halfBentValue >= 0 &&
      fullyBentValue >= 0) {

    Serial.println("\nCopy these values into your program:");

    Serial.print("const int FLEX_FLAT = ");
    Serial.print(flatValue);
    Serial.println(";");

    Serial.print("const int FLEX_HALF_BENT = ");
    Serial.print(halfBentValue);
    Serial.println(";");

    Serial.print("const int FLEX_FULLY_BENT = ");
    Serial.print(fullyBentValue);
    Serial.println(";");
  } else {
    Serial.println("\nCalibration is incomplete.");
    Serial.println("Record F, H and B positions first.");
  }
}

void printValue(int value) {
  if (value < 0) {
    Serial.println("Not recorded");
  } else {
    Serial.println(value);
  }
}