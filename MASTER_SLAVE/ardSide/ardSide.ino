#include <SoftwareSerial.h>

SoftwareSerial BT(10, 11); // RX, TX

void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  Serial.println("Arduino HC-05 ready");
}

void loop() {
  if (BT.available()) {
    char c = BT.read();
    Serial.print("From ESP32: ");
    Serial.println(c);

    if (c == '1') {
      Serial.println("Command 1 received");
      BT.println("Arduino received 1");
    }
  }

  if (Serial.available()) {
    char c = Serial.read();
    BT.write(c);
  }
}