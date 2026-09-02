#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

String hc05Name = "HC-05";   // default HC-05 name

void setup() {
  Serial.begin(115200);

  SerialBT.begin("ESP32_Master", true); 
  Serial.println("ESP32 Bluetooth Master started");

  Serial.print("Connecting to ");
  Serial.println(hc05Name);

  bool connected = SerialBT.connect(hc05Name);

  if (connected) {
    Serial.println("Connected to HC-05");
  } else {
    Serial.println("Failed to connect. Check HC-05 name/password.");
  }
}

void loop() {
  if (SerialBT.connected()) {
    SerialBT.println("1");
    Serial.println("Sent: 1");
    delay(1000);

    while (SerialBT.available()) {
      Serial.write(SerialBT.read());
    }
  } else {
    Serial.println("Disconnected");
    delay(2000);
  }
}