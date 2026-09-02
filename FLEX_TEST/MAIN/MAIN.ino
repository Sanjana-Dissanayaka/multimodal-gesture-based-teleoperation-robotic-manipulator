const int flex1Pin = A3;


void setup() {
  Serial.begin(115200);
}

void loop() {
  int flex1 = analogRead(flex1Pin);

  Serial.println(flex1);
  //Serial.print("\t");
  //Serial.println(flex2);

  delay(100);
}