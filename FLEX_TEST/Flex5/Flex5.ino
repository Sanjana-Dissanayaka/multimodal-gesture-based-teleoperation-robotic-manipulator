const int thumbPin  = A1;
const int indexPin  = A2;
const int middlePin = A3;
const int ringPin   = A6;
const int littlePin = A7;

void setup()
{
    Serial.begin(115200);

    Serial.println("Thumb\tIndex\tMiddle\tRing\tLittle");
}

void loop()
{
    int thumb  = analogRead(thumbPin);
    int index  = analogRead(indexPin);
    int middle = analogRead(middlePin);
    int ring   = analogRead(ringPin);
    int little = analogRead(littlePin);

    Serial.print(thumb);
    Serial.print('\t');

    Serial.print(index);
    Serial.print('\t');

    Serial.print(middle);
    Serial.print('\t');

    Serial.print(ring);
    Serial.print('\t');

    Serial.println(little);

    delay(50);
}