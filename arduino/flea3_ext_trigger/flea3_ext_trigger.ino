int ledPin = 7;                 // LED connected to digital pin 13

void setup()
{
  pinMode(ledPin, OUTPUT);      // sets the digital pin as output
}

void loop()
{
  digitalWrite(ledPin, HIGH);   // sets the LED on
  delayMicroseconds(3333);                  // waits for a second
  digitalWrite(ledPin, LOW);    // sets the LED off
  delayMicroseconds(3333);                  // waits for a second
}
