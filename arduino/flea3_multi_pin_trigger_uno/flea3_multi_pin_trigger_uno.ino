// TTL pulse to Arduino Uno pins 5,7 at 100Hz
//set Port D pins 5,7 as OUTPUT

void setup()
{
  DDRD = B10100000;
}

void loop()
{
  PORTD = B10100000; //Set pins HIGH
  delayMicroseconds(5000);
  PORTD = B00000000; //Set pins LOW
  delayMicroseconds(5000);
}
