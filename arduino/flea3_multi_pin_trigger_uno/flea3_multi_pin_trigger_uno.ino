// TTL pulse to Arduino Uno pins 3,5,7
//set Port D pins 3,5,7 as OUTPUT
void setup()
{
  DDRD = B10101000;
}

void loop()
{
  PORTD = B10101000; //Set pins HIGH
  delayMicroseconds(3333);
  PORTD = B00000000; //Set pins LOW
  delayMicroseconds(3333);
}
