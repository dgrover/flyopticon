// TTL pulse to Arduino Uno pins 5,7 at 100Hz
//set Port D pins 5,7 as OUTPUT

void setup()
{
  DDRD = B10100000;
}

void loop()
{
  //60fps - 8333
  //80fps - 6250
  //100fps - 5000
  PORTD = B10100000; //Set pins HIGH
  delayMicroseconds(8333); 
  PORTD = B00000000; //Set pins LOW
  delayMicroseconds(8333);
}
