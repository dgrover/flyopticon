void setup()
{
  DDRB = B11111111;
}

void loop()
{
  PORTB = B11100000;
  delayMicroseconds(3333);
  PORTB = B00000000;
  delayMicroseconds(3333);
}
