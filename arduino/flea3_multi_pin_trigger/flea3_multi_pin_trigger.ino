// TTL pulse to Arduino Mega 2560 pins 0,2,3
// Unlike the Uno, the Mega ports map differently. 
// Arduino pins 0,2,3 map to PORTE 0,4,5 

//set Port E pins 0-7 as OUTPUT
void setup()
{
  DDRE = B11111111;
}

void loop()
{
  PORTE = B00110001; //Set pins 0,4,5 as HIGH
  delayMicroseconds(3333);
  PORTE = B00000000; //Set pins 0,4,5 as LOW
  delayMicroseconds(3333);
}
