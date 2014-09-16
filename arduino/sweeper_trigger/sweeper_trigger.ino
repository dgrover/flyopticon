#include <Servo.h> 

int incomingByte = 0;	              // for incoming serial data
int led = 13;
Servo myservo;                        // create servo object to control a servo 
int pos = 175;                        // variable to store the servo position 

void setup() {
	Serial.begin(9600);	// opens serial port, sets data rate to 9600 bps         
}

void loop() {

	// send data only when you receive data:
	if (Serial.available() > 0) {
		// read the incoming byte:
		incomingByte = Serial.read();
                if (incomingByte > 0) { 
                 motor();
                 }
               else{
                 }
		// say what you got:
		Serial.print("I received: ");
		Serial.println(incomingByte, DEC);
	}
}


void motor() 
{ 
    myservo.attach(9);                   // attaches the servo on pin 9 to the servo object 
    for(pos = 168; pos > 52; pos -= 1)   // goes from 52 degrees to 168 degrees 
  {                                      // in steps of 1 degrees
    myservo.write(pos);                  // tell servo to go to position in variable 'pos' 
    delay(70);                           // waits 70ms for the servo to reach the position 
  } 
  
  for(pos = 52; pos<168; pos+=1)         // goes from 52 degrees to 173 degrees 
  {                                      // in steps of 5 degrees
    myservo.write(pos);                  // tell servo to go to position in variable 'pos' 
    delay(8);                           // waits 40ms for the servo to reach the position 

    }      
  myservo.detach();
   
}
