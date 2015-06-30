int incomingByte = 0;   // for incoming serial data
const int led0 = 2;
const int led1 = 3;
const int led2 = 4;
const int led3 = 5;
int qt = 1000;

int i = 0;
int cycles = 10;

void setup() {
        Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
        pinMode(led0,OUTPUT);
        pinMode(led1,OUTPUT);
        pinMode(led2,OUTPUT);
        pinMode(led3,OUTPUT);
        digitalWrite(led0,HIGH);
        digitalWrite(led1,HIGH);
        digitalWrite(led2,HIGH);
        digitalWrite(led3,HIGH);
}
void loop() {

        // send data only when you receive data:
        if (Serial.available() > 0) {
                // read the incoming byte:
                incomingByte = Serial.read();
                if( incomingByte >48){
                  i = 0; 
                  while(i<cycles) {          
                  digitalWrite(led0,LOW);
                  delay(qt);
                  digitalWrite(led0,HIGH);
                
                  digitalWrite(led1,LOW);
                  delay(qt);
                  digitalWrite(led1,HIGH);
                
                  digitalWrite(led2,LOW);
                  delay(qt);
                  digitalWrite(led2,HIGH);
                
                  digitalWrite(led3,LOW);
                  delay(qt);
                  digitalWrite(led3,HIGH);
                
                  i++;
                
                
                } 
                }
               
        }
}
