#include <Servo.h> //Using the servo library

Servo servo; //The servo instance (that represents the actual servo);

int userinput;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); //Begin the UART Serial Communication
  servo.attach(9); //Attach pin 9 to the servo
}

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial.available()){ //If there is data transmission
    userinput = Serial.parseInt(); //Get the user input from serial
    servo.writeMicroseconds(userinput); //Rotate the servo
  }
}
