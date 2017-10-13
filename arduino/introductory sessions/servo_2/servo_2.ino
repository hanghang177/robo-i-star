#include <Servo.h> //Using the servo library

Servo servo; //The servo instance (that represents the actual servo);

void setup() {
  // put your setup code here, to run once:
  servo.attach(9); //Attach pin 9 to the servo
}

void loop() {
  // put your main code here, to run repeatedly:
  servo.write(0); //Turn the servo to 0 degrees
  delay(2000); //Delay 2000 milliseconds
  servo.write(180); //Turn the servo to 180 degrees
  delay(2000); //Delay 2000 milliseconds
}
