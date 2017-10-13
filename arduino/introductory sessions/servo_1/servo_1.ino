#include <Servo.h> //Using the servo library

Servo servo; //The servo instance (that represents the actual servo);

void setup() {
  // put your setup code here, to run once:
  servo.attach(9); //Attach pin 9 to the servo
}

void loop() {
  // put your main code here, to run repeatedly:
  servo.write(90); //Turn the servo to 90 degrees
}
