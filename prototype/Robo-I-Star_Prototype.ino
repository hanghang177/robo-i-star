#include <Servo.h>
Servo FrontR, FrontL, BackR, BackL
const int MotorL =5, MotorR = 6, MDL = 4 ,MDR = 7 
//MDL,MDR are motor direction left and motor direction right, when LOW motors go backwards, when HIGH motors go forwards
//MotorL, MotorR, are motor left and right outputs (PWM), they control speed of motors

int dir, fbspeed
//direction and forwardbackward speed inputs are between 1000,2000 pwm



void setup() {
  // put your setup code here, to run once:
  pinMode(MotorL,OUTPUT);
  pinMode(MotorR,OUTPUT);
  pinMode(MDL,OUTPUT);
  pinMode(MDR,OUTPUT);
  FrontR.attach(0)
  FrontL.attach(1)
  BackR.attach(2)
  BackR.attach(3)
  
  Serial.begin()//NEED TO FILL OUT THIS VALUE 
}

void loop() {
  // put your main code here, to run repeatedly:
  if(fbspeed<1500){
    backward((1500-fbspeed)/2) //the motor pins take in speeds between 0-255 so our pwm input is scaled down to between 0-250
  }else if(fbspeed>1500){
    forward((fbspeed-1500)/2)
  }

  //this makes all the wheels turn at the same time
  FrontL.writeMicroseconds(dir)
  FrontR.writeMicroseconds(dir)
  BackR.writeMicroseconds(dir) //if you comment out these BackR,BackL lines then just the front two wheels will turn
  BackL.writeMicroseconds(dir)
}

void forward(int a){
  digitalWrite(MDL,HIGH)
  digitalWrite(MDR,HIGH)
  analogWrite(MotorL,a)
  analogWrite(MotorR,a)
}

void backward(int a){
  digitalWrite(MDL,LOW)
  digitalWrite(MDR,LOW)
  analogWrite(MotorL,a)
  analogWrite(MotorR,a)
}
